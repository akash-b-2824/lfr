# server.py
import time
import uuid
import heapq
import threading
import random
from flask import Flask, request, jsonify, render_template_string
from flask_socketio import SocketIO

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# ---------------------------------------------------------
# 1. Map Graph
# ---------------------------------------------------------
GRAPH ={
        '11': {'s': '21'},
        '12': {'s': '22'},
        '13': {'s': '23'},
        '15': {'s': '25'},
        '21': {'n': '11', 'e': '22', 's': '31'},
        '22': {'n': '12', 's': '32', 'w': '21','e':'23'},
        '23': {'n': '13', 's': '33', 'w': '22'},
        '24': {'e': '25','s':'34'},
        '25': {'n': '15','s': '35','e': '26','w': '24'},
        '26': {'w': '25'},
        '31': {'n': '21', 'e': '32'},
        '32': {'n': '22','e':'33','w': '31'},
        '33': {'n': '23','s': '43','e': '34','w': '32'},
        '34': {'n': '24','s': '44','e': '35','w': '33'},
        '35': {'w':'34','n':'25','e': '36','s': '45'},
        '36': {'w':'35','s':'46'},
        '42': {'s': '52'},
        '43': {'n': '33','s':'53','e': '44'},
        '44': {'w': '43','n': '34','e': '45'},
        '45': {'w':'44','n':'35','s': '65','e': '46'},
        '46': {'w':'45','n':'36'},
        '51': {'e': '52'},
        '52': {'s': '62','e':'53','n': '42','w': '51'},
        '53': {'w': '52','n': '43','s': '63'},
        '56': {'s': '66'},
        '62': {'n': '52'},
        '63': {'s': '73','e':'64','n': '53'},
        '64': {'w': '63','e': '65','s': '84'},
        '65': {'n': '45','s': '75','e': '66','w': '64'},
        '66': {'w':'65','n':'56','s': '76'},
        '71': {'s': '81', 'e': '72'},
        '72': {'s': '82','e':'73','w': '71'},
        '73': {'w': '72','s': '83','n': '63'},
        '75': {'n': '65','s': '85','e': '76'},
        '76': {'w':'75','n':'66','s': '86'},
        '81': {'n': '71'},
        '82': {'n': '72'},
        '83': {'n': '73'},
        '84': {'n': '64'},
        '85': {'n':'75'},
        '86': {'n':'76'},

}

PARKING_NODES = ['81','82','83','84','85','86','11','12','13','15','26','31','46','51','56']

MAX_SEARCH_DEPTH = 60

# ---------------------------------------------------------
# 2. State
# ---------------------------------------------------------
robots = {}       # robot_id -> dict
jobs = {}         # job_id -> dict
job_queue = []    # queued user jobs
reservations = {} # (node, time) -> robot_id
state_lock = threading.Lock()

# ---------------------------------------------------------
# 3. Helpers / Pathfinding / Reservations
# ---------------------------------------------------------
def build_coords(graph):
    coords = {}
    for n in graph:
        try:
            coords[n] = (int(n[1]), int(n[0]))
        except:
            coords[n] = (0,0)
    return coords

NODE_COORDS = build_coords(GRAPH)

def get_manhattan_dist(a,b):
    ax,ay = NODE_COORDS.get(a,(0,0))
    bx,by = NODE_COORDS.get(b,(0,0))
    return abs(ax-bx) + abs(ay-by)

def is_safe(node, t, rid):
    owner = reservations.get((node,t))
    if owner and owner != rid:
        return False
    for orid, info in robots.items():
        if orid != rid and info.get('status') == 'idle' and info.get('node') == node:
            return False
    return True

def space_time_a_star(graph, start, end, t0, rid, max_time=MAX_SEARCH_DEPTH):
    open_set = []
    heapq.heappush(open_set, (0, 0, start, [start]))
    visited = set()
    while open_set:
        f, g, curr, path = heapq.heappop(open_set)
        current_time = t0 + g
        if curr == end:
            return path
        if g >= max_time:
            continue
        neighbors = list(graph[curr].values()) + [curr]  # include wait
        for nb in neighbors:
            nt = current_time + 1
            if (nb, nt) in visited:
                continue
            if is_safe(nb, nt, rid):
                visited.add((nb, nt))
                ng = g + 1
                if nb == curr:
                    ng += 1.1
                h = get_manhattan_dist(nb, end)
                heapq.heappush(open_set, (ng + h, ng, nb, path + [nb]))
    return None

def reserve_path_trajectory(path, t0, rid):
    # clear previous reservations for rid
    keys = [k for k,v in reservations.items() if v == rid]
    for k in keys:
        del reservations[k]
    for i, n in enumerate(path):
        reservations[(n, t0 + i)] = rid

def find_nearest_parking(node):
    candidates = []
    for p in PARKING_NODES:
        occupied = any(r.get('node')==p and r.get('status')=='idle' for r in robots.values())
        if not occupied:
            candidates.append((get_manhattan_dist(node,p), p))
    if not candidates:
        return None
    candidates.sort()
    return candidates[0][1]

# ---------------------------------------------------------
# 4. Instruction generation helpers
# ---------------------------------------------------------
CLOCKWISE = {'n':'e','e':'s','s':'w','w':'n'}
CCW = {v:k for k,v in CLOCKWISE.items()}
OPP = {'n':'s','s':'n','e':'w','w':'e'}

def direction_between(a,b):
    for d, nb in GRAPH[a].items():
        if nb == b:
            return d
    return None

def instruction_from_dirs(cur, target):
    if cur == target:
        return 'S'
    if CLOCKWISE[cur] == target:
        return 'R'
    if CCW[cur] == target:
        return 'L'
    if OPP[cur] == target:
        return 'U'
    return 'S'

def path_to_instr_list(path, initial_dir):
    instrs = []
    cur = initial_dir
    for i in range(len(path)-1):
        a = path[i]; b = path[i+1]
        target = direction_between(a,b)
        if not target:
            instrs.append('U')
            cur = OPP.get(cur, cur)
            continue
        cmd = instruction_from_dirs(cur, target)
        instrs.append(cmd)
        if cmd == 'R':
            cur = CLOCKWISE[cur]
        elif cmd == 'L':
            cur = CCW[cur]
        elif cmd == 'U':
            cur = OPP[cur]
        else:
            cur = cur
    return instrs, cur

def apply_instrs_to_dir(initial_dir, instrs):
    cur = initial_dir
    for c in instrs:
        if c == 'R':
            cur = CLOCKWISE.get(cur, cur)
        elif c == 'L':
            cur = CCW.get(cur, cur)
        elif c == 'U':
            cur = OPP.get(cur, cur)
    return cur

def random_color():
    return "#{:06x}".format(random.randint(0x444444, 0xFFFFFF))

def create_system_job(pickup, drop, rid):
    jid = str(uuid.uuid4())[:8]
    job = {'id': jid, 'pickup': pickup, 'drop': drop, 'submitted_ts': time.time(), 'status': 'assigned', 'assigned_robot': rid}
    jobs[jid] = job
    return job

# ---------------------------------------------------------
# 5. Allocator thread (keeps original behavior)
# ---------------------------------------------------------
def allocator_loop():
    while True:
        with state_lock:
            current_t = int(time.time())
            # cleanup old reservations
            old = [k for k in reservations if k[1] < current_t]
            for k in old: del reservations[k]
            pending = [j for j in job_queue if j['status'] == 'queued']
            for job in pending:
                idle = [r for r, info in robots.items() if info.get('status') == 'idle']
                if not idle:
                    break
                rid = idle[0]
                start_node = robots[rid]['node']
                path = space_time_a_star(GRAPH, start_node, job['pickup'], current_t, rid)
                if path:
                    arrival_t = current_t + len(path) - 1
                    path2 = space_time_a_star(GRAPH, job['pickup'], job['drop'], arrival_t, rid)
                    if path2:
                        full_path = path + path2[1:]
                        reserve_path_trajectory(full_path, current_t, rid)
                        job['assigned_robot'] = rid
                        job['status'] = 'assigned'
                        job['path'] = full_path
                        job_queue.remove(job)
                        robots[rid]['status'] = 'busy'
                        robots[rid]['current_job'] = job['id']
                        robots[rid]['current_path'] = full_path
                        socketio.emit('job_update', {'job': job})
                        socketio.emit('robot_update', {'robot': rid, 'info': robots[rid]})
        time.sleep(0.5)

threading.Thread(target=allocator_loop, daemon=True).start()

# ---------------------------------------------------------
# 6. HTTP API: request_path returns ONLY plan list + job_id
# ---------------------------------------------------------
@app.route('/request_path', methods=['POST'])
def request_path():
    """
    Request payload:
      { "robot_id": "r1", "node": "81", "dir":"s", "pickup":"71", "drop":"73" }

    Response:
      { "ok": true, "plan": [["81","S"],["71","L"],...["73","D"]], "job_id": "abcd" }
    """
    data = request.json or {}
    rid = data.get('robot_id')
    node = data.get('node')
    facing = (data.get('dir') or data.get('facing') or 's').lower()
    pickup = data.get('pickup')
    drop = data.get('drop')

    if not rid or rid not in robots:
        return jsonify({'error': 'unknown robot'}), 400
    if not node:
        return jsonify({'error': 'node missing'}), 400
    if not pickup or not drop:
        return jsonify({'error': 'pickup/drop missing'}), 400

    with state_lock:
        robots[rid]['node'] = node
        robots[rid]['dir'] = facing
        robots[rid]['last_seen'] = time.time()

        now = int(time.time())
        path_to_pickup = space_time_a_star(GRAPH, node, pickup, now, rid)
        if not path_to_pickup:
            return jsonify({'error': 'no path to pickup'}), 500
        arrive_t = now + len(path_to_pickup) - 1
        path_pickup_to_drop = space_time_a_star(GRAPH, pickup, drop, arrive_t, rid)
        if not path_pickup_to_drop:
            return jsonify({'error': 'no path pickup->drop'}), 500

        full_path = path_to_pickup + path_pickup_to_drop[1:]

        reserve_path_trajectory(full_path, now, rid)

        robots[rid]['status'] = 'busy'
        robots[rid]['current_path'] = full_path

        job = create_system_job(pickup, drop, rid)
        job['path'] = full_path
        robots[rid]['current_job'] = job['id']

        instr1, facing_after_pickup = path_to_instr_list(path_to_pickup, facing)
        instr2, _ = path_to_instr_list(path_pickup_to_drop, facing_after_pickup)
        # align instr2 so we don't duplicate the pickup edge
        full_instr = instr1 + (instr2[1:] if len(instr2) > 0 else [])

        plan = []
        for i in range(len(full_path)-1):
            plan.append([ full_path[i], full_instr[i] ])
        plan.append([ full_path[-1], 'D' ])

        socketio.emit('job_update', {'job': job})
        socketio.emit('robot_update', {'robot': rid, 'info': robots[rid]})
        print(plan)
        return jsonify({'ok': True, 'plan': plan, 'job_id': job['id']}), 200

# ---------------------------------------------------------
# rest of API: register, poll, update_location, report_execution, reset
# ---------------------------------------------------------
@app.route('/register_robot', methods=['POST'])
def register_robot():
    data = request.json or {}
    rid = data.get('robot_id') or str(uuid.uuid4())[:6]
    node = data.get('node') or '81'
    direction = (data.get('dir') or data.get('facing') or 's').lower()
    color = random_color()
    with state_lock:
        if rid in robots:
            color = robots[rid].get('color', color)
        robots[rid] = {'status': 'idle', 'node': node, 'last_seen': time.time(), 'color': color, 'current_path': [], 'dir': direction}
    socketio.emit('robot_update', {'robot': rid, 'info': robots[rid]})
    return jsonify({'robot_id': rid, 'color': color}), 200

@app.route('/submit_job', methods=['POST'])
def submit_job():
    data = request.json or {}
    if not data.get('pickup') or not data.get('drop'):
        return jsonify({'error': 'req'}), 400
    job_id = str(uuid.uuid4())[:8]
    job = {'id': job_id, 'pickup': data['pickup'], 'drop': data['drop'], 'submitted_ts': time.time(), 'status': 'queued', 'assigned_robot': None}
    with state_lock:
        job_queue.append(job)
        jobs[job_id] = job
    socketio.emit('job_update', {'job': job})
    return jsonify({'job_id': job_id}), 200

@app.route('/poll_task', methods=['GET'])
def poll_task():
    rid = request.args.get('robot_id')
    with state_lock:
        if rid not in robots:
            return jsonify({'error': 'unknown'}), 400
        robots[rid]['last_seen'] = time.time()
        jid = robots[rid].get('current_job')
        if jid:
            return jsonify({'job': jobs.get(jid)}), 200
        return jsonify({'job': None}), 200

@app.route('/update_location', methods=['POST'])
def update_location():
    data = request.json or {}
    rid = data.get('robot_id')
    node = data.get('node')
    status = data.get('status')
    reported_dir = (data.get('dir') or data.get('facing') or None)
    with state_lock:
        if rid not in robots:
            return jsonify({'error': 'unknown'}), 400
        robots[rid]['node'] = node
        robots[rid]['last_seen'] = time.time()
        if reported_dir:
            robots[rid]['dir'] = reported_dir.lower()
        path = robots[rid].get('current_path', [])
        if node in path:
            robots[rid]['current_path'] = path[path.index(node):]
        if status == 'job_done':
            jid = robots[rid].get('current_job')
            if jid and jid in jobs:
                jobs[jid]['status'] = 'done'
                socketio.emit('job_update', {'job': jobs[jid]})
            robots[rid]['status'] = 'idle'
            robots[rid]['current_path'] = []
            robots[rid].pop('current_job', None)
            keys = [k for k, v in reservations.items() if v == rid]
            for k in keys:
                del reservations[k]
            if node not in PARKING_NODES:
                parking_spot = find_nearest_parking(node)
                if parking_spot:
                    parking_job = create_system_job(node, parking_spot, rid)
                    current_t = int(time.time())
                    park_path = space_time_a_star(GRAPH, node, parking_spot, current_t, rid)
                    if park_path:
                        reserve_path_trajectory(park_path, current_t, rid)
                        robots[rid]['status'] = 'busy'
                        robots[rid]['current_job'] = parking_job['id']
                        robots[rid]['current_path'] = park_path
                        socketio.emit('job_update', {'job': parking_job})
                    else:
                        jobs[parking_job['id']]['status'] = 'failed'
        socketio.emit('robot_update', {'robot': rid, 'info': robots[rid]})
    return jsonify({'ok': True}), 200

@app.route('/report_execution', methods=['POST'])
def report_execution():
    data = request.json or {}
    rid = data.get('robot_id')
    jid = data.get('job_id')
    nodes_with_dir = data.get('nodes_with_dir')
    nodes = data.get('nodes_traversed', [])
    cmds = data.get('commands_executed', "")

    if not rid or rid not in robots:
        return jsonify({'error': 'unknown'}), 400

    with state_lock:
        if nodes_with_dir and isinstance(nodes_with_dir, list) and len(nodes_with_dir) > 0:
            last = nodes_with_dir[-1]
            robots[rid]['node'] = last.get('node', robots[rid].get('node'))
            robots[rid]['dir'] = (last.get('dir') or robots[rid].get('dir', 's')).lower()
            report = {'nodes_with_dir': nodes_with_dir, 'ts': time.time()}
        else:
            if isinstance(cmds, str):
                cmd_list = list(cmds)
            else:
                cmd_list = list(cmds or [])
            if nodes:
                robots[rid]['node'] = nodes[-1]
            cur = robots[rid].get('dir', 's')
            for c in cmd_list:
                if c == 'R':
                    cur = CLOCKWISE.get(cur, cur)
                elif c == 'L':
                    cur = CCW.get(cur, cur)
                elif c == 'U':
                    cur = OPP.get(cur, cur)
            robots[rid]['dir'] = cur
            report = {'nodes': nodes, 'cmds': cmd_list, 'ts': time.time()}

        if jid and jid in jobs:
            jobs[jid].setdefault('reports', []).append({'robot': rid, **report})
            jobs[jid]['status'] = 'done'

        robots[rid]['status'] = 'idle'
        robots[rid]['current_path'] = []
        robots[rid].pop('current_job', None)
        keys = [k for k, v in reservations.items() if v == rid]
        for k in keys:
            del reservations[k]

        socketio.emit('robot_update', {'robot': rid, 'info': robots[rid]})
    return jsonify({'ok': True}), 200

@app.route('/reset_sim', methods=['POST'])
def reset_sim():
    with state_lock:
        job_queue.clear()
        reservations.clear()
        for j in jobs.values():
            if j['status'] == 'assigned':
                j['status'] = 'failed'
                socketio.emit('job_update', {'job': j})
        for r in robots.values():
            r['status'] = 'idle'
            r['current_path'] = []
            r.pop('current_job', None)
            socketio.emit('robot_update', {'robot': r.get('id', 'unknown'), 'info': r})
    return jsonify({'ok': True}), 200

@socketio.on('connect')
def on_connect():
    with state_lock:
        socketio.emit('layout', {'nodes': NODE_COORDS, 'graph': GRAPH})
        socketio.emit('state_snapshot', {'robots': robots, 'jobs': list(jobs.values())})

# ---------------------------------------------------------
# 7. Full dashboard HTML (kept from your original UI)
# ---------------------------------------------------------
HTML_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Fleet Commander</title>
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.2/dist/css/bootstrap.min.css" rel="stylesheet">
    <script src="https://cdn.socket.io/4.7.2/socket.io.min.js"></script>
    <link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;600&display=swap" rel="stylesheet">
  <style>
    :root { --bg-color: #121212; --card-bg: #1e1e1e; --text-color: #e0e0e0; }
    body { font-family: 'Inter', sans-serif; background: var(--bg-color); color: var(--text-color); margin:0; height:100vh; }
    .dashboard-grid { display: grid; grid-template-columns: 300px 1fr 300px; gap: 1rem; padding: 1rem; height: 100%; }
    .card { background: var(--card-bg); border: 1px solid #333; color: var(--text-color); display: flex; flex-direction: column; border-radius: 8px; overflow: hidden; }
    .card-header { border-bottom: 1px solid #333; font-weight: 600; padding: 0.8rem; background: #252525; display: flex; justify-content: space-between; align-items: center; }
    .card-body { padding: 0.8rem; overflow-y: auto; flex: 1; }
    .form-control { background: #2c2c2c; border: 1px solid #444; color: #fff; margin-bottom: 0.5rem; }
    .form-control:focus { background: #333; color: #fff; border-color: #0d6efd; box-shadow: none; }
    .btn-primary { width: 100%; }
    
    /* Map */
    .map-container { background: #1a1a1a; width: 100%; height: 100%; cursor: grab; overflow: hidden; position: relative; }
    .map-container:active { cursor: grabbing; }
    .grid-line { stroke: #333; stroke-width: 1; }
    .node-circle { fill: #2c2c2c; stroke: #444; stroke-width: 1; }
    .node-text { fill: #666; font-size: 5px; text-anchor: middle; dominant-baseline: middle; pointer-events: none; }
    .edge-line { stroke: #444; stroke-width: 2; }
    .robot-group { transition: transform 0.3s linear; }
    .robot-circle { stroke: #fff; stroke-width: 1.5; }
    .robot-text { fill: #fff; font-size: 6px; font-weight: bold; text-anchor: middle; dominant-baseline: middle; }
    .robot-path { fill: none; stroke-width: 3; stroke-linecap: round; stroke-linejoin: round; opacity: 0.7; }

    /* Lists */
    .list-item { padding: 0.5rem; border-bottom: 1px solid #333; font-size: 0.85rem; }
    .badge-status { font-size: 0.65rem; padding: 2px 6px; border-radius: 4px; float: right; }
    .job-queued { border-left: 3px solid #ffc107; background: rgba(255, 193, 7, 0.1); }
    .job-assigned { border-left: 3px solid #198754; background: rgba(25, 135, 84, 0.1); }
    .job-done { border-left: 3px solid #6c757d; background: rgba(108, 117, 125, 0.1); opacity: 0.5; }
  </style>
</head>
<body>

  <div class="dashboard-grid">
    <div class="d-flex flex-column gap-3">
        <div class="card" style="height: auto;">
            <div class="card-header">System Status <span class="badge bg-success">Online</span></div>
            <div class="card-body">
                <div class="d-flex justify-content-between"><span>Active Robots</span> <strong id="stat-robots">0</strong></div>
                <div class="d-flex justify-content-between"><span>Active Jobs</span> <strong id="stat-jobs">0</strong></div>
                <div class="mt-3 small text-muted">Note: Robots finishing jobs in critical zones will auto-park.</div>
            </div>
        </div>
        <div class="card">
            <div class="card-header">Dispatch Job</div>
            <div class="card-body">
                <input id="pickup" class="form-control" placeholder="Pickup Node (e.g. 11)">
                <input id="drop" class="form-control" placeholder="Drop Node (e.g. 84)">
                <button onclick="submitJob()" class="btn btn-primary">Send Job</button>
                <button onclick="resetSim()" class="btn btn-outline-danger btn-sm mt-2 w-100">Reset Simulation</button>
            </div>
        </div>
    </div>

    <div class="card">
        <div class="card-header">
            <span>Facility Map</span>
            <button class="btn btn-sm btn-secondary" onclick="resetView()" style="width:auto;">Recenter</button>
        </div>
        <div class="map-container" id="mapwrap">
            <svg id="map" width="100%" height="100%"><g id="viewport"></g></svg>
        </div>
    </div>

    <div class="d-flex flex-column gap-3">
        <div class="card flex-grow-1">
            <div class="card-header">Robots</div>
            <div class="card-body p-0" id="robots-list"></div>
        </div>
        <div class="card flex-grow-1">
            <div class="card-header">Jobs</div>
            <div class="card-body p-0" id="jobs-list"></div>
        </div>
    </div>
  </div>

<script>
const socket = io();
let NODE_COORDS = {}, GRAPH_DATA = {}, ROBOTS = {}, JOBS = {};
const viewport = document.getElementById('viewport');
const scaleFactor = 40; 
const offsetX = 100, offsetY = 100; 
let transform = { x: 0, y: 0, k: 1 }; 
let isDragging = false, startDrag = { x: 0, y: 0 };

socket.on('connect', () => console.log('Connected'));
socket.on('layout', d => { NODE_COORDS = d.nodes; GRAPH_DATA = d.graph; drawMap(); });
socket.on('state_snapshot', d => { ROBOTS = d.robots||{}; JOBS={}; (d.jobs||[]).forEach(j=>JOBS[j.id]=j); updateUI(); });
socket.on('robot_update', d => { ROBOTS[d.robot] = d.info; updateUI(); });
socket.on('job_update', d => { JOBS[d.job.id] = d.job; updateUI(); });

function updateUI() {
    document.getElementById('stat-robots').innerText = Object.keys(ROBOTS).length;
    document.getElementById('stat-jobs').innerText = Object.values(JOBS).filter(j=>j.status==='assigned').length;
    
    // Robots List
    const rList = document.getElementById('robots-list');
    rList.innerHTML = '';
    Object.keys(ROBOTS).forEach(id => {
        const r = ROBOTS[id];
        const dot = `<span style="background:${r.color}; width:8px; height:8px; border-radius:50%; display:inline-block; margin-right:6px;"></span>`;
        rList.innerHTML += `<div class="list-item">
            ${dot}<strong>${id.substring(0,4)}</strong> <span class="text-muted">@ ${r.node}</span>
            <span class="badge-status bg-secondary">${r.status}</span>
        </div>`;
    });

    // Jobs List
    const jList = document.getElementById('jobs-list');
    jList.innerHTML = '';
    const sorted = Object.values(JOBS).sort((a,b) => b.submitted_ts - a.submitted_ts);
    sorted.forEach(j => {
        let cls = 'job-queued';
        if(j.status === 'assigned') cls = 'job-assigned';
        if(j.status === 'done') cls = 'job-done';
        jList.innerHTML += `<div class="list-item ${cls}">
            <div class="d-flex justify-content-between"><strong>#${j.id.substring(0,4)}</strong> <span>${j.status}</span></div>
            <div class="text-muted" style="font-size:0.8em">${j.pickup} &rarr; ${j.drop}</div>
        </div>`;
    });
    drawMap();
}

function nodeToPixel(id) {
    const n = NODE_COORDS[id];
    return n ? {x: offsetX + n[0]*scaleFactor, y: offsetY + n[1]*scaleFactor} : {x:0,y:0};
}

function drawMap() {
    while (viewport.firstChild) viewport.removeChild(viewport.firstChild);
    
    // Edges
    for(let u in GRAPH_DATA) {
        const p1 = nodeToPixel(u);
        for(let d in GRAPH_DATA[u]) {
            const v = GRAPH_DATA[u][d];
            if(NODE_COORDS[v]) createLine(p1, nodeToPixel(v), 'edge-line');
        }
    }

    // Robot Paths
    for(let id in ROBOTS) {
        const r = ROBOTS[id];
        if(r.current_path && r.current_path.length > 0) {
            let pts = `${nodeToPixel(r.node).x},${nodeToPixel(r.node).y} `;
            r.current_path.forEach(n => { const p = nodeToPixel(n); pts += `${p.x},${p.y} `; });
            const line = document.createElementNS('http://www.w3.org/2000/svg','polyline');
            line.setAttribute('points', pts);
            line.setAttribute('class', 'robot-path');
            line.setAttribute('stroke', r.color);
            viewport.appendChild(line);
        }
    }

    // Nodes
    for(let n in NODE_COORDS) {
        const p = nodeToPixel(n);
        const g = createGroup(p.x, p.y);
        const c = document.createElementNS('http://www.w3.org/2000/svg','circle');
        c.setAttribute('r', 4); c.setAttribute('class', 'node-circle');
        // Highlight parking nodes slightly
        if(['11', '12', '13', '15', '26', '31', '46', '51', '56', '81', '82', '83', '84', '85', '86'].includes(n)) {
            c.setAttribute('fill', '#333');
        }
        const t = document.createElementNS('http://www.w3.org/2000/svg','text');
        t.textContent = n; t.setAttribute('class', 'node-text');
        g.appendChild(c); g.appendChild(t);
        viewport.appendChild(g);
    }

    // Robots
    for(let id in ROBOTS) {
        const r = ROBOTS[id];
        const p = nodeToPixel(r.node);
        const g = createGroup(p.x, p.y);
        g.setAttribute('class', 'robot-group');
        const c = document.createElementNS('http://www.w3.org/2000/svg','circle');
        c.setAttribute('r', 8); c.setAttribute('class', 'robot-circle');
        c.setAttribute('fill', r.color);
        const t = document.createElementNS('http://www.w3.org/2000/svg','text');
        t.textContent = id.substring(0,2); t.setAttribute('class', 'robot-text');
        g.appendChild(c); g.appendChild(t);
        viewport.appendChild(g);
    }
}

function createLine(p1, p2, cls) {
    const l = document.createElementNS('http://www.w3.org/2000/svg','line');
    l.setAttribute('x1', p1.x); l.setAttribute('y1', p1.y); l.setAttribute('x2', p2.x); l.setAttribute('y2', p2.y);
    l.setAttribute('class', cls);
    viewport.appendChild(l);
}
function createGroup(x, y) {
    const g = document.createElementNS('http://www.w3.org/2000/svg','g');
    g.setAttribute('transform', `translate(${x},${y})`);
    return g;
}
function submitJob() {
    const p = document.getElementById('pickup').value, d = document.getElementById('drop').value;
    if(p && d) fetch('/submit_job', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({pickup:p, drop:d})});
}
function resetSim() { fetch('/reset_sim', {method:'POST'}); }

// Zoom/Pan
function updateTransform() { viewport.setAttribute('transform', `translate(${transform.x}, ${transform.y}) scale(${transform.k})`); }
function resetView() { transform = {x:0, y:0, k:1}; updateTransform(); }
const svg = document.getElementById('map');
svg.addEventListener('mousedown', e => { isDragging=true; startDrag={x:e.clientX-transform.x, y:e.clientY-transform.y}; });
window.addEventListener('mousemove', e => { if(!isDragging)return; transform.x=e.clientX-startDrag.x; transform.y=e.clientY-startDrag.y; updateTransform(); });
window.addEventListener('mouseup', () => isDragging=false);
svg.addEventListener('wheel', e => { e.preventDefault(); transform.k *= (1 + (e.deltaY>0?-0.1:0.1)); updateTransform(); });
</script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_PAGE)

# ---------------------------------------------------------
# 8. Run
# ---------------------------------------------------------
if __name__ == '__main__':
    # Note: Your code actually runs on 8080, so I updated the print statement
    print("Server running on port 8080...") 
    socketio.run(app, host='0.0.0.0', port=8080)

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
GRAPH = {
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

def build_plan_array(path, instr_list):
    """Zips path and instructions into [[node, cmd], ...] format"""
    plan = []
    for i in range(len(path)-1):
        plan.append([path[i], instr_list[i]])
    # Add final step
    if len(path) > 0:
        plan.append([path[-1], 'D'])
    return plan

def plan_to_str(plan):
    try:
        return ' '.join([f"{p[0]} {p[1]}" for p in plan])
    except:
        return ""

def random_color():
    return "#{:06x}".format(random.randint(0x444444, 0xFFFFFF))

def create_system_job(pickup, drop, rid):
    jid = str(uuid.uuid4())[:8]
    job = {'id': jid, 'pickup': pickup, 'drop': drop, 'submitted_ts': time.time(), 'status': 'assigned' if rid else 'queued', 'assigned_robot': rid, 'progress_index': None}
    jobs[jid] = job
    return job

# ---------------------------------------------------------
# 5. Allocator thread (assigns idle robots)
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
                # pick nearest idle robot by manhattan
                idle.sort(key=lambda rid: get_manhattan_dist(robots[rid]['node'], job['pickup']))
                rid = idle[0]
                start_node = robots[rid]['node']
                start_dir = robots[rid].get('dir', 's')

                # 1. Path to pickup
                path1 = space_time_a_star(GRAPH, start_node, job['pickup'], current_t, rid)
                if path1:
                    arrival_t = current_t + len(path1) - 1
                    # 2. Path to drop
                    path2 = space_time_a_star(GRAPH, job['pickup'], job['drop'], arrival_t, rid)
                    if path2:
                        full_path = path1 + path2[1:]
                        reserve_path_trajectory(full_path, current_t, rid)
                        
                        # ---- BUILD PLAN HERE ----
                        instr1, facing_after_pickup = path_to_instr_list(path1, start_dir)
                        instr2, _ = path_to_instr_list(path2, facing_after_pickup)

                        # FIX: append entire instr2 (not instr2[1:]) so instruction count matches full_path edges
                        full_instr = instr1 + instr2

                        plan = []
                        if len(full_path) - 1 == len(full_instr):
                            for i in range(len(full_path)-1):
                                plan.append([full_path[i], full_instr[i]])
                            plan.append([full_path[-1], 'D'])
                        else:
                            # fallback: create a simple final D step
                            plan.append([full_path[-1], 'D'])

                        job['assigned_robot'] = rid
                        job['status'] = 'assigned'
                        job['path'] = full_path
                        job['plan'] = plan
                        job['plan_str'] = plan_to_str(plan)
                        job['progress_index'] = None
                        job_queue.remove(job)

                        robots[rid]['status'] = 'busy'
                        robots[rid]['current_job'] = job['id']
                        robots[rid]['current_path'] = full_path

                        socketio.emit('job_update', {'job': job})
                        socketio.emit('robot_update', {'robot': rid, 'info': robots[rid]})
        time.sleep(0.5)

threading.Thread(target=allocator_loop, daemon=True).start()

# ---------------------------------------------------------
# 6. HTTP API
# ---------------------------------------------------------
@app.route('/request_path', methods=['POST'])
def request_path():
    data = request.json or {}
    rid = data.get('robot_id')
    node = data.get('node')
    facing = (data.get('dir') or data.get('facing') or 's').lower()
    pickup = data.get('pickup')
    drop = data.get('drop')

    if not rid or rid not in robots:
        return jsonify({'error': 'unknown robot'}), 400
    
    with state_lock:
        robots[rid]['node'] = node
        robots[rid]['dir'] = facing
        robots[rid]['last_seen'] = time.time()
        now = int(time.time())
        
        path_to_pickup = space_time_a_star(GRAPH, node, pickup, now, rid)
        if not path_to_pickup: return jsonify({'error': 'no path to pickup'}), 500
        
        arrive_t = now + len(path_to_pickup) - 1
        path_pickup_to_drop = space_time_a_star(GRAPH, pickup, drop, arrive_t, rid)
        if not path_pickup_to_drop: return jsonify({'error': 'no path pickup->drop'}), 500

        full_path = path_to_pickup + path_pickup_to_drop[1:]
        reserve_path_trajectory(full_path, now, rid)

        robots[rid]['status'] = 'busy'
        robots[rid]['current_path'] = full_path

        job = create_system_job(pickup, drop, rid)
        job['path'] = full_path
        robots[rid]['current_job'] = job['id']

        instr1, facing_after_pickup = path_to_instr_list(path_to_pickup, facing)
        instr2, _ = path_to_instr_list(path_pickup_to_drop, facing_after_pickup)

        # FIX: include entire instr2 so len(instr_list) == len(full_path)-1
        full_instr = instr1 + instr2

        plan = build_plan_array(full_path, full_instr)
        job['plan'] = plan
        job['plan_str'] = plan_to_str(plan)
        job['progress_index'] = None
        
        socketio.emit('job_update', {'job': job})
        socketio.emit('robot_update', {'robot': rid, 'info': robots[rid]})
        return jsonify({'ok': True, 'plan': plan, 'plan_str': job['plan_str'], 'job_id': job['id']}), 200

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
    step_index = data.get('step_index')

    with state_lock:
        if rid not in robots:
            return jsonify({'error': 'unknown'}), 400
        
        robots[rid]['node'] = node
        robots[rid]['last_seen'] = time.time()
        if reported_dir:
            robots[rid]['dir'] = reported_dir.lower()
        
        # shrink current_path if robot provided node in it
        path = robots[rid].get('current_path', [])
        if node in path:
            robots[rid]['current_path'] = path[path.index(node):]
        
        jid = robots[rid].get('current_job')
        if jid and jid in jobs and step_index is not None:
            job = jobs[jid]
            try:
                si = int(step_index)
            except:
                si = None
            if si is not None:
                job['progress_index'] = si
                job.setdefault('progress_trace', []).append({
                    'step_index': si,
                    'node': node,
                    'dir': robots[rid]['dir'],
                    'ts': time.time()
                })
                socketio.emit('job_update', {'job': job})

        if status == 'job_done':
            jid = robots[rid].get('current_job')
            if jid and jid in jobs:
                jobs[jid]['status'] = 'done'
                socketio.emit('job_update', {'job': jobs[jid]})
            robots[rid]['status'] = 'idle'
            robots[rid]['current_path'] = []
            robots[rid].pop('current_job', None)
            # clear reservations
            keys = [k for k, v in reservations.items() if v == rid]
            for k in keys: del reservations[k]
            # try auto-parking
            if node not in PARKING_NODES:
                parking_spot = find_nearest_parking(node)
                if parking_spot:
                    parking_job = create_system_job(node, parking_spot, rid)
                    current_t = int(time.time())
                    park_path = space_time_a_star(GRAPH, node, parking_spot, current_t, rid)
                    if park_path:
                        reserve_path_trajectory(park_path, current_t, rid)
                        current_dir = robots[rid].get('dir', 's')
                        instrs, _ = path_to_instr_list(park_path, current_dir)
                        plan = build_plan_array(park_path, instrs)
                        parking_job['plan'] = plan
                        parking_job['plan_str'] = plan_to_str(plan)
                        parking_job['path'] = park_path
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

    if not rid or rid not in robots:
        return jsonify({'error': 'unknown'}), 400

    with state_lock:
        if nodes_with_dir and isinstance(nodes_with_dir, list) and len(nodes_with_dir) > 0:
            last = nodes_with_dir[-1]
            robots[rid]['node'] = last.get('node', robots[rid].get('node'))
            robots[rid]['dir'] = (last.get('dir') or robots[rid].get('dir', 's')).lower()
            report = {'nodes_with_dir': nodes_with_dir, 'ts': time.time()}
        else:
            report = {'ts': time.time()}

        if jid and jid in jobs:
            jobs[jid].setdefault('reports', []).append({'robot': rid, **report})
            jobs[jid]['status'] = 'done'
            socketio.emit('job_update', {'job': jobs[jid]})

        robots[rid]['status'] = 'idle'
        robots[rid]['current_path'] = []
        robots[rid].pop('current_job', None)
        keys = [k for k, v in reservations.items() if v == rid]
        for k in keys: del reservations[k]

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
# 7. UI (node labels now drawn inside node circles)
# ---------------------------------------------------------
HTML_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Fleet Commander</title>
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.2/dist/css/bootstrap.min.css" rel="stylesheet">
    <script src="https://cdn.socket.io/4.7.2/socket.io.min.js"></script>
    <style>
        :root { --bg-color: #121212; --card-bg: #1e1e1e; --text-color: #e0e0e0; }
        body { font-family: sans-serif; background: var(--bg-color); color: var(--text-color); margin:0; height:100vh; }
        .dashboard-grid { display: grid; grid-template-columns: 300px 1fr 300px; gap: 1rem; padding: 1rem; height: 100%; }
        .card { background: var(--card-bg); border: 1px solid #333; color: var(--text-color); display: flex; flex-direction: column; border-radius: 8px; overflow: hidden; }
        .card-header { border-bottom: 1px solid #333; padding: 0.8rem; background: #252525; }
        .card-body { padding: 0.8rem; overflow-y: auto; flex: 1; }
        .form-control { background: #2c2c2c; border: 1px solid #444; color: #fff; margin-bottom: 0.5rem; }
        .btn-primary { width: 100%; }
        .map-container { background: #1a1a1a; width: 100%; height: 100%; cursor: grab; overflow: hidden; position: relative; }
        .robot-path { fill: none; stroke-width: 3; stroke-linecap: round; stroke-linejoin: round; opacity: 0.7; }
        .list-item { padding: 0.5rem; border-bottom: 1px solid #333; font-size: 0.85rem; }
        .plan-step { font-family: monospace; font-size: 0.8em; padding: 2px 6px; margin-right:6px; border-radius:4px; background: rgba(255,255,255,0.03); color: #bbb; display:inline-block; }
        .plan-step.active { background: rgba(13,110,253,0.2); color: #fff; border: 1px solid rgba(13,110,253,0.4); }
        .robot-facing { font-size: 0.8em; color: #bbb; margin-left:6px; }
        /* Node label style */
        .node-text { font-size: 10px; fill: #ddd; font-weight: 600; text-anchor: middle; dominant-baseline: middle; pointer-events: none; }
    </style>
</head>
<body>
  <div class="dashboard-grid">
    <div class="d-flex flex-column gap-3">
        <div class="card">
            <div class="card-header">Dispatch Job</div>
            <div class="card-body">
                <input id="pickup" class="form-control" placeholder="Pickup Node">
                <input id="drop" class="form-control" placeholder="Drop Node">
                <button onclick="submitJob()" class="btn btn-primary">Send Job</button>
                
            </div>
        </div>
        <div class="card flex-grow-1">
            <div class="card-header">Robots</div>
            <div class="card-body p-0" id="robots-list"></div>
        </div>
    </div>
    <div class="card">
        <div class="card-header">Map</div>
        <div class="map-container" id="mapwrap">
            <svg id="map" width="100%" height="100%"><g id="viewport"></g></svg>
        </div>
    </div>
    <div class="card">
        <div class="card-header">Jobs</div>
        <div class="card-body p-0" id="jobs-list"></div>
    </div>
  </div>
<script>
const socket = io();
let NODE_COORDS = {}, GRAPH_DATA = {}, ROBOTS = {}, JOBS = {};
const viewport = document.getElementById('viewport');
const scaleFactor = 40, offsetX = 100, offsetY = 100; 
let transform = { x: 0, y: 0, k: 1 }; 
let isDragging = false, startDrag = { x: 0, y: 0 };

socket.on('layout', d => { NODE_COORDS = d.nodes; GRAPH_DATA = d.graph; drawMap(); });
socket.on('state_snapshot', d => { ROBOTS = d.robots||{}; JOBS={}; (d.jobs||[]).forEach(j=>JOBS[j.id]=j); updateUI(); });
socket.on('robot_update', d => { ROBOTS[d.robot] = d.info; updateUI(); });
socket.on('job_update', d => { JOBS[d.job.id] = d.job; updateUI(); });

function updateUI() {
    const rList = document.getElementById('robots-list');
    rList.innerHTML = '';
    Object.keys(ROBOTS).forEach(id => {
        const r = ROBOTS[id];
        const facing = (r.dir || '').toUpperCase();
        rList.innerHTML += `<div class="list-item"><span style="color:${r.color}">●</span> <strong>${id.substring(0,6)}</strong> @ ${r.node} <span class="robot-facing">(${facing})</span> <span style="float:right">${r.status}</span></div>`;
    });

    const jList = document.getElementById('jobs-list');
    jList.innerHTML = '';
    Object.values(JOBS).sort((a,b)=>b.submitted_ts-a.submitted_ts).forEach(j => {
        const planHtml = renderPlan(j);
        jList.innerHTML += `<div class="list-item"><div style="display:flex;justify-content:space-between"><div><strong>#${j.id.substring(0,6)}</strong> ${j.pickup}→${j.drop}</div><div>${j.status}</div></div><div style="margin-top:6px;color:#999;font-size:0.85em">${j.plan_str||''}</div><div style="margin-top:6px;">${planHtml}</div></div>`;
    });

    drawMap();
}

function renderPlan(job) {
    if(!job || !job.plan) return '';
    const steps = job.plan.map((p, idx) => {
        const cls = (job.progress_index !== undefined && job.progress_index !== null && job.progress_index === idx) ? 'plan-step active' : 'plan-step';
        return `<span class="${cls}" title="idx ${idx}">${p[0]} ${p[1]}</span>`;
    });
    return steps.join('');
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
            if(NODE_COORDS[v]) {
                const p2 = nodeToPixel(v);
                const l = document.createElementNS('http://www.w3.org/2000/svg','line');
                l.setAttribute('x1', p1.x); l.setAttribute('y1', p1.y); l.setAttribute('x2', p2.x); l.setAttribute('y2', p2.y);
                l.setAttribute('stroke', '#444'); l.setAttribute('stroke-width', 2);
                viewport.appendChild(l);
            }
        }
    }
    // Paths (from robot.current_path)
    for(let id in ROBOTS) {
        const r = ROBOTS[id];
        if(r.current_path && r.current_path.length > 0) {
            let pts = "";
            r.current_path.forEach(n => { const p = nodeToPixel(n); pts += `${p.x},${p.y} `; });
            const line = document.createElementNS('http://www.w3.org/2000/svg','polyline');
            line.setAttribute('points', pts);
            line.setAttribute('stroke', r.color);
            line.setAttribute('class', 'robot-path');
            viewport.appendChild(line);
        }
    }
    // Nodes (circle + centered label)
    for(let n in NODE_COORDS) {
        const p = nodeToPixel(n);
        // group to hold circle + text
        const g = document.createElementNS('http://www.w3.org/2000/svg','g');
        g.setAttribute('transform', `translate(${p.x}, ${p.y})`);
        // circle
        const c = document.createElementNS('http://www.w3.org/2000/svg','circle');
        c.setAttribute('r', 10); // slightly larger to fit number inside
        c.setAttribute('fill', '#333');
        c.setAttribute('stroke', '#444');
        c.setAttribute('stroke-width', '1');
        g.appendChild(c);
        // centered text label
        const t = document.createElementNS('http://www.w3.org/2000/svg','text');
        t.setAttribute('class', 'node-text');
        t.setAttribute('x', 0);
        t.setAttribute('y', 0);
        t.textContent = n;
        g.appendChild(t);
        viewport.appendChild(g);
    }
    // Robots
    for(let id in ROBOTS) {
        const r = ROBOTS[id];
        const p = nodeToPixel(r.node);
        const g = document.createElementNS('http://www.w3.org/2000/svg','g');
        g.setAttribute('transform', `translate(${p.x}, ${p.y})`);
        const c = document.createElementNS('http://www.w3.org/2000/svg','circle');
        c.setAttribute('r', 8); c.setAttribute('fill', r.color);
        g.appendChild(c);
        const t = document.createElementNS('http://www.w3.org/2000/svg','text');
        t.setAttribute('y', -12); t.setAttribute('text-anchor','middle'); t.setAttribute('font-size','10'); t.setAttribute('fill','#ddd');
        t.textContent = (r.dir||'').toUpperCase();
        g.appendChild(t);
        viewport.appendChild(g);
    }
}
function submitJob() {
    const p = document.getElementById('pickup').value, d = document.getElementById('drop').value;
    if(p && d) fetch('/submit_job', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({pickup:p, drop:d})});
}
function resetSim() { fetch('/reset_sim', {method:'POST'}); }
const svg = document.getElementById('map');
function updateTransform() { viewport.setAttribute('transform', `translate(${transform.x}, ${transform.y}) scale(${transform.k})`); }
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

if __name__ == '__main__':
    print("Server running on port 8080...")
    socketio.run(app, host='0.0.0.0', port=8080)

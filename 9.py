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

# -------------------------
# 1) Graph / Config
# -------------------------
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

PARKING_NODES = ['81', '82', '83', '84', '85', '86',
                 '11', '12', '13', '15', '26', '31', '46', '51', '56']

MAX_SEARCH_DEPTH = 60

# -------------------------
# 2) System State
# -------------------------
robots = {}       # robot_id -> { status, node, dir, last_seen, ... }
jobs = {}         # job_id -> job dict (includes plan, plan_str, path, status, assigned_robot, progress_index)
job_queue = []    # queued jobs (user submitted)
reservations = {} # (node, time) -> robot_id  (space-time reservations)
state_lock = threading.Lock()

# -------------------------
# 3) Coordinate helpers (for heuristics & UI)
# -------------------------
def build_coords(graph):
    coords = {}
    for n in graph:
        try:
            coords[n] = (int(n[1]), int(n[0]))
        except:
            coords[n] = (0, 0)
    return coords

NODE_COORDS = build_coords(GRAPH)

def get_manhattan_dist(a, b):
    ax, ay = NODE_COORDS.get(a, (0, 0))
    bx, by = NODE_COORDS.get(b, (0, 0))
    return abs(ax - bx) + abs(ay - by)

# -------------------------
# 4) Space-time A* pathfinder & reservations
# -------------------------
def is_safe(node, t, rid):
    owner = reservations.get((node, t))
    if owner and owner != rid:
        return False
    # avoid colliding with idle robot parked at node
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
        neighbors = list(graph[curr].values()) + [curr]  # include wait (stay)
        for nb in neighbors:
            nt = current_time + 1
            if (nb, nt) in visited:
                continue
            if is_safe(nb, nt, rid):
                visited.add((nb, nt))
                ng = g + 1
                if nb == curr:
                    ng += 1.1  # small penalty for waiting
                h = get_manhattan_dist(nb, end)
                heapq.heappush(open_set, (ng + h, ng, nb, path + [nb]))
    return None

def reserve_path_trajectory(path, t0, rid):
    # clear previous reservations for rid
    keys = [k for k, v in reservations.items() if v == rid]
    for k in keys:
        del reservations[k]
    for i, n in enumerate(path):
        reservations[(n, t0 + i)] = rid

def find_nearest_parking(node):
    candidates = []
    for p in PARKING_NODES:
        occupied = any(r.get('node') == p and r.get('status') == 'idle' for r in robots.values())
        if not occupied:
            candidates.append((get_manhattan_dist(node, p), p))
    if not candidates:
        return None
    candidates.sort()
    return candidates[0][1]

# -------------------------
# 5) Instruction helpers (server builds plan)
# -------------------------
CLOCKWISE = {'n': 'e', 'e': 's', 's': 'w', 'w': 'n'}
CCW = {v: k for k, v in CLOCKWISE.items()}
OPP = {'n': 's', 's': 'n', 'e': 'w', 'w': 'e'}

def direction_between(a, b):
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
    for i in range(len(path) - 1):
        a = path[i]; b = path[i + 1]
        target = direction_between(a, b)
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

def random_color():
    return "#{:06x}".format(random.randint(0x444444, 0xFFFFFF))

def create_system_job(pickup, drop, rid, plan=None, plan_str=None, path=None):
    jid = str(uuid.uuid4())[:8]
    job = {'id': jid, 'pickup': pickup, 'drop': drop, 'submitted_ts': time.time(),
           'status': 'assigned' if rid else 'queued', 'assigned_robot': rid}
    if plan is not None:
        job['plan'] = plan
    if plan_str is not None:
        job['plan_str'] = plan_str
    if path is not None:
        job['path'] = path
    jobs[jid] = job
    return job

# -------------------------
# 6) Allocator thread
# -------------------------
def allocator_loop():
    while True:
        with state_lock:
            current_t = int(time.time())
            # cleanup old reservations
            old = [k for k in reservations if k[1] < current_t]
            for k in old:
                del reservations[k]

            pending = [j for j in job_queue if j['status'] == 'queued']
            for job in pending:
                # choose nearest idle robot (simple first-fit nearest)
                idle_list = [(r, info) for r, info in robots.items() if info.get('status') == 'idle']
                if not idle_list:
                    break
                # sort by manhattan distance from robot to pickup
                idle_list.sort(key=lambda item: get_manhattan_dist(item[1].get('node'), job['pickup']))
                rid = idle_list[0][0]
                start_node = robots[rid]['node']
                # compute path to pickup
                path_to_pickup = space_time_a_star(GRAPH, start_node, job['pickup'], current_t, rid)
                if not path_to_pickup:
                    continue
                arrival_t = current_t + len(path_to_pickup) - 1
                # compute path pickup->drop
                path_pickup_to_drop = space_time_a_star(GRAPH, job['pickup'], job['drop'], arrival_t, rid)
                if not path_pickup_to_drop:
                    continue
                full_path = path_to_pickup + path_pickup_to_drop[1:]
                # reserve space-time
                reserve_path_trajectory(full_path, current_t, rid)
                # build instruction sequence based on robot's current facing
                instr1, facing_after_pickup = path_to_instr_list(path_to_pickup, robots[rid].get('dir', 's'))
                instr2, _ = path_to_instr_list(path_pickup_to_drop, facing_after_pickup)
                full_instr = instr1 + (instr2[1:] if len(instr2) > 0 else [])
                plan = []
                for i in range(len(full_path) - 1):
                    plan.append([full_path[i], full_instr[i]])
                plan.append([full_path[-1], 'D'])
                plan_str = ' '.join([f"{p[0]} {p[1]}" for p in plan])

                # assign job
                job['assigned_robot'] = rid
                job['status'] = 'assigned'
                job['plan'] = plan
                job['plan_str'] = plan_str
                job['path'] = full_path
                job_queue.remove(job)

                robots[rid]['status'] = 'busy'
                robots[rid]['current_job'] = job['id']
                robots[rid]['current_path'] = []  # server authoritative; robot sends step updates
                socketio.emit('job_update', {'job': job})
                socketio.emit('robot_update', {'robot': rid, 'info': robots[rid]})
        time.sleep(0.5)

threading.Thread(target=allocator_loop, daemon=True).start()

# -------------------------
# 7) HTTP API
# -------------------------
@app.route('/request_path', methods=['POST'])
def request_path():
    """
    Request payload:
      { "robot_id": "r1", "node": "81", "dir":"s", "pickup":"71", "drop":"73" }

    Response:
      { "ok": true, "plan": [["81","S"],["71","L"],...["73","D"]], "plan_str": "...", "job_id": "abcd" }
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

        instr1, facing_after_pickup = path_to_instr_list(path_to_pickup, facing)
        instr2, _ = path_to_instr_list(path_pickup_to_drop, facing_after_pickup)
        full_instr = instr1 + (instr2[1:] if len(instr2) > 0 else [])
        plan = []
        for i in range(len(full_path) - 1):
            plan.append([full_path[i], full_instr[i]])
        plan.append([full_path[-1], 'D'])
        plan_str = ' '.join([f"{p[0]} {p[1]}" for p in plan])

        job = create_system_job(pickup, drop, rid, plan=plan, plan_str=plan_str, path=full_path)
        robots[rid]['status'] = 'busy'
        robots[rid]['current_job'] = job['id']
        robots[rid]['current_path'] = []

        socketio.emit('job_update', {'job': job})
        socketio.emit('robot_update', {'robot': rid, 'info': robots[rid]})
        return jsonify({'ok': True, 'plan': plan, 'plan_str': plan_str, 'job_id': job['id']}), 200

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
        robots[rid] = {'status': 'idle', 'node': node, 'last_seen': time.time(),
                       'color': color, 'current_path': [], 'dir': direction}
    socketio.emit('robot_update', {'robot': rid, 'info': robots[rid]})
    return jsonify({'robot_id': rid, 'color': color}), 200

@app.route('/submit_job', methods=['POST'])
def submit_job():
    data = request.json or {}
    if not data.get('pickup') or not data.get('drop'):
        return jsonify({'error': 'req'}), 400
    job_id = str(uuid.uuid4())[:8]
    job = {'id': job_id, 'pickup': data['pickup'], 'drop': data['drop'],
           'submitted_ts': time.time(), 'status': 'queued', 'assigned_robot': None}
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
            # return authoritative job including plan & progress
            return jsonify({'job': jobs.get(jid)}), 200
        return jsonify({'job': None}), 200

@app.route('/update_location', methods=['POST'])
def update_location():
    """
    Expected payload:
      { robot_id, node, dir, step_index (optional), status (optional: 'job_done') }

    Server stores robots[rid]['node'] and ['dir'] and if robot has current job,
    updates job.progress_index and job.progress_trace (for UI live updates).
    """
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

        jid = robots[rid].get('current_job')
        if jid and jid in jobs and step_index is not None:
            job = jobs[jid]
            # normalize step index to int if possible
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

        # job done handling
        if status == 'job_done' and jid and jid in jobs:
            jobs[jid]['status'] = 'done'
            socketio.emit('job_update', {'job': jobs[jid]})
            # cleanup robot current job
            robots[rid]['status'] = 'idle'
            robots[rid].pop('current_job', None)
            robots[rid]['current_path'] = []
            # clear reservations for this robot
            keys = [k for k, v in reservations.items() if v == rid]
            for k in keys:
                del reservations[k]
            # if not in parking, schedule parking job
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
                        robots[rid]['current_path'] = []
                        socketio.emit('job_update', {'job': parking_job})
                    else:
                        jobs[parking_job['id']]['status'] = 'failed'

        socketio.emit('robot_update', {'robot': rid, 'info': robots[rid]})
    return jsonify({'ok': True}), 200

@app.route('/report_execution', methods=['POST'])
def report_execution():
    """
    Robots send full trace at the end:
      { robot_id, job_id, nodes_with_dir: [{node, dir}, ...] }
    """
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
            # fallback: compute final dir from commands if given
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
            socketio.emit('job_update', {'job': jobs[jid]})

        robots[rid]['status'] = 'idle'
        robots[rid]['current_path'] = []
        robots[rid].pop('current_job', None)
        # clear reservations for this robot
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
            if j.get('status') == 'assigned':
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

# Optional simple UI page for debugging (keeps prior dashboard if you want to use it)
HTML_PAGE = """
<!doctype html>
<title>Fleet Commander (Minimal)</title>
<h3>Fleet Commander server running.</h3>
<p>Use the API endpoints: /register_robot, /submit_job, /poll_task, /update_location, /report_execution</p>
"""

@app.route('/')
def index():
    return render_template_string(HTML_PAGE)

# -------------------------
# 8) Run server
# -------------------------
if __name__ == '__main__':
    print("Server running on port 8080...")
    socketio.run(app, host='0.0.0.0', port=8080)

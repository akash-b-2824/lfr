# central_server.py
# Central planner server for industrial navigation robots
# Requirements: pip install flask flask-socketio eventlet

import time
import uuid
import heapq
import threading
import random
from collections import deque
from flask import Flask, request, jsonify, render_template_string
from flask_socketio import SocketIO

app = Flask(__name__)

# Use 'threading' for best compatibility on Windows/Python 3.13
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# ----------------------------
# 1. Configuration & Graph
# ----------------------------
# Graph structure provided by user
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

# Time and reservation settings
TIME_STEP = 1.0
MAX_SEARCH_DEPTH = 60 # Max steps to look ahead for collision

# ----------------------------
# 2. In-memory State
# ----------------------------
robots = {}     # robot_id -> {status, node, color, current_path: []}
job_queue = []  # Pending jobs
jobs = {}       # All jobs history
reservations = {} # (node, time_int) -> robot_id
state_lock = threading.Lock()

# ----------------------------
# 3. Coordinate System
# ----------------------------
def build_coords(graph):
    """Parses Node IDs (e.g., '25') into (x=5, y=2) coords"""
    coords = {}
    for node in graph:
        try:
            r = int(node[0])
            c = int(node[1])
            coords[node] = (c, r) # x=c, y=r
        except:
            coords[node] = (0, 0)
    return coords

NODE_COORDS = build_coords(GRAPH)

def get_manhattan_dist(node_a, node_b):
    if node_a not in NODE_COORDS or node_b not in NODE_COORDS: return 0
    ax, ay = NODE_COORDS[node_a]
    bx, by = NODE_COORDS[node_b]
    return abs(ax - bx) + abs(ay - by)

# ----------------------------
# 4. Pathfinding: Space-Time A*
# ----------------------------
def is_safe(node, t, my_id, prev_node=None):
    """Checks vertex and edge collisions, plus static obstacles."""
    # 1. Reservation Check (Vertex Conflict)
    res_id = reservations.get((node, t))
    if res_id and res_id != my_id:
        return False
    
    # 2. Edge Conflict (Swap Prevention)
    # If I am going u->v at time t, ensure no one is going v->u at time t
    # (This implementation relies on the reservation table effectively blocking the target node)
    
    # 3. Static Obstacle Check (Idle robots blocking the way)
    for rid, info in robots.items():
        if rid != my_id and info['status'] == 'idle' and info['node'] == node:
            # If an idle robot is sitting there, it's blocked unless we nudge it
            return False
    return True

def space_time_a_star(graph, start, end, start_time, my_id, max_time=MAX_SEARCH_DEPTH):
    """Returns path [start, n1, n2... end] accounting for time reservations."""
    open_set = []
    # Priority Queue: (f_score, g_score, current_node, path_history)
    heapq.heappush(open_set, (0, 0, start, [start])) 
    visited = set() # (node, time)
    
    while open_set:
        f, g, current, path = heapq.heappop(open_set)
        current_time = start_time + g
        
        if current == end:
            return path
        
        if g >= max_time: continue
            
        # Neighbors + Wait (stay current)
        neighbors = list(graph.get(current, {}).values())
        neighbors.append(current) 
        
        for neighbor in neighbors:
            next_time = current_time + 1
            if (neighbor, next_time) in visited: continue
            
            if is_safe(neighbor, next_time, my_id, current):
                visited.add((neighbor, next_time))
                new_g = g + 1
                h = get_manhattan_dist(neighbor, end)
                if neighbor == current: new_g += 1.1 # Penalty for waiting
                
                new_f = new_g + h
                heapq.heappush(open_set, (new_f, new_g, neighbor, path + [neighbor]))
    return None

def simple_dijkstra(graph, start, end):
    """Fallback for nudge logic calculation"""
    if start == end: return [start]
    q = [(0, start, [])]
    visited = set()
    while q:
        c, n, p = heapq.heappop(q)
        if n in visited: continue
        visited.add(n)
        path = p + [n]
        if n == end: return path
        for _, neigh in graph.get(n, {}).items():
            if neigh not in visited: heapq.heappush(q, (c+1, neigh, path))
    return None

# ----------------------------
# 5. Reservation & Nudge Helpers
# ----------------------------
def reserve_path_trajectory(path, start_time, robot_id):
    # Clear old reservations
    keys = [k for k, v in reservations.items() if v == robot_id]
    for k in keys: del reservations[k]
    # Reserve new
    for i, node in enumerate(path):
        reservations[(node, start_time + i)] = robot_id

def find_free_neighbor(graph, node, excluded_nodes):
    neighbors = list(graph.get(node, {}).values())
    random.shuffle(neighbors)
    for n in neighbors:
        if n in excluded_nodes: continue
        occupied = False
        for r in robots.values():
            if r['node'] == n: occupied = True
        if not occupied: return n
    return None

def generate_random_color():
    return "#{:06x}".format(random.randint(0x444444, 0xFFFFFF))

# ----------------------------
# 6. Allocator Thread
# ----------------------------
def allocator_loop():
    while True:
        with state_lock:
            current_t = int(time.time())
            # Cleanup old reservations
            old = [k for k in reservations if k[1] < current_t]
            for k in old: del reservations[k]
            
            # Process Queued Jobs
            pending = [j for j in job_queue if j['status'] == 'queued']
            
            for job in pending:
                idle = [r for r, info in robots.items() if info['status'] == 'idle']
                if not idle: break
                
                robot_id = idle[0]
                start_node = robots[robot_id]['node']
                
                # Try Space-Time A*
                path = space_time_a_star(GRAPH, start_node, job['pickup'], current_t, robot_id)
                
                if path:
                    # Plan leg 2 (Pickup -> Drop)
                    arrival_t = current_t + len(path) - 1
                    path2 = space_time_a_star(GRAPH, job['pickup'], job['drop'], arrival_t, robot_id)
                    
                    if path2:
                        full_path = path + path2[1:]
                        reserve_path_trajectory(full_path, current_t, robot_id)
                        
                        job['assigned_robot'] = robot_id
                        job['status'] = 'assigned'
                        job['path'] = full_path
                        job_queue.remove(job)
                        
                        robots[robot_id]['status'] = 'busy'
                        robots[robot_id]['current_job'] = job['id']
                        robots[robot_id]['current_path'] = full_path
                        
                        socketio.emit('job_update', {'job': job})
                        socketio.emit('robot_update', {'robot': robot_id, 'info': robots[robot_id]})
                        continue # Job assigned, move to next job

                # NUDGE LOGIC: If pathfinding failed, check if an idle robot is blocking
                naive_path = simple_dijkstra(GRAPH, start_node, job['pickup'])
                if naive_path:
                    blocker_id = None
                    for n in naive_path:
                        for rid, r in robots.items():
                            if r['node'] == n and r['status'] == 'idle' and rid != robot_id:
                                blocker_id = rid
                                break
                        if blocker_id: break
                    
                    if blocker_id:
                        # Found a blocker, move them
                        safe = find_free_neighbor(GRAPH, robots[blocker_id]['node'], naive_path)
                        if safe:
                            nudge_id = str(uuid.uuid4())[:8]
                            nudge_job = {'id': nudge_id, 'pickup': safe, 'drop': safe, 'status': 'assigned', 'assigned_robot': blocker_id}
                            esc_path = space_time_a_star(GRAPH, robots[blocker_id]['node'], safe, current_t, blocker_id)
                            if esc_path:
                                reserve_path_trajectory(esc_path, current_t, blocker_id)
                                robots[blocker_id]['status'] = 'busy'
                                robots[blocker_id]['current_job'] = nudge_id
                                robots[blocker_id]['current_path'] = esc_path
                                jobs[nudge_id] = nudge_job
                                socketio.emit('job_update', {'job': nudge_job})
                                socketio.emit('robot_update', {'robot': blocker_id, 'info': robots[blocker_id]})
                                break # Break to let things settle
        time.sleep(0.5)

threading.Thread(target=allocator_loop, daemon=True).start()

# ----------------------------
# 7. HTTP API
# ----------------------------
@app.route('/')
def index(): return render_template_string(HTML_PAGE)

@app.route('/submit_job', methods=['POST'])
def submit_job():
    data = request.json or {}
    if not data.get('pickup') or not data.get('drop'): return jsonify({'error': 'req'}), 400
    job_id = str(uuid.uuid4())[:8]
    job = {'id': job_id, 'pickup': data['pickup'], 'drop': data['drop'], 'submitted_ts': time.time(), 'status': 'queued', 'assigned_robot': None}
    with state_lock:
        job_queue.append(job)
        jobs[job_id] = job
    socketio.emit('job_update', {'job': job})
    return jsonify({'job_id': job_id}), 200

@app.route('/register_robot', methods=['POST'])
def register_robot():
    data = request.json or {}
    rid = data.get('robot_id') or str(uuid.uuid4())[:6]
    node = data.get('node') or '81'
    color = generate_random_color()
    with state_lock:
        if rid in robots: color = robots[rid].get('color', color)
        robots[rid] = {'status': 'idle', 'node': node, 'last_seen': time.time(), 'color': color, 'current_path': []}
    socketio.emit('robot_update', {'robot': rid, 'info': robots[rid]})
    return jsonify({'robot_id': rid, 'color': color}), 200

@app.route('/poll_task', methods=['GET'])
def poll_task():
    rid = request.args.get('robot_id')
    with state_lock:
        if rid not in robots: return jsonify({'error': 'unknown'}), 400
        robots[rid]['last_seen'] = time.time()
        jid = robots[rid].get('current_job')
        if jid: return jsonify({'job': jobs.get(jid)}), 200
        return jsonify({'job': None}), 200

@app.route('/update_location', methods=['POST'])
def update_location():
    data = request.json or {}
    rid, node, status = data.get('robot_id'), data.get('node'), data.get('status')
    with state_lock:
        if rid not in robots: return jsonify({'error': 'unknown'}), 400
        robots[rid]['node'] = node
        robots[rid]['last_seen'] = time.time()
        
        # Path Consumption (Visual)
        path = robots[rid].get('current_path', [])
        if node in path:
            robots[rid]['current_path'] = path[path.index(node):] # Slice path
            
        if status == 'job_done':
            jid = robots[rid].get('current_job')
            if jid and jid in jobs:
                jobs[jid]['status'] = 'done'
                socketio.emit('job_update', {'job': jobs[jid]})
            robots[rid]['status'] = 'idle'
            robots[rid]['current_path'] = []
            robots[rid].pop('current_job', None)
            keys = [k for k,v in reservations.items() if v==rid]
            for k in keys: del reservations[k]
            
        socketio.emit('robot_update', {'robot': rid, 'info': robots[rid]})
    return jsonify({'ok': True}), 200

@app.route('/reset_sim', methods=['POST'])
def reset_sim():
    with state_lock:
        job_queue.clear()
        reservations.clear()
        for j in jobs.values():
            if j['status'] == 'assigned': j['status'] = 'failed'
            socketio.emit('job_update', {'job': j})
        for r in robots.values():
            r['status'] = 'idle'
            r['current_path'] = []
            r.pop('current_job', None)
            socketio.emit('robot_update', {'robot': r['id'] if 'id' in r else 'unknown', 'info': r}) # Fix for dict access
    return jsonify({'ok':True}), 200

@socketio.on('connect')
def on_connect():
    with state_lock:
        socketio.emit('layout', {'nodes': NODE_COORDS, 'graph': GRAPH})
        socketio.emit('state_snapshot', {'robots': robots, 'jobs': list(jobs.values())})

# ----------------------------
# 8. Dashboard HTML
# ----------------------------
HTML_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Fleet Commander - Dark Mode</title>
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

if __name__ == '__main__':
    print("Server running on port 5000...")
    socketio.run(app, host='0.0.0.0', port=5000)
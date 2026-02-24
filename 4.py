# central_server.py
# Central planner server for industrial navigation robots
# Requirements: pip install flask flask-socketio

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
# Configuration & Graph
# ----------------------------
# Graph using "RowCol" naming convention for coordinate extraction
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
RESERVATION_LOOKAHEAD = 300

# ----------------------------
# In-memory state
# ----------------------------
robots = {}     # robot_id -> {status, node, color, current_path: []}
job_queue = []  # Pending jobs
jobs = {}       # All jobs (including done)
reservations = {}
state_lock = threading.Lock()

# ----------------------------
# Utilities: Dijkstra
# ----------------------------
def dijkstra(graph, start, end):
    if start == end: return [start]
    queue = [(0, start, [])]
    visited = set()
    while queue:
        cost, node, path = heapq.heappop(queue)
        if node in visited: continue
        visited.add(node)
        path = path + [node]
        if node == end: return path
        for _, neighbor in graph.get(node, {}).items():
            if neighbor not in visited:
                heapq.heappush(queue, (cost + 1, neighbor, path))
    return None

# ----------------------------
# Layout: ABSOLUTE COORDINATES extraction
# ----------------------------
def build_coords(graph):
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

# ----------------------------
# Reservation helpers
# ----------------------------
def now_int(): return int(time.time())

def can_reserve_path(path, start_time_int, robot_id):
    for i, node in enumerate(path):
        t = start_time_int + i
        if (node, t) in reservations and reservations[(node, t)] != robot_id:
            return False
    return True

def reserve_path(path, start_time_int, robot_id):
    for i, node in enumerate(path):
        t = start_time_int + i
        reservations[(node, t)] = robot_id

def release_reservations_of_robot(robot_id):
    keys = [k for k, v in reservations.items() if v == robot_id]
    for k in keys: del reservations[k]

def generate_random_color():
    return "#{:06x}".format(random.randint(0, 0xFFFFFF))

# ----------------------------
# Job allocator thread
# ----------------------------
def allocator_loop():
    while True:
        with state_lock:
            # Only look at queued jobs
            pending = [j for j in job_queue if j['status'] == 'queued']
            
            for job in pending:
                idle = [r for r, info in robots.items() if info.get('status') == 'idle']
                if not idle: break
                
                robot_id = idle[0]
                robot_info = robots[robot_id]
                start_node = robot_info.get('node', '81')
                
                path1 = dijkstra(GRAPH, start_node, job['pickup'])
                path2 = dijkstra(GRAPH, job['pickup'], job['drop'])
                
                if not path1 or not path2:
                    job['status'] = 'failed'
                    socketio.emit('job_update', {'job': job})
                    continue
                
                combined = path1 + path2[1:]
                start_time = now_int()
                scheduled = False
                scheduled_start = start_time
                
                for offset in range(0, 15): 
                    if can_reserve_path(combined, start_time + offset, robot_id):
                        reserve_path(combined, start_time + offset, robot_id)
                        scheduled = True
                        scheduled_start = start_time + offset
                        break
                
                if not scheduled: continue
                
                # Assign
                job['assigned_robot'] = robot_id
                job['status'] = 'assigned'
                job['path'] = combined
                job['scheduled_start'] = scheduled_start
                job['assigned_ts'] = time.time()
                
                # Update Robot
                robots[robot_id]['status'] = 'busy'
                robots[robot_id]['current_job'] = job['id']
                robots[robot_id]['current_path'] = combined # Store full path for visualization
                
                # Remove from pending queue, but keep in global dict
                job_queue.remove(job)
                
                socketio.emit('job_assigned', {'robot': robot_id, 'job': job})
                socketio.emit('job_update', {'job': job})
                socketio.emit('robot_update', {'robot': robot_id, 'info': robots[robot_id]})
        
        time.sleep(1.0)

alloc_thread = threading.Thread(target=allocator_loop, daemon=True)
alloc_thread.start()

# ----------------------------
# HTTP endpoints
# ----------------------------
@app.route('/')
def index(): return render_template_string(HTML_PAGE)

@app.route('/submit_job', methods=['POST'])
def submit_job():
    data = request.json or {}
    pickup = data.get('pickup')
    drop = data.get('drop')
    if not pickup or not drop: return jsonify({'error': 'required'}), 400
    job_id = str(uuid.uuid4())[:8]
    job = {'id': job_id, 'pickup': pickup, 'drop': drop, 'submitted_ts': time.time(), 'status': 'queued', 'assigned_robot': None}
    with state_lock:
        job_queue.append(job)
        jobs[job_id] = job
    socketio.emit('job_update', {'job': job})
    return jsonify({'job_id': job_id}), 200

@app.route('/register_robot', methods=['POST'])
def register_robot():
    data = request.json or {}
    robot_id = data.get('robot_id') or str(uuid.uuid4())[:6]
    node = data.get('node') or '81'
    direction = data.get('direction') or 's'
    
    # Generate random color if new
    color = generate_random_color()
    
    with state_lock:
        if robot_id in robots:
            color = robots[robot_id].get('color', color)
            
        robots[robot_id] = {
            'status': 'idle', 
            'node': node, 
            'dir': direction, 
            'last_seen': time.time(),
            'color': color,
            'current_path': []
        }
    socketio.emit('robot_update', {'robot': robot_id, 'info': robots[robot_id]})
    return jsonify({'robot_id': robot_id, 'color': color}), 200

@app.route('/poll_task', methods=['GET'])
def poll_task():
    robot_id = request.args.get('robot_id')
    if not robot_id: return jsonify({'error': 'id req'}), 400
    with state_lock:
        if robot_id not in robots: return jsonify({'error': 'unknown robot'}), 400
        robots[robot_id]['last_seen'] = time.time()
        cur_job_id = robots[robot_id].get('current_job')
        
        if cur_job_id: return jsonify({'job': jobs.get(cur_job_id)}), 200
        return jsonify({'job': None}), 200

@app.route('/update_location', methods=['POST'])
def update_location():
    data = request.json or {}
    robot_id = data.get('robot_id')
    node = data.get('node')
    status = data.get('status')
    with state_lock:
        if robot_id not in robots: return jsonify({'error': 'unknown'}), 400
        
        robots[robot_id]['node'] = node
        robots[robot_id]['last_seen'] = time.time()
        
        # PATH CONSUMPTION LOGIC: Remove nodes from path as they are reached
        if 'current_path' in robots[robot_id]:
            path = robots[robot_id]['current_path']
            if node in path:
                # Slice path to keep only nodes from current location onwards
                idx = path.index(node)
                robots[robot_id]['current_path'] = path[idx:]
        
        if status == 'job_done':
            cur_job = robots[robot_id].get('current_job')
            if cur_job:
                jobs[cur_job]['status'] = 'done'
                jobs[cur_job]['completed_ts'] = time.time()
                socketio.emit('job_update', {'job': jobs[cur_job]})
                robots[robot_id].pop('current_job', None)
            
            robots[robot_id]['status'] = 'idle'
            robots[robot_id]['current_path'] = [] # Clear path
            release_reservations_of_robot(robot_id)
            
        socketio.emit('robot_update', {'robot': robot_id, 'info': robots[robot_id]})
    return jsonify({'ok': True}), 200

# ----------------------------
# Socket Events
# ----------------------------
@socketio.on('connect')
def on_connect():
    with state_lock:
        socketio.emit('layout', {'nodes': NODE_COORDS, 'graph': GRAPH})
        socketio.emit('state_snapshot', {'robots': robots, 'jobs': list(jobs.values())})

# ----------------------------
# DASHBOARD HTML
# ----------------------------
HTML_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Fleet Commander</title>
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.2/dist/css/bootstrap.min.css" rel="stylesheet">
    <link href="https://cdn.jsdelivr.net/npm/bootstrap-icons@1.11.1/font/bootstrap-icons.css" rel="stylesheet">
    <script src="https://cdn.socket.io/4.7.2/socket.io.min.js"></script>
    <link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;600&display=swap" rel="stylesheet">
  <style>
    :root { --bg-color: black; }
    body { font-family: 'Inter', sans-serif; background: var(--bg-color); color: #212529; }
    .dashboard-grid { display: grid; grid-template-columns: 350px 1fr 350px; gap: 1.5rem; padding: 1.5rem; height: 95vh; }
    .card { background: #fff; border: 1px solid #e0e0e0; border-radius: 10px; display: flex; flex-direction: column; overflow: hidden; box-shadow: 0 2px 4px rgba(0,0,0,0.05); }
    .card-header { font-weight: 600; background: #fff; border-bottom: 1px solid #e0e0e0; padding: 1rem; display: flex; justify-content: space-between; align-items: center; }
    .card-body { padding: 1rem; overflow-y: auto; }
    
    /* Map Styles */
    .map-container { background: #fff; width: 100%; height: 100%; cursor: grab; overflow: hidden; position: relative; }
    .map-container:active { cursor: grabbing; }
    .grid-line { stroke: #f0f2f5; stroke-width: 1; }
    .node-circle { fill: #f8f9fa; stroke: #dee2e6; stroke-width: 1.5; }
    .node-text { font-size: 5px; text-anchor: middle; dominant-baseline: middle; fill: #adb5bd; font-weight: bold; pointer-events: none; }
    .edge-line { stroke: #e9ecef; stroke-width: 2; stroke-linecap: round; }
    
    /* Robot & Path Styles */
    .robot-group { transition: transform 0.3s ease; }
    .robot-circle { stroke: white; stroke-width: 2; }
    .robot-text { font-size: 6px; fill: white; font-weight: bold; text-anchor: middle; dominant-baseline: middle; }
    .robot-path { fill: none; stroke-width: 3; stroke-linecap: round; stroke-linejoin: round; stroke-dasharray: 5,5; opacity: 0.6; }

    /* Job List Styles */
    .list-item { padding: 0.75rem; border-bottom: 1px solid #f0f0f0; }
    .job-done { background-color: #f8f9fa; border-left: 4px solid #6c757d; }
    .job-assigned { background-color: #d1e7dd; border-left: 4px solid #198754; }
    .job-queued { background-color: #fff3cd; border-left: 4px solid #ffc107; }
    
    .badge-status { font-size: 0.7rem; padding: 4px 8px; border-radius: 12px; font-weight: 600; }

    @media (max-width: 1200px) { .dashboard-grid { grid-template-columns: 1fr 1fr; grid-template-rows: auto auto; height: auto; } .map-card { grid-column: 1 / -1; height: 500px; } }
    @media (max-width: 768px) { .dashboard-grid { grid-template-columns: 1fr; } }
  </style>
</head>
<body>

  <div class="dashboard-grid">
    
    <div class="d-flex flex-column gap-3">
        <div class="card">
            <div class="card-header">
                <span><i class="bi bi-cpu"></i> System Status</span>
                <span class="badge bg-success">Online</span>
            </div>
            <div class="card-body">
                <div class="d-flex justify-content-between mb-2"><span class="text-muted">Time</span><span id="svtime" class="fw-bold">--:--</span></div>
                <div class="d-flex justify-content-between mb-2"><span class="text-muted">Robots</span><span id="stat-robots" class="fw-bold">0</span></div>
                <div class="d-flex justify-content-between"><span class="text-muted">Active Jobs</span><span id="stat-jobs" class="fw-bold">0</span></div>
            </div>
        </div>
        <div class="card">
            <div class="card-header"><i class="bi bi-send"></i> Dispatch Job</div>
            <div class="card-body">
                <div class="mb-2"><label class="small text-muted">Pickup</label><input id="pickup" class="form-control" placeholder="e.g. 11"></div>
                <div class="mb-3"><label class="small text-muted">Drop</label><input id="drop" class="form-control" placeholder="e.g. 84"></div>
                <button onclick="submitJob()" class="btn btn-primary w-100">Send</button>
            </div>
        </div>
    </div>

    <div class="card map-card">
        <div class="card-header">
            <span><i class="bi bi-map"></i> Facility Map</span>
            <button class="btn btn-sm btn-outline-secondary" onclick="resetView()">Recenter</button>
        </div>
        <div class="map-container" id="mapwrap">
            <svg id="map" width="100%" height="100%"><g id="viewport"></g></svg>
        </div>
    </div>

    <div class="d-flex flex-column gap-3">
        <div class="card flex-grow-1">
            <div class="card-header"><i class="bi bi-robot"></i> Robots</div>
            <div class="card-body p-0" id="robots-list"><div class="p-3 text-center text-muted small">No robots</div></div>
        </div>
        <div class="card flex-grow-1">
            <div class="card-header"><i class="bi bi-list-check"></i> Jobs</div>
            <div class="card-body p-0" id="jobs-list"><div class="p-3 text-center text-muted small">Queue empty</div></div>
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
socket.on('state_snapshot', d => { 
    ROBOTS = d.robots || {}; 
    JOBS = {}; 
    (d.jobs||[]).forEach(j=>JOBS[j.id]=j); 
    updateUI(); 
});
socket.on('robot_update', d => { ROBOTS[d.robot] = d.info; updateUI(); });
socket.on('job_update', d => { JOBS[d.job.id] = d.job; updateUI(); });

function updateUI() {
    // Update Stats
    document.getElementById('stat-robots').innerText = Object.keys(ROBOTS).length;
    document.getElementById('stat-jobs').innerText = Object.values(JOBS).filter(j => j.status === 'assigned').length;
    
    // Update Robots List
    const rList = document.getElementById('robots-list');
    rList.innerHTML = '';
    for(let id in ROBOTS) {
        const r = ROBOTS[id];
        const colorStyle = `background-color: ${r.color}; width: 12px; height: 12px; display: inline-block; border-radius: 50%; margin-right: 5px;`;
        rList.innerHTML += `<div class="list-item d-flex justify-content-between align-items-center">
            <div>
                <div class="fw-bold small"><span style="${colorStyle}"></span>${id.substring(0,6)}</div>
                <div class="text-muted small">Node ${r.node}</div>
            </div>
            <span class="badge bg-secondary">${r.status}</span></div>`;
    }
    
    // Update Jobs List with Colors (Yellow=Queued, Green=Assigned, Grey=Done)
    const jList = document.getElementById('jobs-list');
    jList.innerHTML = '';
    
    // Sort: Assigned -> Queued -> Done
    const sortedJobs = Object.values(JOBS).sort((a,b) => {
        const order = { 'assigned': 1, 'queued': 2, 'done': 3, 'failed': 4 };
        return order[a.status] - order[b.status];
    });

    if(sortedJobs.length === 0) jList.innerHTML = '<div class="p-3 text-center text-muted small">No jobs</div>';
    
    sortedJobs.forEach(j => {
        let cls = 'job-queued'; // Default yellow
        let statusText = 'Pending';
        
        if(j.status === 'assigned') { cls = 'job-assigned'; statusText = 'Active'; }
        else if(j.status === 'done') { cls = 'job-done'; statusText = 'Done'; }
        else if(j.status === 'failed') { cls = 'job-done'; statusText = 'Failed'; }

        jList.innerHTML += `<div class="list-item ${cls}">
            <div class="d-flex justify-content-between">
                <span class="fw-bold small">#${j.id}</span>
                <span class="small fw-bold text-muted">${statusText}</span>
            </div>
            <div class="small text-muted">${j.pickup} &rarr; ${j.drop}</div>
        </div>`;
    });
    
    drawMap();
}

function nodeToPixel(nId) {
    const n = NODE_COORDS[nId];
    return n ? {x: offsetX + n[0]*scaleFactor, y: offsetY + n[1]*scaleFactor} : {x:0,y:0};
}

function drawMap() {
    while (viewport.firstChild) viewport.removeChild(viewport.firstChild);
    
    // 1. Grid
    if(Object.keys(NODE_COORDS).length) {
        let maxX=0, maxY=0;
        for(let n in NODE_COORDS) { maxX = Math.max(maxX, NODE_COORDS[n][0]); maxY = Math.max(maxY, NODE_COORDS[n][1]); }
        for(let x=0; x<=maxX+1; x++) createLine({x: offsetX+x*scaleFactor, y: offsetY}, {x: offsetX+x*scaleFactor, y: offsetY+(maxY+1)*scaleFactor}, 'grid-line');
        for(let y=0; y<=maxY+1; y++) createLine({x: offsetX, y: offsetY+y*scaleFactor}, {x: offsetX+(maxX+1)*scaleFactor, y: offsetY+y*scaleFactor}, 'grid-line');
    }

    // 2. Edges
    for(let u in GRAPH_DATA) {
        const p1 = nodeToPixel(u);
        for(let d in GRAPH_DATA[u]) {
            const v = GRAPH_DATA[u][d];
            if(NODE_COORDS[v]) createLine(p1, nodeToPixel(v), 'edge-line');
        }
    }

    // 3. Robot Paths (Dynamic & Colored)
    for(let id in ROBOTS) {
        const r = ROBOTS[id];
        if(r.current_path && r.current_path.length > 1) {
            let points = "";
            // Start path from current robot visual position (node)
            const currentPixel = nodeToPixel(r.node);
            points += `${currentPixel.x},${currentPixel.y} `;
            
            // Add rest of the nodes in the path
            for(let i=1; i<r.current_path.length; i++) {
                const p = nodeToPixel(r.current_path[i]);
                points += `${p.x},${p.y} `;
            }
            
            const polyline = document.createElementNS('http://www.w3.org/2000/svg','polyline');
            polyline.setAttribute('points', points);
            polyline.setAttribute('class', 'robot-path');
            polyline.setAttribute('stroke', r.color || '#000');
            viewport.appendChild(polyline);
        }
    }

    // 4. Nodes
    for(let n in NODE_COORDS) {
        const p = nodeToPixel(n);
        const g = createGroup(p.x, p.y);
        const c = document.createElementNS('http://www.w3.org/2000/svg','circle');
        c.setAttribute('r', 7); c.setAttribute('class', 'node-circle');
        const t = document.createElementNS('http://www.w3.org/2000/svg','text');
        t.textContent = n; t.setAttribute('class', 'node-text');
        g.appendChild(c); g.appendChild(t);
        viewport.appendChild(g);
    }

    // 5. Robots
    for(let id in ROBOTS) {
        const info = ROBOTS[id];
        const p = nodeToPixel(info.node || Object.keys(NODE_COORDS)[0]);
        const g = createGroup(p.x, p.y);
        g.setAttribute('class', 'robot-group');
        
        const r = document.createElementNS('http://www.w3.org/2000/svg','circle');
        r.setAttribute('r', 9); 
        r.setAttribute('class', 'robot-circle');
        r.setAttribute('fill', info.color || '#007bff'); // Use random color
        
        const t = document.createElementNS('http://www.w3.org/2000/svg','text');
        t.textContent = id.substring(0,2); t.setAttribute('class', 'robot-text');
        g.appendChild(r); g.appendChild(t);
        viewport.appendChild(g);
    }
}

function createLine(p1, p2, cls) {
    const l = document.createElementNS('http://www.w3.org/2000/svg','line');
    l.setAttribute('x1', p1.x); l.setAttribute('y1', p1.y);
    l.setAttribute('x2', p2.x); l.setAttribute('y2', p2.y);
    l.setAttribute('class', cls);
    viewport.appendChild(l);
}
function createGroup(x, y) {
    const g = document.createElementNS('http://www.w3.org/2000/svg','g');
    g.setAttribute('transform', `translate(${x},${y})`);
    return g;
}

// Zoom/Pan
function updateTransform() { viewport.setAttribute('transform', `translate(${transform.x}, ${transform.y}) scale(${transform.k})`); }
function resetView() { transform = {x:0, y:0, k:1}; updateTransform(); }
const svg = document.getElementById('map');
svg.addEventListener('mousedown', e => { isDragging=true; startDrag={x:e.clientX-transform.x, y:e.clientY-transform.y}; });
window.addEventListener('mousemove', e => { if(!isDragging)return; transform.x=e.clientX-startDrag.x; transform.y=e.clientY-startDrag.y; updateTransform(); });
window.addEventListener('mouseup', () => isDragging=false);
svg.addEventListener('wheel', e => { e.preventDefault(); transform.k *= (1 + (e.deltaY>0?-0.1:0.1)); updateTransform(); });

function submitJob() {
    fetch('/submit_job', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({pickup:document.getElementById('pickup').value, drop:document.getElementById('drop').value})});
    document.getElementById('pickup').value=''; document.getElementById('drop').value='';
}
setInterval(() => document.getElementById('svtime').innerText = new Date().toLocaleTimeString(), 1000);
</script>
</body>
</html>
"""

if __name__ == '__main__':
    print("Server running on port 5000...")
    socketio.run(app, host='0.0.0.0', port=5000)
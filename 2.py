# central_server.py
# Central planner server for industrial navigation robots
# Requirements: pip install flask flask-socketio eventlet

import time
import uuid
import heapq
import threading
from collections import deque
from flask import Flask, request, jsonify, render_template_string
from flask_socketio import SocketIO

app = Flask(__name__)

# Use 'threading' for best compatibility on Windows/Python 3.13
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# ----------------------------
# Configuration & Graph
# ----------------------------
GRAPH = {
    # Row 1
    '11': {'s': '21'},
    '12': {'s': '22'},
    '13': {'s': '23'},
    '15': {'s': '25'},  

    # Row 2
    '21': {'n': '11', 'e': '22', 's': '31'},
    '22': {'n': '12', 's': '32', 'w': '21', 'e': '23'},
    '23': {'n': '13', 's': '33', 'w': '22'}, 
    '24': { 'e': '25', 's': '34'}, 
    '25': {'n':'15','s': '35', 'e': '26', 'w': '24'},
    '26': {'w': '25'},

    # Row 3
    '31': {'n': '21', 'e': '32'},
    '32': {'n': '22', 'e': '33', 'w': '31'},
    '33': {'n': '23', 's': '43', 'e': '34', 'w': '32'},
    '34': {'n': '24', 's': '44', 'e': '35', 'w': '33'},
    '35': {'w': '34', 'n': '25', 'e': '36', 's': '45'},
    '36': {'w': '35', 's': '46'},

    # Row 4
    '42': {'s': '52'},
    '43': {'n': '33', 's': '53', 'e': '44'},
    '44': {'w': '43', 'n': '34', 'e': '45'},
    '45': {'n': '35', 's': '65', 'e': '46', 'w': '44'},
    '46': {'w': '45', 'n': '36'},

    # Row 5
    '51': {'e': '52'},
    '52': {'s': '62', 'e': '53', 'n': '42', 'w': '51'},
    '53': {'w': '52', 'n': '43', 's': '63'},
    '56': {'s': '66'},

    # Row 6
    '62': {'n': '52'},
    '63': {'n': '53', 'e': '64', 's': '73'}, 
    '64': {'w': '63', 'e': '65', 's': '84'}, 
    '65': {'n': '45', 's': '75', 'e': '66', 'w': '64'},
    '66': {'w': '65', 'n': '56', 's': '76'},

    # Row 7
    '71': {'s': '81', 'e': '72'},
    '72': { 'e': '73', 'w': '71', 's': '82'}, 
    '73': {'w': '72', 's': '83', 'n': '63'}, 
    '75': {'e': '76', 'n': '65', 's': '85'},
    '76': {'w': '75', 'n': '66', 's': '86'},

    # Row 8
    '81': {'n': '71'},
    '82': {'n': '72'},
    '83': {'n': '73'},
    '84': {'n': '64'}, 
    '85': {'n': '75'}, 
    '86': {'n': '76'}, 
}

# Time and reservation settings
TIME_STEP = 1.0
RESERVATION_LOOKAHEAD = 300

# ----------------------------
# In-memory state
# ----------------------------
robots = {}
job_queue = []
jobs = {}
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
# Layout: convert node -> (x,y)
# ----------------------------
DIR_VECTORS = {'n': (0, -1), 's': (0, 1), 'e': (1, 0), 'w': (-1, 0)}

def build_coords(graph, start_node=None):
    if not start_node: start_node = next(iter(graph))
    coords = {start_node: (0, 0)}
    visited = set([start_node])
    q = deque([start_node])
    while q:
        cur = q.popleft()
        cur_x, cur_y = coords[cur]
        for d, neigh in graph.get(cur, {}).items():
            vec = DIR_VECTORS.get(d)
            if not vec: continue
            nx = cur_x + vec[0]
            ny = cur_y + vec[1]
            if neigh not in coords:
                coords[neigh] = (nx, ny)
            if neigh not in visited:
                visited.add(neigh)
                q.append(neigh)
    return coords

NODE_COORDS = build_coords(GRAPH, start_node='81')

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

# ----------------------------
# Job allocator thread
# ----------------------------
def allocator_loop():
    while True:
        with state_lock:
            for job in list(job_queue):
                idle = [r for r, info in robots.items() if info.get('status') == 'idle']
                if not idle: break
                robot_id = idle[0]
                robot_info = robots[robot_id]
                start_node = robot_info.get('node', '81')
                path1 = dijkstra(GRAPH, start_node, job['pickup'])
                path2 = dijkstra(GRAPH, job['pickup'], job['drop'])
                
                if not path1 or not path2:
                    job['status'] = 'failed'
                    job_queue.remove(job)
                    jobs[job['id']] = job
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
                
                job_queue.remove(job)
                job['assigned_robot'] = robot_id
                job['status'] = 'assigned'
                job['path'] = combined
                job['scheduled_start'] = scheduled_start
                job['assigned_ts'] = time.time()
                jobs[job['id']] = job
                
                robots[robot_id]['status'] = 'busy'
                robots[robot_id]['current_job'] = job['id']
                
                socketio.emit('job_assigned', {'robot': robot_id, 'job': job})
                socketio.emit('job_update', {'job': job})
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
    with state_lock:
        robots[robot_id] = {'status': 'idle', 'node': node, 'dir': direction, 'last_seen': time.time()}
    socketio.emit('robot_update', {'robot': robot_id, 'info': robots[robot_id]})
    return jsonify({'robot_id': robot_id}), 200

@app.route('/poll_task', methods=['GET'])
def poll_task():
    robot_id = request.args.get('robot_id')
    if not robot_id: return jsonify({'error': 'id req'}), 400
    with state_lock:
        if robot_id not in robots: return jsonify({'error': 'unknown robot'}), 400
        robots[robot_id]['last_seen'] = time.time()
        cur_job_id = robots[robot_id].get('current_job')
        if cur_job_id: return jsonify({'job': jobs.get(cur_job_id)}), 200
        for job in jobs.values():
            if job.get('assigned_robot') == robot_id and job.get('status') == 'assigned':
                robots[robot_id]['current_job'] = job['id']
                robots[robot_id]['status'] = 'busy'
                return jsonify({'job': job}), 200
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
        if status == 'job_done':
            cur_job = robots[robot_id].get('current_job')
            if cur_job:
                jobs[cur_job]['status'] = 'done'
                jobs[cur_job]['completed_ts'] = time.time()
                socketio.emit('job_update', {'job': jobs[cur_job]})
                robots[robot_id].pop('current_job', None)
            robots[robot_id]['status'] = 'idle'
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
        socketio.emit('state_snapshot', {'robots': robots, 'jobs': list(jobs.values()), 'queue': job_queue})

# ----------------------------
# INDUSTRIAL DASHBOARD HTML
# ----------------------------
HTML_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Fleet Commander - Central Server</title>
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.2/dist/css/bootstrap.min.css" rel="stylesheet">
    <link href="https://cdn.jsdelivr.net/npm/bootstrap-icons@1.11.1/font/bootstrap-icons.css" rel="stylesheet">
    <script src="https://cdn.socket.io/4.7.2/socket.io.min.js"></script>
    <link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;600&display=swap" rel="stylesheet">
  <style>
    :root {
      --primary-color: #0d6efd;
      --secondary-color: #6c757d;
      --success-color: #198754;
      --danger-color: #dc3545;
      --warning-color: #ffc107;
      --background-color: #f4f6f8;
      --card-bg: #ffffff;
      --text-color: #212529;
      --border-color: #e0e0e0;
      --shadow: 0 4px 6px rgba(0, 0, 0, 0.05);
    }

    body {
      font-family: 'Inter', sans-serif;
      margin: 0;
      background: var(--background-color);
      color: var(--text-color);
    }

    .dashboard {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
      gap: 1.5rem;
      padding: 1.5rem;
      max-width: 1800px;
      margin: auto;
    }
    
    /* Specialized Grid for Map focus */
    .dashboard-grid {
        display: grid;
        grid-template-columns: 350px 1fr 350px; /* Panels | Map | Lists */
        gap: 1.5rem;
        height: 95vh;
    }

    .card {
      background: var(--card-bg);
      border: 1px solid var(--border-color);
      border-radius: 10px;
      box-shadow: var(--shadow);
      display: flex;
      flex-direction: column;
      overflow: hidden;
    }

    .card-header {
      font-weight: 600;
      font-size: 1rem;
      color: var(--text-color);
      background: white;
      border-bottom: 1px solid var(--border-color);
      padding: 1rem;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }

    .card-body {
        padding: 1rem;
        overflow-y: auto;
    }
    
    /* Map Container Styles (SVG) */
    .map-container {
      background: #ffffff;
      position: relative;
      width: 100%;
      height: 100%;
      cursor: grab;
      overflow: hidden;
    }
    .map-container:active { cursor: grabbing; }
    
    .grid-line { stroke: #e9ecef; stroke-width: 1; }
    .node-circle { fill: #f8f9fa; stroke: #ced4da; stroke-width: 1.5; }
    .node-text { font-size: 6px; text-anchor: middle; dominant-baseline: middle; pointer-events: none; fill: #6c757d; font-weight: 600;}
    .edge-line { stroke: #dee2e6; stroke-width: 2; stroke-linecap: round; }
    
    .robot-group { transition: transform 0.3s ease; }
    .robot-circle { fill: var(--primary-color); stroke: white; stroke-width: 2; }
    .robot-text { font-size: 7px; fill: white; font-weight: bold; text-anchor: middle; dominant-baseline: middle; }
    .robot-indicator { fill: rgba(13, 110, 253, 0.2); stroke: none; animation: pulse 2s infinite; }

    @keyframes pulse {
        0% { transform: scale(1); opacity: 0.8; }
        100% { transform: scale(2); opacity: 0; }
    }

    /* List Items */
    .list-item {
        padding: 0.75rem;
        border-bottom: 1px solid #f0f0f0;
        font-size: 0.9rem;
    }
    .list-item:last-child { border-bottom: none; }
    .badge-status { padding: 4px 8px; border-radius: 12px; font-size: 0.75rem; font-weight: 600; }
    .status-idle { background: #e9ecef; color: #495057; }
    .status-busy { background: #cfe2ff; color: #084298; }
    
    /* Controls */
    .form-control, .form-select { font-size: 0.9rem; }
    .btn-primary { background-color: var(--primary-color); border: none; }
    .btn-primary:hover { background-color: #0b5ed7; }

    @media (max-width: 1200px) {
        .dashboard-grid { grid-template-columns: 1fr 1fr; grid-template-rows: auto auto; height: auto;}
        .map-card { grid-column: 1 / -1; height: 500px; }
    }
    @media (max-width: 768px) {
        .dashboard-grid { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>

  <div class="dashboard dashboard-grid">
    
    <div style="display: flex; flex-direction: column; gap: 1.5rem;">
        
        <div class="card">
            <div class="card-header">
                <span><i class="bi bi-server"></i> System Status</span>
                <span class="badge bg-success">Online</span>
            </div>
            <div class="card-body">
                <div class="d-flex justify-content-between mb-2">
                    <span class="text-muted">Server Time</span>
                    <span id="svtime" class="fw-bold">--:--:--</span>
                </div>
                <div class="d-flex justify-content-between mb-2">
                    <span class="text-muted">Active Robots</span>
                    <span id="stat-robots" class="fw-bold">0</span>
                </div>
                <div class="d-flex justify-content-between">
                    <span class="text-muted">Pending Jobs</span>
                    <span id="stat-jobs" class="fw-bold">0</span>
                </div>
            </div>
        </div>

        <div class="card">
            <div class="card-header"><i class="bi bi-joystick"></i> Task Control</div>
            <div class="card-body">
                <div class="mb-3">
                    <label class="form-label text-muted small">Pickup Node</label>
                    <input type="text" class="form-control" id="pickup" placeholder="e.g. 33">
                </div>
                <div class="mb-3">
                    <label class="form-label text-muted small">Drop Node</label>
                    <input type="text" class="form-control" id="drop" placeholder="e.g. 64">
                </div>
                <button onclick="submitJob()" class="btn btn-primary w-100">
                    <i class="bi bi-send"></i> Dispatch Job
                </button>
            </div>
        </div>

        <div class="card flex-grow-1">
            <div class="card-header"><i class="bi bi-info-circle"></i> Controls</div>
            <div class="card-body small text-muted">
                <ul class="ps-3">
                    <li><strong>Scroll</strong> to Zoom Map</li>
                    <li><strong>Drag</strong> to Pan Map</li>
                    <li>Robots update in real-time</li>
                    <li>Green lines show active paths</li>
                </ul>
            </div>
        </div>
    </div>

    <div class="card map-card" style="height: 100%; min-height: 500px;">
        <div class="card-header">
            <span><i class="bi bi-map"></i> Warehouse Floor Plan</span>
            <button class="btn btn-sm btn-outline-secondary" onclick="resetView()">Reset View</button>
        </div>
        <div class="map-container" id="mapwrap">
            <svg id="map" width="100%" height="100%">
                <g id="viewport"></g>
            </svg>
        </div>
    </div>

    <div style="display: flex; flex-direction: column; gap: 1.5rem; overflow: hidden;">
        
        <div class="card" style="flex: 1; min-height: 0;">
            <div class="card-header"><i class="bi bi-robot"></i> Fleet</div>
            <div class="card-body p-0" id="robots-list">
                <div class="p-3 text-center text-muted">Waiting for robots...</div>
            </div>
        </div>

        <div class="card" style="flex: 1; min-height: 0;">
            <div class="card-header"><i class="bi bi-list-task"></i> Job Queue</div>
            <div class="card-body p-0" id="jobs-list">
                <div class="p-3 text-center text-muted">No active jobs</div>
            </div>
        </div>

    </div>
  </div>

<script>
const socket = io();
let NODE_COORDS = {}, GRAPH_DATA = {}, ROBOTS = {}, JOBS = {};

// Map Visualization Variables
const viewport = document.getElementById('viewport');
const scaleFactor = 40; 
const offsetX = 400, offsetY = 100; // Initial offset
let transform = { x: 0, y: 0, k: 1 }; 
let isDragging = false, startDrag = { x: 0, y: 0 };

// --- Socket Logic ---
socket.on('connect', () => console.log('Connected to Central Server'));

socket.on('layout', data => {
  NODE_COORDS = data.nodes || {};
  GRAPH_DATA = data.graph || {};
  // Normalize tuples
  for (let k in NODE_COORDS) NODE_COORDS[k] = [NODE_COORDS[k][0], NODE_COORDS[k][1]];
  drawMap();
});

socket.on('state_snapshot', data => {
  if (data.robots) ROBOTS = data.robots;
  if (data.jobs) { 
      JOBS = {}; 
      data.jobs.forEach(j => JOBS[j.id] = j); 
  }
  updateUI();
});

socket.on('robot_update', data => {
  ROBOTS[data.robot] = data.info;
  updateUI();
});

socket.on('job_update', data => {
  JOBS[data.job.id] = data.job;
  updateUI();
});

// --- UI Updating ---
function updateUI() {
    drawMap(); // Redraw robots on map
    updateStats();
    updateRobotList();
    updateJobList();
}

function updateStats() {
    document.getElementById('stat-robots').innerText = Object.keys(ROBOTS).length;
    document.getElementById('stat-jobs').innerText = Object.values(JOBS).filter(j => j.status !== 'done').length;
}

function updateRobotList() {
    const container = document.getElementById('robots-list');
    container.innerHTML = '';
    
    if (Object.keys(ROBOTS).length === 0) {
        container.innerHTML = '<div class="p-3 text-center text-muted">No robots connected</div>';
        return;
    }

    for (let id in ROBOTS) {
        const r = ROBOTS[id];
        const statusClass = r.status === 'busy' ? 'status-busy' : 'status-idle';
        const html = `
            <div class="list-item d-flex justify-content-between align-items-center">
                <div>
                    <div class="fw-bold"><i class="bi bi-robot"></i> ${id}</div>
                    <div class="text-muted small">Node: ${r.node}</div>
                </div>
                <span class="badge-status ${statusClass}">${r.status.toUpperCase()}</span>
            </div>
        `;
        container.innerHTML += html;
    }
}

function updateJobList() {
    const container = document.getElementById('jobs-list');
    container.innerHTML = '';
    const jobsArr = Object.values(JOBS).filter(j => j.status !== 'done');

    if (jobsArr.length === 0) {
        container.innerHTML = '<div class="p-3 text-center text-muted">Queue empty</div>';
        return;
    }

    jobsArr.forEach(j => {
        let badgeColor = 'bg-secondary';
        if(j.status === 'assigned') badgeColor = 'bg-primary';
        if(j.status === 'failed') badgeColor = 'bg-danger';

        const html = `
            <div class="list-item">
                <div class="d-flex justify-content-between">
                    <span class="fw-bold small">#${j.id}</span>
                    <span class="badge ${badgeColor} small">${j.status}</span>
                </div>
                <div class="small mt-1">
                    <i class="bi bi-geo-alt"></i> ${j.pickup} &rarr; <i class="bi bi-box-seam"></i> ${j.drop}
                </div>
            </div>
        `;
        container.innerHTML += html;
    });
}

// --- Map Logic (SVG) ---
function nodeToPixel(nodeId) {
  const n = NODE_COORDS[nodeId];
  return n ? {x: offsetX + n[0]*scaleFactor, y: offsetY + n[1]*scaleFactor} : {x:0,y:0};
}

function drawMap() {
  while (viewport.firstChild) viewport.removeChild(viewport.firstChild);

  // 1. Grid Lines
  if (Object.keys(NODE_COORDS).length > 0) {
      let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
      for (let n in NODE_COORDS) {
          minX = Math.min(minX, NODE_COORDS[n][0]); maxX = Math.max(maxX, NODE_COORDS[n][0]);
          minY = Math.min(minY, NODE_COORDS[n][1]); maxY = Math.max(maxY, NODE_COORDS[n][1]);
      }
      // Vertical
      for (let x = minX - 1; x <= maxX + 1; x++) {
          const p1 = {x: offsetX + x*scaleFactor, y: offsetY + (minY-1)*scaleFactor};
          const p2 = {x: offsetX + x*scaleFactor, y: offsetY + (maxY+1)*scaleFactor};
          createLine(p1, p2, 'grid-line');
      }
      // Horizontal
      for (let y = minY - 1; y <= maxY + 1; y++) {
          const p1 = {x: offsetX + (minX-1)*scaleFactor, y: offsetY + y*scaleFactor};
          const p2 = {x: offsetX + (maxX+1)*scaleFactor, y: offsetY + y*scaleFactor};
          createLine(p1, p2, 'grid-line');
      }
  }

  // 2. Edges
  for (let u in GRAPH_DATA) {
      const start = nodeToPixel(u);
      const neighbors = GRAPH_DATA[u];
      for (let dir in neighbors) {
          const v = neighbors[dir];
          if (!NODE_COORDS[v]) continue;
          const end = nodeToPixel(v);
          createLine(start, end, 'edge-line');
      }
  }

  // 3. Nodes
  for (let n in NODE_COORDS) {
    const p = nodeToPixel(n);
    const g = createGroup(p.x, p.y);
    const c = document.createElementNS('http://www.w3.org/2000/svg','circle');
    c.setAttribute('r', 8); c.setAttribute('class', 'node-circle');
    const t = document.createElementNS('http://www.w3.org/2000/svg','text');
    t.textContent = n; t.setAttribute('class', 'node-text');
    g.appendChild(c); g.appendChild(t);
    viewport.appendChild(g);
  }

  // 4. Robots
  for (let id in ROBOTS) {
    const info = ROBOTS[id];
    const p = nodeToPixel(info.node || Object.keys(NODE_COORDS)[0]);
    const g = createGroup(p.x, p.y);
    g.setAttribute('class', 'robot-group');

    // Pulse effect
    const pulse = document.createElementNS('http://www.w3.org/2000/svg','circle');
    pulse.setAttribute('r', 14); pulse.setAttribute('class', 'robot-indicator');
    
    const r = document.createElementNS('http://www.w3.org/2000/svg','circle');
    r.setAttribute('r', 10); r.setAttribute('class', 'robot-circle');
    if(info.status === 'busy') r.setAttribute('fill', '#dc3545'); // Red if busy

    const t = document.createElementNS('http://www.w3.org/2000/svg','text');
    t.textContent = id.substring(0,2).toUpperCase(); t.setAttribute('class', 'robot-text');
    
    g.appendChild(pulse);
    g.appendChild(r);
    g.appendChild(t);
    viewport.appendChild(g);
  }
}

// SVG Helpers
function createLine(p1, p2, className) {
    const line = document.createElementNS('http://www.w3.org/2000/svg','line');
    line.setAttribute('x1', p1.x); line.setAttribute('y1', p1.y);
    line.setAttribute('x2', p2.x); line.setAttribute('y2', p2.y);
    line.setAttribute('class', className);
    viewport.appendChild(line);
}
function createGroup(x, y) {
    const g = document.createElementNS('http://www.w3.org/2000/svg','g');
    g.setAttribute('transform', `translate(${x}, ${y})`);
    return g;
}

// --- Zoom/Pan Logic ---
function updateTransform() {
    viewport.setAttribute('transform', `translate(${transform.x}, ${transform.y}) scale(${transform.k})`);
}
function resetView() {
    transform = {x:0, y:0, k:1};
    updateTransform();
}
const svg = document.getElementById('map');
svg.addEventListener('mousedown', (e) => {
    isDragging = true;
    startDrag = { x: e.clientX - transform.x, y: e.clientY - transform.y };
});
window.addEventListener('mousemove', (e) => {
    if (!isDragging) return;
    e.preventDefault();
    transform.x = e.clientX - startDrag.x;
    transform.y = e.clientY - startDrag.y;
    updateTransform();
});
window.addEventListener('mouseup', () => { isDragging = false; });
svg.addEventListener('wheel', (e) => {
    e.preventDefault();
    const zoomIntensity = 0.1;
    const direction = e.deltaY > 0 ? -1 : 1;
    transform.k *= (1 + (direction * zoomIntensity));
    transform.k = Math.min(Math.max(0.2, transform.k), 5); 
    updateTransform();
});

// --- Job Submission ---
function submitJob(){
  const pickup = document.getElementById('pickup').value;
  const drop = document.getElementById('drop').value;
  if(!pickup || !drop) { alert("Please enter both pickup and drop nodes"); return; }
  
  fetch('/submit_job', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({pickup, drop})
  }).then(r => {
      document.getElementById('pickup').value = '';
      document.getElementById('drop').value = '';
  });
}

setInterval(()=> { document.getElementById('svtime').innerText = new Date().toLocaleTimeString(); }, 1000);
</script>
</body>
</html>
"""

if __name__ == '__main__':
    print("Server running on port 5000...")
    socketio.run(app, host='0.0.0.0', port=5000)
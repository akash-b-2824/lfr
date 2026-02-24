# central_server.py
# Central planner server for industrial navigation robots
# Requirements: pip install flask flask-socketio

import time
import uuid
import heapq
import threading
from collections import deque
from flask import Flask, request, jsonify, render_template_string
from flask_socketio import SocketIO

app = Flask(__name__)

# Use 'threading' for best compatibility
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# ----------------------------
# Configuration & Graph (Fixed Layout)
# ----------------------------
GRAPH = {
    # Row 1
    '11': {'s': '21'},
    '12': {'s': '22'},
    '13': {'s': '23'},
    '15': {'s': '25'},  # FIXED: Connects to 24 now (was 25), putting it in line with 13

    # Row 2
    '21': {'n': '11', 'e': '22', 's': '31'},
    '22': {'n': '12', 's': '32', 'w': '21', 'e': '23'},
    '23': {'n': '13', 's': '33', 'w': '22'}, # FIXED: Added 'e': '24'
    '24': { 'e': '25', 's': '34'}, # FIXED: Added 'w': '23' and 'n': '14'
    '25': {'n':'15','s': '35', 'e': '26', 'w': '24'},            # FIXED: Removed 'n': '14'
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
    '63': {'n': '53', 'e': '64', 's': '73'}, # FIXED: Points South to 72 (was 73)
    '64': {'w': '63', 'e': '65', 's': '84'}, # FIXED: Points South to 73 (was 84)
    '65': {'n': '45', 's': '75', 'e': '66', 'w': '64'},
    '66': {'w': '65', 'n': '56', 's': '76'},

    # Row 7
    '71': {'s': '81', 'e': '72'},
    '72': { 'e': '73', 'w': '71', 's': '82'}, # FIXED: 'n' is 62
    '73': {'w': '72', 's': '83', 'n': '63'}, # FIXED: 'n' is 63
     # FIXED: 's' is 84 (was 85)
    '75': {'e': '76', 'n': '65', 's': '85'},
    '76':{'w': '75', 'n': '66', 's': '86'},

    # Row 8
    '81': {'n': '71'},
    '82': {'n': '72'},
    '83': {'n': '73'},
    '84': {'n': '64'}, # FIXED: n is 74 (was 63)
    '85': {'n': '75'}, # FIXED: n is 75 (was 74)
    '86': {'n': '76'}, # Keeping as is (hanging node)
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
# Frontend
# ----------------------------
HTML_PAGE = """
<!doctype html>
<html>
<head>
  <title>Central Planner</title>
  <script src="https://cdn.socket.io/4.7.2/socket.io.min.js"></script>
  <style>
    body { font-family: 'Segoe UI', sans-serif; margin: 20px; background: #f4f4f9; }
    h2 { margin-top: 0; }
    #container { display: flex; gap: 20px; }
    #left-panel { width: 500px; }
    #right-panel { flex: 1; }
    #mapwrap { 
      border: 2px solid #ccc; width: 500px; height: 400px; 
      background: #fff; overflow: hidden; position: relative; cursor: grab;
    }
    #mapwrap:active { cursor: grabbing; }
    pre { background: #fff; border: 1px solid #ddd; padding: 10px; max-height: 200px; overflow: auto; border-radius: 4px;}
    .controls { background: #fff; padding: 15px; border: 1px solid #ddd; border-radius: 4px; margin-bottom: 15px; }
    input { padding: 5px; width: 60px; }
    button { padding: 5px 10px; cursor: pointer; background: #007bff; color: white; border: none; border-radius: 3px;}
    .node-circle { fill: #eee; stroke: #555; stroke-width: 1; }
    .node-text { font-size: 6px; text-anchor: middle; dominant-baseline: middle; pointer-events: none; fill: #333; font-weight: bold;}
    .edge-line { stroke: #ccc; stroke-width: 1.5; stroke-linecap: round; }
    .robot-circle { fill: #ff4757; stroke: white; stroke-width: 1; transition: all 0.3s ease; }
    .robot-text { font-size: 8px; fill: black; font-weight: bold; text-anchor: middle; }
  </style>
</head>
<body>
  <div id="container">
    <div id="left-panel">
      <h2>Navigation Map</h2>
      <div class="controls">
        <div>Mouse Wheel to Zoom, Drag to Pan</div>
      </div>
      <div id="mapwrap">
        <svg id="map" width="100%" height="100%">
            <g id="viewport"></g>
        </svg>
      </div>
      <div class="controls" style="margin-top:15px;">
        <h3>Task Control</h3>
        Pickup: <input id="pickup" placeholder="33"/>
        Drop: <input id="drop" placeholder="64"/>
        <button onclick="submitJob()">Submit Job</button>
      </div>
    </div>
    <div id="right-panel">
      <h3>System Status <span style="font-size:0.8em; font-weight:normal" id="svtime"></span></h3>
      <h4>Robots</h4>
      <pre id="robots">Waiting...</pre>
      <h4>Jobs</h4>
      <pre id="jobs">Waiting...</pre>
    </div>
  </div>
<script>
const socket = io();
let NODE_COORDS = {}, GRAPH_DATA = {}, ROBOTS = {}, JOBS = {};
const svg = document.getElementById('map');
const viewport = document.getElementById('viewport');
const scaleFactor = 40; 
const offsetX = 250, offsetY = 50;
let transform = { x: 0, y: 0, k: 1 }; 
let isDragging = false, startDrag = { x: 0, y: 0 };

function updateTransform() {
    viewport.setAttribute('transform', `translate(${transform.x}, ${transform.y}) scale(${transform.k})`);
}
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
    transform.k *= (1 + (e.deltaY > 0 ? -0.1 : 0.1));
    transform.k = Math.min(Math.max(0.2, transform.k), 5); 
    updateTransform();
});

function nodeToPixel(nodeId) {
  const n = NODE_COORDS[nodeId];
  return n ? {x: offsetX + n[0]*scaleFactor, y: offsetY + n[1]*scaleFactor} : {x:0,y:0};
}

function drawMap() {
  while (viewport.firstChild) viewport.removeChild(viewport.firstChild);
  // Edges
  for (let u in GRAPH_DATA) {
      const start = nodeToPixel(u);
      const neighbors = GRAPH_DATA[u];
      for (let dir in neighbors) {
          const v = neighbors[dir];
          if (!NODE_COORDS[v]) continue;
          const end = nodeToPixel(v);
          const line = document.createElementNS('http://www.w3.org/2000/svg','line');
          line.setAttribute('x1', start.x); line.setAttribute('y1', start.y);
          line.setAttribute('x2', end.x); line.setAttribute('y2', end.y);
          line.setAttribute('class', 'edge-line');
          viewport.appendChild(line);
      }
  }
  // Nodes
  for (let n in NODE_COORDS) {
    const p = nodeToPixel(n);
    const g = document.createElementNS('http://www.w3.org/2000/svg','g');
    g.setAttribute('transform', `translate(${p.x}, ${p.y})`);
    const c = document.createElementNS('http://www.w3.org/2000/svg','circle');
    c.setAttribute('r', 6); c.setAttribute('class', 'node-circle');
    const t = document.createElementNS('http://www.w3.org/2000/svg','text');
    t.textContent = n; t.setAttribute('class', 'node-text');
    g.appendChild(c); g.appendChild(t);
    viewport.appendChild(g);
  }
  // Robots
  for (let id in ROBOTS) {
    const info = ROBOTS[id];
    const p = nodeToPixel(info.node || Object.keys(NODE_COORDS)[0]);
    const g = document.createElementNS('http://www.w3.org/2000/svg','g');
    g.setAttribute('transform', `translate(${p.x}, ${p.y})`);
    const r = document.createElementNS('http://www.w3.org/2000/svg','circle');
    r.setAttribute('r', 9); r.setAttribute('class', 'robot-circle');
    const t = document.createElementNS('http://www.w3.org/2000/svg','text');
    t.textContent = id.substring(0,4); t.setAttribute('y', -12); t.setAttribute('class', 'robot-text');
    g.appendChild(r); g.appendChild(t);
    viewport.appendChild(g);
  }
}

socket.on('layout', data => {
  NODE_COORDS = data.nodes || {};
  GRAPH_DATA = data.graph || {};
  for (let k in NODE_COORDS) NODE_COORDS[k] = [NODE_COORDS[k][0], NODE_COORDS[k][1]];
  drawMap();
  updateTransform();
});
socket.on('state_snapshot', data => {
  if (data.robots) ROBOTS = data.robots;
  if (data.jobs) { JOBS = {}; data.jobs.forEach(j => JOBS[j.id] = j); }
  document.getElementById('robots').innerText = JSON.stringify(ROBOTS, null, 2);
  document.getElementById('jobs').innerText = JSON.stringify(JOBS, null, 2);
  drawMap();
});
socket.on('robot_update', data => {
  ROBOTS[data.robot] = data.info;
  document.getElementById('robots').innerText = JSON.stringify(ROBOTS, null, 2);
  drawMap();
});
socket.on('job_update', data => {
  JOBS[data.job.id] = data.job;
  document.getElementById('jobs').innerText = JSON.stringify(JOBS, null, 2);
});
function submitJob(){
  const pickup = document.getElementById('pickup').value;
  const drop = document.getElementById('drop').value;
  if(!pickup || !drop) return;
  fetch('/submit_job', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({pickup, drop})
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
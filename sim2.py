# robot_sim.py
import threading
import time
import requests
import random
from datetime import datetime

# ---------------- CONFIG ----------------
SERVER_BASE = "http://127.0.0.1:8080"   # matches your server.py (port 8080)
PARKING_NODES = ['81','82','83','84','85','86']
POLL_INTERVAL = 1.0     # seconds between poll_task calls
STEP_TIME = 1        # simulated time to move between nodes
LOG_LOCK = threading.Lock()

# ---------------- GRAPH (same as server) ----------------
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

# ---------------- turn helpers ----------------
CLOCKWISE = {'n':'e','e':'s','s':'w','w':'n'}
CCW = {v:k for k,v in CLOCKWISE.items()}
OPP = {'n':'s','s':'n','e':'w','w':'e'}

def log(msg):
    with LOG_LOCK:
        print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}")

def direction_between(a,b):
    for d, nb in GRAPH.get(a, {}).items():
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

def build_plan_from_path(path, start_dir):
    """Convert server-provided path (list of nodes) into plan pairs [[node,cmd],...,'D']"""
    if not path:
        return []
    instrs = []
    cur = start_dir
    for i in range(len(path)-1):
        a = path[i]; b = path[i+1]
        target = direction_between(a,b)
        if not target:
            cmd = 'U'
            cur = OPP.get(cur, cur)
        else:
            cmd = instruction_from_dirs(cur, target)
            if cmd == 'R':
                cur = CLOCKWISE[cur]
            elif cmd == 'L':
                cur = CCW[cur]
            elif cmd == 'U':
                cur = OPP[cur]
            else:
                cur = cur
        instrs.append(cmd)
    plan = []
    for i in range(len(path)-1):
        plan.append([ path[i], instrs[i] ])
    plan.append([ path[-1], 'D' ])
    print(plan)
    return plan

# ---------------- HTTP helpers ----------------
def post(path, payload, timeout=6):
    url = SERVER_BASE.rstrip('/') + path
    try:
        log(f"-> POST {url} payload={payload}")
        r = requests.post(url, json=payload, timeout=timeout)
        log(f"<- {r.status_code} {r.text}")
        try: return r.status_code, r.json()
        except: return r.status_code, None
    except Exception as e:
        log(f"<- ERROR POST {url} {e}")
        return None, None

def get(path, params=None, timeout=6):
    url = SERVER_BASE.rstrip('/') + path
    try:
        log(f"-> GET {url} params={params}")
        r = requests.get(url, params=params or {}, timeout=timeout)
        log(f"<- {r.status_code} {r.text}")
        try: return r.status_code, r.json()
        except: return r.status_code, None
    except Exception as e:
        log(f"<- ERROR GET {url} {e}")
        return None, None

# ---------------- Robot thread ----------------
class SimRobot(threading.Thread):
    def __init__(self, rid, start_node, start_dir='s'):
        super().__init__(daemon=True)
        self.rid = rid
        self.node = start_node
        self.dir = start_dir
        self.running = True
        self.nodes_with_dir = []

    def register(self):
        code, resp = post('/register_robot', {'robot_id': self.rid, 'node': self.node, 'dir': self.dir})
        return code == 200

    def poll_task(self):
        code, resp = get('/poll_task', params={'robot_id': self.rid})
        if code == 200 and resp:
            return resp.get('job')
        return None

    def update_location(self, node, status=None):
        payload = {'robot_id': self.rid, 'node': node, 'dir': self.dir}
        if status:
            payload['status'] = status
        post('/update_location', payload)

    def report_execution(self, job_id):
        payload = {'robot_id': self.rid, 'job_id': job_id,
                   'nodes_with_dir': [{'node': n, 'dir': d} for n,d in self.nodes_with_dir]}
        post('/report_execution', payload)

    def apply_turn(self, cmd):
        if cmd == 'S':
            return self.dir
        if cmd == 'R':
            return CLOCKWISE.get(self.dir, self.dir)
        if cmd == 'L':
            return CCW.get(self.dir, self.dir)
        if cmd == 'U':
            return OPP.get(self.dir, self.dir)
        return self.dir

    def execute_plan(self, plan, job_id):
        """Follow plan: post update_location for each node, exec cmd, on 'D' send job_done + report_execution."""
        self.nodes_with_dir = []
        for node, cmd in plan:
            # simulate arrival
            log(f"{self.rid}: ARRIVED {node} facing={self.dir} cmd_at_node={cmd}")
            self.node = node
            self.nodes_with_dir.append((self.node, self.dir))
            if cmd == 'D':
                # final
                self.update_location(self.node, status='job_done')
                self.report_execution(job_id)
                log(f"{self.rid}: Job {job_id} DONE at {self.node}")
                return
            # normal update
            self.update_location(self.node)
            # execute instruction (simulate)
            log(f"{self.rid}: EXEC '{cmd}' (sleep {STEP_TIME}s)")
            time.sleep(STEP_TIME)
            self.dir = self.apply_turn(cmd)
        # safety: if plan had no 'D'
        self.update_location(self.node, status='job_done')
        self.report_execution(job_id)
        log(f"{self.rid}: Job {job_id} finished (no explicit D)")

    def run(self):
        # register once
        if not self.register():
            log(f"{self.rid}: registration failed -> abort")
            return
        log(f"{self.rid}: registered at {self.node} facing={self.dir}")

        while self.running:
            try:
                job = self.poll_task()
                if not job:
                    time.sleep(POLL_INTERVAL)
                    continue

                job_id = job.get('id') or job.get('job_id')
                path = job.get('path')
                if not job_id or not path:
                    log(f"{self.rid}: got job but missing path/job_id -> ignoring")
                    time.sleep(POLL_INTERVAL)
                    continue

                log(f"{self.rid}: assigned job {job_id} path={path}")

                # Build plan and execute (robot uses only path & own facing)
                plan = build_plan_from_path(path, self.dir)
                log(f"{self.rid}: built plan {plan}")

                self.execute_plan(plan, job_id)

                # short pause to let server update
                time.sleep(0.2)

            except Exception as e:
                log(f"{self.rid}: EXCEPTION {e}")
                time.sleep(1.0)

# ---------------- main ----------------
def start_sim(n):
    robots = []
    for i in range(n):
        rid = f"r{i+1}"
        start_node = PARKING_NODES[i % len(PARKING_NODES)]
        r = SimRobot(rid, start_node, start_dir='s')
        r.start()
        robots.append(r)
        time.sleep(0.1)
    log(f"Started {n} simulated robots (registering at {PARKING_NODES}). Use UI to add jobs; allocator will assign them.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        log("Stopping simulation...")
        for r in robots:
            r.running = False
        for r in robots:
            r.join(timeout=2.0)
        log("All robots stopped.")

if __name__ == "__main__":
    try:
        n = int(input("Number of robots to simulate (default 3): ").strip() or "3")
    except Exception:
        n = 3
    start_sim(n)

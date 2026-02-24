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
STEP_TIME = 0.6         # simulated time to move between nodes
LOG_LOCK = threading.Lock()

# ---------------- turn helpers ----------------
# Minimal logic needed just to track own direction for reporting
CLOCKWISE = {'n':'e','e':'s','s':'w','w':'n'}
CCW = {v:k for k,v in CLOCKWISE.items()}
OPP = {'n':'s','s':'n','e':'w','w':'e'}

def log(msg):
    with LOG_LOCK:
        print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}")

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
        # Robot just follows the command blindly to update its internal state
        if cmd == 'S': return self.dir
        if cmd == 'R': return CLOCKWISE.get(self.dir, self.dir)
        if cmd == 'L': return CCW.get(self.dir, self.dir)
        if cmd == 'U': return OPP.get(self.dir, self.dir)
        return self.dir

    def execute_plan(self, plan, job_id):
        """
        Follow plan directly: [[node, cmd], [node, cmd], ..., [node, 'D']]
        Server calculated all turns.
        """
        self.nodes_with_dir = []
        for node, cmd in plan:
            # simulate arrival
            log(f"{self.rid}: ARRIVED {node} facing={self.dir} next_cmd={cmd}")
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
            
            # execute instruction (simulate time)
            log(f"{self.rid}: EXEC '{cmd}' (sleep {STEP_TIME}s)")
            time.sleep(STEP_TIME)
            
            # Update internal direction based on server command
            self.dir = self.apply_turn(cmd)

        # safety catch
        self.update_location(self.node, status='job_done')
        self.report_execution(job_id)

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
                # CHECK FOR 'plan' INSTEAD OF 'path'
                plan = job.get('plan')

                if not job_id or not plan:
                    log(f"{self.rid}: got job but missing plan/job_id -> ignoring")
                    time.sleep(POLL_INTERVAL)
                    continue

                log(f"{self.rid}: assigned job {job_id} plan_len={len(plan)}")

                # Execute directly
                self.execute_plan(plan, job_id)

                # short pause
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
    log(f"Started {n} simulated robots. Use UI to add jobs.")
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
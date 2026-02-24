# robot_sim.py
# Simulated robot that polls the server, receives job.plan (list of [node,cmd]),
# follows the sequence, sends update_location with step_index at each arrival,
# and finally sends report_execution.

import threading
import time
import requests
import random
from datetime import datetime

# ---------------- CONFIG ----------------
SERVER_BASE = "http://127.0.0.1:8080"
PARKING_NODES = ['81','82','83','84','85','86']
POLL_INTERVAL = 1.0
STEP_TIME = 0.6  # time to execute a turn / step
LOG_LOCK = threading.Lock()

# ---------------- turn helpers ----------------
CLOCKWISE = {'n':'e','e':'s','s':'w','w':'n'}
CCW = {v:k for k,v in CLOCKWISE.items()}
OPP = {'n':'s','s':'n','e':'w','w':'e'}

def log(msg):
    with LOG_LOCK:
        print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}")

def apply_turn(cur, cmd):
    if cmd == 'S':
        return cur
    if cmd == 'R':
        return CLOCKWISE.get(cur, cur)
    if cmd == 'L':
        return CCW.get(cur, cur)
    if cmd == 'U':
        return OPP.get(cur, cur)
    return cur

# ---------------- HTTP helpers ----------------
def post(path, payload, timeout=6):
    url = SERVER_BASE.rstrip('/') + path
    try:
        log(f"-> POST {url} {payload}")
        r = requests.post(url, json=payload, timeout=timeout)
        log(f"<- {r.status_code} {r.text}")
        try:
            return r.status_code, r.json()
        except:
            return r.status_code, None
    except Exception as e:
        log(f"<- ERROR POST {url} {e}")
        return None, None

def get(path, params=None, timeout=6):
    url = SERVER_BASE.rstrip('/') + path
    try:
        log(f"-> GET {url} params={params}")
        r = requests.get(url, params=params or {}, timeout=timeout)
        log(f"<- {r.status_code} {r.text}")
        try:
            return r.status_code, r.json()
        except:
            return r.status_code, None
    except Exception as e:
        log(f"<- ERROR GET {url} {e}")
        return None, None

# ---------------- Robot class ----------------
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

    def update_location(self, node, status=None, step_index=None):
        payload = {'robot_id': self.rid, 'node': node, 'dir': self.dir}
        if status:
            payload['status'] = status
        if step_index is not None:
            payload['step_index'] = step_index
        post('/update_location', payload)

    def report_execution(self, job_id):
        payload = {'robot_id': self.rid, 'job_id': job_id,
                   'nodes_with_dir': [{'node': n, 'dir': d} for n,d in self.nodes_with_dir]}
        post('/report_execution', payload)

    def execute_plan(self, plan, job_id):
        """
        plan: list of [node, cmd]
        Robot uses plan ordering to determine node arrival and sends step_index each arrival.
        """
        self.nodes_with_dir = []
        for idx, (node, cmd) in enumerate(plan):
            # arrival (robot senses node)
            log(f"{self.rid}: ARRIVED {node} facing={self.dir} idx={idx} cmd={cmd}")
            self.node = node
            self.nodes_with_dir.append((self.node, self.dir))

            # send arrival with step_index
            if cmd == 'D':
                # include job_done status on final
                self.update_location(self.node, status='job_done', step_index=idx)
                self.report_execution(job_id)
                log(f"{self.rid}: Job {job_id} DONE at {self.node}")
                return
            else:
                self.update_location(self.node, step_index=idx)

            # simulate time to execute the turning/motion instruction
            log(f"{self.rid}: EXEC '{cmd}' (sleep {STEP_TIME}s)")
            time.sleep(STEP_TIME)

            # apply turn (update facing)
            self.dir = apply_turn(self.dir, cmd)

            # optional: send a post-turn update so server knows new facing while still at same node
            self.update_location(self.node, step_index=idx)

        # safety: if plan had no 'D'
        self.update_location(self.node, status='job_done')
        self.report_execution(job_id)
        log(f"{self.rid}: Job {job_id} finished (no explicit D)")

    def run(self):
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
                plan = job.get('plan')
                if not job_id or not plan:
                    log(f"{self.rid}: received job without plan -> ignoring")
                    time.sleep(POLL_INTERVAL)
                    continue

                log(f"{self.rid}: assigned job {job_id} plan_str={job.get('plan_str') or ''}")
                # Execute the plan exactly as server provided
                self.execute_plan(plan, job_id)
                # small pause before polling again
                time.sleep(0.2)

            except Exception as e:
                log(f"{self.rid}: EXCEPTION {e}")
                time.sleep(1.0)

# ---------------- main ----------------
if __name__ == "__main__":
    try:
        n = int(input("Number of robots to simulate (default 3): ").strip() or "3")
    except Exception:
        n = 3
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

"""
robot_simulator.py

A realistic multi-robot simulator (terminal) that talks to your Fleet Commander server
(assumes server.py from your message is running at SERVER_URL).

Features:
- Registers N robots with the server (/register_robot)
- Each robot polls /poll_task for assigned jobs
- When assigned a job with a `plan`, the robot "executes" it step-by-step:
  - sleeps per step (configurable seconds_per_step) to simulate travel time
  - updates server with /update_location (node, dir, step_index)
  - collects nodes+dir history and finally reports /report_execution
- Simulates network jitter, occasional packet loss (configurable)
- Console visualization of robot positions + status
- Optional: submit a manual job with --pickup and --drop to test request_path

Usage:
    python robot_simulator.py --server http://127.0.0.1:8080 --robots 4

Dependencies:
    pip install requests

Note: This is a simulator for testing the server behaviour. It DOES NOT attempt
to override server reservations or internal logic; it follows the plan the server
hands out and reports progress back.

"""

import requests
import threading
import time
import uuid
import random
import argparse
import sys
from collections import deque

# -------------------- Config --------------------
DEFAULT_SERVER = 'http://127.0.0.1:8080'
NUM_ROBOTS = 3
SECONDS_PER_STEP = 0.9         # realistic per-edge travel time (seconds)
NETWORK_JITTER = 0.08          # plus/minus seconds jitter on each request
PACKET_LOSS_PROB = 0.02        # small chance a request "fails" (simulated)
PRINT_INTERVAL = 1.0

# Robot behaviour
RETRY_BACKOFF = 1.0
MAX_RETRIES = 5

# Direction helpers (must match server's semantics)
CLOCKWISE = {'n':'e','e':'s','s':'w','w':'n'}
CCW = {v:k for k,v in CLOCKWISE.items()}
OPP = {'n':'s','s':'n','e':'w','w':'e'}

# -------------------- Simulator Code --------------------

class RobotSimulator(threading.Thread):
    def __init__(self, server, robot_id=None, start_node='81', start_dir='s', seconds_per_step=SECONDS_PER_STEP):
        super().__init__(daemon=True)
        self.server = server.rstrip('/')
        self.robot_id = robot_id or str(uuid.uuid4())[:6]
        self.node = start_node
        self.dir = start_dir.lower()
        self.color = None
        self.status = 'init'
        self.current_job = None
        self.current_plan = []
        self.seconds_per_step = seconds_per_step
        self._stop = threading.Event()
        self.nodes_with_dir = []

    def log(self, *parts):
        print(f"[{self.robot_id}]", *parts)

    def jitter_sleep(self):
        jitter = random.uniform(-NETWORK_JITTER, NETWORK_JITTER)
        t = max(0.01, self.seconds_per_step + jitter)
        time.sleep(t)

    def http_post(self, path, payload):
        url = f"{self.server}{path}"
        # simulate packet loss
        if random.random() < PACKET_LOSS_PROB:
            # simulate a broken request by returning None
            return None
        for attempt in range(1, MAX_RETRIES+1):
            try:
                time.sleep(random.uniform(-NETWORK_JITTER, NETWORK_JITTER))
                r = requests.post(url, json=payload, timeout=5)
                if r.status_code == 200:
                    return r.json()
                else:
                    self.log('HTTP', r.status_code, r.text)
            except Exception as e:
                # backoff
                time.sleep(RETRY_BACKOFF * attempt)
        return None

    def http_get(self, path, params=None):
        url = f"{self.server}{path}"
        if random.random() < PACKET_LOSS_PROB:
            return None
        for attempt in range(1, MAX_RETRIES+1):
            try:
                time.sleep(random.uniform(-NETWORK_JITTER, NETWORK_JITTER))
                r = requests.get(url, params=params, timeout=5)
                if r.status_code == 200:
                    return r.json()
                else:
                    self.log('HTTP GET', r.status_code, r.text)
            except Exception as e:
                time.sleep(RETRY_BACKOFF * attempt)
        return None

    def register(self):
        payload = {'robot_id': self.robot_id, 'node': self.node, 'dir': self.dir}
        ans = self.http_post('/register_robot', payload)
        if ans:
            self.color = ans.get('color')
            self.log('registered at', self.node, 'color', self.color)
            self.status = 'idle'
            return True
        self.log('registration failed')
        return False

    def poll_for_job(self):
        ans = self.http_get('/poll_task', params={'robot_id': self.robot_id})
        if not ans:
            return None
        return ans.get('job')

    def request_path(self, pickup, drop):
        payload = {'robot_id': self.robot_id, 'node': self.node, 'dir': self.dir, 'pickup': pickup, 'drop': drop}
        ans = self.http_post('/request_path', payload)
        return ans

    def send_update_location(self, node, step_index=None, status=None, dir_report=None):
        payload = {'robot_id': self.robot_id, 'node': node}
        if step_index is not None:
            payload['step_index'] = step_index
        if status:
            payload['status'] = status
        if dir_report:
            payload['dir'] = dir_report
        return self.http_post('/update_location', payload)

    def report_execution(self, job_id=None):
        payload = {'robot_id': self.robot_id}
        if job_id:
            payload['job_id'] = job_id
        # attach nodes_with_dir for richer server record
        payload['nodes_with_dir'] = [{'node': n, 'dir': d} for n,d in self.nodes_with_dir]
        return self.http_post('/report_execution', payload)

    def apply_cmd_to_dir(self, cmd):
        # cmd: 'R','L','U','S'
        if not cmd: return
        cur = self.dir
        if cmd == 'R':
            self.dir = CLOCKWISE.get(cur, cur)
        elif cmd == 'L':
            self.dir = CCW.get(cur, cur)
        elif cmd == 'U':
            self.dir = OPP.get(cur, cur)
        else:
            self.dir = cur

    def execute_plan(self, plan, job_id=None):
        # plan: list of [node, cmd] ... final node likely has 'D'
        self.status = 'executing'
        self.current_plan = plan
        self.current_job = job_id
        self.nodes_with_dir = []
        for idx, step in enumerate(plan):
            node, cmd = step[0], (step[1] if len(step) > 1 else None)
            # "move" to node
            # Sleep to simulate travel
            self.jitter_sleep()
            # update internal state
            self.node = node
            if cmd and cmd != 'D':
                # changes facing for next edge
                self.apply_cmd_to_dir(cmd)
            # record for final report
            self.nodes_with_dir.append((node, self.dir))

            # send update to server
            # For the simulator we send step index as idx
            status = None
            if cmd == 'D':
                status = 'job_done'
            self.send_update_location(node=node, step_index=idx, status=status, dir_report=self.dir)
        # after finishing plan, also call report_execution
        self.report_execution(job_id=job_id)
        self.status = 'idle'
        self.current_plan = []
        self.current_job = None

    def run(self):
        # register first
        if not self.register():
            # try a few times
            for _ in range(3):
                time.sleep(1)
                if self.register(): break
        # main loop: poll for tasks and execute
        while not self._stop.is_set():
            try:
                job = self.poll_for_job()
                if job:
                    # server may send full job object or just id; handle gracefully
                    job_id = job.get('id') if isinstance(job, dict) else None
                    plan = job.get('plan') if isinstance(job, dict) else None
                    # fallback: server might send a job with path but no plan -> just follow path
                    if not plan and isinstance(job, dict) and job.get('path'):
                        # convert path to trivial plan: each node + 'S' except last 'D'
                        path = job.get('path')
                        plan = [[n, 'S'] for n in path]
                        if len(plan) > 0:
                            plan[-1][1] = 'D'
                    if plan:
                        self.log('received job', job_id, 'plan_len', len(plan))
                        self.execute_plan(plan, job_id=job_id)
                    else:
                        # nothing yet - sleep and poll again
                        time.sleep(0.5)
                else:
                    # idle - sleep a bit before polling again
                    time.sleep(0.8 + random.random()*0.2)
            except Exception as e:
                self.log('error in main loop', e)
                time.sleep(1.0)

    def stop(self):
        self._stop.set()


# -------------------- Console Monitor --------------------
class Monitor(threading.Thread):
    def __init__(self, robots, interval=PRINT_INTERVAL):
        super().__init__(daemon=True)
        self.robots = robots
        self.interval = interval
        self._stop = threading.Event()

    def run(self):
        while not self._stop.is_set():
            lines = []
            for r in self.robots:
                lines.append(f"{r.robot_id[:6]} @{r.node} ({r.dir.upper()}) {r.status}")
            sys.stdout.write('\r' + ' | '.join(lines) + ' ' * 10)
            sys.stdout.flush()
            time.sleep(self.interval)

    def stop(self):
        self._stop.set()


# -------------------- Main --------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--server', default=DEFAULT_SERVER, help='Fleet server base URL')
    parser.add_argument('--robots', type=int, default=None)
    parser.add_argument('--seconds_per_step', type=float, default=SECONDS_PER_STEP)
    parser.add_argument('--pickup', help='(optional) immediately request a path pickup node')
    parser.add_argument('--drop', help='(optional) immediately request a path drop node')
    parser.add_argument('--start_node', default='81')
    parser.add_argument('--seed', type=int, default=None)
    args = parser.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    # Ask user for N robots if not provided
    if args.robots is None:
        try:
            args.robots = int(input("Enter number of robots to simulate: "))
        except:
            print("Invalid number, defaulting to 1 robot")
            args.robots = 1

    # ensure robots start on unique parking nodes
    PARKING_NODES = ['81','82','83','84','85','86','11','12','13','15','26','31','46','51','56']
    if args.robots > len(PARKING_NODES):
        print(f"Requested {args.robots} robots but only {len(PARKING_NODES)} unique parking nodes available.\nLimiting to {len(PARKING_NODES)} robots to ensure unique start positions.")
        args.robots = len(PARKING_NODES)

    robots = []
    # sample unique parking spots
    parking_sample = random.sample(PARKING_NODES, args.robots)
    for i in range(args.robots):
        rid = f"r{i+1:02d}"
        start = parking_sample[i]
        r = RobotSimulator(server=args.server, robot_id=rid, start_node=start, seconds_per_step=args.seconds_per_step)
        robots.append(r)
        r.start()

    mon = Monitor(robots)
    mon.start()

    # optionally submit a manual request_path for the first robot
    if args.pickup and args.drop:
        time.sleep(1.0)
        first = robots[0]
        res = first.request_path(args.pickup, args.drop)
        if res:
            print('\nRequested path for', first.robot_id, '->', res)
        else:
            print('\nRequest path failed (no response)')

    try:
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        print('\nStopping simulator...')
        for r in robots: r.stop()
        mon.stop()
        time.sleep(0.5)


if __name__ == '__main__':
    main()

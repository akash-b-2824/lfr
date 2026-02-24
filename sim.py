# robot_simulator.py
# Simulates two robot clients (R1 @81 south, R2 @82 south) for the central server API.
# Usage: python robot_simulator.py
# Requires: pip install requests

import threading
import time
import requests
import random
import sys

# ---------- CONFIG ----------
SERVER = "http://127.0.0.1:8080"   # change if central_server runs elsewhere
POLL_INTERVAL = 1.0                # seconds between poll attempts
NODE_TRAVEL_BASE = 1.2             # base seconds to travel between nodes
NODE_TRAVEL_JITTER = 0.25          # random jitter added/subtracted
REGISTER_RETRY = 3                 # retry register attempts

# Robot definitions
ROBOTS = [
    {"robot_id": "Ram1", "start_node": "81", "start_dir": "s"},
    {"robot_id": "R2", "start_node": "82", "start_dir": "s"},
    {"robot_id": "R3", "start_node": "83", "start_dir": "s"},
    {"robot_id": "R4", "start_node": "84", "start_dir": "s"},
    {"robot_id": "R5", "start_node": "85", "start_dir": "s"},
]

# ---------- helpers ----------
def safe_post(path, json=None, timeout=5):
    url = SERVER.rstrip('/') + path
    try:
        r = requests.post(url, json=json, timeout=timeout)
        r.raise_for_status()
        return r.json()
    except Exception as e:
        print(f"[HTTP POST ERROR] {path} -> {e}")
        return None

def safe_get(path, timeout=5):
    url = SERVER.rstrip('/') + path
    try:
        r = requests.get(url, timeout=timeout)
        r.raise_for_status()
        return r.json()
    except Exception as e:
        print(f"[HTTP GET ERROR] {path} -> {e}")
        return None

# ---------- Robot simulator class ----------
class RobotSim(threading.Thread):
    def __init__(self, robot_id, start_node, start_dir):
        super().__init__(daemon=True)
        self.robot_id = robot_id
        self.node = start_node
        self.dir = start_dir
        self.running = True

    def register(self):
        payload = {"robot_id": self.robot_id, "node": self.node, "direction": self.dir}
        for attempt in range(REGISTER_RETRY):
            resp = safe_post("/register_robot", json=payload)
            if resp and isinstance(resp, dict) and resp.get("robot_id"):
                print(f"[{self.robot_id}] Registered successfully: {resp}")
                return True
            print(f"[{self.robot_id}] register attempt {attempt+1} failed, retrying...")
            time.sleep(1)
        print(f"[{self.robot_id}] registration failed after retries.")
        return False

    def poll_task(self):
        resp = safe_get(f"/poll_task?robot_id={self.robot_id}")
        if not resp:
            return None
        # resp expected: {'job': {...}} or {'job': None}
        if isinstance(resp, dict) and 'job' in resp:
            return resp['job']
        return None

    def send_update(self, node, status=None):
        payload = {"robot_id": self.robot_id, "node": node}
        if status:
            payload["status"] = status
        resp = safe_post("/update_location", json=payload)
        # ignore errors but print them
        print(f"[{self.robot_id}] update_location -> node={node} status={status} resp={resp}")
        return resp

    def simulate_travel(self, path):
        """
        Simulate moving along a path (list of node IDs).
        Sends update_location for each node. Blinks on pickup node (simulated).
        """
        if not path or len(path) == 0:
            return
        print(f"[{self.robot_id}] Simulating path: {path}")
        # assume job has 'pickup' info will be handled in caller
        for i, node in enumerate(path):
            # "arrive" at node
            travel_time = NODE_TRAVEL_BASE + random.uniform(-NODE_TRAVEL_JITTER, NODE_TRAVEL_JITTER)
            # for the first node, we are already there so skip travel
            if i > 0:
                print(f"[{self.robot_id}] traveling to {node} (sleep {travel_time:.2f}s)")
                time.sleep(max(0.05, travel_time))
            self.node = node
            self.send_update(node)
        print(f"[{self.robot_id}] finished path")

    def run(self):
        if not self.register():
            print(f"[{self.robot_id}] aborting due to register failure.")
            return
        # send initial location
        self.send_update(self.node)
        while self.running:
            job = self.poll_task()
            if not job:
                # nothing assigned, sleep
                time.sleep(POLL_INTERVAL)
                continue
            # If job is present
            if isinstance(job, dict) and job.get("status") in ("assigned",) and job.get("path"):
                job_id = job.get("id")
                path = job.get("path")
                # ensure path is a list
                if isinstance(path, str):
                    try:
                        import json as _json
                        path = _json.loads(path)
                    except Exception:
                        path = path.split(",")
                pickup = job.get("pickup")
                drop = job.get("drop")
                print(f"[{self.robot_id}] Received job {job_id} pickup={pickup} drop={drop} path={path}")
                # Simulate moving node by node; blink at pickup
                for i, node in enumerate(path):
                    # travel to next node (skip if i==0 since initial)
                    if i > 0:
                        travel_time = NODE_TRAVEL_BASE + random.uniform(-NODE_TRAVEL_JITTER, NODE_TRAVEL_JITTER)
                        print(f"[{self.robot_id}] -> moving to {node}, will take {travel_time:.2f}s")
                        time.sleep(max(0.05, travel_time))
                    self.node = node
                    # send live update on arrival
                    self.send_update(node)
                    # if we arrived at pickup, simulate blink
                    if node == pickup:
                        print(f"[{self.robot_id}] arrived at PICKUP {pickup} - blinking 3x")
                        for b in range(3):
                            print(f"[{self.robot_id}] BLINK {b+1}")
                            time.sleep(0.12)
                        # brief pause after pickup
                        time.sleep(0.2)
                # finished path -> notify job_done
                final_node = path[-1] if path else self.node
                print(f"[{self.robot_id}] job {job_id} finished at {final_node}, sending job_done")
                self.send_update(final_node, status="job_done")
                # small cooldown
                time.sleep(0.5)
            else:
                # job present but not assigned or no path yet
                time.sleep(POLL_INTERVAL)

    def stop(self):
        self.running = False

# ---------- main ----------
def main():
    sims = []
    for r in ROBOTS:
        sim = RobotSim(r["robot_id"], r["start_node"], r["start_dir"])
        sims.append(sim)
        sim.start()
    print("[simulator] Started robot simulations. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("[simulator] Stopping...")
        for s in sims:
            s.stop()
        # wait a little for threads to finish
        time.sleep(1.0)
        print("[simulator] Exited.")

if __name__ == "__main__":
    main()

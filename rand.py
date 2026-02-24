#!/usr/bin/env python3
"""
batch_submit_tasks.py

Submit a hard-coded list of (pickup, drop) tasks to the /submit_job endpoint.

Edit the TASKS list below and run:
    python batch_submit_tasks.py
"""

import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import List, Tuple
import requests

# ====== CONFIG ======
HOST = "http://127.0.0.1:8080"   # change to your server if needed
SUBMIT_PATH = "/submit_job"
CONCURRENCY = 8                  # number of concurrent HTTP submissions
RETRIES = 3                      # retries per task on failure
DELAY_BETWEEN_STARTS = 0.0       # seconds to wait between starting each submission (throttle)
TIMEOUT = 6                      # seconds for HTTP requests
SHOW_ALL_RESPONSES = False       # set True to print full response JSON for each success
# ====================

# Paste / edit tasks here as (pickup, drop) tuples
TASKS: List[Tuple[str, str]] = [
    ("11", "86"),
    ("13", "15"),
    ("46", "51"),
    ("15", "81"),
    ("65", "12")
]

def submit_once(base_url: str, pickup: str, drop: str, timeout: int = TIMEOUT):
    url = base_url.rstrip('/') + SUBMIT_PATH
    payload = {"pickup": pickup, "drop": drop}
    headers = {"Content-Type": "application/json"}
    resp = requests.post(url, json=payload, headers=headers, timeout=timeout)
    resp.raise_for_status()
    try:
        return resp.json()
    except Exception:
        return {"raw_text": resp.text}

def submit_with_retries(base_url: str, pickup: str, drop: str, retries: int = RETRIES):
    last_exc = None
    for attempt in range(1, retries + 1):
        try:
            r = submit_once(base_url, pickup, drop)
            return {"ok": True, "response": r, "attempts": attempt, "pickup": pickup, "drop": drop}
        except requests.RequestException as e:
            last_exc = e
            backoff = 0.5 * attempt
            time.sleep(backoff)
    return {"ok": False, "error": str(last_exc), "pickup": pickup, "drop": drop}

def main():
    total = len(TASKS)
    if total == 0:
        print("No tasks in TASKS list. Edit the script and add tasks.")
        return

    print(f"Submitting {total} tasks to {HOST} (concurrency={CONCURRENCY}, retries={RETRIES})")
    start_time = time.time()
    results = []

    with ThreadPoolExecutor(max_workers=CONCURRENCY) as ex:
        futures = []
        for i, (p, d) in enumerate(TASKS):
            # optional pacing between start of each submission
            if DELAY_BETWEEN_STARTS and i > 0:
                time.sleep(DELAY_BETWEEN_STARTS)
            futures.append(ex.submit(submit_with_retries, HOST, p, d))

        for idx, fut in enumerate(as_completed(futures), 1):
            res = fut.result()
            results.append(res)
            if res.get("ok"):
                resp = res.get("response", {})
                jid = resp.get("job_id") or (resp.get("job") or {}).get("id") or resp.get("id")
                if SHOW_ALL_RESPONSES:
                    print(f"[OK] ({res['pickup']} -> {res['drop']}) attempt={res['attempts']} response={resp}")
                else:
                    print(f"[OK] ({res['pickup']} -> {res['drop']}) job_id={jid} attempts={res['attempts']}")
            else:
                print(f"[ERR] ({res['pickup']} -> {res['drop']}) error={res.get('error')}")

    elapsed = time.time() - start_time
    succ = sum(1 for r in results if r.get("ok"))
    fail = len(results) - succ

    print("\nSummary:")
    print(f"  total submitted: {total}")
    print(f"  succeeded:       {succ}")
    print(f"  failed:          {fail}")
    print(f"  elapsed:         {elapsed:.2f}s")

    if fail:
        print("\nFailed tasks:")
        for r in results:
            if not r.get("ok"):
                print(f"  {r.get('pickup')} -> {r.get('drop')}  error: {r.get('error')}")

if __name__ == "__main__":
    main()

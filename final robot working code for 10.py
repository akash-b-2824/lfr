# robot_conn.py
# MicroPython helper: network + server protocol skeleton
# - registers robot
# - polls /poll_task while idle (using urequests compatible code)
# - prints any assigned job.plan to serial
# - exposes send_location_update() and send_report_execution() functions
#
# Usage:
#  - flash this to your board
#  - update WIFI_SSID/WIFI_PASS/SERVER_BASE
#  - your line-follow code should call send_location_update(node, dir, step_index, status)
#  - when job is done call send_report_execution(job_id)

from machine import Pin,PWM
import socket
import heapq
import network
import urequests as requests
import ujson as json
import time
import ubinascii
import machine
from time import sleep









from machine import Pin,PWM
from time import sleep
import network
import socket
import heapq
#line follower meow meow

direction='s'
def change_dir(turn):
    global direction

    # How each direction changes on turns
    right_turn = {
        "n": "e",
        "e": "s",
        "s": "w",
        "w": "n"
    }

    left_turn = {
        "n": "w",
        "w": "s",
        "s": "e",
        "e": "n"
    }

    # U-turn mapping
    u_turn = {
        "n": "s",
        "s": "n",
        "e": "w",
        "w": "e"
    }

    # Decide which mapping to apply
    if turn == "r":
        direction = right_turn[direction]
    elif turn == "l":
        direction = left_turn[direction]
    elif turn == "u":
        direction = u_turn[direction]
    elif turn == "s":
        pass   # Straight = no change
    else:
        print("Invalid turn command:", turn)

    return direction


#lfr functions and initialization
lmf=Pin(4,Pin.OUT)
lmb=Pin(2,Pin.OUT)
lmp=PWM(Pin(15))

# #right motor connect 
rmf=Pin(22,Pin.OUT)
rmb=Pin(21,Pin.OUT)
rmp=PWM(Pin(23))




#recurance prevention

#pwm frequency
lmp.freq(1500)
rmp.freq(1500)

#ir connect
BL=Pin(34,Pin.IN,Pin.PULL_DOWN)
FL=Pin(35,Pin.IN,Pin.PULL_DOWN)
FS=Pin(32,Pin.IN,Pin.PULL_DOWN)
FR=Pin(33,Pin.IN,Pin.PULL_DOWN)
BR=Pin(25,Pin.IN,Pin.PULL_DOWN)

#motor forward
def forward():
    lmp.duty_u16(65000)
    rmp.duty_u16(65000)
    lmf.value(1)
    rmf.value(1)
    lmb.value(0)
    rmb.value(0)

#motor turn left
def turn_right():
    lmp.duty_u16(000)
    rmp.duty_u16(65000)
    lmf.value(0)
    rmf.value(1)
    lmb.value(1)
    rmb.value(0)

#motor turn right
def turn_left():
    lmp.duty_u16(65000)
    rmp.duty_u16(000)
    lmf.value(1)
    rmf.value(0)
    lmb.value(0)
    rmb.value(1)

#motor tilt left
def tilt_right():
    lmp.duty_u16(45000)
    rmp.duty_u16(65000)
    lmf.value(1)
    rmf.value(1)
    lmb.value(0)
    rmb.value(0)

#motor tilt right
def tilt_left():
    lmp.duty_u16(65000)
    rmp.duty_u16(45000)
    lmf.value(1)
    rmf.value(1)
    lmb.value(0)
    rmb.value(0)
    
#motor uturn left
def uturn_right():
    lmp.duty_u16(52000)
    rmp.duty_u16(52000)
    lmf.value(0)
    rmf.value(1)
    lmb.value(1)
    rmb.value(0)
#motor uturn right
def uturn_left():
    lmp.duty_u16(65000)
    rmp.duty_u16(65000)
    lmf.value(1)
    rmf.value(0)
    lmb.value(0)
    rmb.value(1)

#motor stop
def stop():
    lmf.value(0)
    rmf.value(0)
    lmb.value(0)
    rmb.value(0)
def reverse():
    lmp.duty_u16(65000)
    rmp.duty_u16(65000)
    lmf.value(0)
    rmf.value(0)
    lmb.value(1)
    rmb.value(1)
    
def readsensor():
    global fl,fs,fr,br,bl
    fl = FL.value()   
    fs = FS.value()
    fr = FR.value()
    br = BR.value()
    bl = BL.value()












# ---------- CONFIG ----------
WIFI_SSID = "As"
WIFI_PASS = "12345678"
SERVER_BASE = "http://10.241.189.98:8080/"  # change to your server ip/port
ROBOT_ID = None            # if None, generated from MAC
START_NODE = "81"
START_DIR = "s"           # 'n','e','s','w'
POLL_INTERVAL = 1.0       # seconds between polls when idle
HTTP_TIMEOUT = 6
curr="s"
job_id=None
# ---------- optional LED ----------
try:
    led = machine.Pin(2, machine.Pin.OUT)
except:
    led = None

def blink(times=1, t=0.06):
    if not led: return
    for _ in range(times):
        led.value(1); time.sleep(t)
        led.value(0); time.sleep(t)

# ---------- network helpers ----------
def get_mac_robot_id():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    try:
        mac = wlan.config('mac')
        if mac:
            return ubinascii.hexlify(mac[-3:]).decode('utf-8')
    except:
        pass
    return "r000"

def connect_wifi(ssid, pwd, max_retries=12):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if wlan.isconnected():
        print("WiFi already connected:", wlan.ifconfig())
        return True
    print("Connecting to WiFi:", ssid)
    wlan.connect(ssid, pwd)
    tries = 0
    while not wlan.isconnected() and tries < max_retries:
        blink(1, 0.04)
        time.sleep(1)
        tries += 1
    if wlan.isconnected():
        print("Connected:", wlan.ifconfig())
        return True
    print("WiFi connect failed")
    return False

# ---------- safe HTTP wrappers (urequests-compatible) ----------
def _encode_qs(params):
    # simple url-encoding for keys/values that contain only safe chars (digits/letters/_)
    # for more complex values, pre-encode them before passing to safe_get
    parts = []
    for k, v in params.items():
        parts.append(str(k) + "=" + str(v))
    return "&".join(parts)

def safe_post(path, payload, timeout=HTTP_TIMEOUT):
    url = SERVER_BASE.rstrip('/') + path
    try:
        r = requests.post(url, json=payload, timeout=timeout)
        try:
            data = r.json()
        except:
            data = None
        status = getattr(r, 'status_code', None)
        r.close()
        return status, data
    except Exception as e:
        print("POST error", url, e)
        return None, None

def safe_get(path, params=None, timeout=HTTP_TIMEOUT):
    # Build URL manually because urequests.get does not support params kwarg
    url = SERVER_BASE.rstrip('/') + path
    if params:
        qs = _encode_qs(params)
        url = url + "?" + qs
    try:
        r = requests.get(url, timeout=timeout)
        try:
            data = r.json()
        except:
            data = None
        status = getattr(r, 'status_code', None)
        r.close()
        return status, data
    except Exception as e:
        print("GET error", url, e)
        return None, None

# ---------- protocol functions you can call ----------
_nodes_with_dir_trace = []   # local trace buffer for report
# robot id final resolved at runtime
ROBOT_ID_FINAL = None

def send_location_update(node, dir, step_index=None, status=None):
    """
    POST /update_location
    node: node id string (e.g. "23")
    dir : 'n'|'e'|'s'|'w'
    step_index: optional integer index of plan step you're at
    status: optional string, e.g. 'job_done'
    Returns (status_code, response_json)
    """
    global _nodes_with_dir_trace
    payload = {'robot_id': ROBOT_ID_FINAL, 'node': node, 'dir': dir}
    if step_index is not None:
        try:
            payload['step_index'] = int(step_index)
        except:
            payload['step_index'] = step_index
    if status is not None:
        payload['status'] = status
    # keep local trace for final report
    _nodes_with_dir_trace.append({'node': node, 'dir': dir, 'ts': time.time(), 'step_index': step_index})
    code, resp = safe_post('/update_location', payload)
    print("Sent location update:", payload, "->", code)
    return code, resp

def send_report_execution(job_id):
    """
    POST /report_execution with nodes_with_dir
    call when job is finished (after you send job_done)
    """
    global _nodes_with_dir_trace
    payload = {'robot_id': ROBOT_ID_FINAL, 'job_id': job_id,
               'nodes_with_dir': [{'node': x['node'], 'dir': x['dir']} for x in _nodes_with_dir_trace]}
    code, resp = safe_post('/report_execution', payload)
    print("Sent report_execution for", job_id, "->", code)
    # reset local trace after reporting
    _nodes_with_dir_trace = []
    return code, resp

# ---------- register & poll helpers ----------
def register_robot(start_node=START_NODE, start_dir=START_DIR):
    payload = {'robot_id': ROBOT_ID_FINAL, 'node': start_node, 'dir': start_dir}
    code, resp = safe_post('/register_robot', payload)
    print("Register response:", code, resp)
    return code == 200

def poll_task():
    # Note: safe_get expects params dict as second arg
    code, resp = safe_get('/poll_task', params={'robot_id': ROBOT_ID_FINAL})
    if code == 200 and resp is not None:
        # resp = {'job': {...}} or {'job': None}
        job = resp.get('job')
        return job
    return None

# ---------- main loop: polls while idle and prints assigned job plan ----------
def main_loop():
    global plan,job_id
    print("Starting main loop. Poll interval:", POLL_INTERVAL)
    while True:
        try:
            job = poll_task()
            if not job:
                # idle => poll after sleep
                time.sleep(POLL_INTERVAL)
                continue
            # we have a job assigned
            job_id = job.get('id') or job.get('job_id')
            plan = job.get('plan')            # list of [node, cmd]
            plan_str = job.get('plan_str')    # human readable string if server provided
            assigned_robot = job.get('assigned_robot')
            print("=== ASSIGNED JOB ===")
            print("job_id:", job_id)
            print("assigned_robot:", assigned_robot)
            print("plan_str:", plan_str)
            print("plan raw:", plan)
            print("You should now start executing the plan. When you detect node arrivals call:")
            print("  send_location_update(node, dir, step_index)")
            print("After final step call:")
            print("  send_report_execution(job_id)")
            
            # small delay to avoid busy-looping; real execution replaces this
            time.sleep(0.5)
            return plan
        except Exception as e:
            print("Main loop exception:", e)
            time.sleep(1.0)

# ---------- boot / startup ----------
if __name__ == "__main__":
    # prepare robot id
    if ROBOT_ID is None:
        ROBOT_ID_FINAL = get_mac_robot_id()
    else:
        ROBOT_ID_FINAL = ROBOT_ID

    print("Robot ID:", ROBOT_ID_FINAL)
    # connect wifi
    if not connect_wifi(WIFI_SSID, WIFI_PASS):
        print("WiFi connection failed. Will retry in 10s.")
        time.sleep(10)
        # Optionally add a reboot or retry loop, but for now just try once more then continue
        if not connect_wifi(WIFI_SSID, WIFI_PASS):
            print("Second WiFi attempt failed; continuing but network calls will fail.")
    else:
        blink(2, 0.05)

    # best-effort registration
    try:
        register_ok = register_robot()
        if not register_ok:
            print("Initial registration failed; continue polling anyway.")
    except Exception as e:
        print("Registration exception:", e)
b=-1
stepindex=-1
    # start polling loop
def st():
    global path,rec,lef,reg,k,b,stepindex
    plan=main_loop()
    y=""
    for i in range(len(plan)):
        y+=plan[i][1]
    path=y.lower()[:-1]
    print(path)
    path=path.replace('l','z')
    path=path.replace("r","l")
    path=path.replace('z','r')
    rec=True
    lef=False
    reg=False
    b=-1
    stepindex=-1
    if not (path[0]=='u' or path[0]=='s'):
        reverse()
        sleep(.05)
        stop()
        sleep(.1)
        reverse()
        sleep(1)
        stop()
        sleep(.5)
    if path[0]=='s':
        path=path[1:]
    path+='sj'
    k=-len(path)
lef=False
rig=True
rec=True
while True:
    st()
    def preo(g):
        global rig
        global lef,k,b,stepindex,job_id
        if path[g]=='j':
            
            forward()
            sleep(.3)
            stop()
            sleep(1)
            send_report_execution(job_id)
            sleep(1)
            st()
        if b!=k:
            stop()
            stepindex+=1
            send_location_update(plan[k+1][0],direction,stepindex)
            b=k

        if path[g]=='r':
            lef=True
            rig=False
            change_dir('l')
        elif path[g]=='u':
            stop()
            sleep(.4)
            forward()
            sleep(.3)
            uturn_left()
            sleep(1.3)
            stop()
            sleep(.3)
            k+=1
            change_dir('u')
        elif path[g]=='s':
            lef=False
            rig=False
        else:
            lef=False
            rig=True
            change_dir('r')
    #main loop

    #main loop
    while True:
        preo(k)
        #initialize short cuts
        readsensor()

        #condition strict left
        if (bl==1 or br==1)and(rec and (not rig)and(not lef)):
            rec=False
            k+=1
            forward()
            sleep(.3)
        elif bl==1 and rec and lef:
            forward()
            sleep(.5)
            uturn_left()
            k+=1
            rec=False
            sleep(.55)
        elif br==1 and rec and rig:
            forward()
            sleep(.5)
            uturn_right()
            k+=1
            rec=False
            sleep(.55)
        elif fl==0 and fs==1 and fr==0:
            forward()
            rec=True
        elif fl==1 and fs==1 and fr==0:
            tilt_left()
            rec=True
        elif fl==1 and fs==0 and fr==0:turn_left()
        elif fl==0 and fs==1 and fr==1:
            tilt_right()
            rec=True
        elif fl==0 and fs==0 and fr==1:turn_right()
        elif fl==1 and fs==1 and fr==1:forward()
        sleep(0.001)

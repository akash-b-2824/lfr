import cv2
import mediapipe as mp
import time
import math
import requests

# Server Configuration
SERVER_URL = "http://192.168.1.116:8080/submit_job"

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.7)
mp_draw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)
tip_ids = [4, 8, 12, 16, 20]

# Logic Variables
last_number = None
start_time = None
current_input = []  # Current numbers being typed
pickup_node = None  # Confirmed pickup
drop_node = None    # Confirmed drop
already_saved = False
cooldown_until = 0  # To prevent double diamond triggers

# Thresholds
PINKY_TOUCH_THRESHOLD = 45 
CLOSE_THRESHOLD = 50 

while True:
    ret, frame = cap.read()
    if not ret: break

    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = hands.process(rgb)

    total = 0
    hand_visible = False
    pinky_points = []
    thumbs = {}
    indices = {}

    if result.multi_hand_landmarks:
        hand_visible = True
        for handLms, handedness in zip(result.multi_hand_landmarks, result.multi_handedness):
            label = handedness.classification[0].label
            lm = []
            h, w, _ = frame.shape
            for i, p in enumerate(handLms.landmark):
                px, py = int(p.x * w), int(p.y * h)
                lm.append((px, py))
                if i == 4: thumbs[label] = (px, py)
                if i == 8: indices[label] = (px, py)
                if i == 20: pinky_points.append((px, py))

            fingers = []
            if label == "Right": fingers.append(lm[4][0] < lm[3][0])
            else: fingers.append(lm[4][0] > lm[3][0])
            for i in range(1, 5): fingers.append(lm[tip_ids[i]][1] < lm[tip_ids[i] - 2][1])
            total += fingers.count(True)
            mp_draw.draw_landmarks(frame, handLms, mp_hands.HAND_CONNECTIONS)

    # -------- DIAMOND GESTURE (ENTER/SUBMIT) --------
    if len(thumbs) == 2 and len(indices) == 2 and time.time() > cooldown_until:
        d_thumb = math.hypot(thumbs['Left'][0] - thumbs['Right'][0], thumbs['Left'][1] - thumbs['Right'][1])
        d_index = math.hypot(indices['Left'][0] - indices['Right'][0], indices['Left'][1] - indices['Right'][1])

        if d_thumb < CLOSE_THRESHOLD and d_index < CLOSE_THRESHOLD:
            input_val = "".join(map(str, current_input))
            
            if pickup_node is None:
                # First Diamond: Confirm Pickup
                if input_val:
                    pickup_node = input_val
                    current_input = []
                    print(f"Pickup Confirmed: {pickup_node}")
            else:
                # Second Diamond: Confirm Drop and Send to Server
                if input_val:
                    drop_node = input_val
                    print(f"Sending Job: {pickup_node} -> {drop_node}")
                    try:
                        resp = requests.post(SERVER_URL, json={'pickup': pickup_node, 'drop': drop_node})
                        if resp.status_code == 200:
                            print("Job Submitted Successfully!")
                    except Exception as e:
                        print("Server error:", e)
                    
                    # Reset all for next job
                    pickup_node = None
                    drop_node = None
                    current_input = []
            
            cooldown_until = time.time() + 2.0  # 2 second cooldown

    # -------- PINKY TOUCH (CLEAR CURRENT ENTRY) --------
    if len(pinky_points) == 2:
        dist = math.hypot(pinky_points[0][0] - pinky_points[1][0], pinky_points[0][1] - pinky_points[1][1])
        if dist < PINKY_TOUCH_THRESHOLD:
            current_input = []
            time.sleep(0.5)

    # -------- TIMER SAVE LOGIC --------
    if hand_visible:
        if total != last_number:
            last_number = total
            start_time = time.time()
            already_saved = False
        elif start_time and not already_saved:
            if time.time() - start_time >= 2: # Reduced to 2s for faster input
                current_input.append(total)
                already_saved = True
    else:
        last_number = None

    # -------- DISPLAY --------
    # UI Overlay
    cv2.rectangle(frame, (20, 20), (500, 160), (0, 0, 0), -1)
    
    # State tracking display
    status_text = "Step 1: Set Pickup" if pickup_node is None else "Step 2: Set Drop"
    cv2.putText(frame, status_text, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    
    # Show entries
    p_display = pickup_node if pickup_node else "___"
    d_display = "".join(map(str, current_input)) if pickup_node else ""
    
    if pickup_node:
        cv2.putText(frame, f"Pickup: {p_display}", (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(frame, f"Drop Input: {d_display}", (30, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    else:
        cv2.putText(frame, f"Pickup Input: {''.join(map(str, current_input))}", (30, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    cv2.imshow("Hand Control - Fleet Commander", frame)
    if cv2.waitKey(1) & 0xFF == 27: break

cap.release()
cv2.destroyAllWindows()
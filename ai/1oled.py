from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont
import cv2
import mediapipe as mp
import time
import math
import requests

# ---------------- OLED SETUP ----------------
serial = i2c(port=1, address=0x3C)
device = ssd1306(serial, width=128, height=64)

width = device.width
height = device.height

try:
    big_font = ImageFont.truetype("DejaVuSans-Bold.ttf", 48)
    small_font = ImageFont.truetype("DejaVuSans.ttf", 12)
except:
    big_font = ImageFont.load_default()
    small_font = ImageFont.load_default()

def display_oled(number_str="", status=""):
    image = Image.new("1", (width, height))
    draw = ImageDraw.Draw(image)

    draw.rectangle((0, 0, width, height), outline=0, fill=0)

    # Status (top)
    draw.text((2, 0), status, font=small_font, fill=255)

    # Big number
    bbox = draw.textbbox((0, 0), number_str, font=big_font)
    w = bbox[2] - bbox[0]
    h = bbox[3] - bbox[1]

    x = (width - w) // 2
    y = (height - h) // 2 + 5

    draw.text((x, y), number_str, font=big_font, fill=255)

    device.display(image)

# ---------------- HAND TRACKING SETUP ----------------
SERVER_URL = "https://freefox.fun/submit_job"

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.7)

cap = cv2.VideoCapture(0)
tip_ids = [4, 8, 12, 16, 20]

# ---------------- LOGIC VARIABLES ----------------
last_number = None
start_time = None
current_input = []
pickup_node = None
drop_node = None
already_saved = False
cooldown_until = 0

# Thresholds
PINKY_TOUCH_THRESHOLD = 45
CLOSE_THRESHOLD = 50

while True:
    ret, frame = cap.read()
    if not ret:
        break

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
            if label == "Right":
                fingers.append(lm[4][0] < lm[3][0])
            else:
                fingers.append(lm[4][0] > lm[3][0])

            for i in range(1, 5):
                fingers.append(lm[tip_ids[i]][1] < lm[tip_ids[i] - 2][1])

            total += fingers.count(True)

    # -------- DIAMOND (CONFIRM) --------
    if len(thumbs) == 2 and len(indices) == 2 and time.time() > cooldown_until:
        d_thumb = math.hypot(thumbs['Left'][0] - thumbs['Right'][0], thumbs['Left'][1] - thumbs['Right'][1])
        d_index = math.hypot(indices['Left'][0] - indices['Right'][0], indices['Left'][1] - indices['Right'][1])

        if d_thumb < CLOSE_THRESHOLD and d_index < CLOSE_THRESHOLD:
            input_val = "".join(map(str, current_input))

            if pickup_node is None:
                if input_val:
                    pickup_node = input_val
                    current_input = []
            else:
                if input_val:
                    drop_node = input_val

                    # OLED → Sending
                    display_oled("...", "Sending")

                    try:
                        requests.post(SERVER_URL, json={
                            'pickup': pickup_node,
                            'drop': drop_node
                        })
                    except:
                        pass

                    # Reset
                    pickup_node = None
                    drop_node = None
                    current_input = []

            cooldown_until = time.time() + 2.0

    # -------- PINKY CLEAR --------
    if len(pinky_points) == 2:
        dist = math.hypot(
            pinky_points[0][0] - pinky_points[1][0],
            pinky_points[0][1] - pinky_points[1][1]
        )
        if dist < PINKY_TOUCH_THRESHOLD:
            current_input = []
            time.sleep(0.5)

    # -------- NUMBER CAPTURE --------
    if hand_visible:
        if total != last_number:
            last_number = total
            start_time = time.time()
            already_saved = False
        elif start_time and not already_saved:
            if time.time() - start_time >= 2:
                current_input.append(total)
                already_saved = True
    else:
        last_number = None

    # -------- OLED DISPLAY LOGIC --------
    if pickup_node is None:
        status = "Pickup"
        number = "".join(map(str, current_input))[-2:]
    else:
        status = "Drop"
        number = "".join(map(str, current_input))[-2:]

    if number == "":
        number = "--"

    display_oled(number, status)

    # -------- CAMERA PREVIEW (OPTIONAL) --------
    cv2.imshow("Camera", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
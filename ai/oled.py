from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont
import time
import RPi.GPIO as GPIO

# ---------------- GPIO SETUP ----------------
RED = 17
GREEN = 27
BLUE = 22

GPIO.setmode(GPIO.BCM)
GPIO.setup(RED, GPIO.OUT)
GPIO.setup(GREEN, GPIO.OUT)
GPIO.setup(BLUE, GPIO.OUT)

def set_color(r, g, b):
    GPIO.output(RED, r)
    GPIO.output(GREEN, g)
    GPIO.output(BLUE, b)

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

def display_number(number, status_text="Status"):
    image = Image.new("1", (width, height))
    draw = ImageDraw.Draw(image)

    draw.rectangle((0, 0, width, height), outline=0, fill=0)

    draw.text((2, 0), status_text, font=small_font, fill=255)

    num_str = f"{number:02d}"

    bbox = draw.textbbox((0, 0), num_str, font=big_font)
    text_width = bbox[2] - bbox[0]
    text_height = bbox[3] - bbox[1]

    x = (width - text_width) // 2
    y = (height - text_height) // 2 + 5

    draw.text((x, y), num_str, font=big_font, fill=255)

    device.display(image)

# ---------------- MAIN LOOP ----------------
try:
    while True:
        for i in range(100):

            # LED Logic
            if i % 3 == 0:
                set_color(1, 0, 0)  # Red
            elif i % 3 == 1:
                set_color(0, 1, 0)  # Green
            else:
                set_color(0, 0, 1)  # Blue

            display_number(i, status_text="Running")
            time.sleep(1)

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()
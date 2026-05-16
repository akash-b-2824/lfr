import cv2
import board
import busio
from PIL import Image
import adafruit_ssd1306

# OLED setup
WIDTH = 128
HEIGHT = 64

i2c = busio.I2C(board.SCL, board.SDA)
oled = adafruit_ssd1306.SSD1306_I2C(WIDTH, HEIGHT, i2c)

oled.fill(0)
oled.show()

# Open USB camera
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # Resize to OLED resolution
    frame = cv2.resize(frame, (WIDTH, HEIGHT))

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Convert to PIL image
    image = Image.fromarray(gray)

    # Convert to 1-bit (black & white)
    image = image.convert("1")

    # Display on OLED
    oled.image(image)
    oled.show()

# Cleanup (never reached normally)
cap.release()
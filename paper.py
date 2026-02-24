"""
sheet_crop_live.py

Detect a white sheet in a webcam stream, draw a clear border, perform perspective warp
and display the cropped/flattened sheet in real time.

Controls:
  - 'c' : save the current cropped (warped) sheet as sheet_<n>.png
  - 'q' : quit

Dependencies:
  pip install opencv-python numpy
"""
import cv2
import numpy as np
import time
import os

# ---------- helper functions ----------
def order_points(pts):
    # pts: (4,2) array of points
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]   # top-left (smallest sum)
    rect[2] = pts[np.argmax(s)]   # bottom-right (largest sum)
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]  # top-right (smallest difference)
    rect[3] = pts[np.argmax(diff)]  # bottom-left (largest difference)
    return rect

def four_point_transform(image, pts):
    rect = order_points(pts)
    (tl, tr, br, bl) = rect

    # compute the width of the new image
    widthA = np.linalg.norm(br - bl)
    widthB = np.linalg.norm(tr - tl)
    maxWidth = int(max(widthA, widthB))

    # compute the height of the new image
    heightA = np.linalg.norm(tr - br)
    heightB = np.linalg.norm(tl - bl)
    maxHeight = int(max(heightA, heightB))

    # destination points for "birds eye view"
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype="float32")

    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
    return warped

# ---------- parameters (tweak as needed) ----------
BLUR_KERNEL = (7, 7)
MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
MIN_CONTOUR_AREA = 20000   # minimum area for a contour to be considered a sheet (tweak)
CANNY_LOW = 50
CANNY_HIGH = 150

# ---------- setup ----------
cap = cv2.VideoCapture(1)   # change index if you have multiple cameras
if not cap.isOpened():
    raise RuntimeError("Could not open camera. Try changing camera index (0 -> 1 -> 2).")

save_dir = "sheet_crops"
os.makedirs(save_dir, exist_ok=True)
save_count = 0

print("Press 'c' to save crop, 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame, exiting.")
        break

    # Resize for speed (optional) - keep a copy for high-res crop
    frame_w = frame.copy()
    h, w = frame_w.shape[:2]
    scale = 800.0 / max(h, w) if max(h,w) > 800 else 1.0
    if scale != 1.0:
        frame = cv2.resize(frame, dsize=None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    else:
        frame = frame.copy()

    orig = frame.copy()

    # Convert to HSV and isolate bright/white regions by high V and low saturation
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h_channel, s_channel, v_channel = cv2.split(hsv)

    # White has low saturation and high value. Tune these thresholds if needed.
    # These thresholds work reasonably under diffuse indoor light.
    white_mask = cv2.inRange(hsv, np.array([0, 0, 180]), np.array([180, 60, 255]))

    # Optional: combine with brightness threshold from grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, bright_mask = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

    mask = cv2.bitwise_or(white_mask, bright_mask)

    # Clean up mask
    mask = cv2.medianBlur(mask, 5)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL, iterations=1)

    # Find edges on mask for better contour detection
    edges = cv2.Canny(mask, CANNY_LOW, CANNY_HIGH)

    # Find contours (external)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    sheet_contour = None
    max_area = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_CONTOUR_AREA:
            continue
        # approximate to quad
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        if len(approx) == 4 and area > max_area:
            sheet_contour = approx
            max_area = area

    # If a quad was not found, try the largest contour and approximate anyway
    if sheet_contour is None and contours:
        # find largest contour by area
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) >= MIN_CONTOUR_AREA:
            peri = cv2.arcLength(largest, True)
            approx = cv2.approxPolyDP(largest, 0.02 * peri, True)
            if len(approx) >= 4:
                # take the 4 extreme points from approx by using convex hull then selecting 4 corners
                hull = cv2.convexHull(approx)
                # fallback: bounding rect corners
                x,y,ww,hh = cv2.boundingRect(hull)
                sheet_contour = np.array([[[x,y]], [[x+ww,y]], [[x+ww,y+hh]], [[x,y+hh]]], dtype=np.int32)
                max_area = cv2.contourArea(largest)

    display = frame.copy()
    warped = None

    if sheet_contour is not None:
        # draw a thick clear border
        cv2.drawContours(display, [sheet_contour], -1, (0,255,0), 3)  # green contour
        # get float points for transform
        pts = sheet_contour.reshape(4,2).astype("float32")
        try:
            warped = four_point_transform(frame, pts)
        except Exception as e:
            # If perspective transform fails, fallback to bounding rect crop
            x,y,wc,hc = cv2.boundingRect(sheet_contour)
            warped = frame[y:y+hc, x:x+wc].copy()

        # show small preview of warped on the corner
        if warped is not None:
            # resize preview to fit
            preview_h = int(display.shape[0] * 0.4)
            preview_w = int(warped.shape[1] * (preview_h / warped.shape[0]))
            preview = cv2.resize(warped, (preview_w, preview_h))
            # put a white border behind preview for clarity
            px, py = 10, 10
            cv2.rectangle(display, (px-2, py-2), (px+preview_w+2, py+preview_h+2), (255,255,255), -1)
            display[py:py+preview_h, px:px+preview_w] = preview

    else:
        cv2.putText(display, "No sheet detected", (20,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)

    # show windows
    cv2.imshow("Live - Sheet Detection", display)
    cv2.imshow("Mask", mask)

    if warped is not None:
        cv2.imshow("Cropped / Warped Sheet", warped)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    if key == ord('c') and warped is not None:
        fname = os.path.join(save_dir, f"sheet_{int(time.time())}_{save_count}.png")
        cv2.imwrite(fname, warped)
        print(f"Saved {fname}")
        save_count += 1

# cleanup
cap.release()
cv2.destroyAllWindows()

"""
a4_object_coords_axes.py

Detect A4 sheet, warp to top-down, find objects on the sheet, draw center X/Y axes,
show cm tick marks, and overlay a legend at each object's center with distances (cm).

Controls:
 - 'c' to save current warped sheet image
 - 'q' to quit

Dependencies:
 pip install opencv-python numpy
"""

import cv2
import numpy as np
import time
import os
import math

# ---------- user params ----------
CAM_INDEX = 1            # camera index (0,1,2...)
MIN_CONTOUR_AREA = 2000  # minimum area for an "object" on sheet (px) - tune for your objects
SAVE_DIR = "a4_crops_axes"
A4_WIDTH_MM = 210.0
A4_HEIGHT_MM = 297.0
TICK_CM = 2              # tick spacing in cm along axes
FONT = cv2.FONT_HERSHEY_SIMPLEX

os.makedirs(SAVE_DIR, exist_ok=True)

# ---------- helper functions ----------
def order_points(pts):
    rect = np.zeros((4,2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]   # top-left
    rect[2] = pts[np.argmax(s)]   # bottom-right
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]  # top-right
    rect[3] = pts[np.argmax(diff)]  # bottom-left
    return rect

def four_point_transform(image, pts):
    rect = order_points(pts)
    (tl, tr, br, bl) = rect
    widthA = np.linalg.norm(br - bl)
    widthB = np.linalg.norm(tr - tl)
    maxWidth = int(max(widthA, widthB))
    heightA = np.linalg.norm(tr - br)
    heightB = np.linalg.norm(tl - bl)
    maxHeight = int(max(heightA, heightB))
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype="float32")
    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
    return warped, rect, M

def draw_dashed_line(img, p1, p2, color, thickness=1, dash_length=10):
    p1 = tuple(map(int, p1))
    p2 = tuple(map(int, p2))
    dist = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
    if dist == 0:
        return
    dx = (p2[0]-p1[0]) / dist
    dy = (p2[1]-p1[1]) / dist
    num_dashes = int(dist / dash_length)
    for i in range(num_dashes):
        start = (int(p1[0] + dx * i * dash_length), int(p1[1] + dy * i * dash_length))
        end = (int(p1[0] + dx * (i+0.5) * dash_length), int(p1[1] + dy * (i+0.5) * dash_length))
        cv2.line(img, start, end, color, thickness)

# ---------- open camera ----------
cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)  # CAP_DSHOW for Windows (remove on Linux/mac)
time.sleep(0.2)
if not cap.isOpened():
    print(f"Could not open camera index {CAM_INDEX}. Try another index (0/1/2).")
    exit()

save_count = 0
print("Press 'c' to save warped sheet image. 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to read frame from camera.")
        break

    # Resize for speed but keep original for full-res warped crop
    H, W = frame.shape[:2]
    scale = 1000.0 / max(H, W) if max(H, W) > 1000 else 1.0
    if scale != 1.0:
        frame_small = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    else:
        frame_small = frame.copy()

    # detect white sheet by HSV thresholding + morphology
    hsv = cv2.cvtColor(frame_small, cv2.COLOR_BGR2HSV)
    white_mask = cv2.inRange(hsv, np.array([0, 0, 150]), np.array([180, 60, 255]))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7,7))
    mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    edges = cv2.Canny(mask, 50, 150)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    sheet_cnt = None
    max_area = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 5000:
            continue
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        if len(approx) == 4 and area > max_area:
            sheet_cnt = approx
            max_area = area

    display = frame_small.copy()
    warped = None

    if sheet_cnt is not None:
        # draw contour on preview
        cv2.drawContours(display, [sheet_cnt], -1, (0,255,0), 2)

        # small warp for preview
        pts_small = sheet_cnt.reshape(4,2).astype("float32")
        warped_small, rect_small, M_small = four_point_transform(frame_small, pts_small)

        # compute transform and warped full-resolution version for accurate measurements
        if scale != 1.0:
            rect_orig = rect_small / scale
        else:
            rect_orig = rect_small.copy()
        warped_full, rect_full, M_full = four_point_transform(frame, rect_orig)
        warped = warped_full
        warped_vis = warped.copy()

        sheet_h, sheet_w = warped.shape[:2]

        # mm per pixel
        mm_per_px_x = A4_WIDTH_MM / sheet_w
        mm_per_px_y = A4_HEIGHT_MM / sheet_h

        # px per cm for tick drawing
        px_per_cm_x = sheet_w / (A4_WIDTH_MM / 10.0)   # sheet_w / 21.0
        px_per_cm_y = sheet_h / (A4_HEIGHT_MM / 10.0)  # sheet_h / 29.7

        # center (origin) at sheet center
        cx0 = sheet_w / 2.0
        cy0 = sheet_h / 2.0

        # draw center axes (X to right, Y up visually)
        axis_color = (50, 200, 255)  # light blue/orangeish
        cv2.line(warped_vis, (0, int(cy0)), (sheet_w-1, int(cy0)), axis_color, 2)  # horizontal (X)
        cv2.line(warped_vis, (int(cx0), 0), (int(cx0), sheet_h-1), axis_color, 2)  # vertical (Y)

        # draw ticks and labels every TICK_CM
        tick_px_x = int(round(px_per_cm_x * TICK_CM))
        tick_px_y = int(round(px_per_cm_y * TICK_CM))

        # horizontal axis ticks and labels (along X at center Y)
        # positive X to right, label in cm relative to center (0)
        half_ticks = int(sheet_w / tick_px_x) + 2
        for t in range(-half_ticks, half_ticks+1):
            x = int(cx0 + t * tick_px_x)
            if 0 <= x < sheet_w:
                # tick mark
                cv2.line(warped_vis, (x, int(cy0-8)), (x, int(cy0+8)), axis_color, 1)
                # label every tick (avoid overlapping many labels) - show every tick with small font
                cm_val = t * TICK_CM
                text = f"{cm_val}cm"
                # place labels below axis for ticks near top/bottom safety
                txt_size, _ = cv2.getTextSize(text, FONT, 0.4, 1)
                txt_x = x - txt_size[0]//2
                txt_y = int(cy0 + 22)
                # background rectangle
                cv2.rectangle(warped_vis, (txt_x-2, txt_y-txt_size[1]-2), (txt_x + txt_size[0]+2, txt_y+2), (255,255,255), -1)
                cv2.putText(warped_vis, text, (txt_x, txt_y), FONT, 0.4, (0,0,0), 1)

        # vertical axis ticks and labels (along Y at center X)
        half_ticks_y = int(sheet_h / tick_px_y) + 2
        for t in range(-half_ticks_y, half_ticks_y+1):
            y = int(cy0 + t * tick_px_y)
            if 0 <= y < sheet_h:
                cv2.line(warped_vis, (int(cx0-8), y), (int(cx0+8), y), axis_color, 1)
                cm_val = -t * TICK_CM  # negative sign because Y grows downward in image coords
                text = f"{cm_val}cm"
                txt_size, _ = cv2.getTextSize(text, FONT, 0.4, 1)
                txt_x = int(cx0 + 12)
                txt_y = y + txt_size[1]//2
                cv2.rectangle(warped_vis, (txt_x-2, txt_y-txt_size[1]-2), (txt_x + txt_size[0]+2, txt_y+2), (255,255,255), -1)
                cv2.putText(warped_vis, text, (txt_x, txt_y), FONT, 0.4, (0,0,0), 1)

        # detect darker objects on the sheet
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        _, obj_mask = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY_INV)
        k2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        obj_mask = cv2.morphologyEx(obj_mask, cv2.MORPH_OPEN, k2, iterations=1)
        obj_mask = cv2.morphologyEx(obj_mask, cv2.MORPH_CLOSE, k2, iterations=1)

        obj_cnts, _ = cv2.findContours(obj_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        results = []
        for i, oc in enumerate(obj_cnts):
            area = cv2.contourArea(oc)
            if area < MIN_CONTOUR_AREA:
                continue
            x,y,wc,hc = cv2.boundingRect(oc)
            cx = x + wc/2.0
            cy = y + hc/2.0

            # distances relative to sheet center, in mm/cm
            dx_mm = (cx - cx0) * mm_per_px_x
            dy_mm = (cy0 - cy) * mm_per_px_y   # positive = upward from center
            dx_cm = dx_mm / 10.0
            dy_cm = dy_mm / 10.0
            r_cm = math.hypot(dx_cm, dy_cm)

            results.append({
                "index": i,
                "bbox_px": (int(x), int(y), int(wc), int(hc)),
                "centroid_px": (float(cx), float(cy)),
                "dx_cm": dx_cm,
                "dy_cm": dy_cm,
                "r_cm": r_cm,
                "area": area
            })

            # draw bbox and centroids and crosshair
            cv2.rectangle(warped_vis, (x,y), (x+wc, y+hc), (0,0,255), 2)
            cv2.drawMarker(warped_vis, (int(cx), int(cy)), (255,0,0), markerType=cv2.MARKER_CROSS, markerSize=12, thickness=2)

            # dashed helper line from centroid to center
            draw_dashed_line(warped_vis, (cx, cy), (cx0, cy), (200,200,200), thickness=1, dash_length=8)
            draw_dashed_line(warped_vis, (cx, cy), (cx, cy0), (200,200,200), thickness=1, dash_length=8)

            # legend text (centered on object)
            # prepare text lines
            line1 = f"dx={dx_cm:+.2f}cm"
            line2 = f"dy={dy_cm:+.2f}cm"
            line3 = f"r={r_cm:.2f}cm"
            lines = [line1, line2, line3]
            # measure block size
            pad = 6
            line_heights = []
            max_w = 0
            for ln in lines:
                (tw, th), _ = cv2.getTextSize(ln, FONT, 0.5, 1)
                line_heights.append(th)
                if tw > max_w:
                    max_w = tw
            block_w = max_w + pad*2
            block_h = sum(line_heights) + pad*(len(lines)+1)

            # top-left of block such that block is centered on centroid
            bx = int(cx - block_w/2)
            by = int(cy - block_h/2)

            # keep block inside image bounds
            bx = max(2, min(bx, sheet_w - block_w - 2))
            by = max(2, min(by, sheet_h - block_h - 2))

            # draw semi-transparent rectangle (simulate by solid white with alpha-like look)
            cv2.rectangle(warped_vis, (bx, by), (bx + block_w, by + block_h), (255,255,255), -1)
            cv2.rectangle(warped_vis, (bx, by), (bx + block_w, by + block_h), (0,0,0), 1)

            # put lines
            ty = by + pad + line_heights[0]
            for idx_ln, ln in enumerate(lines):
                cv2.putText(warped_vis, ln, (bx + pad, ty), FONT, 0.5, (0,0,0), 1)
                ty += line_heights[idx_ln] + pad

        # print results to console
        if results:
            print("Detected objects (relative to sheet center):")
            for r in results:
                print(f"  Obj {r['index']}: bbox(px)={r['bbox_px']}, center(px)={tuple(round(v,2) for v in r['centroid_px'])}, "
                      f"dx={r['dx_cm']:+.2f}cm, dy={r['dy_cm']:+.2f}cm, r={r['r_cm']:.2f}cm, area={int(r['area'])}")
        else:
            print("No objects detected on sheet.")

        # show overlays
        cv2.imshow("Warped Sheet - axes & legends", warped_vis)
        cv2.imshow("Object mask", obj_mask)

    else:
        cv2.putText(display, "A4 sheet not detected", (20,30), FONT, 0.9, (0,0,255), 2)

    cv2.imshow("Frame", display)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    if key == ord('c') and warped is not None:
        fname = os.path.join(SAVE_DIR, f"warped_axes_{int(time.time())}_{save_count}.png")
        cv2.imwrite(fname, warped_vis)
        print("Saved", fname)
        save_count += 1

cap.release()
cv2.destroyAllWindows()

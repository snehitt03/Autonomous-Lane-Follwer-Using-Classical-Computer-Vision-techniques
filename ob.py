import cv2
import numpy as np
from collections import deque
import requests
import time
import logging
import math

# ---------------- LOGGING SETUP ----------------
logging.basicConfig(
    filename="esp32_log.txt",
    level=logging.INFO,
    format="%(asctime)s | %(message)s",
)

# ------------------ CONFIG ------------------
ESP32_IP = "http://10.201.85.238"   # 🔴 Change to your ESP32 IP
CAM_URL  = "http://10.201.85.111:8080/video"  # 🔴 IP Webcam Stream URL
# ------------------------------------------------


def send_command(cmd, frame_proc_ms=None, frame_gap_ms=None):
    """Send command to ESP32 with logging and RTT measurement.
    Optionally include frame processing and frame gap metrics for logging.
    """
    t0 = time.time()
    try:
        r = requests.get(f"{ESP32_IP}/cmd", params={"dir": cmd}, timeout=0.25)
        t1 = time.time()

        elapsed_ms = (t1 - t0) * 1000   # round-trip time
        response = r.text.strip()

        # Try to parse ESP32 millis if response format OK:<CMD>:<millis>
        esp32_millis = None
        try:
            if response.startswith("OK:"):
                parts = response.split(":")
                # Expecting "OK", "<CMD>", "<millis>"
                if len(parts) >= 3 and parts[-1].isdigit():
                    esp32_millis = int(parts[-1])
        except Exception:
            esp32_millis = None

        # Prepare frame metrics text
        if frame_proc_ms is None:
            fp_text = "N/A"
        else:
            fp_text = f"{frame_proc_ms:.1f}"

        if frame_gap_ms is None or (isinstance(frame_gap_ms, float) and math.isnan(frame_gap_ms)):
            fg_text = "N/A"
        else:
            fg_text = f"{frame_gap_ms:.1f}"

        # Console print
        print(f"➡ Sent {cmd} → ESP32: {response} | RTT={elapsed_ms:.1f} ms | "
              f"FRAME_PROC={fp_text} ms | FRAME_GAP={fg_text} ms")

        # Build log message in required format (Option 1)
        log_msg = f"CMD={cmd} | RESPONSE={response} | RTT={elapsed_ms:.1f} ms | " \
                  f"FRAME_PROC={fp_text} ms | FRAME_GAP={fg_text} ms"

        logging.info(log_msg)

        # Return useful info in case caller wants to use it
        return {
            "response": response,
            "rtt_ms": elapsed_ms,
            "esp32_millis": esp32_millis,
            "frame_proc_ms": frame_proc_ms,
            "frame_gap_ms": frame_gap_ms
        }

    except Exception as e:
        print("❌ ESP32 Command Error:", e)
        # If request failed, still log but mark RTT as ERROR
        fp_text = f"{frame_proc_ms:.1f}" if frame_proc_ms is not None else "N/A"
        fg_text = f"{frame_gap_ms:.1f}" if (frame_gap_ms is not None and not math.isnan(frame_gap_ms)) else "N/A"
        logging.error(f"CMD={cmd} | ERROR={e} | FRAME_PROC={fp_text} ms | FRAME_GAP={fg_text} ms")
        return None



def nothing(x):
    pass


# ---------- Perspective Warp Utilities ----------
def getWarpPoints():
    t1 = cv2.getTrackbarPos("Top X1", "Warp Trackbars")
    t2 = cv2.getTrackbarPos("Top X2", "Warp Trackbars")
    b1 = cv2.getTrackbarPos("Bottom X1", "Warp Trackbars")
    b2 = cv2.getTrackbarPos("Bottom X2", "Warp Trackbars")
    ty = cv2.getTrackbarPos("Top Y", "Warp Trackbars")
    by = cv2.getTrackbarPos("Bottom Y", "Warp Trackbars")
    return np.float32([[t1, ty], [t2, ty], [b1, by], [b2, by]])


def warpImg(img, src, dstSize):
    dst = np.float32([[0, 0], [dstSize[0], 0], [0, dstSize[1]], [dstSize[0], dstSize[1]]])
    matrix = cv2.getPerspectiveTransform(src, dst)
    imgWarp = cv2.warpPerspective(img, matrix, dstSize)
    return imgWarp, matrix


def unwarpImg(img, matrix, dstSize):
    inv = np.linalg.inv(matrix)
    return cv2.warpPerspective(img, inv, dstSize)


# ---------- Lane Polynomial Fit ----------
def fit_polynomial_lane(binary):
    nz = binary.nonzero()
    y = np.array(nz[0])
    x = np.array(nz[1])

    if len(x) == 0:
        return None, None

    mid = binary.shape[1] // 2
    lx, ly = x[x < mid], y[x < mid]
    rx, ry = x[x >= mid], y[x >= mid]

    lf = np.polyfit(ly, lx, 2) if len(lx) > 50 else None
    rf = np.polyfit(ry, rx, 2) if len(rx) > 50 else None

    return lf, rf


# ---------- Steering Angle Calculation ----------
def calculate_steering_angle(lf, rf, img_height, img_width, lookahead=0.6):
    y_eval = img_height * (1 - lookahead)
    left_x = lf[0] * y_eval**2 + lf[1] * y_eval + lf[2]
    right_x = rf[0] * y_eval**2 + rf[1] * y_eval + rf[2]

    lane_center = (left_x + right_x) / 2
    vehicle_center = img_width / 2
    offset = lane_center - vehicle_center

    curvature = ((2 * lf[0]) + (2 * rf[0])) / 2
    base_angle = np.arctan2(offset, img_height * lookahead) * 180 / np.pi
    steering_angle = base_angle + (curvature * 50)

    return steering_angle, offset, curvature


def get_steering_command(steering_angle):
    if abs(steering_angle) < 2:
        return "STRAIGHT"
    elif steering_angle > 0:
        return "RIGHT"
    else:
        return "LEFT"


def read_latest_frame(cap, discard=5):
    for _ in range(discard):
        cap.grab()
    return cap.read()



# === Load Object Detection Model ===
net = cv2.dnn.readNetFromCaffe("deploy.prototxt", "mobilenet_iter_73000.caffemodel")
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant",
           "sheep", "sofa", "train", "tvmonitor"]


print("✅ Lane + Object Detection Started")

cap = cv2.VideoCapture(CAM_URL, cv2.CAP_FFMPEG)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
if not cap.isOpened():
    print("❌ Could not open video stream.")
    exit()


# ---------- Trackbar Setup ----------
cv2.namedWindow("HSV Trackbars")
cv2.createTrackbar("HMin", "HSV Trackbars", 0, 179, nothing)
cv2.createTrackbar("HMax", "HSV Trackbars", 179, 179, nothing)
cv2.createTrackbar("SMin", "HSV Trackbars", 0, 255, nothing)
cv2.createTrackbar("SMax", "HSV Trackbars", 255, 255, nothing)
cv2.createTrackbar("VMin", "HSV Trackbars", 160, 255, nothing)
cv2.createTrackbar("VMax", "HSV Trackbars", 255, 255, nothing)

cv2.namedWindow("Warp Trackbars")
cv2.createTrackbar("Top X1", "Warp Trackbars", 140, 480, nothing)
cv2.createTrackbar("Top X2", "Warp Trackbars", 340, 480, nothing)
cv2.createTrackbar("Bottom X1", "Warp Trackbars", 60, 480, nothing)
cv2.createTrackbar("Bottom X2", "Warp Trackbars", 420, 480, nothing)
cv2.createTrackbar("Top Y", "Warp Trackbars", 130, 240, nothing)
cv2.createTrackbar("Bottom Y", "Warp Trackbars", 230, 240, nothing)

cv2.namedWindow("Steering Tuning")
cv2.createTrackbar("Lookahead", "Steering Tuning", 60, 100, nothing)


# ---------- MAIN LOOP ----------
N = 5
leftQ, rightQ = deque(maxlen=N), deque(maxlen=N)
last_dir = "STOP"
last_sent = time.time()
frame_counter = 0
object_present = False

# frame timing helpers
last_frame_time = None  # for FRAME_GAP calculation (seconds, perf_counter)
print("🚗 Press 'q' to quit")


while True:

    # ----- read frame and mark arrival time -----
    ret, img = read_latest_frame(cap)
    if not ret:
        continue

    # Use high-resolution monotonic clock for timing
    t_frame_recv = time.perf_counter()

    # compute frame gap (ms)
    if last_frame_time is None:
        frame_gap_ms = float('nan')
    else:
        frame_gap_ms = (t_frame_recv - last_frame_time) * 1000.0

    # update last_frame_time for next iteration
    last_frame_time = t_frame_recv

    img = cv2.resize(img, (480, 240))
    frame_counter += 1

    # Start processing timer for this frame
    t_proc_start = time.perf_counter()

    # ===== OBJECT DETECTION =====
    if frame_counter % 5 == 0 or object_present:

        h, w = img.shape[:2]
        x1, y1 = int(w * 0.35), int(h * 0.7)
        x2, y2 = int(w * 0.65), int(h * 0.95)

        roi = img[y1:y2, x1:x2].copy()

        blob = cv2.dnn.blobFromImage(
            cv2.resize(roi, (300, 300)),
            0.007843, (300, 300), 127.5
        )
        net.setInput(blob)
        detections = net.forward()

        object_present = False

        for i in range(detections.shape[2]):
            conf = detections[0, 0, i, 2]
            if conf > 0.5:
                object_present = True

        # End processing timer (we made a decision for this frame)
        t_proc_end = time.perf_counter()
        frame_proc_ms = (t_proc_end - t_proc_start) * 1000.0

        if object_present:
            # include frame timing in the command log
            send_command("STOP", frame_proc_ms=frame_proc_ms, frame_gap_ms=frame_gap_ms)
            # Skip lane detection when obstacle present
            continue


    # ===== LANE DETECTION =====
    blur = cv2.GaussianBlur(img, (5, 5), 0)
    canny = cv2.Canny(blur, 50, 150)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    hMin = cv2.getTrackbarPos("HMin", "HSV Trackbars")
    hMax = cv2.getTrackbarPos("HMax", "HSV Trackbars")
    sMin = cv2.getTrackbarPos("SMin", "HSV Trackbars")
    sMax = cv2.getTrackbarPos("SMax", "HSV Trackbars")
    vMin = cv2.getTrackbarPos("VMin", "HSV Trackbars")
    vMax = cv2.getTrackbarPos("VMax", "HSV Trackbars")

    mask = cv2.inRange(hsv, np.array([hMin, sMin, vMin]), np.array([hMax, sMax, vMax]))
    comb = cv2.bitwise_and(mask, canny)

    src = getWarpPoints()
    warp, mat = warpImg(comb, src, (480, 240))

    lf, rf = fit_polynomial_lane(warp)

    if lf is not None:
        leftQ.append(lf)
    if rf is not None:
        rightQ.append(rf)

    lf = np.mean(leftQ, axis=0) if len(leftQ) else None
    rf = np.mean(rightQ, axis=0) if len(rightQ) else None

    overlay = img.copy()
    direction = "STOP"

    # ===== Compute steering =====
    if lf is not None and rf is not None:

        lookahead = cv2.getTrackbarPos("Lookahead", "Steering Tuning") / 100.0
        lookahead = max(0.3, min(0.9, lookahead))

        steering_angle, offset, curvature = calculate_steering_angle(
            lf, rf, warp.shape[0], warp.shape[1], lookahead
        )

        direction = get_steering_command(steering_angle)

    # End processing timer (we made a decision for this frame)
    t_proc_end = time.perf_counter()
    frame_proc_ms = (t_proc_end - t_proc_start) * 1000.0

    # Visualization (same as before)
    if lf is not None and rf is not None:
        y = np.linspace(0, warp.shape[0]-1, warp.shape[0])
        lfx = lf[0]*y**2 + lf[1]*y + lf[2]
        rfx = rf[0]*y**2 + rf[1]*y + rf[2]
        pts = np.hstack((np.transpose(np.vstack([lfx,y])),
                         np.flipud(np.transpose(np.vstack([rfx,y])))))
        pts = np.array(pts, np.int32).reshape((-1,1,2))

        lane = np.zeros_like(cv2.cvtColor(warp, cv2.COLOR_GRAY2BGR))
        cv2.fillPoly(lane, [pts], (0,255,0))
        lane_unwarp = unwarpImg(lane, mat, (480,240))
        overlay = cv2.addWeighted(img, 1, lane_unwarp, 0.5, 0)

        cv2.putText(overlay, f"Dir:{direction} | Angle:{steering_angle:.1f}", 
                    (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
    else:
        cv2.putText(overlay, "NO LANES DETECTED", (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

    # Show windows
    cv2.imshow("Lane + Object Detection", overlay)
    cv2.imshow("Warped", warp)
    cv2.imshow("Canny+HSV", comb)

    # ===== Send command only if changed (include frame metrics in log) =====
    if direction != last_dir and time.time() - last_sent > 0.3:
        send_command(direction, frame_proc_ms=frame_proc_ms, frame_gap_ms=frame_gap_ms)
        last_dir = direction
        last_sent = time.time()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()
print("✅ Script Ended")

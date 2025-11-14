import cv2
import numpy as np
import serial
import time

# ============================
# Arduino Serial Setup
# ============================
try:
    arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=0.1)  # Change COM port if needed
    time.sleep(2)  # Allow Arduino to reset
    print("Arduino connected.")
except:
    arduino = None
    print(" Arduino not connected. Running in camera-only mode.")


def send_arduino(cmd: str):
    """Send a single-character command to the Arduino."""
    if arduino is not None:
        arduino.write(cmd.encode())
        time.sleep(0.01)


# ============================
# Auto Brightening
# ============================
def auto_brighten(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    v_mean = np.mean(hsv[:, :, 2])

    if v_mean < 90:
        frame = cv2.convertScaleAbs(frame, alpha=1.8, beta=60)
    elif v_mean < 130:
        frame = cv2.convertScaleAbs(frame, alpha=1.5, beta=40)

    return frame



# Camera Initialization
cap = cv2.VideoCapture(1)

# Grab one frame to init
_, frame = cap.read()

# Logitech C270 manual exposure
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  
cap.set(cv2.CAP_PROP_EXPOSURE, -6)
cap.set(cv2.CAP_PROP_GAIN, 0)

# Grab again to apply settings
_, frame = cap.read()

print("Camera initialized.")


# Main Loop
while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, (640, 480))
    frame = auto_brighten(frame)

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_green = np.array([35, 60, 60])
    upper_green = np.array([85, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)

    # Morphological noise clean
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.GaussianBlur(mask, (7, 7), 2)

    # Cube Detection (Contour-based)
    output = frame.copy()
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cube_found = False
    cube_center = None

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 500:  # ignore tiny noise
            continue

        # Approximate the contour to identify shape
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)

        # Square/cube-like shape has 4 corners
        if len(approx) == 4:
            cube_found = True

            # Get center
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cube_center = (cx, cy)

            # Draw bounding box
            cv2.drawContours(output, [approx], -1, (0, 255, 0), 3)
            cv2.circle(output, cube_center, 6, (0, 0, 255), -1)
            cv2.putText(output, "Green Cube", (cube_center[0] - 40, cube_center[1] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    # Arduino Control Logic
    if cube_found and cube_center is not None:
        frame_center = output.shape[1] // 2

        cx = cube_center[0]
        offset = cx - frame_center

        threshold = 40  # dead-zone for forward movement

        if offset < -threshold:
            print("➡️ Turn Left")
            send_arduino('L')
        elif offset > threshold:
            print("⬅️ Turn Right")
            send_arduino('R')
        else:
            print("⬆️ Forward")
            send_arduino('F')
    else:
        print("No cube detected — stopping")
        send_arduino('S')

    # Display Windows
    cv2.imshow("Mask", mask)
    cv2.imshow("Cube Detection", output)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        send_arduino('S')
        break

cap.release()
cv2.destroyAllWindows()

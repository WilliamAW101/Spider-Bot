import cv2 
import numpy as np
import serial
import time

# --- Initialize Serial (CHANGE PORT IF NEEDED) ---
try:
    arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    time.sleep(2)  # let Arduino reset
except:
    arduino = None
# --- Initialize webcam with DEFAULT settings ---
cap = cv2.VideoCapture(0)

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CENTER_X = FRAME_WIDTH // 2
TOLERANCE = 50   # dead zone where bot doesn't move

def send(cmd):
    """Send a single character to Arduino."""
    if arduino is not None:
        arduino.write(cmd.encode())
        time.sleep(0.01)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_green = np.array([35, 60, 60])
    upper_green = np.array([85, 255, 255])

    mask = cv2.inRange(hsv, lower_green, upper_green)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    output = frame.copy()

    object_found = False
    cx = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 500:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(output, (x, y), (x+w, y+h), (0,255,0), 2)

        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(output, (cx, cy), 4, (0,0,255), -1)

        object_found = True
        break  # only track first detected object

    # --- SERIAL OUTPUT ONLY ---
    if object_found:
        if cx < CENTER_X - TOLERANCE:
            send('L')   # object left
        elif cx > CENTER_X + TOLERANCE:
            send('R')   # object right
        else:
            send('C')   # centered
    else:
        send('N')       # no object

    cv2.imshow("Green Mask", mask)
    cv2.imshow("Green Object Detection", output)

    if cv2.waitKey(1) & 0xFF == 'q':
        break

send('N')
cap.release()
cv2.destroyAllWindows()
arduino.close()

import cv2
import numpy as np

def auto_brighten(frame):
    """
    Brighten frame dynamically if it's too dark
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    v_mean = np.mean(hsv[:, :, 2])
    if v_mean < 90:
        frame = cv2.convertScaleAbs(frame, alpha=1.8, beta=60)
    elif v_mean < 130:
        frame = cv2.convertScaleAbs(frame, alpha=1.5, beta=40)
    return frame

# --- Initialize camera ---
cap = cv2.VideoCapture(0)

# Step 1: Grab the first frame so the camera initializes
ret, frame = cap.read()

# Step 2: Set manual exposure & auto-exposure off
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 0.25 = manual mode in OpenCV
cap.set(cv2.CAP_PROP_EXPOSURE, -6)         # Adjust depending on your camera/light
cap.set(cv2.CAP_PROP_GAIN, 0)              # Optional: reduce noise

# Optional: capture one more frame to apply settings
ret, frame = cap.read()

# --- Main loop ---
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Optional: downscale to speed up processing
    frame = cv2.resize(frame, (640, 480))

    # --- Auto-brighten if necessary ---
    frame = auto_brighten(frame)

    # --- Convert to HSV and create green mask ---
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_green = np.array([35, 60, 60])
    upper_green = np.array([85, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)

    # --- Morphological filtering to remove noise ---
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.GaussianBlur(mask, (9,9), 2)

    # --- Detect circles ---
    circles = cv2.HoughCircles(
        mask,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=100,
        param1=100,
        param2=45,       # Higher = fewer false positives
        minRadius=10,
        maxRadius=200
    )

    output = frame.copy()
    if circles is not None:
        circles = np.uint16(np.around(circles[0, :]))
        # Pick the largest circle (most likely the real sphere)
        largest_circle = max(circles, key=lambda c: c[2])
        x, y, r = largest_circle

        # Draw circle and center
        cv2.circle(output, (x, y), r, (0, 255, 0), 3)
        cv2.circle(output, (x, y), 2, (0, 0, 255), 3)
        cv2.putText(output, "Green Sphere", (x-40, y-r-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(output, f"Radius: {r}", (x-40, y-r+15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    # --- Display ---
    cv2.imshow("Green Mask", mask)
    cv2.imshow("Green Sphere Detection", output)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

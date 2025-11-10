import cv2
import numpy as np

# --- Initialize webcam ---
cap = cv2.VideoCapture(1)

# Optional: lock exposure & gain for more stable brightness
# (These values may need tuning for your lighting conditions)
# cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Manual mode
# cap.set(cv2.CAP_PROP_EXPOSURE, -5)         # Try between -4 and -7
# cap.set(cv2.CAP_PROP_GAIN, 0)              # Reduce sensor noise
# cap.set(cv2.CAP_PROP_BRIGHTNESS, 128)      # Optional baseline
# cap.set(cv2.CAP_PROP_CONTRAST, 32)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Resize (optional for speed)
    frame = cv2.resize(frame, (640, 480))

    # Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # --- Define green color range (adjust as needed) ---
    lower_green = np.array([35, 60, 60])
    upper_green = np.array([85, 255, 255])

    # Create green mask
    mask = cv2.inRange(hsv, lower_green, upper_green)

    # --- Morphological filtering to remove noise ---
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.GaussianBlur(mask, (9, 9), 2)

    # --- Detect circles ---
    circles = cv2.HoughCircles(
        mask,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=100,
        param1=100,
        param2=45,   # Increase to reduce false positives (40–60 recommended)
        minRadius=10,
        maxRadius=200
    )

    output = frame.copy()

    if circles is not None:
        circles = np.uint16(np.around(circles[0, :]))
        # Pick the largest circle (most likely the actual sphere)
        largest_circle = max(circles, key=lambda c: c[2])
        x, y, r = largest_circle

        # Draw detection
        cv2.circle(output, (x, y), r, (0, 255, 0), 3)
        cv2.circle(output, (x, y), 2, (0, 0, 255), 3)
        cv2.putText(output, f"Green Sphere", (x - 40, y - r - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(output, f"Radius: {r}", (x - 40, y - r + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    # --- Display both mask and detection ---
    cv2.imshow("Green Mask", mask)
    cv2.imshow("Green Sphere Detection", output)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

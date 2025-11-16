import cv2
import os
import signal
import numpy as np
import serial
import sys
import time
from flask import Flask, render_template, Response, jsonify
from threading import Thread

app = Flask(__name__)

# --- Initialize Serial ---
try:
    arduino = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
    time.sleep(2)
except:
    arduino = None

# --- Initialize webcam ---
cap = cv2.VideoCapture(0)

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CENTER_X = FRAME_WIDTH // 2
TOLERANCE =  70

# Global state for web display
frame_rgb = None
mask_frame = None
last_command = None
object_detected = False
center_x = 0
frame_lock = __import__('threading').Lock()


def send(cmd):
    """Send a single character to Arduino."""
    global last_command
    if arduino is not None:
        print(cmd)
        arduino.write(cmd.encode())
    last_command = cmd
    time.sleep(0.01)

def handle_sigint(signum, frame):
    print(f"\nReceived signal {signum} ({signal.Signals(signum).name}). Cleaning up and exiting...")
    # Perform any necessary cleanup here
    cmd =  'N'
    send(cmd)
    sys.exit(0)

def process_frames():
    """Continuously process video frames."""
    global frame_rgb, mask_frame, object_detected, center_x
    
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
            cv2.rectangle(output, (x, y), (x+w, y+h), (0, 255, 0), 2)

            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(output, (cx, cy), 4, (0, 0, 255), -1)

            object_found = True
            break

        # Draw center line and tolerance zones
        cv2.line(output, (CENTER_X, 0), (CENTER_X, FRAME_HEIGHT), (255, 0, 0), 1)
        cv2.line(output, (CENTER_X - TOLERANCE, 0), (CENTER_X - TOLERANCE, FRAME_HEIGHT), (100, 100, 255), 1)
        cv2.line(output, (CENTER_X + TOLERANCE, 0), (CENTER_X + TOLERANCE, FRAME_HEIGHT), (100, 100, 255), 1)

        # Send commands
        if object_found:
            center_x = cx
            if cx < CENTER_X - TOLERANCE:
                send('L')
            elif cx > CENTER_X + TOLERANCE:
                send('R')
            else:
                send('C')
        else:
            send('N')

        object_detected = object_found

        # Convert to RGB for web display
        with frame_lock:
            frame_rgb = cv2.cvtColor(output, cv2.COLOR_BGR2RGB)
            mask_frame = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)

        time.sleep(0.033)  # ~30 FPS

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """Streaming route for main video."""
    def generate():
        while True:
            with frame_lock:
                frame = frame_rgb
            if frame is not None:
                ret, buffer = cv2.imencode('.jpg', cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n'
                           b'Content-Length: ' + str(len(buffer)).encode() + b'\r\n\r\n' + 
                           buffer.tobytes() + b'\r\n')
            time.sleep(0.033)
    
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/mask_feed')
def mask_feed():
    """Streaming route for mask."""
    def generate():
        while True:
            with frame_lock:
                frame = mask_frame
            if frame is not None:
                ret, buffer = cv2.imencode('.jpg', cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n'
                           b'Content-Length: ' + str(len(buffer)).encode() + b'\r\n\r\n' + 
                           buffer.tobytes() + b'\r\n')
            time.sleep(0.033)
    
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/status')
def status():
    """Return tracking status as JSON."""
    return jsonify({
        'object_detected': object_detected,
        'center_x': center_x,
        'frame_width': FRAME_WIDTH,
        'last_command': last_command,
        'commands': {
            'L': 'Object Left',
            'R': 'Object Right',
            'C': 'Centered',
            'N': 'No Object'
        }
    })

if __name__ == '__main__':
    signal.signal(signal.SIGINT, handle_sigint)

    # Start video processing thread
    video_thread = Thread(target=process_frames, daemon=True)
    video_thread.start()
    
    # Start Flask
    app.run(debug=False, host='0.0.0.0', port=5000)

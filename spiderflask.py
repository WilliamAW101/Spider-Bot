import cv2
import os
import signal
import numpy as np
import serial
import sys
import time
from flask import Flask, render_template, Response, jsonify
from threading import Thread
import math

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
TOLERANCE = 150

# Camera calibration (adjust these based on your camera)
# Focal length and reference object width in pixels (calibrate for your camera)
FOCAL_LENGTH = 500  # Adjust based on your camera calibration
REFERENCE_WIDTH = 50  # Expected width of object in real world (cm)

# Global state for web display
frame_rgb = None
mask_frame = None
last_command = None
previous_command = None
object_detected = False
center_x = 0
object_distance = 0  # Distance in cm
frame_lock = __import__('threading').Lock()
manual_mode = False
manual_command = None

# Command delay tracking
pending_command = None
pending_command_time = None
COMMAND_DELAYS = {
    'L': 1.0,      # Left: 1 second delay
    'R': 1.0,      # Right: 1 second delay
    'F': 0.5,      # Forward: 0.5 second delay
    'B': 0.0,      # Backward: no delay
    'N': 0.0       # No object: no delay
}


def calculate_distance(object_width_pixels):
    """
    Calculate distance from object using focal length method.
    Distance = (REFERENCE_WIDTH * FOCAL_LENGTH) / object_width_pixels
    Adjust FOCAL_LENGTH and REFERENCE_WIDTH for your setup.
    """
    if object_width_pixels > 0:
        distance = (REFERENCE_WIDTH * FOCAL_LENGTH) / object_width_pixels
        return distance
    return 0


def send(cmd):
    """Send a single character to Arduino only if command changed."""
    global last_command, previous_command
    
    # Only send if command is different from last command
    if cmd != previous_command:
        if arduino is not None:
            print(f"Sending: {cmd}")
            arduino.write(cmd.encode())
        last_command = cmd
        previous_command = cmd
        time.sleep(0.01)


def send_with_delay(cmd):
    """Send command with appropriate delay based on command type."""
    global pending_command, pending_command_time
    
    delay = COMMAND_DELAYS.get(cmd, 0.0)
    
    if delay > 0:
        # Set pending command
        pending_command = cmd
        pending_command_time = time.time()
    else:
        # Send immediately
        send(cmd)


def check_pending_commands():
    """Check if pending command should be sent based on elapsed time."""
    global pending_command, pending_command_time
    
    if pending_command is not None and pending_command_time is not None:
        elapsed = time.time() - pending_command_time
        delay = COMMAND_DELAYS.get(pending_command, 0.0)
        
        if elapsed >= delay:
            send(pending_command)
            pending_command = None
            pending_command_time = None


def process_frames():
    """Continuously process video frames."""
    global frame_rgb, mask_frame, object_detected, center_x, manual_mode, manual_command
    
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

        # Send commands based on mode
        if manual_mode:
            # In manual mode, use manual_command if set
            if manual_command:
                last_command = manual_command
        else:
            # In auto mode, track object
            if object_found:
                center_x = cx
                if cx < CENTER_X - TOLERANCE:
                    cmd = 'L'
                elif cx > CENTER_X + TOLERANCE:
                    cmd = 'R'
                else:
                    cmd = 'F'
            else:
                cmd = 'N'
            
            # Only send if command changed
            if cmd != previous_command:
                send_with_delay(cmd)
        
        # Check if any pending commands should be sent
        check_pending_commands()

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
        'manual_mode': manual_mode,
        'commands': {
            'L': 'Left',
            'R': 'Right',
            'F': 'Forward',
            'B': 'Backward',
            'N': 'No Object'
        }
    })

@app.route('/toggle_mode')
def toggle_mode():
    """Toggle between manual and automatic mode."""
    global manual_mode, previous_command
    manual_mode = not manual_mode
    # Reset previous command when switching modes to allow fresh commands
    if manual_mode:
        previous_command = None
    return jsonify({'manual_mode': manual_mode})

@app.route('/manual_control/<cmd>')
def manual_control(cmd):
    """Send manual command."""
    global manual_command, previous_command, last_command
    if cmd in ['L', 'R', 'F', 'B', 'N']:
        # Only send if it's a change from the previous command
        if cmd != previous_command:
            manual_command = cmd
            if manual_mode:
                if arduino is not None:
                    print(f"Manual: {cmd}")
                    send_with_delay(cmd)
                previous_command = cmd
            last_command = cmd
            return jsonify({'status': 'ok', 'command': cmd, 'sent': True})
        else:
            return jsonify({'status': 'ok', 'command': cmd, 'sent': False, 'message': 'Same command, not sent'})
    return jsonify({'status': 'error', 'message': 'Invalid command'}), 400

@app.route('/grabber/<action>')
def grabber_control(action):
    """Control grabber - open or close."""
    if action in ['O', 'C']:  # O for open, C for close
        if arduino is not None:
            arduino.write(action.encode())
        return jsonify({'status': 'ok', 'action': action})
    return jsonify({'status': 'error', 'message': 'Invalid action'}), 400

if __name__ == '__main__':
    # Start video processing thread
    video_thread = Thread(target=process_frames, daemon=True)
    video_thread.start()
    
    # Start Flask
    app.run(debug=False, host='0.0.0.0', port=5000)
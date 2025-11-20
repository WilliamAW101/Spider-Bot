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
TOLERANCE = 100

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

# Position tracking
current_position = None  # 'L', 'R', 'F', or None
position_start_time = None
POSITION_THRESHOLD = 0.5  # 0.5 seconds before sending command

# Distance to object tracking
last_object_x = None  # Last known X position of object
last_object_center_offset = 0  # Offset from center when object was last seen
object_lost_time = None  # Time when object was last lost from view
frames_since_detection = 0  # Frame count since last detection

# Grabber auto-trigger settings
GRABBER_DELAY = 1  # Wait 0.5 seconds after object is lost before grabbing
grabber_triggered = False  # Track if grabber sequence has been initiated
object_ever_detected = False  # Track if object has been seen at least once


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


def update_position_tracking(new_position):
    """
    Track how long object is in a specific position (L, R, F, or None).
    Only send command if position is maintained for POSITION_THRESHOLD seconds.
    """
    global current_position, position_start_time
    
    # If position changed, reset tracking
    if new_position != current_position:
        current_position = new_position
        position_start_time = time.time()
        return None  # Don't send command yet
    
    # Position hasn't changed, check if threshold met
    if position_start_time is not None:
        elapsed = time.time() - position_start_time
        if elapsed >= POSITION_THRESHOLD:
            return new_position  # Return command to send
    
    return None  # Command not ready yet


def update_object_tracking(object_found, object_cx):
    """
    Track object position and distance to use for grabber arm positioning.
    When object is lost, calculate estimated distance based on last known position.
    """
    global last_object_x, last_object_center_offset, object_lost_time, frames_since_detection
    
    if object_found:
        # Object is visible - update tracking
        last_object_x = object_cx
        last_object_center_offset = object_cx - CENTER_X
        object_lost_time = None
        frames_since_detection = 0
    else:
        # Object is lost - track time and frame count
        if object_lost_time is None:
            object_lost_time = time.time()
        frames_since_detection += 1
    
    return {
        'last_x': last_object_x,
        'offset_from_center': last_object_center_offset,
        'frames_lost': frames_since_detection,
        'time_since_lost': time.time() - object_lost_time if object_lost_time else None
    }


def calculate_grabber_position():
    """
    Calculate where grabber arms should move based on last known object position.
    Returns command indicating which direction to move arms.
    """
    if last_object_center_offset is None:
        return None
    
    # If object was to the left of center
    if last_object_center_offset < -TOLERANCE:
        return 'L'  # Move grabber left
    # If object was to the right of center
    elif last_object_center_offset > TOLERANCE:
        return 'R'  # Move grabber right
    # Object was centered
    else:
        return 'F'  # Move grabber forward


def check_and_trigger_grabber():
    """
    Automatically trigger grabber sequence when object is lost.
    Only triggers if object was detected at least once.
    Waits GRABBER_DELAY seconds after object disappears, then:
    1. Moves grabber in direction object was last seen
    2. Closes the grabber
    """
    global grabber_triggered, object_lost_time, object_ever_detected
    
    # Only trigger if object was detected before, is now lost, and hasn't been triggered yet
    if object_ever_detected and object_lost_time is not None and not grabber_triggered:
        time_since_lost = time.time() - object_lost_time
        
        # Wait for delay threshold before triggering
        if time_since_lost >= GRABBER_DELAY:
            grabber_triggered = True
            grabber_dir = calculate_grabber_position()
            
            # Send movement command based on where object was
            if grabber_dir:
                print(f"Auto-grabbing: Moving {grabber_dir}")
                send(grabber_dir)
            
            # After a short delay, close the grabber
            if arduino is not None:
                time.sleep(0.2)  # Give robot time to move
                print("Auto-grabbing: Closing grabber")
                arduino.write(b'C')  # C for close
            
            return True
    
    return False


def reset_grabber_trigger():
    """Reset grabber trigger when object is detected again."""
    global grabber_triggered
    grabber_triggered = False


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
                    position = 'L'
                elif cx > CENTER_X + TOLERANCE:
                    position = 'R'
                else:
                    position = 'F'
            else:
                position = 'N'
            
            # Update object tracking (for grabber positioning)
            update_object_tracking(object_found, cx if object_found else None)
            
            # If object found, reset grabber trigger; if lost, check if should auto-grab
            if object_found:
                reset_grabber_trigger()
                global object_ever_detected
                object_ever_detected = True  # Mark that object has been seen
                # Check if position has been held long enough to send command
                cmd_to_send = update_position_tracking(position)
                if cmd_to_send is not None:
                    send(cmd_to_send)
            else:
                # Object is lost - check if we should trigger auto-grab
                check_and_trigger_grabber()

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
        'current_position': current_position,
        'manual_mode': manual_mode,
        'last_object_x': last_object_x,
        'last_object_offset': last_object_center_offset,
        'frames_since_lost': frames_since_detection,
        'time_since_lost': round(time.time() - object_lost_time, 2) if object_lost_time else None,
        'estimated_grabber_position': calculate_grabber_position(),
        'commands': {
            'L': 'Left',
            'R': 'Right',
            'F': 'Forward',
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
                    arduino.write(cmd.encode())
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
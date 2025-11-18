#!/usr/bin/env python3
import cv2 
import numpy as np
import serial
import time
import rospy
from std_msgs.msg import String

# --- Initialize ROS Node ---
rospy.init_node('green_object_tracker', anonymous=True)

# --- Get parameters from ROS param server ---
serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')
baud_rate = rospy.get_param('~baud_rate', 9600)
frame_width = rospy.get_param('~frame_width', 640)
frame_height = rospy.get_param('~frame_height', 480)
tolerance = rospy.get_param('~tolerance', 50)

# --- Initialize Publisher ---
pub = rospy.Publisher('object_command', String, queue_size=10)

# --- Initialize Serial (CHANGE PORT IF NEEDED) ---
try:
    arduino = serial.Serial(serial_port, baud_rate, timeout=1)
    time.sleep(2)  # let Arduino reset
    rospy.loginfo(f"Serial connection established on {serial_port}")
except Exception as e:
    rospy.logerr(f"Failed to connect to Arduino: {e}")
    arduino = None

# --- Initialize webcam with DEFAULT settings ---
cap = cv2.VideoCapture(0)

CENTER_X = frame_width // 2

def send(cmd):
    """Send a single character to Arduino and publish to ROS."""
    if arduino is not None:
        arduino.write(cmd.encode())
        time.sleep(0.01)
    
    # Publish to ROS topic
    pub.publish(cmd)
    rospy.logdebug(f"Command sent: {cmd}")

def main():
    rospy.loginfo("Green object tracker started")
    
    try:
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                rospy.logwarn("Failed to read frame from camera")
                break

            frame = cv2.resize(frame, (frame_width, frame_height))

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

            # --- SEND COMMAND ---
            if object_found:
                if cx < CENTER_X - tolerance:
                    send('L')   # object left
                elif cx > CENTER_X + tolerance:
                    send('R')   # object right
                else:
                    send('C')   # centered
            else:
                send('N')       # no object

            cv2.imshow("Green Mask", mask)
            cv2.imshow("Green Object Detection", output)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")
    
    finally:
        send('N')
        cap.release()
        cv2.destroyAllWindows()
        if arduino is not None:
            arduino.close()
        rospy.loginfo("Green object tracker stopped")

if __name__ == '__main__':
    main()
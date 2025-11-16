import serial
import time

arduino = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
time.sleep(2)
cmd = "Connected"
arduino.write(cmd.encode())

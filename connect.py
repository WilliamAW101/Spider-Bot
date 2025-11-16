import serial
import time

arduino = serial.Serial('/dev/ttyAMAO', 9600, timeout=1)
time.sleep(2)
cmd = "Connected"
arduino.write(cmd.encode())

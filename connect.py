import serial
import time

arduino = serial.Serial('/dev/ttyAMA10', 9600, timeout=1)
time.sleep(2)
cmd = "Connected"
arduino.write(cmd.encode())

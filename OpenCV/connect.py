import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 9600)  # Open serial port
time.sleep(2)  # Wait for connection to establish

ser.write(b'1')  # Send command to Arduino
response = ser.readline().decode()  # Read response
print(response)

ser.close()
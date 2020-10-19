import serial
import time

SERIAL_FREQUENCE = 57600

ser = serial.Serial('/dev/ttyACM0', SERIAL_FREQUENCE)
time.sleep(2)
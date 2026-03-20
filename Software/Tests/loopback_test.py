import serial
import time

# 확실하게 ttyAMA0 지정
ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)

print("Final Loopback Test...")
try:
    while True:
        ser.write(b'\xFF\xAA') # FF AA 전송
        time.sleep(0.1)
        
        if ser.in_waiting > 0:
            recv = ser.read(ser.in_waiting)
            print(f"Success! Received: {recv.hex().upper()}")
        else:
            print("Trying...")
        time.sleep(0.5)
except:
    ser.close()
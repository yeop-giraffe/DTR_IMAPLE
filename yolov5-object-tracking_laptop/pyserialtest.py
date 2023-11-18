import serial
import time

# Set up serial connection (Change 'COM3' to whatever port your Arduino is on)
ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
time.sleep(2)  # wait for the serial connection to initialize


# Example of sending data
ct = 0
while True:
    cmd_text = f"Y:{ct}, X:-20, S:0, F:1, B:0\n"
    ser.write(cmd_text.encode())
    #ser.flush()
    ct+= 1
    time.sleep(1)

# Close serial connection
ser.close()

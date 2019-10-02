import serial as serial
import time 
from ctypes import *

LSB_CONNECT = b'\xB9'
LSB_MOTOR = b'\xA8'

s = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

def MotorCmd(m1,m2,m3,m4,m5,m6):
    s.write(LSB_MOTOR)
    time.sleep(0.001)
    s.write(c_int8(m1))
    s.write(c_int8(m2))
    s.write(c_int8(m3))
    s.write(c_int8(m4))
    s.write(c_int8(m5))
    s.write(c_int8(m6))
    s.write(c_uint8(1))

if __name__ == "__main__":
    # Establish connection
    print ("CONNECTING...\n")
    r = "asfd"
    while("ACK" not in r):
        s.write(LSB_CONNECT)
        r = str(s.read(3))
    print ("CONNECTED\n")

    while(1):
        MotorCmd(50,0,0,0,0,0)
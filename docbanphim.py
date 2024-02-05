from threading import Thread
import threading
import serial
import time
import os
import sys, select, termios, tty

import rospy
from std_msgs.msg import Int16

os.system("clear")
msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
       w    
   a   s    d
q:stop
CTRL-C to quit
"""
moveBindings = {
        'w':(0.15,0),
        's':(0.15,0),
        'd':(0.08,0.05),
        'a':(0.08,0.05),
    }

speedBindings={
        'z':(0.1,0.1),
        'x':(-0.1,-0.1),
    }

#Ser1 -> banh phai (right)
ser = serial.Serial(
    port='/dev/ttyUSB1',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1 
) 

DriverAddr = 0x01
                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
# def send_byte(byte):
#     ser.write(bytes([byte]))
##
def lo_hi_bit(num):
    # bitwise de lay bit thap bit cao/ tham khao
    lo = num & 0xFF  # Lấy 1byte thấp
    hi = (num >> 8) & 0xFF  # Dịch phải 8 bit để lấy 1byte cao
    return hi, lo
##
def tohex(val,nbits):
    return hex((val + (1<<nbits)) % (1<<nbits))
##
class myThread(Thread):
    def __init__(self, byte):
        super(myThread, self).__init__()
        self.byte=byte
    def run(self):
        ser.write(bytes([self.byte]))
##
def vels(speed, turn):
    return "speed change:\tspeed %s\tturn %s " % (speed,turn)
def speed_data(linearX, angularZ):
    return "currently:\tspeed %s\tturn %s " % (linearX,angularZ)
##

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    key_timeout = 0.0
    if key_timeout == 0.0:
        key_timeout = None
    while 1:
        key = getKey(key_timeout)
        if key in moveBindings.keys():
            
            if key == 'w':
               
            elif key == "s":
                
            elif key == "d":
                
            elif key == "a":
         
            
        elif key in speedBindings.keys():
            acc_speed = speedBindings[key][0]+acc_speed
            # acc_turn = speedBindings[key][1]+acc_turn
            acc_turn = 0
            print(vels(acc_speed,acc_turn))
            if (status == 14):
                print(msg)
            status = (status + 1) % 15
        else:
            if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                # continue
                pass
            x = 0
            y = 0
            z = 0
            th = 0
            if (key == '\x03'):
                # break
                pass
            elif ord(key) >= 48 and ord(key) <= 57:
                print("stop")
                Fstop()
            # else:
            #     print("Key pressed:", key)


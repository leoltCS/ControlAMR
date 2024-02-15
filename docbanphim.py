from threading import Thread
import threading
import serial
import time
import os
import sys, select, termios, tty

# import rospy
# from std_msgs.msg import Int16

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
        'w':(0.10,0),
        's':(0.10,0),
        'd':(0.08,-0.15),
        'a':(0.08,0.15),
       'q':(0,0),
       'e':(0,0),
       'r':(0,0),
       'c':(0,0),
    }
speedBindings={
        'z':(0.1,0.1),
        'x':(-0.1,-0.1),
    }
##########################################
################ DEFINE ##################
##########################################
ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)
DriverAddr = 0x01

######################################
######################################
class putc_uart0(Thread):
    def __init__(self, hexcode):
        super(putc_uart0, self).__init__()
        self.hexcode=hexcode
    def run(self):
        ser.write(serial.to_bytes(self.hexcode))
        
def send_byte(byte):
    ser.write(bytes([byte]))
######################################
def lo_hi_bit(num):
    # bitwise de lay bit thap bit cao/ tham khao
    lo = num & 0xFF  # Lấy 1byte thấp
    hi = (num >> 8) & 0xFF  # Dịch phải 8 bit để lấy 1byte cao
    return hi, lo
######################################
# def crc_fn(data):
#     return sum(data) % 256
def crc_fn(dpacket):#tham khao cua a Phát vippro
    crc = 0xffff
    poly = 0xa001
    for byte in dpacket:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc >>= 1
                crc ^= poly
            else:
                crc >>= 1
    return crc
######################################
def sync_speed(LSpd, RSpd):
    # global TempL, TempR, FSendS
    LSpd = LSpd
    RSpd = -RSpd
    # if LSpd != TempL:
    TempL = LSpd
    #     FSendS = 1
    # if RSpd != TempR:
    TempR = RSpd
    #     FSendS = 1
    # if FSendS == 1:
    if 1:
        FSendS = 0
        hi_TempL, lo_TempL = lo_hi_bit(TempL)
        hi_TempR, lo_TempR = lo_hi_bit(TempR)
        ibuf = bytearray([DriverAddr, 0x10, 0x20, 0x88, 0x00, 0x02, 0x04,
                          hi_TempL, lo_TempL, hi_TempR, lo_TempR])
        ck = crc_fn(ibuf)
        hi_ck, lo_ck = lo_hi_bit(ck)
        ibuf.extend([lo_ck, hi_ck])

        # iibuf = [DriverAddr, 0x10, 0x20, 0x88, 0x00, 0x02, 0x04,
        #         hi_TempL, lo_TempL, hi_TempR, lo_TempR]
        # iibuf.append(lo_ck)
        # iibuf.append(hi_ck)

        send_speed = putc_uart0(ibuf)
        send_speed.start()
        # for byte in ibuf:
        #     print(hex(byte))
            # send_byte(byte)
        print("send ok")
##################################################################################################
##################################################################################################
def Fstop():
    sync_speed(0,0)
def Enable_motor():
    cmd_enable = bytearray([0x01, 0x06, 0x20, 0x0e, 0x00, 0x08, 0xe2, 0x0f])
    for byte in cmd_enable:
        send_byte(byte)
def Idle_motor():
    cmd_stop = bytearray([0x01, 0x06, 0x20, 0x0e, 0x00, 0x07, 0xa2, 0x0b])
    for byte in cmd_stop:
        send_byte(byte)
##################################################################################################
##################################################################################################
def tohex(val,nbits):
    return hex((val + (1<<nbits)) % (1<<nbits))
##################################################################################################
def vels(speed, turn):
    return "speed change:\tspeed %s\tturn %s " % (speed,turn)
def speed_data(linearX, angularZ):
    return "currently:\tspeed %s\tturn %s " % (linearX,angularZ)
######################################
def getKey(key_timeout=0.0):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
##################################################################################################################

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    speed = 0.0
    turn = 0.0
    acc_speed = 0.0
    acc_turn = 0.0
    repeat = 0.0
    key_timeout = 0.0
    if key_timeout == 0.0:
        key_timeout = None
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0
    print(msg)
    cmd_enable = bytearray([0x01, 0x06, 0x20, 0x0e, 0x00, 0x08, 0xe2, 0x0f])
    for byte in cmd_enable:
        send_byte(byte)
    while 1:
        key = getKey(key_timeout)
        if key in moveBindings.keys():
            speed = moveBindings[key][0] 
            turn = moveBindings[key][1] 
            linearX = speed + acc_speed   ###   
            angularZ = turn  ###
            ###
            L = 0.356
            R = 0.085
            v = linearX
            ow = angularZ
            vg_Right = v +((ow*L)/2)
            vg_Left = v -((ow*L)/2)
            RPM_right = (60 * vg_Right)/(2*3.14*R)
            RPM_left = (60 * vg_Left)/(2*3.14*R)
            ###w
            RPM_right = int(round(RPM_right,0))
            RPM_left = int(round(RPM_left,0))
            print(RPM_right)
            print(RPM_left)
            # sync_speed(RPM_left,RPM_right)
            if key == 'w':
                print(RPM_right)
                print(RPM_left)
                sync_speed(RPM_left,RPM_right)
            elif key == 'a':
                print(RPM_right)
                print(RPM_left)
                sync_speed(RPM_left,RPM_right)
            elif key == 'd':
                print(RPM_right)
                print(RPM_left)
                sync_speed(RPM_left,RPM_right)
            elif key == 's':
                print(RPM_right)
                print(RPM_left)
                sync_speed(-15,-15)
            elif key == "q":
                print(speed_data(moveBindings[key][0],moveBindings[key][1]))
                Fstop()
            elif key == "e":
                print(speed_data(moveBindings[key][0],moveBindings[key][1]))
                Idle_motor()
            elif key == "c":
                # Enable_motor()
                cmd_enable = bytearray([0x01, 0x06, 0x20, 0x0e, 0x00, 0x08, 0xe2, 0x0f])
                for byte in cmd_enable:
                    send_byte(byte)
                print("enable motor")
            if (status == 14):
                os.system("clear")
                print(msg)
            status = (status + 1) % 15
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


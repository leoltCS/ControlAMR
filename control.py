from threading import Thread
import threading
import serial
import time
import os
os.system("clear")

ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1 
) 

DriverAddr = 0x01
FuncCode = 0x06
StartAddr = 0x20
MotorLeft = 0x88
MotorRight = 0x89

# 30RPM -> left: 00 1e/ right: ff e2 / ck: c2 17 / lo-hi
# cmd_vel= bytearray([0x1, 0x10, 0x20, 0x88, 0x0, 0x2, 0x4, 0x0, 0x1e, 0xff, 0xe2, 0xc2, 0x17])

def send_byte(byte):
    ser.write(bytes([byte]))

def lo_hi_bit(num):
    # bitwise de lay bit thap bit cao/ tham khao
    lo = num & 0xFF  # Lấy 1byte thấp
    hi = (num >> 8) & 0xFF  # Dịch phải 8 bit để lấy 1byte cao
    return hi, lo

# def crc_fn(data):
#     return sum(data) % 256
def crc_fn(dpacket):
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

def sync_speed(LSpd, RSpd):
    global TempL, TempR, FSendS

    LSpd = LSpd
    RSpd = -RSpd

    # if LSpd != TempL:
    TempL = LSpd
    #     FSendS = 1

    # if RSpd != TempR:
    TempR = RSpd
    #     FSendS = 1

    if 1:
        FSendS = 0
        hi_TempL, lo_TempL = lo_hi_bit(TempL)
        hi_TempR, lo_TempR = lo_hi_bit(TempR)
        ibuf = bytearray([DriverAddr, 0x10, 0x20, 0x88, 0x00, 0x02, 0x04,
                          hi_TempL, lo_TempL, hi_TempR, lo_TempR])
        ck = crc_fn(ibuf)
        hi_ck, lo_ck = lo_hi_bit(ck)
        ibuf.extend([lo_ck, hi_ck])

        # Sending the data (replace this part with your actual transmission logic)
        for byte in ibuf:
            print(hex(byte))
            send_byte(byte)
        print("send ok")
    # Resetting flags if needed
    # if FZL:
    #     Ci = 0
    #     FZL = 0
 
# TempL = 0
# TempR = 0
# FSendS = 0
cmd_enable = bytearray([0x01, 0x06, 0x20, 0x0e, 0x00, 0x08, 0xe2, 0x0f])
for byte in cmd_enable:
    send_byte(byte)
time.sleep(0.2)
sync_speed(30, 30)
time.sleep(3)
sync_speed(0, 0)
# time.sleep(3)
# cmd_stop = bytearray([0x01, 0x06, 0x20, 0x0e, 0x00, 0x07, 0xa2, 0x0b])
# for byte in cmd_stop:
#     send_byte(byte)

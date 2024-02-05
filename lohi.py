from threading import Thread
import threading
import serial
import time
import os
os.system("clear")

ser = serial.Serial(port='/dev/ttyUSB0',baudrate=115200) 
def send_byte(byte):
    ser.write(bytes([byte])) 

def lo_hi_bit(num):
    if num < 0:
        num = 0xFFFF + num + 1  # Two's complement for negative values

    lo_bit = num & 0xFF
    hi_bit = (num >> 8) & 0xFF

    return hi_bit, lo_bit

# def crc_fn1(data): #sai
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
cmd_enable = bytearray([0x01, 0x06, 0x20, 0x0e, 0x00, 0x08, 0xe2, 0x0f])
# cmd_stop = bytearray([0x01, 0x06, 0x20, 0x0e, 0x00, 0x07, 0xa2, 0x0b])
cmd_stop = bytearray([0x01, 0x06, 0x20, 0x0e, 0x00, 0x08])
for byte in cmd_stop:
    send_byte(byte)
ck = crc_fn(cmd_stop)
hi_ck, lo_ck = lo_hi_bit(ck)
cmd_stop.extend([lo_ck, hi_ck])
print(cmd_stop)
# DriverAddr = 0x01
# TempL = 30
# TempR = -30

# hi_TempL, lo_TempL = lo_hi_bit(TempL)
# hi_TempR, lo_TempR = lo_hi_bit(TempR)
# # hi_TempL , lo_TempL = 0x1e , 0x00
# # hi_TempR, lo_TempR = 0x1e , 0x00
# ibuf = bytearray([DriverAddr, 0x10, 0x20, 0x88, 0x00, 0x02, 0x04,
#                   hi_TempL, lo_TempL, hi_TempR, lo_TempR])
# ck = crc_fn(ibuf)
# hi_ck, lo_ck = lo_hi_bit(ck)
# ibuf.extend([lo_ck, hi_ck])

# # print(ibuf)
# for byte in ibuf:
#     print(hex(byte))
#     # send_byte(byte)
# print("send ok")
# for byte in cmd_stop:
#     send_byte(byte)


# zlac706
# calc = 273
# hi_bit, lo_bit = lo_hi_bit(calc)

# data = bytearray([0x6, hi_bit, lo_bit])
# # data = bytearray([0x6, 0x00, lo_bit])
# # Tính checksum
# checksum = sum(data) % 256

# # Lấy bit cao và bit thấp của checksum
# hi_checksum, lo_checksum = lo_hi_bit(checksum)

# print(f"Bit cao của checksum: {hi_checksum}, Bit thấp của checksum: {lo_checksum}")
# # Gán checksum vào cuối ibuf
# data.extend([hi_checksum, lo_checksum])

# # In ra ibuf với checksum
# print(data)

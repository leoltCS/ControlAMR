#!/usr/bin/env python3

#qt gui import
from PyQt5 import QtWidgets, uic
from PyQt5 import QtGui
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtMultimedia import *
from PyQt5.QtMultimediaWidgets import *
# from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QPixmap, QMouseEvent
########################--OS--################################
# import numpy as np
# import cv2
import sys, os, serial, time, select, termios, tty
from threading import Thread
# import subprocess
# import socket

import serial.tools.list_ports
import configparser #ini file import lib
#covert rcc to py trước khi import form
os.system("Pyrcc5 icon_navigation.qrc -o icon_navigation_rc.py")
#######################--GUI--#################################
from main_body import Ui_MainWindow
####################### DRIVER ZLAC8015D #################################
# from zlac8015d import ZLAC8015D
import ZLAC8015D
##----------------ver------------------------##
version_beta = "0.1.build.15-02-2024.byTL"
##----------------set dir------------------------##
dirpath = os.path.dirname(os.path.abspath(__file__))
os.chdir(dirpath)
#################################
# Lấy đường dẫn của thư mục chứa file exe đang thực thi
exe_dir = os.path.dirname(os.path.abspath(__file__))
##----------------------------------------------##
config = configparser.ConfigParser()
config.read('./setting/Setting_Config.ini')
# min_area_set = int(config.get('OpenCV','min_area'))
# max_area_set = int(config.get('OpenCV','max_area'))
##----------------------------------------------#
WINDOW_SIZE = 1
##----------------------------------------------##
os.system("clear")
# os.system("cls")
##----------------------------------------------##
DriverAddr = 0X01
#######################################################################
'''######################### DEF ######################################'''
#######################################################################
def getKey(key_timeout=0.0):
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
#######################################################################
def lo_hi_bit(num):
        # bitwise de lay bit thap bit cao/ tham khao
        lo = num & 0xFF  # Lấy 1byte thấp
        hi = (num >> 8) & 0xFF  # Dịch phải 8 bit để lấy 1byte cao
        return hi, lo
    ######################################
    # def get_checksum(data):
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
#######################################################################
#######################################################################
class putc_uart0(Thread):
    def __init__(self, serIalport, hexcode):
        super(putc_uart0, self).__init__()
        self.ser = serIalport
        self.hexcode = hexcode
    def run(self):
        self.ser.write(serial.to_bytes(self.hexcode))
#######################################################################
#######################################################################
class Serial_Reader_Thread_KEYPAD(QThread):
    signal = pyqtSignal(bytes)
    def __init__(self, Serial_c):
        self.Serial = Serial_c
        super(Serial_Reader_Thread_KEYPAD, self).__init__()
    def run(self):
        while True:
            # data_port = self.Serial.readline().decode().strip()
            data_port = self.Serial.read(3)
            if data_port:
                # print(data_port)
                self.signal.emit(data_port)
            time.sleep(0.08)
    def stop(self):
        self.terminate()
#######################################################################
class Control_motors_thread(QThread):
    signal = pyqtSignal(bytes)
    def __init__(self, motors,speed,LR):
        self.motors = motors
        self.speed = speed
        self.LR = LR
        super(Control_motors_thread, self).__init__()
    def run(self):
        # self.motor
        if self.LR == "FW":
            left_RPM = self.speed
            right_RPM = -(self.speed)
            self.motors.set_rpm(left_RPM,right_RPM)
            print("run forward") 
        elif self.LR == "BW":
            left_RPM = -(self.speed)
            right_RPM = self.speed
            self.motors.set_rpm(left_RPM,right_RPM) 
            print("run backward")  
        elif self.LR == "stop":
            cmds = [0, 0]
            self.motors.set_rpm(cmds[0],cmds[1])
            print("stop motor")
    def stop(self):
        self.terminate()
#######################################################################
class encoder_request_data_thread(QThread):
    signal_L = pyqtSignal(int)
    signal_R = pyqtSignal(int)
    def __init__(self, serial):
        self.serial = serial
        super(encoder_request_data_thread, self).__init__()
    def run(self):
        hexcode = [0x01, 0x03, 0x20, 0xA7, 0x00, 0x04, 0xfe, 0x2a]
        self.serial.write(serial.to_bytes(hexcode))
        # time.sleep(0.05)
        # self.serial.write(b'\x20\xA7\x00\x04')  # Gửi yêu cầu đọc 4 byte từ địa chỉ 0x20A7
        data = self.serial.read(8)  # Đọc phản hồi 8 byte
        l_pul_hi = data[2]
        l_pul_lo = data[3]
        r_pul_hi = data[4]
        r_pul_lo = data[5]
        l_tick = ((l_pul_hi & 0xFF) << 8) | (l_pul_lo & 0xFF)
        r_tick = ((r_pul_hi & 0xFF) << 8) | (r_pul_lo & 0xFF)
        self.signal_L.emit(l_tick) 
        self.signal_R.emit(-r_tick)
    def stop(self):
        self.terminate()
#######################################################################
class encoder_thread_ticks(QThread):
    signal_L = pyqtSignal(int)
    signal_R = pyqtSignal(int)
    def __init__(self, motors):
        self.motors = motors
        super(encoder_thread_ticks, self).__init__()
    def run(self):
        while True:
            l_tick, r_tick = self.motors.get_wheels_tick()
            # vl,vr = motors.get_linear_velocities()
            # print("period: {:.4f} l_tick: {:.1f} | r_tick: {:.1f}".format(period,l_tick,r_tick))
            self.signal_L.emit(l_tick) 
            self.signal_R.emit(-r_tick)   
    def stop(self):
        self.terminate()
#######################################################################
class KeyboardReaderThread(QThread):
    signal_key_pressed = pyqtSignal(str)
    def __init__(self):
        super(KeyboardReaderThread, self).__init__()
    def run(self):
        while True:
            key = getKey()
            if key:
                self.signal_key_pressed.emit(key)
#######################################################################
class thread_custom(QThread):
    signal = pyqtSignal(str)
    def __init__(self, self_uic, name_func):
        super(thread_custom, self).__init__()
        self.self_uic = self_uic
        self.name_func = name_func
    def run(self):
        # Check if the method exists before calling it
        # if hasattr(self.self_uic, name_func):
        getattr(self.self_uic, self.name_func)()
    def stop(self):
        self.terminate()
#######################################################################
##########################---Main Class---#############################
#######################################################################
class Main_form(QtWidgets.QMainWindow):
    def __init__(self):
        super(Main_form, self).__init__()
        self.uic = Ui_MainWindow()
        self.uic.setupUi(self)
        '''set up UI main'''
        self.initUI()
    def initUI(self): 
        #---init---#
        # self.setWindowTitle('demo')
        # print("Cv2 version: ",cv2.__version__)
        self.index_port = None
        self.port_open = False
        self.setWindowFlag(Qt.FramelessWindowHint)
        '''############scale lb video and cap####################'''
        # self.uic.lb_video.setScaledContents(True)
        # self.uic.lb_result.setScaledContents(True)
        '''######################################################'''
        '''----Set value in ini file to setting tab----'''
        # self.uic.txt_soluonglo.setText(str(So_luong_full))
        #set default in HOME tab
        self.uic.stackedWidget.setCurrentWidget(self.uic.form_control)
        '''#---event---#'''
        #Window navigation event
        self.uic.minimum_btn.clicked.connect(lambda: self.showMinimized())
        self.uic.maximum_btn.clicked.connect(self.restore_or_maximize_window)
        self.uic.close_btn.clicked.connect(lambda: self.close())
        #drag window
        def moveWindow(e:QMouseEvent):
            if self.isMaximized() == False:
                if e.buttons() == Qt.LeftButton:
                    self.move(self.pos() + e.globalPos() - self.clickPosition)
                    self.clickPosition = e.globalPos()
                    e.accept()
        self.uic.main_header.mouseMoveEvent = moveWindow
        self.uic.menu_btn.clicked.connect(self.extend_menu)
        
        '''#---button top left---#'''
        self.uic.btn_home.clicked.connect(lambda: self.uic.stackedWidget.setCurrentWidget(self.uic.form_rong))
        self.uic.btn_setting.clicked.connect(lambda: self.uic.stackedWidget.setCurrentWidget(self.uic.form_control))
        self.uic.btn_information.clicked.connect(lambda: self.uic.stackedWidget.setCurrentWidget(self.uic.form_information))
        '''#---button mode---#'''
        self.uic.btn_speedmode.clicked.connect(self.speedmode)
        self.uic.btn_positionmode.clicked.connect(self.positionmode)
        '''#---button control---#'''
        self.uic.btn_forward.clicked.connect(self.run_forward)
        self.uic.btn_backward.clicked.connect(self.run_backward)
        self.uic.btn_left.clicked.connect(self.run_left)
        self.uic.btn_right.clicked.connect(self.run_right)
        self.uic.btn_stop.clicked.connect(self.stop_run)
        self.uic.btn_keypad.clicked.connect(self.start_thread_read_keypad)
        '''#---button spindle mode---#'''
        self.uic.btn_runspeed.clicked.connect(self.run_speed)
        self.uic.btn_enablemotor.clicked.connect(self.enable_motor)
        self.uic.btn_idle.clicked.connect(self.idle_motor)
        # combobox.addItem("Item 1")
        #create null thread 
        self.thread = {}
        self.get_name_port()
        
        self.show() #show form
        # self.showMaximized()
        # self.connect_driver_zlac8015d()

        self.connect_port()

        self.thread_ticks_encoder()
        # self.uic.maximum_btn.setIcon(QtGui.QIcon(u":/images/icon/icons8_restore_window_64px.png"))
    ##############################################################################################################################################################################  
    ##############################################################################################################################################################################  
    '''########################################################################################################################################################################'''
    def start_thread(self, name_func):
        try:
            self.thread[3].stop()
            self.thread[3] = thread_custom(self, name_func)
            self.thread[3].start()
        except:
            print("Stop Error!")
            self.thread[3] = thread_custom(self, name_func)
            self.thread[3].start()
    #######################################################################################  
    #######################################################################################
    '''################################ sync speed ######################################'''
    def sync_speed(self, LSpd:int, RSpd:int):
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

            send_speed = putc_uart0(self.serial_port, ibuf)
            send_speed.start()
            # for byte in ibuf:
            #     print(hex(byte))
                # send_byte(byte)
            print("send ok")  
    '''####################### start thread get encoder ####################################'''
    def thread_ticks_encoder(self):
        # self.thread[10] = encoder_thread_ticks(self.motors)
        self.thread[10] = encoder_request_data_thread(self.serial_port)
        self.thread[10].start()
        self.thread[10].signal_L.connect(self.left_ticks_encoder)
        self.thread[10].signal_R.connect(self.right_ticks_encoder)
    '''####################### get encoder ticks ####################################'''
    def left_ticks_encoder(self,leftticks):
        self.uic.txt_endcoder_left.setText(str(leftticks))
    def right_ticks_encoder(self,rightticks):
        self.uic.txt_encoder_right.setText(str(rightticks))
    def port_read(self,data_port):
        data_port_decode = data_port.decode().strip()
        # print(data_port_decode)
        if data_port_decode == "$KM":
            # self.uic.lb_keypad.setText(data_port_decode)
            self.enable_motor()
        elif data_port_decode == "$KL":
            self.idle_motor()
        elif data_port_decode == "$K2":
            cmds = [5, -5]
            self.motors.set_rpm(cmds[0],cmds[1])
        elif data_port_decode == "$K5":
            cmds = [-5, 5]
            self.motors.set_rpm(cmds[0],cmds[1])
        elif data_port_decode == "$K4":
            cmds = [0, -5]
            self.motors.set_rpm(cmds[0],cmds[1])
        elif data_port_decode == "$K6":
            cmds = [5, 0]
            self.motors.set_rpm(cmds[0],cmds[1])
        elif data_port_decode == "$K1":
            cmds = [5, 0]
            self.stop_motor()
    '''####################### MODE ####################################'''
    def speedmode(self):
        self.motors.set_mode(3)
        self.enable_motor()
        print("speed mode")
    def positionmode(self):
        self.motors.set_mode(3)
        self.enable_motor()
        print("speed mode")
    '''####################### Spinde MODE ####################################'''
    def enable_motor(self):
        # self.motors.enable_motor()
        # print("enable motor")  
        cmd_enable = [0x01, 0x06, 0x20, 0x0e, 0x00, 0x08, 0xe2, 0x0f]
        send_speed = putc_uart0(self.serial_port, cmd_enable)
        send_speed.start()
    def idle_motor(self):
        # self.motors.disable_motor()
        # print("idle")
        # cmd_stop = bytearray([0x01, 0x06, 0x20, 0x0e, 0x00, 0x07, 0xa2, 0x0b])
        # for byte in cmd_stop:
        #     send_byte(byte)
        cmd_stop = [0x01, 0x06, 0x20, 0x0e, 0x00, 0x07, 0xa2, 0x0b]
        send_speed = putc_uart0(self.serial_port, cmd_stop)
        send_speed.start()
    '''####################### Control ####################################'''
    def run_forward(self):
        get_speed = int(self.uic.txt_speed.text())
        L_Speed = get_speed
        R_Speed = get_speed
        self.sync_speed(L_Speed, R_Speed)
    def run_backward(self):
        get_speed = int(self.uic.txt_speed.text())
        L_Speed = -get_speed
        R_Speed = -get_speed
        self.sync_speed(L_Speed, R_Speed)
    def run_left(self):
        get_speed = int(self.uic.txt_speed.text())
        L_Speed = 0
        R_Speed = get_speed
        self.sync_speed(L_Speed, R_Speed)
    def run_right(self):
        get_speed = int(self.uic.txt_speed.text())
        L_Speed = get_speed
        R_Speed = 0
        self.sync_speed(L_Speed, R_Speed)
    def stop_run(self):
        L_Speed = 0
        R_Speed = 0
        self.sync_speed(L_Speed, R_Speed)

    # def start_thread_read_keypad(self):
    #     self.timer = QTimer(self)
    #     self.timer.timeout.connect(self.checkKey)
    #     self.timer.start(10)  # Milliseconds
    # def checkKey(self):
    #     key = getKey(0.0)
    #     if key:
    #         self.label.setText(f"Pressed key: {key}")
    def start_thread_read_keypad(self):
        try:
            self.thread[2].stop()
            self.uic.btn_keypad.setText("Keyboard")
        except:
            print("Stop Error!")
            self.thread[2] = KeyboardReaderThread()
            self.thread[2].start()
            self.thread[2].signal_key_pressed.connect(self.on_key_pressed)
            self.uic.btn_keypad.setText("Cls")
    def on_key_pressed(self, key):
        print(f"Pressed key: {key}")
        if key == "w":
            self.run_forward()
        elif key == "s":
            self.run_backward()
        elif key == "a":
            self.run_left()
        elif key == "d":
            self.run_right()
        elif key == "q":
            self.stop_run()
        elif key == "e":
            self.enable_motor()
        elif key == "r":
            self.idle_motor()
    #######################################################################################
    def run_speed(self):
        L_Speed = int(self.uic.txt_left.text())
        R_Speed = int(self.uic.txt_right.text())
        self.sync_speed(L_Speed, R_Speed)
        print("run speed")
    #######################################################################################  
    #######################################################################################
    '''####################### Connect to driver ####################################'''
    def connect_driver_zlac8015d(self):
        # port = self.uic.cbb_nameport.itemText(self.index_port) #"get value from combobox"
        # self.motors = ZLAC8015D.Controller(port)
        self.motors = ZLAC8015D.Controller(port="/dev/ttyUSB0")
    #######################################################################################  
    #######################################################################################
    def get_name_port(self):
        config.read('./setting/Setting_Config.ini')
        com_name = config.get('ComPort','nameport')
        self.com_name = com_name
        index_port = 0
        available_ports = serial.tools.list_ports.comports()   # lấy tất cả các port hiện có
        all_port_names = [port.device for port in available_ports]  # lấy tên tất cả các port thả vô danh sách
        for name_port in all_port_names:
            self.uic.cbb_nameport.addItem(name_port)
            if name_port == com_name:
                self.index_port = index_port
                self.uic.cbb_nameport.setCurrentIndex(index_port)
            index_port = index_port + 1
    def connect_port(self):
        try:
            # self.get_name_port()
            config.read('./setting/Setting_Config.ini')
            com_name = config.get('ComPort','nameport')
            self.com_name = com_name
            get_nameport = self.com_name
        except Exception as e:
            print(f"Error get comport: {e}")
        if self.port_open:
            self.port_open = False
            self.serial_port.close()
        else:
            try:
                self.serial_port = serial.Serial(get_nameport, 115200, timeout=0.1)
                self.port_open = True
                # self.read_dataport()
                self.uic.lb_status_connect_port.setText(f"OK port {com_name}")
            except serial.SerialException:
                QtWidgets.QMessageBox.warning(self,"Connect Fail!", "Lỗi kết nối Port")
    # def read_dataport(self):
    #     self.thread[1] = Thread_Serial_Reader(self.serial_port)
    #     self.thread[1].start()
    #     self.thread[1].signal.connect(self.port_read)
    '''##################### def windown navigation event ########################'''
    def extend_menu(self):
        width = self.uic.left_side_menu.width()
        if width == 45:
            newwidth = 90
        else:
            newwidth = 45
        self.animation = QPropertyAnimation(self.uic.left_side_menu,b"minimumWidth")
        # self.animation = QPropertyAnimation(self.uic.left_side_menu,b"maximumWidth")
        self.animation.setDuration(250)
        self.animation.setStartValue(width)
        self.animation.setEndValue(newwidth)
        self.animation.setEasingCurve(QEasingCurve.InOutQuart)
        self.animation.start()
    def restore_or_maximize_window(self):
        global WINDOW_SIZE
        win_status = WINDOW_SIZE
        if win_status == 0:
            WINDOW_SIZE = 1
            self.showMaximized()
            self.uic.maximum_btn.setIcon(QtGui.QIcon(u":/images/icon/icons8_restore_window_64px.png"))
        else:
            WINDOW_SIZE = 0
            self.showNormal()
            self.uic.maximum_btn.setIcon(QtGui.QIcon(u":/images/icon/icons8_maximize_window_64px.png"))
    def closeEvent(self, event):
        # msgbox_info("thông báo","bạn có muốn thoát")
        result = QtWidgets.QMessageBox.question(self, 'Confirm', 'Are you sure you want to close?',
                                                 QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
        if result == QtWidgets.QMessageBox.Yes:
            event.accept()
            self.close()
        else:
            event.ignore()
    def mousePressEvent(self, event: QMouseEvent):
        # return super().mousePressEvent(a0)
        self.clickPosition = event.globalPos()
        # print('nhan chuot')
    ################################################################    
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = Main_form()
    sys.exit(app.exec_())
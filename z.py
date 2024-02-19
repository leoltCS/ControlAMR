import serial
import requests

# Open the COM port
ser = serial.Serial('COM24', 9600)  # Adjust the port and baud rate as needed

# # Define your web server endpoint
# url = 'http://your-web-server.com/data-endpoint'

try:
    start_marker = b'$'
    data = b''  # Biến lưu trữ dữ liệu
    flag_find_index = True
    while True:
        if flag_find_index:
            byte = ser.read(10)
            data += byte
            # kiểm tra xem đã đọc đủ một gói dữ liệu hay chưa
            if start_marker in data:
                start_index = data.find(start_marker)
                packet = data[start_index:]
                print(hex(packet))
                # data = data[end_index + 1:]
              
except KeyboardInterrupt:
    # Close the COM port when the program is terminated
    ser.close()

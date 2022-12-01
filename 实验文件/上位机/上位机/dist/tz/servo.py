import sys
import serial
import serial.tools.list_ports
import re
import time
import threading
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtCore import QTimer
from PyQt5 import uic


class MyWindow(QWidget):

    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        self.ui = uic.loadUi("./Servo.ui")
        self.sensor_data = self.ui.textBrowser
        self.servo_angle = self.ui.textBrowser_2
        self.target = self.ui.lineEdit
        self.clear = self.ui.pushButton
        self.open_com = self.ui.pushButton_3
        self.control = self.ui.pushButton_2
        self.calibration = self.ui.pushButton_4
        self.auto = self.ui.pushButton_5
        self.stop = self.ui.pushButton_6
        self.choose_com = self.ui.comboBox
        self.baudRate = self.ui.comboBox_2
        self.ser = serial.Serial()
        self.ser.baudrate = int(self.baudRate.currentText())
        self.ser.bytesize = 8  # 设置数据位
        self.ser.stopbits = 1  # 设置停止位
        self.ser.parity = "N"  # 设置校验位
        self.search_com()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.getdata)
        self.count = 0
        self.ccount = [0, 0]


        # 按键绑定槽函数
        self.clear.clicked.connect(self.clear_all)
        self.open_com.clicked.connect(self.openport)
        self.control.clicked.connect(self.control_servo)
        self.calibration.clicked.connect(self.cali)
        self.auto.clicked.connect(self.senddata)
        self.stop.clicked.connect(self.stoproll)

        # 定义函数
        # 查找可用串口

    def search_com(self):
        self.choose_com.clear()
        port_list = list(serial.tools.list_ports.comports())
        com_numbers = len(port_list)
        p1 = re.compile(r'[(](.*?)[)]', re.S)
        for i in range(com_numbers):
            com_list = str(port_list[i])
            com_name = re.findall(p1, com_list)
            com_name = str(com_name)
            strlist = com_name.split("'")
            self.choose_com.addItem(strlist[1])

        # 打开/关闭串口

    def openport(self):
        self.ser.port = self.choose_com.currentText()
        self.ser.baudrate = int(self.baudRate.currentText())
        if self.ser.is_open:
            self.timer.stop()
            time.sleep(0.5)
            self.ser.close()
            self.open_com.setText('打开串口')
        else:
            self.ser.open()
            flag = self.ser.is_open
            if flag:
                # self.sensor_data.setPlainText("串口连接成功!")
                self.open_com.setText('关闭串口')
                self.timer.start(5)

        # 清除所有数据

    def clear_all(self):
        self.sensor_data.clear()
        self.servo_angle.clear()
        self.target.clear()

        # 控制舵机

    def control_servo(self):
        if self.ser.is_open:
            input_s = 'hand' + self.target.text()
            self.ser.write((input_s + '\r\n').encode('UTF-8'))

        # 标定

    def cali(self):
        self.ccount[self.count] = int(self.sensor_data.toPlainText())
        self.count = self.count + 1
        if self.count > 1:
            self.count = 0

        # 自动控制

    def stoproll(self):
        if self.ser.is_open:
            input_s = 'stop'
            self.ser.write((input_s + '\r\n').encode('UTF-8'))

    def getdata(self):
        if self.ser.is_open:
            self.rc_data = self.ser.read_all()
            if self.rc_data != b'':
                data = self.rc_data.decode('UTF-8')
                if data.find('sens') != -1:
                    st = data.find('sens')
                    jd = data.find('/')
                    self.sensor_data.setPlainText(data[st+4: jd])
                    self.servo_angle.setPlainText(data[jd + 1:])

    def senddata(self):
        input_string = 'auto' + 'n' + str(self.ccount[0]) + 'x' + str(self.ccount[1])
        self.ser.write((input_string + '\r\n').encode('UTF-8'))
        time.sleep(1)
        input_string = 'status'
        self.ser.write((input_string + '\r\n').encode('UTF-8'))


if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = MyWindow()
    w.ui.show()
    app.exec()

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
        self.ui = uic.loadUi("./motor.ui")
        self.speed = self.ui.textBrowser
        self.target_speed = self.ui.lineEdit
        self.PID_P = self.ui.lineEdit_2
        self.PID_I = self.ui.lineEdit_3
        self.PID_D = self.ui.lineEdit_4
        self.ratio = self.ui.lineEdit_5
        self.choose_com = self.ui.comboBox
        self.baudRate = self.ui.comboBox_2
        self.clear = self.ui.pushButton
        self.open_com = self.ui.pushButton_3
        self.PID_control = self.ui.pushButton_2
        self.control = self.ui.pushButton_4
        self.stopmotor = self.ui.pushButton_5
        self.pg = self.ui.graphicsView
        self.pg.setBackground('w')
        self.pg.setLabel('left', '速度')
        self.pg.setLabel('bottom', '时间')
        self.ser = serial.Serial()
        self.ser.baudrate = int(self.baudRate.currentText())
        self.ser.bytesize = 8  # 设置数据位
        self.ser.stopbits = 1  # 设置停止位
        self.ser.parity = "N"  # 设置校验位
        self.search_com()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.getdata)
        self.signal = False
        self.judge = False
        self.arr = []
        self.x = []
        self.count = 0

        self.clear.clicked.connect(self.clear_all)
        self.open_com.clicked.connect(self.openport)
        self.PID_control.clicked.connect(self.PID)
        self.control.clicked.connect(self.senddata)
        self.stopmotor.clicked.connect(self.stop)

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
                self.timer.start(20)

        # 清除所有数据

    def clear_all(self):
        self.speed.clear()
        self.target_speed.clear()
        self.PID_P.clear()
        self.PID_I.clear()
        self.PID_D.clear()
        self.ratio.clear()

    def PID(self):
        if self.ser.is_open:
            self.judge = not self.judge
            if self.judge:
                self.PID_control.setText('关闭')
                input_string = 'targ' + self.target_speed.text() + 'P' + self.PID_P.text() + 'I' + self.PID_I.text() + 'D' + self.PID_D.text()
                self.ser.write((input_string + '\r\n').encode('UTF-8'))
                time.sleep(1)
                input_string = 'start'
                self.ser.write((input_string + '\r\n').encode('UTF-8'))
                self.signal = True
            else:
                self.PID_control.setText('PID控制')
                input_string = 'stop'
                self.ser.write((input_string + '\r\n').encode('UTF-8'))
                self.signal = False
                self.arr = []
                self.x = []
                self.count = 0
                self.pg.clear()

    def senddata(self):
        if self.ser.is_open:
            input_string = 'ratio' + self.ratio.text()
            self.ser.write((input_string + '\r\n').encode('UTF-8'))

    def stop(self):
        if self.ser.is_open:
            input_string = 'close'
            self.ser.write((input_string + '\r\n').encode('UTF-8'))

    def getdata(self):
        if self.ser.is_open:
            self.rc_data = self.ser.read_all()
            if self.rc_data != b'':
                data = self.rc_data.decode('UTF-8')
                self.speed.setPlainText(data)
                if self.signal:
                    self.arr.append(int(data))
                    self.x.append(0.1*self.count)
                    self.count = self.count + 1
                    self.pg.plot(self.x, self.arr, pen=(255, 0, 0), title="转速时间图")


if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = MyWindow()
    w.ui.show()
    app.exec()

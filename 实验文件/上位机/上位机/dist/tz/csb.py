import sys
import serial
import serial.tools.list_ports
import re
import time
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtCore import QTimer
from PyQt5 import uic


class MyWindow(QWidget):

    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        self.ui = uic.loadUi("./chaoshengbo.ui")
        self.distance = self.ui.textBrowser_2
        self.target = self.ui.lineEdit
        self.frequency = self.ui.lineEdit_2
        self.clear = self.ui.pushButton
        self.open_com = self.ui.pushButton_3
        self.setdistance = self.ui.pushButton_2
        self.openfmq = self.ui.pushButton_4
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
        self.signal = False
        self.judge = False

        # 按键绑定槽函数
        self.clear.clicked.connect(self.clear_all)
        self.open_com.clicked.connect(self.openport)
        self.setdistance.clicked.connect(self.setD)
        self.openfmq.clicked.connect(self.control)

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
        self.target.clear()
        self.distance.clear()
        self.frequency.clear()

        # 控制舵机

    def setD(self):
        if self.ser.is_open:
            self.judge = not self.judge
            if self.judge:
                self.setdistance.setText('关闭')
                input_string = 'hz' + self.frequency.text()
                self.ser.write((input_string + '\r\n').encode('UTF-8'))
                time.sleep(1)
                input_string = 'Dis' + self.target.text()
                self.ser.write((input_string + '\r\n').encode('UTF-8'))
            else:
                self.setdistance.setText('设置')
                input_string = 'stop'
                self.ser.write((input_string + '\r\n').encode('UTF-8'))

    def control(self):
        if self.ser.is_open:
            self.signal = not self.signal
            if self.signal:
                self.openfmq.setText('关闭')
                input_string = 'hz' + self.frequency.text()
                self.ser.write((input_string + '\r\n').encode('UTF-8'))
                time.sleep(1)
                input_string = 'start'
                self.ser.write((input_string + '\r\n').encode('UTF-8'))
            else:
                self.openfmq.setText('打开')
                input_string = 'close'
                self.ser.write((input_string + '\r\n').encode('UTF-8'))

    def getdata(self):
        if self.ser.is_open:
            self.rc_data = self.ser.read_all()
            if self.rc_data != b'':
                data = self.rc_data.decode('UTF-8')
                if data.find('D') != -1:
                    if data.find('F') != -1:
                        D = data.find('D')
                        F = data.find('F')
                        self.distance.setText(data[D + 1: F])
                        self.frequency.setText(data[F + 1:])
                    else:
                        D = data.find('D')
                        self.distance.setText(data[D + 1:])


if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = MyWindow()
    w.ui.show()
    app.exec()

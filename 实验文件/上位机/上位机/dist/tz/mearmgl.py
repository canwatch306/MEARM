import sys
import serial
import serial.tools.list_ports
import re
import time
import cv2
import numpy as np
import threading
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer
from PyQt5 import uic


class MyWindow(QWidget):

    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        self.ui = uic.loadUi("./mearmgl.ui")
        self.base_angle = self.ui.textBrowser
        self.rArm_angle = self.ui.textBrowser_2
        self.fArm_angle = self.ui.textBrowser_3
        self.claw_angle = self.ui.textBrowser_4
        self.Z_angle = self.ui.textBrowser_5
        self.W_data = self.ui.textBrowser_6
        self.X_angle = self.ui.textBrowser_7
        self.Pressure = self.ui.textBrowser_8
        self.base_target = self.ui.lineEdit
        self.rArm_target = self.ui.lineEdit_2
        self.fArm_target = self.ui.lineEdit_3
        self.claw_target = self.ui.lineEdit_4
        self.camera_ip = self.ui.lineEdit_5
        self.c1_hmin = self.ui.lineEdit_6
        self.c1_smin = self.ui.lineEdit_7
        self.c1_vmin = self.ui.lineEdit_8
        self.c1_hmax = self.ui.lineEdit_9
        self.c1_smax = self.ui.lineEdit_10
        self.c1_vmax = self.ui.lineEdit_11
        self.c2_hmin = self.ui.lineEdit_12
        self.c2_smin = self.ui.lineEdit_13
        self.c2_vmin = self.ui.lineEdit_14
        self.c2_hmax = self.ui.lineEdit_15
        self.c2_smax = self.ui.lineEdit_16
        self.c2_vmax = self.ui.lineEdit_17
        self.findcom = self.ui.pushButton
        self.opencom = self.ui.pushButton_2
        self.control = self.ui.pushButton_3
        self.opencam = self.ui.pushButton_4
        self.recognize = self.ui.pushButton_5
        self.cali = self.ui.pushButton_6
        self.reset = self.ui.pushButton_7
        self.clear = self.ui.pushButton_8
        self.closecam = self.ui.pushButton_9
        self.autocontrol = self.ui.pushButton_10
        self.recieve = self.ui.pushButton_11
        self.changecolor = self.ui.pushButton_13
        self.choose_com = self.ui.comboBox
        self.baudRate = self.ui.comboBox_2
        self.color1 = self.ui.comboBox_3
        self.color2 = self.ui.comboBox_4
        self.picture = self.ui.label_10
        self.hsvtab = self.ui.label_30
        self.ser = serial.Serial()
        self.ser.baudrate = int(self.baudRate.currentText())
        self.ser.bytesize = 8  # 设置数据位
        self.ser.stopbits = 1  # 设置停止位
        self.ser.parity = "N"  # 设置校验位
        self.search_com()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.getdata)
        self.stopEvent1 = threading.Event()
        self.stopEvent1.clear()
        self.stopEvent2 = threading.Event()
        self.stopEvent2.clear()
        pix = QPixmap('camera1.jpg')
        self.picture.setPixmap(pix)
        self.picture.setScaledContents(True)
        pix1 = QPixmap('hsv.png')
        self.hsvtab.setPixmap(pix1)
        self.hsvtab.setScaledContents(True)
        self.judge = False
        self.signal = False
        self.rc_signal = False
        self.count = 0
        self.ccount = [0, 0]
        self.w_count = 0
        self.w_ccount = [0, 0]
        self.c1_min1 = [0, 0, 0]
        self.c1_min2 = [0, 0, 0]
        self.c1_max1 = [0, 0, 0]
        self.c1_max2 = [0, 0, 0]
        self.c2_min1 = [0, 0, 0]
        self.c2_min2 = [0, 0, 0]
        self.c2_max1 = [0, 0, 0]
        self.c2_max2 = [0, 0, 0]
        self.c1_hmin.setText('0/156')
        self.c1_smin.setText('43/43')
        self.c1_vmin.setText('46/46')
        self.c1_hmax.setText('10/180')
        self.c1_smax.setText('255/255')
        self.c1_vmax.setText('255/255')
        self.c2_hmin.setText('110')
        self.c2_smin.setText('43')
        self.c2_vmin.setText('46')
        self.c2_hmax.setText('155')
        self.c2_smax.setText('255')
        self.c2_vmax.setText('255')
        self.change_hsv()
        # slider
        self.gl = self.ui.widget
        self.bslider = self.ui.horizontalSlider
        self.rslider = self.ui.horizontalSlider_2
        self.fslider = self.ui.horizontalSlider_3
        self.cslider = self.ui.horizontalSlider_4
        self.mode_choose = self.ui.comboBox_5

        self.mode_choose.currentIndexChanged.connect(self.modechange)
        self.bslider.valueChanged.connect(self.value_changed)
        self.bslider.sliderPressed.connect(self.mousep)
        self.bslider.sliderReleased.connect(self.mouser)
        self.rslider.valueChanged.connect(self.value_changed)
        self.rslider.sliderPressed.connect(self.mousep)
        self.rslider.sliderReleased.connect(self.mouser)
        self.fslider.valueChanged.connect(self.value_changed)
        self.fslider.sliderPressed.connect(self.mousep)
        self.fslider.sliderReleased.connect(self.mouser)
        self.cslider.valueChanged.connect(self.value_changed)
        self.cslider.sliderPressed.connect(self.mousep)
        self.cslider.sliderReleased.connect(self.mouser)
        self.base_angle.setPlainText(str(self.bslider.value() + 90))
        self.rArm_angle.setPlainText(str(-self.rslider.value() + 159))
        self.fArm_angle.setPlainText(str(self.fslider.value() + 145))
        self.claw_angle.setPlainText(str(int(self.cslider.value() / 2) + 65))
        self.state = False
        # 绑定槽函数
        self.findcom.clicked.connect(self.search_com)
        self.opencom.clicked.connect(self.openport)
        self.control.clicked.connect(self.control_servo)
        self.opencam.clicked.connect(self.connect_cam)
        self.recognize.clicked.connect(self.reco)
        self.cali.clicked.connect(self.calibration)
        self.reset.clicked.connect(self.reset_all)
        self.clear.clicked.connect(self.clear_all)
        self.closecam.clicked.connect(self.close_cam)
        self.autocontrol.clicked.connect(self.auto)
        self.recieve.clicked.connect(self.RC)
        self.changecolor.clicked.connect(self.change_hsv)

    # 函数
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

    def openport(self):
        self.ser.port = self.choose_com.currentText()
        self.ser.baudrate = int(self.baudRate.currentText())
        if self.ser.is_open:
            self.timer.stop()
            time.sleep(0.1)
            self.ser.close()
            self.opencom.setText('打开串口')
        else:
            self.ser.open()
            flag = self.ser.is_open
            if flag:
                # self.sensor_data.setPlainText("串口连接成功!")
                self.opencom.setText('关闭串口')
                self.timer.start(5)
                self.judge = False

    def change_hsv(self):
        color1data = self.color1.currentText()
        color2data = self.color2.currentText()
        if color1data == "red":
            a = self.c1_hmin.text().find('/')
            b = self.c1_smin.text().find('/')
            c = self.c1_vmin.text().find('/')
            d = self.c1_hmax.text().find('/')
            e = self.c1_smax.text().find('/')
            f = self.c1_vmax.text().find('/')
            self.c1_min1[0] = int(self.c1_hmin.text()[0: a])
            self.c1_min2[0] = int(self.c1_hmin.text()[a + 1:])
            self.c1_min1[1] = int(self.c1_smin.text()[0: b])
            self.c1_min2[1] = int(self.c1_smin.text()[b + 1:])
            self.c1_min1[2] = int(self.c1_vmin.text()[0: c])
            self.c1_min2[2] = int(self.c1_vmin.text()[c + 1:])
            self.c1_max1[0] = int(self.c1_hmax.text()[0: d])
            self.c1_max2[0] = int(self.c1_hmax.text()[d + 1:])
            self.c1_max1[1] = int(self.c1_smax.text()[0: e])
            self.c1_max2[1] = int(self.c1_smax.text()[e + 1:])
            self.c1_max1[2] = int(self.c1_vmax.text()[0: f])
            self.c1_max2[2] = int(self.c1_vmax.text()[f + 1:])
        else:
            self.c1_min1[0] = int(self.c1_hmin.text())
            self.c1_min1[1] = int(self.c1_smin.text())
            self.c1_min1[2] = int(self.c1_vmin.text())
            self.c1_max1[0] = int(self.c1_hmax.text())
            self.c1_max1[1] = int(self.c1_smax.text())
            self.c1_max1[2] = int(self.c1_vmax.text())
        if color2data == "red":
            a = self.c2_hmin.text().find('/')
            b = self.c2_smin.text().find('/')
            c = self.c2_vmin.text().find('/')
            d = self.c2_hmax.text().find('/')
            e = self.c2_smax.text().find('/')
            f = self.c2_vmax.text().find('/')
            self.c2_min1[0] = int(self.c2_hmin.text()[0: a])
            self.c2_min2[0] = int(self.c2_hmin.text()[a + 1:])
            self.c2_min1[1] = int(self.c2_smin.text()[0: b])
            self.c2_min2[1] = int(self.c2_smin.text()[b + 1:])
            self.c2_min1[2] = int(self.c2_vmin.text()[0: c])
            self.c2_min2[2] = int(self.c2_vmin.text()[c + 1:])
            self.c2_max1[0] = int(self.c2_hmax.text()[0: d])
            self.c2_max2[0] = int(self.c2_hmax.text()[d + 1:])
            self.c2_max1[1] = int(self.c2_smax.text()[0: e])
            self.c2_max2[1] = int(self.c2_smax.text()[e + 1:])
            self.c2_max1[2] = int(self.c2_vmax.text()[0: f])
            self.c2_max2[2] = int(self.c2_vmax.text()[f + 1:])
        else:
            self.c2_min1[0] = int(self.c2_hmin.text())
            self.c2_min1[1] = int(self.c2_smin.text())
            self.c2_min1[2] = int(self.c2_vmin.text())
            self.c2_max1[0] = int(self.c2_hmax.text())
            self.c2_max1[1] = int(self.c2_smax.text())
            self.c2_max1[2] = int(self.c2_vmax.text())

    def control_servo(self):
        if self.ser.is_open:
            input_s = 'b' + self.base_target.text() + 'r' + self.rArm_target.text() + 'f' + self.fArm_target.text() + 'c' + self.claw_target.text()
            self.ser.write((input_s + '\r\n').encode('UTF-8'))

    def connect_cam(self):
        self.cap = cv2.VideoCapture(self.camera_ip.text())
        th = threading.Thread(target=self.Display)
        th.start()

    def close_cam(self):
        self.stopEvent1.set()

    def Display(self):
        while self.cap.isOpened():
            success, frame = self.cap.read()
            if success:
                image = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                img = QImage(image.data, image.shape[1], image.shape[0], QImage.Format_RGB888)
                self.picture.setPixmap(QPixmap.fromImage(img))
                if self.color1.currentText() == "red":
                    lowerr1 = np.array([self.c1_min1[0], self.c1_min1[1], self.c1_min1[2]])
                    upperr1 = np.array([self.c1_max1[0], self.c1_max1[1], self.c1_max1[2]])
                    mask1 = cv2.inRange(HSV, lowerr1, upperr1)
                    ret1, thresh1 = cv2.threshold(mask1, 127, 255, 0)
                    lowerr2 = np.array([self.c1_min2[0], self.c1_min2[1], self.c1_min2[2]])
                    upperr2 = np.array([self.c1_max2[0], self.c1_max2[1], self.c1_max2[2]])
                    mask2 = cv2.inRange(HSV, lowerr2, upperr2)
                    ret2, thresh2 = cv2.threshold(mask2, 127, 255, 0)
                    thresh = thresh1 + thresh2
                else:
                    lowerr1 = np.array([self.c1_min1[0], self.c1_min1[1], self.c1_min1[2]])
                    upperr1 = np.array([self.c1_max1[0], self.c1_max1[1], self.c1_max1[2]])
                    mask1 = cv2.inRange(HSV, lowerr1, upperr1)
                    ret1, thresh1 = cv2.threshold(mask1, 127, 255, 0)
                    thresh = thresh1
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
                retr = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)
                retr = cv2.morphologyEx(retr, cv2.MORPH_CLOSE, kernel, iterations=3)
                cnts, hierarchy = cv2.findContours(retr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                for c in cnts:
                    (x, y, w, h) = cv2.boundingRect(c)
                    if w >= 40 and 40 <= h:
                        cv2.rectangle(frame, (x - 1, y - 1), (x + w - 1, y + h - 1), (0, 255, 0), 1)
                        cv2.putText(frame, self.color1.currentText(), (x - 5, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                (0, 255, 0), 2)
                        if self.judge:
                            input_s = 'red'
                            self.ser.write((input_s + '\r\n').encode('UTF-8'))
                            self.judge = False
                if self.color2.currentText() == "red":
                    lowery1 = np.array([self.c2_min1[0], self.c2_min1[1], self.c2_min1[2]])
                    uppery1 = np.array([self.c2_max1[0], self.c2_max1[1], self.c2_max1[2]])
                    mask3 = cv2.inRange(HSV, lowery1, uppery1)
                    ret3, thresh3 = cv2.threshold(mask3, 127, 255, 0)
                    lowery2 = np.array([self.c2_min2[0], self.c2_min2[1], self.c2_min2[2]])
                    uppery2 = np.array([self.c2_max2[0], self.c2_max2[1], self.c2_max2[2]])
                    mask4 = cv2.inRange(HSV, lowery2, uppery2)
                    ret4, thresh4 = cv2.threshold(mask4, 127, 255, 0)
                    thresh5 = thresh3 + thresh4
                else:
                    lowery1 = np.array([self.c2_min1[0], self.c2_min1[1], self.c2_min1[2]])
                    uppery1 = np.array([self.c2_max1[0], self.c2_max1[1], self.c2_max1[2]])
                    mask3 = cv2.inRange(HSV, lowery1, uppery1)
                    ret3, thresh3 = cv2.threshold(mask3, 127, 255, 0)
                    thresh5 = thresh3
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
                rety = cv2.morphologyEx(thresh5, cv2.MORPH_OPEN, kernel, iterations=2)
                rety = cv2.morphologyEx(rety, cv2.MORPH_CLOSE, kernel, iterations=3)
                cnts, hierarchy = cv2.findContours(rety, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                for c in cnts:
                    (x, y, w, h) = cv2.boundingRect(c)
                    if w >= 40 and 40 <= h:
                        cv2.rectangle(frame, (x - 1, y - 1), (x + w - 1, y + h - 1), (0, 255, 0), 1)
                        cv2.putText(frame, self.color2.currentText(), (x - 5, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                (0, 255, 0), 2)
                        if self.judge:
                            input_s = 'yellow'
                            self.ser.write((input_s + '\r\n').encode('UTF-8'))
                            self.judge = False
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                img = QImage(frame.data, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
                self.picture.setPixmap(QPixmap.fromImage(img))
                self.picture.setScaledContents(True)
                cv2.waitKey(1)
            if self.stopEvent1.is_set():
                # 关闭事件置为未触发，清空显示label
                self.stopEvent1.clear()
                self.picture.clear()
                pix = QPixmap('camera1.jpg')
                self.picture.setPixmap(pix)
                self.picture.setScaledContents(True)
                break

    def reco(self):
        if self.ser.is_open:
            input_s = 'reco'
            self.ser.write((input_s + '\r\n').encode('UTF-8'))

    def calibration(self):
        self.ccount[self.count] = int(self.Pressure.toPlainText())
        self.count = self.count + 1
        if self.count > 1:
            self.count = 0

    def auto(self):
        if self.ser.is_open:
            self.signal = not self.signal
            if self.signal:
                self.autocontrol.setText('停止')
                input_string = 'auto' + 'y' + str(self.ccount[0]) + 'z' + str(self.ccount[1])
                self.ser.write((input_string + '\r\n').encode('UTF-8'))
                time.sleep(1)
                input_string = 'status'
                self.ser.write((input_string + '\r\n').encode('UTF-8'))
            else:
                self.autocontrol.setText('传感器控制')
                input_string = 'over'
                self.ser.write((input_string + '\r\n').encode('UTF-8'))

    def getdata(self):
        if self.ser.is_open:
            self.rc_data = self.ser.read_all()
            if self.rc_data != b'':
                data = self.rc_data.decode('UTF-8')
                if data.find('ready') != -1:
                    self.judge = True
                elif data.find('sens') != -1:
                    ss0 = data.find('X')
                    ss1 = data.find('Z')
                    ss2 = data.find('P')
                    ss3 = data.find('W')
                    jd0 = data.find('b')
                    jd1 = data.find('r')
                    jd2 = data.find('f')
                    jd3 = data.find('c')
                    self.X_angle.setPlainText(data[ss0 + 1: ss1])
                    self.Z_angle.setPlainText(data[ss1 + 1: ss2])
                    self.Pressure.setPlainText(data[ss2 + 1: ss3])
                    self.W_data.setPlainText(data[ss3 + 1:jd0])
                    bvalue = int(data[jd0 + 1:jd1]) - 90
                    rvalue = int(data[jd1 + 1:jd2]) + 157
                    fvalue = int(data[jd2 + 1:jd3]) - 150
                    cvalue = (int(data[jd3 + 1:]) - 65) * 2
                    self.base_angle.setPlainText(str(bvalue))
                    self.rArm_angle.setPlainText(str(rvalue))
                    self.fArm_angle.setPlainText(str(fvalue))
                    self.claw_angle.setPlainText(str(cvalue))
                else:
                    jd0 = data.find('b')
                    jd1 = data.find('r')
                    jd2 = data.find('f')
                    jd3 = data.find('c')
                    self.base_angle.setPlainText(data[jd0 + 1: jd1])
                    self.rArm_angle.setPlainText(data[jd1 + 1: jd2])
                    self.fArm_angle.setPlainText(data[jd2 + 1: jd3])
                    self.claw_angle.setPlainText(data[jd3 + 1: jd3 + 3])
                    self.bslider.setValue(int(self.base_angle.toPlainText()) - 90)
                    self.rslider.setValue(-int(self.rArm_angle.toPlainText()) + 157)
                    self.fslider.setValue(int(self.fArm_angle.toPlainText()) - 150)
                    self.cslider.setValue((int(self.claw_angle.toPlainText()) - 65) * 2)
                    self.gl.schange(int(self.base_angle.toPlainText()) - 90, -int(self.rArm_angle.toPlainText()) + 157,
                                    int(self.fArm_angle.toPlainText()) - 150,
                                    (int(self.claw_angle.toPlainText()) - 65) * 2)

    def RC(self):
        if self.ser.is_open:
            self.rc_signal = not self.rc_signal
            if self.rc_signal:
                self.recieve.setText('停止')
                input_s = 'start'
                self.ser.write((input_s + '\r\n').encode('UTF-8'))
            else:
                self.recieve.setText('接收数据')
                input_s = 'stop'
                self.ser.write((input_s + '\r\n').encode('UTF-8'))

    def senddata(self):
        while self.ser.is_open:
            input_string = 'autob90r70f145c95'
            self.ser.write((input_string + '\r\n').encode('UTF-8'))
            time.sleep(0.2)
            if self.stopEvent2.is_set():
                self.stopEvent2.clear()
                break

    def reset_all(self):
        self.judge = False
        if self.ser.is_open:
            input_s = 'b90r70f145c95'
            self.ser.write((input_s + '\r\n').encode('UTF-8'))
        else:
            self.bslider.setValue(0)
            self.rslider.setValue(90)
            self.fslider.setValue(-5)
            self.cslider.setValue(60)
            self.base_angle.setPlainText(str(self.bslider.value() + 90))
            self.rArm_angle.setPlainText(str(-self.rslider.value() + 157))
            self.fArm_angle.setPlainText(str(self.fslider.value() + 145))
            self.claw_angle.setPlainText(str(int(self.cslider.value() / 2) + 65))
            self.gl.schange(self.bslider.value(), self.rslider.value(), self.fslider.value(), self.cslider.value())

    def clear_all(self):
        self.base_target.clear()
        self.rArm_target.clear()
        self.fArm_target.clear()
        self.claw_target.clear()
        self.camera_ip.clear()
        pix = QPixmap('camera1.jpg')
        self.picture.setPixmap(pix)
        self.picture.setScaledContents(True)

    def value_changed(self):
        if self.state:
            self.base_angle.setPlainText(str(self.bslider.value() + 90))
            self.rArm_angle.setPlainText(str(-self.rslider.value() + 157))
            self.fArm_angle.setPlainText(str(self.fslider.value() + 145))
            self.claw_angle.setPlainText(str(int(self.cslider.value() / 2) + 65))
            self.gl.schange(self.bslider.value(), self.rslider.value(), self.fslider.value(), self.cslider.value())
            if self.mode_choose.currentIndex() == 1:
                if self.ser.is_open:
                    input_s = 'b' + self.base_angle.toPlainText() + 'r' + self.rArm_angle.toPlainText() + 'f' + self.fArm_angle.toPlainText() + 'c' + self.claw_angle.toPlainText()
                    print(input_s)
                    self.ser.write((input_s + '\r\n').encode('UTF-8'))

    def mousep(self):
        self.state = True

    def mouser(self):
        self.state = False

    def modechange(self):
        if self.mode_choose.currentIndex() == 1:
            if self.ser.is_open:
                input_s = 'mode2'
                self.ser.write((input_s + '\r\n').encode('UTF-8'))
        else:
            if self.ser.is_open:
                input_s = 'mode1'
                self.ser.write((input_s + '\r\n').encode('UTF-8'))


if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = MyWindow()
    w.ui.show()
    app.exec()

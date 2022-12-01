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
import pyqtgraph
import Qopengl


class MyWindow(QWidget):

    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        self.ui = uic.loadUi("./tz.ui")
        self.dj = self.ui.pushButton
        self.mearm = self.ui.pushButton_2
        self.csb = self.ui.pushButton_3
        self.syn = self.ui.pushButton_4
        self.motor = self.ui.pushButton_5

        self.dj.clicked.connect(self.tztodj)
        self.mearm.clicked.connect(self.tztomearm)
        self.csb.clicked.connect(self.tztocsb)
        self.syn.clicked.connect(self.tztompu)
        self.motor.clicked.connect(self.tztopid)

    def tztodj(self):
        import servo  # 调用servo.py
        self.s = servo.MyWindow()  # 实例化servo.py中的MyWindow类
        self.s.ui.show()  # show()方法显示窗口

    def tztomearm(self):
        import mearmgl  # 调用servo.py
        self.m = mearmgl.MyWindow()  # 实例化servo.py中的MyWindow类
        self.m.ui.show()  # show()方法显示窗口

    def tztocsb(self):
        import csb  # 调用servo.py
        self.m = csb.MyWindow()  # 实例化servo.py中的MyWindow类
        self.m.ui.show()  # show()方法显示窗口

    def tztompu(self):
        import mpu  # 调用servo.py
        self.m = mpu.MyWindow()  # 实例化servo.py中的MyWindow类
        self.m.ui.show()  # show()方法显示窗口

    def tztopid(self):
        import pid  # 调用servo.py
        self.m = pid.MyWindow()  # 实例化servo.py中的MyWindow类
        self.m.ui.show()  # show()方法显示窗口


if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = MyWindow()
    w.ui.show()
    app.exec()

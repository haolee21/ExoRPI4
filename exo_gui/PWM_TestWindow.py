from PyQt5 import uic
from PyQt5.QtWidgets import QWidget,QLabel,QLineEdit,QPushButton
class PWM_TestWindow(QWidget):
    def __init__(self,parent=None):
        super().__init__()
        self.parent =parent
        uic.loadUi('PWM_TestFun.ui',self)
        self.btn_LKnePWM_start = self.findChild(QPushButton,'btn_LKnePWM_start')
        self.btn_LAnkPWM_start = self.findChild(QPushButton,'btn_LAnkPWM_start')
        self.btn_LTankPWM_start= self.findChild(QPushButton,'btn_LTankPWM_start')
        self.btn_RKnePWM_start = self.findChild(QPushButton,'btn_RKnePWM_start')
        self.btn_RAnkPWM_start = self.findChild(QPushButton,'btn_RAnkPWM_start')
        self.btn_RTankPWM_start=self.findChild(QPushButton,'btn_RTankPWM_start')

        self.btn_LKnePWM_stop = self.findChild(QPushButton,'btn_LKnePWM_stop')
        self.btn_LAnkPWM_stop = self.findChild(QPushButton,'btn_LAnkPWM_stop')
        self.btn_LTankPWM_stop= self.findChild(QPushButton,'btn_LTankPWM_stop')
        self.btn_RKnePWM_stop = self.findChild(QPushButton,'btn_RKnePWM_stop')
        self.btn_RAnkPWM_stop = self.findChild(QPushButton,'btn_RAnkPWM_stop')
        self.btn_RTankPWM_stop=self.findChild(QPushButton,'btn_RTankPWM_stop')

        self.parent=parent
        self.LKne_duty=self.findChild(QLineEdit,'lineEdit_LKneDuty')
        self.LAnk_duty=self.findChild(QLineEdit,'lineEdit_LAnkDuty')
        self.LTank_duty=self.findChild(QLineEdit,'lineEdit_LTankDuty')

        self.RKne_duty=self.findChild(QLineEdit,'lineEdit_RKneDuty')
        self.RAnk_duty=self.findChild(QLineEdit,'lineEdit_RAnkDuty')
        self.RTank_duty=self.findChild(QLineEdit,'lineEdit_RTankDuty')


        self.LKne_pre=self.findChild(QLineEdit,'lineEdit_LKnePre')
        self.LAnk_pre=self.findChild(QLineEdit,'lineEdit_LAnkPre')
        self.RKne_pre=self.findChild(QLineEdit,'lineEdit_RKnePre')
        self.RAnk_pre=self.findChild(QLineEdit,'lineEdit_RAnkPre')

        self.btn_LKnePreStr=self.findChild(QPushButton,'btn_LKneCharge_str')
        self.btn_LAnkPreStr=self.findChild(QPushButton,'btn_LAnkCharge_str')
        self.btn_LTankPreStr=self.findChild(QPushButton,'btn_RAnkCharge_str')
        self.btn_RKnePreStr=self.findChild(QPushButton,'btn_RKneCharge_str')
        self.btn_RAnkPreStr=self.findChild(QPushButton,'btn_RAnkCharge_str')
        self.btn_RTankPreStr=self.findChild(QPushButton,'btn_RTankCharge_str')

        self.btn_LKnePreRel=self.findChild(QPushButton,'btn_LKneCharge_rel')
        self.btn_LAnkPreRel=self.findChild(QPushButton,'btn_LAnkCharge_rel')
        self.btn_LTankPreRel=self.findChild(QPushButton,'btn_LTankCharge_rel')
        self.btn_RKnePreRel=self.findChild(QPushButton,'btn_RKneCharge_rel')
        self.btn_RAnkPreRel=self.findChild(QPushButton,'btn_RAnkCharge_rel')
        self.btn_RTankPreRel=self.findChild(QPushButton,'btn_RTankCharge_rel')

        #bind clicked and functions
        self.btn_LKnePWM_start.clicked.connect(self.btn_LKnePWM_start_clicked)
        self.btn_LAnkPWM_start.clicked.connect(self.btn_LAnkPWM_start_clicked)
        self.btn_LTankPWM_start.clicked.connect(self.btn_LTankPWM_start_clicked)
        self.btn_RKnePWM_start.clicked.connect(self.btn_RKnePWM_start_clicked)
        self.btn_RAnkPWM_start.clicked.connect(self.btn_RAnkPWM_start_clicked)
        self.btn_RTankPWM_start.clicked.connect(self.btn_RTankPWM_start_clicked)

        self.btn_LKnePWM_stop.clicked.connect(self.btn_LKnePWM_stop_clicked)
        self.btn_LAnkPWM_stop.clicked.connect(self.btn_LAnkPWM_stop_clicked)
        self.btn_LTankPWM_stop.clicked.connect(self.btn_LTankPWM_stop_clicked)
        self.btn_RKnePWM_stop.clicked.connect(self.btn_RKnePWM_stop_clicked)
        self.btn_RAnkPWM_stop.clicked.connect(self.btn_RAnkPWM_stop_clicked)
        self.btn_RTankPWM_stop.clicked.connect(self.btn_RTankPWM_stop_clicked)

    # btn clicked 
    def btn_LKnePWM_start_clicked(self):
        self.parent.tcp_port.SendCmd('SET:PWM:LKNE:'+str(int(float(self.LKne_duty.text()))),1,True)
    def btn_LAnkPWM_start_clicked(self):
        self.parent.tcp_port.SendCmd('SET:PWM:LANK:'+str(int(float(self.LAnk_duty.text()))),1,True)
    def btn_LTankPWM_start_clicked(self):
        self.parent.tcp_port.SendCmd('SET:PWM:LTANK:'+str(int(float(self.LTank_duty.text()))),1,True)
    def btn_RKnePWM_start_clicked(self):
        self.parent.tcp_port.SendCmd('SET:PWM:RKNE:'+str(int(float(self.RKne_duty.text()))),1,True)
    def btn_RAnkPWM_start_clicked(self):
        self.parent.tcp_port.SendCmd('SET:PWM:RANK:'+str(int(float(self.RAnk_duty.text()))),1,True)
    def btn_RTankPWM_start_clicked(self):
        self.parent.tcp_port.SendCmd('SET:PWM:RTANK:'+str(int(float(self.RTank_duty.text()))),1,True)

    def btn_LKnePWM_stop_clicked(self):
        self.parent.tcp_port.SendCmd('SET:PWM:LKNE:0',1,True)
    def btn_LAnkPWM_stop_clicked(self):
        self.parent.tcp_port.SendCmd('SET:PWM:LANK:0',1,True)
    def btn_LTankPWM_stop_clicked(self):
        self.parent.tcp_port.SendCmd('SET:PWM:LTANK:0',1,True)
    def btn_RKnePWM_stop_clicked(self):
        self.parent.tcp_port.SendCmd('SET:PWM:RKNE:0',1,True)
    def btn_RAnkPWM_stop_clicked(self):
        self.parent.tcp_port.SendCmd('SET:PWM:RANK:0',1,True)
    def btn_RTankPWM_stop_clicked(self):
        self.parent.tcp_port.SendCmd('SET:PWM:RTANK:0',1,True)
    
        
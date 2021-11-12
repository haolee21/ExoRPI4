from PyQt5 import uic
from PyQt5.QtWidgets import QWidget,QLabel,QLineEdit,QPushButton
class PWM_TestWindow(QWidget):
    def __init__(self,parent=None):
        super().__init__()
        self.parent =parent
        uic.loadUi('PWM_TestFun.ui',self)
        self.btn_LKnePWM = self.findChild(QPushButton,'btn_LKnePWM')
        self.btn_LAnkPWM = self.findChild(QPushButton,'btn_LAnkPWM')
        self.btn_RKnePWM = self.findChild(QPushButton,'btn_RKnePWM')
        self.btn_RAnkPWM = self.findChild(QPushButton,'btn_RAnkPWM')

        self.parent=parent
        self.LKne_duty=self.findChild(QLineEdit,'lineEdit_LKneDuty')
        self.LAnk_duty=self.findChild(QLineEdit,'lineEdit_LAnkDuty')
        self.RKne_duty=self.findChild(QLineEdit,'lineEdit_RKneDuty')
        self.RAnk_duty=self.findChild(QLineEdit,'lineEdit_RAnkDuty')


        self.LKne_pre=self.findChild(QLineEdit,'lineEdit_LKnePre')
        self.LAnk_pre=self.findChild(QLineEdit,'lineEdit_LAnkPre')
        self.RKne_pre=self.findChild(QLineEdit,'lineEdit_RKnePre')
        self.RAnk_pre=self.findChild(QLineEdit,'lineEdit_RAnkPre')

        self.btn_LKnePreStr=self.findChild(QPushButton,'btn_LKneCharge_str')
        self.btn_LAnkPreStr=self.findChild(QPushButton,'btn_LAnkCharge_str')
        self.btn_RKnePreStr=self.findChild(QPushButton,'btn_RKneCharge_str')
        self.btn_RAnkPreStr=self.findChild(QPushButton,'btn_RAnkCharge_str')

        self.btn_LKnePreRel=self.findChild(QPushButton,'btn_LKneCharge_rel')
        self.btn_LAnkPreRel=self.findChild(QPushButton,'btn_LAnkCharge_rel')
        self.btn_RKnePreRel=self.findChild(QPushButton,'btn_RKneCharge_rel')
        self.btn_RAnkPreRel=self.findChild(QPushButton,'btn_RAnkCharge_rel')
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget,QLabel,QLineEdit,QPushButton
from functools import partial
# from TCP_Con import TCP, TextToFloat
from UdpClient import *
from UdpClient import UdpClient
class PressureConWindow(QWidget):
    def __init__(self,parent=None):
        super().__init__()
        self.parent =parent
        uic.loadUi('PressureCon_window.ui',self)


        self.LKneExt_pre=self.findChild(QLineEdit,'lineEdit_LKneExtPre')
        self.LKneFlex_pre=self.findChild(QLineEdit,'lineEdit_LKneFlexPre')
        self.LAnkExt_pre=self.findChild(QLineEdit,'lineEdit_LAnkExtPre')
        self.LTank_pre = self.findChild(QLineEdit,'lineEdit_LTankPre')


        self.RKneExt_pre=self.findChild(QLineEdit,'lineEdit_RKneExtPre')
        self.RKneFlex_pre=self.findChild(QLineEdit,'lineEdit_RKneFlexPre')
        self.RAnkExt_pre=self.findChild(QLineEdit,'lineEdit_RAnkExtPre')
        self.RTank_pre = self.findChild(QLineEdit,'lineEdit_RTankPre')

        self.btn_LKneExtPre_start=self.findChild(QPushButton,'btn_LKneExtPre_start')
        self.btn_LKneFlexPre_start=self.findChild(QPushButton,'btn_LKneFlexPre_start')
        self.btn_LAnkExtPre_start = self.findChild(QPushButton,'btn_LAnkExtPre_start')
        self.btn_LTankPre_start = self.findChild(QPushButton,'btn_LTankPre_start')

        self.btn_RKneExtPre_start = self.findChild(QPushButton,'btn_RKneExtPre_start')
        self.btn_RKneFlexPre_start=self.findChild(QPushButton,'btn_RKneFlexPre_start')
        self.btn_RAnkExtPre_start=self.findChild(QPushButton,'btn_RAnkExtPre_start')
        self.btn_RTankPre_start=self.findChild(QPushButton,'btn_RTankPre_start')

        self.btn_LKneExtPre_stop=self.findChild(QPushButton,'btn_LKneExtPre_stop')
        self.btn_LKneFlexPre_stop=self.findChild(QPushButton,'btn_LKneFlexPre_stop')
        self.btn_LAnkExtPre_stop = self.findChild(QPushButton,'btn_LAnkExtPre_stop')
        self.btn_LTankPre_stop = self.findChild(QPushButton,'btn_LTankPre_stop')

        self.btn_RKneExtPre_stop = self.findChild(QPushButton,'btn_RKneExtPre_stop')
        self.btn_RKneFlexPre_stop=self.findChild(QPushButton,'btn_RKneFlexPre_stop')
        self.btn_RAnkExtPre_stop=self.findChild(QPushButton,'btn_RAnkExtPre_stop')
        self.btn_RTankPre_stop=self.findChild(QPushButton,'btn_RTankPre_stop')

        ## start
        self.btn_LKneExtPre_start.clicked.connect(partial(self.PreStartClicked,LKNE_EXT,self.LKneExt_pre))
        self.btn_LKneFlexPre_start.clicked.connect(partial(self.PreStartClicked,LKNE_FLEX,self.LKneFlex_pre))
        self.btn_LAnkExtPre_start.clicked.connect(partial(self.PreStartClicked,LANK_EXT,self.LAnkExt_pre))
        self.btn_LTankPre_start.clicked.connect(partial(self.PreStartClicked,LTANK,self.LTank_pre))
        
        self.btn_RKneExtPre_start.clicked.connect(partial(self.PreStartClicked,RKNE_EXT,self.RKneExt_pre))
        self.btn_RKneFlexPre_start.clicked.connect(partial(self.PreStartClicked,RKNE_FLEX,self.RKneFlex_pre))
        self.btn_RAnkExtPre_start.clicked.connect(partial(self.PreStartClicked,RANK_EXT,self.RAnkExt_pre))
        self.btn_RTankPre_start.clicked.connect(partial(self.PreStartClicked,RTANK,self.RTank_pre))
        ## stop

        self.btn_LKneExtPre_stop.clicked.connect(partial(self.DutyStopClicked,LKNE_EXT))
        self.btn_LKneFlexPre_stop.clicked.connect(partial(self.DutyStopClicked,LKNE_FLEX))
        self.btn_LAnkExtPre_stop.clicked.connect(partial(self.DutyStopClicked,LANK_EXT))
        self.btn_LTankPre_stop.clicked.connect(partial(self.DutyStopClicked,LTANK))

        self.btn_RKneExtPre_stop.clicked.connect(partial(self.DutyStopClicked,RKNE_EXT))
        self.btn_RKneFlexPre_stop.clicked.connect(partial(self.DutyStopClicked,RKNE_FLEX))
        self.btn_RAnkExtPre_stop.clicked.connect(partial(self.DutyStopClicked,RANK_EXT))
        self.btn_RTankPre_stop.clicked.connect(partial(self.DutyStopClicked,RTANK))



    def PreStartClicked(self,chamber_idx,pre_lineEdit):
        self.parent.udp_port.udp_cmd_packet.des_pre_data[chamber_idx]=TextToFloat(pre_lineEdit.text())
        self.parent.udp_port.udp_cmd_packet.des_pre_flag[chamber_idx]=True
    def DutyStopClicked(self,chamber_idx):
        self.parent.udp_port.udp_cmd_packet.pwm_duty_data[chamber_idx]=0
        self.parent.udp_port.udp_cmd_packet.pwm_duty_flag[chamber_idx]=True
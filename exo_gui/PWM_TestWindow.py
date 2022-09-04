from cgitb import text
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget,QLabel,QLineEdit,QPushButton
from functools import partial
# from TCP_Con import TCP, TextToFloat
from UdpClient import *
from UdpClient import UdpClient
class PWM_TestWindow(QWidget):
    def __init__(self,parent=None):
        super().__init__()
        self.parent =parent
        uic.loadUi('PWM_TestFun.ui',self)
        self.btn_LKneExtDuty_start = self.findChild(QPushButton,'btn_LKneExtDuty_start')
        self.btn_LKneFlexDuty_start=self.findChild(QPushButton,'btn_LKneFlexDuty_start')
        self.btn_LAnkExtDuty_start = self.findChild(QPushButton,'btn_LAnkExtDuty_start')
        self.btn_LAnkFlexDuty_start = self.findChild(QPushButton,'btn_LAnkFlexDuty_start')
        self.btn_LTankDuty_start= self.findChild(QPushButton,'btn_LTankDuty_start')

        
        self.btn_RKneExtDuty_start = self.findChild(QPushButton,'btn_RKneExtDuty_start')
        self.btn_RKneFlexDuty_start = self.findChild(QPushButton,'btn_RKneFlexDuty_start')
        self.btn_RAnkExtDuty_start = self.findChild(QPushButton,'btn_RAnkExtDuty_start')
        self.btn_RAnkFlexDuty_start = self.findChild(QPushButton,'btn_RAnkFlexDuty_start')
        self.btn_RTankDuty_start=self.findChild(QPushButton,'btn_RTankDuty_start')

        self.btn_LKneExtDuty_stop = self.findChild(QPushButton,'btn_LKneExtDuty_stop')
        self.btn_LKneFlexDuty_stop=self.findChild(QPushButton,'btn_LKneFlexDuty_stop')
        self.btn_LAnkExtDuty_stop = self.findChild(QPushButton,'btn_LAnkExtDuty_stop')
        self.btn_LAnkFlexDuty_stop = self.findChild(QPushButton,'btn_LAnkFlexDuty_stop')
        self.btn_LTankDuty_stop= self.findChild(QPushButton,'btn_LTankDuty_stop')

        self.btn_RKneExtDuty_stop = self.findChild(QPushButton,'btn_RKneExtDuty_stop')
        self.btn_RKneFlexDuty_stop=self.findChild(QPushButton,'btn_RKneFlexDuty_stop')
        self.btn_RAnkExtDuty_stop = self.findChild(QPushButton,'btn_RAnkExtDuty_stop')
        self.btn_RAnkFlexDuty_stop = self.findChild(QPushButton,'btn_RAnkFlexDuty_stop')
        self.btn_RTankDuty_stop= self.findChild(QPushButton,'btn_RTankDuty_stop')

        self.parent=parent
        self.LKneExt_duty=self.findChild(QLineEdit,'lineEdit_LKneExtDuty')
        self.LKneFlex_duty=self.findChild(QLineEdit,'lineEdit_LKneFlexDuty')
        self.LAnkExt_duty=self.findChild(QLineEdit,'lineEdit_LAnkExtDuty')
        self.LAnkFlex_duty=self.findChild(QLineEdit,'lineEdit_LAnkFlexDuty')
        self.LTank_duty=self.findChild(QLineEdit,'lineEdit_LTankDuty')

        self.RKneExt_duty=self.findChild(QLineEdit,'lineEdit_RKneExtDuty')
        self.RKneFlex_duty=self.findChild(QLineEdit,'lineEdit_RKneFlexDuty')
        self.RAnkExt_duty=self.findChild(QLineEdit,'lineEdit_RAnkExtDuty')
        self.RAnkFlex_duty=self.findChild(QLineEdit,'lineEdit_RAnkFlexDuty')
        self.RTank_duty=self.findChild(QLineEdit,'lineEdit_RTankDuty')


        


        self.LKneExt_pre=self.findChild(QLineEdit,'lineEdit_LKneExtPre')
        self.LKneFlex_pre=self.findChild(QLineEdit,'lineEdit_LKneFlexPre')
        self.LAnkExt_pre=self.findChild(QLineEdit,'lineEdit_LAnkExtPre')
        self.LAnkFlex_pre=self.findChild(QLineEdit,'lineEdit_LAnkFlexPre')
        self.LTank_pre = self.findChild(QLineEdit,'lineEdit_LTankPre')


        self.RKneExt_pre=self.findChild(QLineEdit,'lineEdit_RKneExtPre')
        self.RKneFlex_pre=self.findChild(QLineEdit,'lineEdit_RKneFlexPre')
        self.RAnkExt_pre=self.findChild(QLineEdit,'lineEdit_RAnkExtPre')
        self.RAnkFlex_pre=self.findChild(QLineEdit,'lineEdit_RAnkFlexPre')
        self.RTank_pre = self.findChild(QLineEdit,'lineEdit_RTankPre')

        


        self.btn_LKneExtPre_start=self.findChild(QPushButton,'btn_LKneExtPre_start')
        self.btn_LKneFlexPre_start=self.findChild(QPushButton,'btn_LKneFlexPre_start')
        self.btn_LAnkExtPre_start = self.findChild(QPushButton,'btn_LAnkExtPre_start')
        self.btn_LAnkFlexPre_start=self.findChild(QPushButton,'btn_LAnkFlexPre_start')
        self.btn_LTankPre_start = self.findChild(QPushButton,'btn_LTankPre_start')

        self.btn_RKneExtPre_start = self.findChild(QPushButton,'btn_RKneExtPre_start')
        self.btn_RKneFlexPre_start=self.findChild(QPushButton,'btn_RKneFlexPre_start')
        self.btn_RAnkExtPre_start=self.findChild(QPushButton,'btn_RAnkExtPre_start')
        self.btn_RAnkFlexPre_start=self.findChild(QPushButton,'btn_RAnkFlexPre_start')
        self.btn_RTankPre_start=self.findChild(QPushButton,'btn_RTankPre_start')

        self.btn_LKneExtPre_stop=self.findChild(QPushButton,'btn_LKneExtPre_stop')
        self.btn_LKneFlexPre_stop=self.findChild(QPushButton,'btn_LKneFlexPre_stop')
        self.btn_LAnkExtPre_stop = self.findChild(QPushButton,'btn_LAnkExtPre_stop')
        self.btn_LAnkFlexPre_stop=self.findChild(QPushButton,'btn_LAnkFlexPre_stop')
        self.btn_LTankPre_stop = self.findChild(QPushButton,'btn_LTankPre_stop')

        self.btn_RKneExtPre_stop = self.findChild(QPushButton,'btn_RKneExtPre_stop')
        self.btn_RKneFlexPre_stop=self.findChild(QPushButton,'btn_RKneFlexPre_stop')
        self.btn_RAnkExtPre_stop=self.findChild(QPushButton,'btn_RAnkExtPre_stop')
        self.btn_RAnkFlexPre_stop=self.findChild(QPushButton,'btn_RAnkFlexPre_stop')
        self.btn_RTankPre_stop=self.findChild(QPushButton,'btn_RTankPre_stop')


        #bind clicked and functions
        
        self.btn_LKneExtDuty_start.clicked.connect(partial(self.DutyStartClicked,'LKNE_EXT',self.LKneExt_duty))
        self.btn_LKneFlexDuty_start.clicked.connect(partial(self.DutyStartClicked,'LKNE_FLEX',self.LKneFlex_duty))
        self.btn_LAnkExtDuty_start.clicked.connect(partial(self.DutyStartClicked,'LANK_EXT',self.LAnkExt_duty))
        self.btn_LAnkFlexDuty_start.clicked.connect(partial(self.DutyStartClicked,'LANK_FLEX',self.LAnkFlex_duty))
        self.btn_LTankDuty_start.clicked.connect(partial(self.DutyStartClicked,'LTANK',self.LTank_duty))
        
        self.btn_RKneExtDuty_start.clicked.connect(partial(self.DutyStartClicked,'RKNE_EXT',self.RKneExt_duty))
        self.btn_RKneFlexDuty_start.clicked.connect(partial(self.DutyStartClicked,'RKNE_FLEX',self.RKneFlex_duty))
        self.btn_RAnkExtDuty_start.clicked.connect(partial(self.DutyStartClicked,'RANK_EXT',self.RAnkExt_duty))
        self.btn_RAnkFlexDuty_start.clicked.connect(partial(self.DutyStartClicked,'RANK_FLEX',self.RAnkFlex_duty))
        self.btn_RTankDuty_start.clicked.connect(partial(self.DutyStartClicked,'RTANK',self.RTank_duty))

        self.btn_LKneExtDuty_stop.clicked.connect(partial(self.DutyStopClicked,'LKNE_EXT'))
        self.btn_LKneFlexDuty_stop.clicked.connect(partial(self.DutyStopClicked,'LKNE_FLEX'))
        self.btn_LAnkExtDuty_stop.clicked.connect(partial(self.DutyStopClicked,'LANK_EXT'))
        self.btn_LAnkFlexDuty_stop.clicked.connect(partial(self.DutyStopClicked,'LANK_FLEX'))
        self.btn_LTankDuty_stop.clicked.connect(partial(self.DutyStopClicked,'LTANK'))

        self.btn_RKneExtDuty_stop.clicked.connect(partial(self.DutyStopClicked,'RKNE_EXT'))
        self.btn_RKneFlexDuty_stop.clicked.connect(partial(self.DutyStopClicked,'RKNE_FLEX'))
        self.btn_RAnkExtDuty_stop.clicked.connect(partial(self.DutyStopClicked,'RANK_EXT'))
        self.btn_RAnkFlexDuty_stop.clicked.connect(partial(self.DutyStopClicked,'RANK_FLEX'))
        self.btn_RTankDuty_stop.clicked.connect(partial(self.DutyStopClicked,'RTANK'))


        #pressure control

        ## start
        self.btn_LKneExtPre_start.clicked.connect(partial(self.PreStartClicked,'LKNE_EXT',self.LKneExt_pre))
        self.btn_LKneFlexPre_start.clicked.connect(partial(self.PreStartClicked,'LKNE_FLEX',self.LKneFlex_pre))
        self.btn_LAnkExtPre_start.clicked.connect(partial(self.PreStartClicked,'LANK_EXT',self.LAnkExt_pre))
        self.btn_LAnkFlexPre_start.clicked.connect(partial(self.PreStartClicked,'LANK_FLEX',self.LAnkFlex_pre))
        self.btn_LTankPre_start.clicked.connect(partial(self.PreStartClicked,'LTANK',self.LTank_pre))
        
        self.btn_RKneExtPre_start.clicked.connect(partial(self.PreStartClicked,'RKNE_EXT',self.RKneExt_pre))
        self.btn_RKneFlexPre_start.clicked.connect(partial(self.PreStartClicked,'RKNE_FLEX',self.RKneFlex_pre))
        self.btn_RAnkExtPre_start.clicked.connect(partial(self.PreStartClicked,'RANK_EXT',self.RAnkExt_pre))
        self.btn_RAnkFlexPre_start.clicked.connect(partial(self.PreStartClicked,'RANK_FLEX',self.RAnkFlex_pre))
        self.btn_RTankPre_start.clicked.connect(partial(self.PreStartClicked,'RTANK',self.RTank_pre))
        ## stop

        self.btn_LKneExtPre_stop.clicked.connect(partial(self.DutyStopClicked,'LKNE_EXT'))
        self.btn_LKneFlexPre_stop.clicked.connect(partial(self.DutyStopClicked,'LKNE_FLEX'))
        self.btn_LAnkExtPre_stop.clicked.connect(partial(self.DutyStopClicked,'LANK_EXT'))
        self.btn_LAnkFlexPre_stop.clicked.connect(partial(self.DutyStopClicked,'LANK_FLEX'))
        self.btn_LTankPre_stop.clicked.connect(partial(self.DutyStopClicked,'LTANK'))

        self.btn_RKneExtPre_stop.clicked.connect(partial(self.DutyStopClicked,'RKNE_EXT'))
        self.btn_RKneFlexPre_stop.clicked.connect(partial(self.DutyStopClicked,'RKNE_FLEX'))
        self.btn_RAnkExtPre_stop.clicked.connect(partial(self.DutyStopClicked,'RANK_EXT'))
        self.btn_RAnkFlexPre_stop.clicked.connect(partial(self.DutyStopClicked,'RANK_FLEX'))
        self.btn_RTankPre_stop.clicked.connect(partial(self.DutyStopClicked,'RTANK'))
        
        
    # btn clicked


    def DutyStartClicked(self,chamber_idx,duty_lineEdit):
        self.udp_port = UdpClient()
        self.udp_port.udp_cmd_packet.pwm_duty_data[chamber_idx] = TextToFloat(duty_lineEdit.text())
        self.udp_port.udp_cmd_packet.pwm_duty_flag[chamber_idx] = True
        
    def DutyStopClicked(self,chamber_idx):
        self.udp_port.udp_cmd_packet.pwm_duty_data[chamber_idx]=0
        self.udp_port.udp_cmd_packet.pwm_duty_flag[chamber_idx]=True
        self.udp_port.udp_cmd_packet.con_on_off_data[chamber_idx]=False
        self.udp_port.udp_cmd_packet.con_on_off_flag[chamber_idx]=True

    def PreStartClicked(self,chamber_idx,pre_lineEdit):
        self.udp_port.udp_cmd_packet.des_pre_data[chamber_idx]=TextToFloat(pre_lineEdit.text())
        self.udp_port.udp_cmd_packet.des_pre_flag[chamber_idx]=True
    
        
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
        self.btn_LKneAnkDuty_start = self.findChild(QPushButton,'btn_LKneAnkDuty_start')
        self.btn_LKneExutDuty_start = self.findChild(QPushButton,'btn_LKneExutDuty_start')
        self.btn_LAnkExutDuty_start = self.findChild(QPushButton,'btn_LAnkExutDuty_start')
        self.btn_LTankDuty_start= self.findChild(QPushButton,'btn_LTankDuty_start')

        self.btn_RKneExtDuty_start = self.findChild(QPushButton,'btn_RKneExtDuty_start')
        self.btn_RKneFlexDuty_start=self.findChild(QPushButton,'btn_RKneFlexDuty_start')
        self.btn_RAnkExtDuty_start = self.findChild(QPushButton,'btn_RAnkExtDuty_start')
        self.btn_RKneAnkDuty_start = self.findChild(QPushButton,'btn_RKneAnkDuty_start')
        self.btn_RKneExutDuty_start = self.findChild(QPushButton,'btn_RKneExutDuty_start')
        self.btn_RAnkExutDuty_start = self.findChild(QPushButton,'btn_RAnkExutDuty_start')
        self.btn_RTankDuty_start= self.findChild(QPushButton,'btn_RTankDuty_start')

        self.btn_LKneExtDuty_stop = self.findChild(QPushButton,'btn_LKneExtDuty_stop')
        self.btn_LKneFlexDuty_stop=self.findChild(QPushButton,'btn_LKneFlexDuty_stop')
        self.btn_LAnkExtDuty_stop = self.findChild(QPushButton,'btn_LAnkExtDuty_stop')
        self.btn_LKneAnkDuty_stop = self.findChild(QPushButton,'btn_LKneAnkDuty_stop')
        self.btn_LKneExutDuty_stop = self.findChild(QPushButton,'btn_LKneExutDuty_stop')
        self.btn_LAnkExutDuty_stop = self.findChild(QPushButton,'btn_LAnkExutDuty_stop')
        self.btn_LTankDuty_stop= self.findChild(QPushButton,'btn_LTankDuty_stop')

        self.btn_RKneExtDuty_stop = self.findChild(QPushButton,'btn_RKneExtDuty_stop')
        self.btn_RKneFlexDuty_stop=self.findChild(QPushButton,'btn_RKneFlexDuty_stop')
        self.btn_RAnkExtDuty_stop = self.findChild(QPushButton,'btn_RAnkExtDuty_stop')
        self.btn_RKneAnkDuty_stop = self.findChild(QPushButton,'btn_RKneAnkDuty_stop')
        self.btn_RKneExutDuty_stop = self.findChild(QPushButton,'btn_RKneExutDuty_stop')
        self.btn_RAnkExutDuty_stop = self.findChild(QPushButton,'btn_RAnkExutDuty_stop')
        self.btn_RTankDuty_stop= self.findChild(QPushButton,'btn_RTankDuty_stop')
        

        self.LKneExt_duty=self.findChild(QLineEdit,'lineEdit_LKneExtDuty')
        self.LKneFlex_duty=self.findChild(QLineEdit,'lineEdit_LKneFlexDuty')
        self.LAnkExt_duty=self.findChild(QLineEdit,'lineEdit_LAnkExtDuty')
        self.LKneAnk_duty=self.findChild(QLineEdit,'lineEdit_LKneAnkDuty')
        self.LKneExut_duty=self.findChild(QLineEdit,'lineEdit_LKneExutDuty')
        self.LAnkExut_duty=self.findChild(QLineEdit,'lineEdit_LAnkExutDuty')
        self.LTank_duty=self.findChild(QLineEdit,'lineEdit_LTankDuty')

        self.RKneExt_duty=self.findChild(QLineEdit,'lineEdit_RKneExtDuty')
        self.RKneFlex_duty=self.findChild(QLineEdit,'lineEdit_RKneFlexDuty')
        self.RAnkExt_duty=self.findChild(QLineEdit,'lineEdit_RAnkExtDuty')
        self.RKneAnk_duty=self.findChild(QLineEdit,'lineEdit_RKneAnkDuty')
        self.RKneExut_duty=self.findChild(QLineEdit,'lineEdit_RKneExutDuty')
        self.RAnkExut_duty=self.findChild(QLineEdit,'lineEdit_RAnkExutDuty')
        self.RTank_duty=self.findChild(QLineEdit,'lineEdit_RTankDuty')


        


        


        #bind clicked and functions
        
        self.btn_LKneExtDuty_start.clicked.connect(partial(self.DutyStartClicked,LKNE_EXT_PWM,self.LKneExt_duty))
        self.btn_LKneFlexDuty_start.clicked.connect(partial(self.DutyStartClicked,LKNE_FLEX_PWM,self.LKneFlex_duty))
        self.btn_LAnkExtDuty_start.clicked.connect(partial(self.DutyStartClicked,LANK_EXT_PWM,self.LAnkExt_duty))
        self.btn_LKneAnkDuty_start.clicked.connect(partial(self.DutyStartClicked,LKNE_ANK_PWM,self.LKneAnk_duty))
        self.btn_LKneExutDuty_start.clicked.connect(partial(self.DutyStartClicked,LKNE_EXUT_PWM,self.LKneExut_duty))
        self.btn_LAnkExutDuty_start.clicked.connect(partial(self.DutyStartClicked,LANK_EXUT_PWM,self.LAnkExut_duty))
        self.btn_LTankDuty_start.clicked.connect(partial(self.DutyStartClicked,LTANK_PWM,self.LTank_duty))

        self.btn_RKneExtDuty_start.clicked.connect(partial(self.DutyStartClicked,RKNE_EXT_PWM,self.RKneExt_duty))
        self.btn_RKneFlexDuty_start.clicked.connect(partial(self.DutyStartClicked,RKNE_FLEX_PWM,self.RKneFlex_duty))
        self.btn_RAnkExtDuty_start.clicked.connect(partial(self.DutyStartClicked,RANK_EXT_PWM,self.RAnkExt_duty))
        self.btn_RKneAnkDuty_start.clicked.connect(partial(self.DutyStartClicked,RKNE_ANK_PWM,self.RKneAnk_duty))
        self.btn_RKneExutDuty_start.clicked.connect(partial(self.DutyStartClicked,RKNE_EXUT_PWM,self.RKneExut_duty))
        self.btn_RAnkExutDuty_start.clicked.connect(partial(self.DutyStartClicked,RANK_EXUT_PWM,self.RAnkExut_duty))
        self.btn_RTankDuty_start.clicked.connect(partial(self.DutyStartClicked,RTANK_PWM,self.RTank_duty))


        self.btn_LKneExtDuty_stop.clicked.connect(partial(self.DutyStopClicked,LKNE_EXT_PWM))
        self.btn_LKneFlexDuty_stop.clicked.connect(partial(self.DutyStopClicked,LKNE_FLEX_PWM))
        self.btn_LAnkExtDuty_stop.clicked.connect(partial(self.DutyStopClicked,LANK_EXT_PWM))
        self.btn_LKneAnkDuty_stop.clicked.connect(partial(self.DutyStopClicked,LKNE_ANK_PWM))
        self.btn_LKneExutDuty_stop.clicked.connect(partial(self.DutyStopClicked,LKNE_EXUT_PWM))
        self.btn_LAnkExutDuty_stop.clicked.connect(partial(self.DutyStopClicked,LANK_EXUT_PWM))
        self.btn_LTankDuty_stop.clicked.connect(partial(self.DutyStopClicked,LTANK_PWM))

        self.btn_RKneExtDuty_stop.clicked.connect(partial(self.DutyStopClicked,RKNE_EXT_PWM))
        self.btn_RKneFlexDuty_stop.clicked.connect(partial(self.DutyStopClicked,RKNE_FLEX_PWM))
        self.btn_RAnkExtDuty_stop.clicked.connect(partial(self.DutyStopClicked,RANK_EXT_PWM))
        self.btn_RKneAnkDuty_stop.clicked.connect(partial(self.DutyStopClicked,RKNE_ANK_PWM))
        self.btn_RKneExutDuty_stop.clicked.connect(partial(self.DutyStopClicked,RKNE_EXUT_PWM))
        self.btn_RAnkExutDuty_stop.clicked.connect(partial(self.DutyStopClicked,RANK_EXUT_PWM))
        self.btn_RTankDuty_stop.clicked.connect(partial(self.DutyStopClicked,RTANK_PWM))


        
        
        
    # btn clicked


    def DutyStartClicked(self,chamber_idx,duty_lineEdit):
       
        self.parent.udp_port.udp_cmd_packet.pwm_duty_data[chamber_idx] = TextToInt(duty_lineEdit.text())
        self.parent.udp_port.udp_cmd_packet.pwm_duty_flag[chamber_idx] = True
        
        
        
    def DutyStopClicked(self,chamber_idx):
        self.parent.udp_port.udp_cmd_packet.pwm_duty_data[chamber_idx]=0
        self.parent.udp_port.udp_cmd_packet.pwm_duty_flag[chamber_idx]=True

    
        
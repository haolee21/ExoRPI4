from PyQt5 import uic
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QLineEdit,QPushButton,QDialog
from functools import partial
# from TCP_Con import TCP, TextToFloat
from UdpClient import *
from Common import *
class PressureConWindow(QDialog):
    def __init__(self,parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.Window) # if you don't set it, it will not show up in alt-tab (for QDialog)
        uic.loadUi('UI/PressureCon_window.ui',self)


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
        self.btn_LKneExtPre_start.clicked.connect(partial(self.PreStartClicked,Chamber.KneExt.value,self.LKneExt_pre,True))
        self.btn_LKneFlexPre_start.clicked.connect(partial(self.PreStartClicked,Chamber.KneFlex.value,self.LKneFlex_pre,True))
        self.btn_LAnkExtPre_start.clicked.connect(partial(self.PreStartClicked,Chamber.AnkPla.value,self.LAnkExt_pre,False))
        self.btn_LTankPre_start.clicked.connect(partial(self.PreStartClicked,Chamber.SubTank.value,self.LTank_pre,True))
        
        self.btn_RKneExtPre_start.clicked.connect(partial(self.PreStartClicked,Chamber.KneExt.value,self.RKneExt_pre,False))
        self.btn_RKneFlexPre_start.clicked.connect(partial(self.PreStartClicked,Chamber.KneFlex.value,self.RKneFlex_pre,False))
        self.btn_RAnkExtPre_start.clicked.connect(partial(self.PreStartClicked,Chamber.AnkPla.value,self.RAnkExt_pre,True))
        self.btn_RTankPre_start.clicked.connect(partial(self.PreStartClicked,Chamber.SubTank.value,self.RTank_pre,False))
        ## stop

        self.btn_LKneExtPre_stop.clicked.connect(partial(self.DutyStopClicked,LKRA))
        self.btn_LKneFlexPre_stop.clicked.connect(partial(self.DutyStopClicked,LKRA))
        self.btn_LAnkExtPre_stop.clicked.connect(partial(self.DutyStopClicked,RKLA))
        self.btn_LTankPre_stop.clicked.connect(partial(self.DutyStopClicked,LKRA))

        self.btn_RKneExtPre_stop.clicked.connect(partial(self.DutyStopClicked,RKLA))
        self.btn_RKneFlexPre_stop.clicked.connect(partial(self.DutyStopClicked,RKLA))
        self.btn_RAnkExtPre_stop.clicked.connect(partial(self.DutyStopClicked,LKRA))
        self.btn_RTankPre_stop.clicked.connect(partial(self.DutyStopClicked,RKLA))



    def PreStartClicked(self,chamber_idx,pre_lineEdit,is_lkra):
        with self.parent().udp_port.lock:
            if is_lkra:    
                pre_data = self.parent().udp_port.udp_cmd_packet.lkra_des_pre_data
                pre_flag = self.parent().udp_port.udp_cmd_packet.lkra_des_pre_flag
            else:
                pre_data = self.parent().udp_port.udp_cmd_packet.rkla_des_pre_data
                pre_flag = self.parent().udp_port.udp_cmd_packet.rkla_des_pre_flag
            
            pre_data[chamber_idx]=TextToFloat(pre_lineEdit.text())
            pre_flag[chamber_idx]=True
    def DutyStopClicked(self,chamber_idx):
        with self.parent().udp_port.lock:
            self.parent().udp_port.udp_cmd_packet.con_on_off_data[chamber_idx] = False
            self.parent().udp_port.udp_cmd_packet.con_on_off_flag[chamber_idx] = True
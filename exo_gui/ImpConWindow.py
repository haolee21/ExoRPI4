from pickle import FALSE
from socketserver import UDPServer
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget,QLabel,QLineEdit,QPushButton
from PyQt5.QtGui import QPalette
from PyQt5.QtGui import QColor
from PyQt5.QtCore import Qt
from functools import partial
# from TCP_Con import TextToFloat
from UdpClient import *
class ImpWindow(QWidget):
    def __init__(self, parent= None):
        super().__init__()
        self.parent = parent
        uic.loadUi("Imp_window.ui",self)
        self.btn_lkne_set = self.findChild(QPushButton,'btn_lkne_set')
        self.btn_lkne_stop = self.findChild(QPushButton,'btn_lkne_stop')
        self.btn_lank_set = self.findChild(QPushButton,'btn_lank_set')
        self.btn_lank_stop = self.findChild(QPushButton,'btn_lank_stop')
        self.btn_rkne_set = self.findChild(QPushButton,'btn_rkne_set')
        self.btn_rkne_stop = self.findChild(QPushButton,'btn_rkne_stop')
        self.btn_rank_set = self.findChild(QPushButton,'btn_rank_set')
        self.btn_rank_stop = self.findChild(QPushButton,'btn_rank_stop')

        self.lineEdit_lkne = self.findChild(QLineEdit,'lineEdit_lkne')
        self.lineEdit_lank = self.findChild(QLineEdit,'lineEdit_lank')
        self.lineEdit_rkne = self.findChild(QLineEdit,'lineEdit_rkne')
        self.lineEdit_rank = self.findChild(QLineEdit,'lineEdit_rank')
        
        self.lineEdit_lkne.textChanged.connect(partial(self.TextChange,self.lineEdit_lkne))
        self.btn_lkne_set.clicked.connect(partial(self.ImpSet,self.lineEdit_lkne,JOINT_LKNE))
        self.btn_lkne_stop.clicked.connect(partial(self.ImpStop,JOINT_LKNE))

        self.lineEdit_lank.textChanged.connect(partial(self.TextChange,self.lineEdit_lank))
        self.btn_lank_set.clicked.connect(partial(self.ImpSet,self.lineEdit_lank,JOINT_LANK))
        self.btn_lank_stop.clicked.connect(partial(self.ImpStop,JOINT_LANK))

        self.lineEdit_rkne.textChanged.connect(partial(self.TextChange,self.lineEdit_rkne))
        self.btn_rkne_set.clicked.connect(partial(self.ImpSet,self.lineEdit_rkne,JOINT_RKNE))
        self.btn_rkne_stop.clicked.connect(partial(self.ImpStop,JOINT_RKNE))

        self.lineEdit_rank.textChanged.connect(partial(self.TextChange,self.lineEdit_rank))
        self.btn_rank_set.clicked.connect(partial(self.ImpSet,self.lineEdit_rank,JOINT_RANK))
        self.btn_rank_stop.clicked.connect(partial(self.ImpStop,JOINT_RANK))
       

    def TextChange(self,line_edit):
        palette = QPalette()
        palette.setColor(QPalette.Text,Qt.blue)
        line_edit.setPalette(palette)
    def ImpSet(self,line_edit,joint_idx):
        palette = QPalette()
        palette.setColor(QPalette.Text,Qt.black)
        line_edit.setPalette(palette)
        self.parent.udp_port.udp_cmd_packet.des_imp_data[joint_idx]=TextToFloat(line_edit.text())
        self.parent.udp_port.udp_cmd_packet.des_imp_flag[joint_idx]=True
    def ImpStop(self,joint_idx):
        
        self.parent.udp_port.udp_cmd_packet.con_on_off_data[joint_idx]=False
        self.parent.udp_port.udp_cmd_packet.con_on_off_flag[joint_idx]=True
        # self.parent.udp_port.udp_cmd_packet.pwm_duty_data[chamber_idx]=0
        # self.parent.udp_port.udp_cmd_packet.pwm_duty_flag[chamber_idx]=True

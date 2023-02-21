# from re import T
from PyQt5 import uic
from PyQt5.QtWidgets import QLineEdit,QPushButton,QDialog
from PyQt5.QtGui import QPalette
from PyQt5.QtGui import QColor
from PyQt5.QtCore import Qt
from functools import partial
from UdpClient import *
from Common import *
class ImpactCon(QDialog):
    def __init__(self,parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.Window)
        uic.loadUi('UI/ImpactCon.ui',self)

        self.lineEdit_lkne_init_f = self.findChild(QLineEdit,'lineEdit_lkne_init_f')
        self.lineEdit_lank_init_f = self.findChild(QLineEdit,'lineEdit_lank_init_f')
        self.lineEdit_rkne_init_f = self.findChild(QLineEdit,'lineEdit_rkne_init_f')
        self.lineEdit_rank_init_f = self.findChild(QLineEdit,'lineEdit_rank_init_f')

        self.lineEdit_lkne_impact_imp = self.findChild(QLineEdit,'lineEdit_lkne_impact_imp')
        self.lineEdit_lank_impact_imp = self.findChild(QLineEdit,'lineEdit_lank_impact_imp')
        self.lineEdit_rkne_impact_imp = self.findChild(QLineEdit,'lineEdit_rkne_impact_imp')
        self.lineEdit_rank_impact_imp = self.findChild(QLineEdit,'lineEdit_rank_impact_imp')

        self.lineEdit_lkne_restore_imp = self.findChild(QLineEdit,'lineEdit_lkne_restore_imp')
        self.lineEdit_lank_restore_imp = self.findChild(QLineEdit,'lineEdit_lank_restore_imp')
        self.lineEdit_rkne_restore_imp = self.findChild(QLineEdit,'lineEdit_rkne_restore_imp')
        self.lineEdit_rank_restore_imp = self.findChild(QLineEdit,'lineEdit_rank_restore_imp')

        self.btn_lkne_reset = self.findChild(QPushButton,'btn_lkne_reset')
        self.btn_lank_reset = self.findChild(QPushButton,'btn_lank_reset')
        self.btn_rkne_reset = self.findChild(QPushButton,'btn_rkne_reset')
        self.btn_rank_reset = self.findChild(QPushButton,'btn_rank_reset')

        self.btn_lkne_off = self.findChild(QPushButton,'btn_lkne_off')
        self.btn_lank_off = self.findChild(QPushButton,'btn_lank_off')
        self.btn_rkne_off = self.findChild(QPushButton,'btn_rkne_off')
        self.btn_rank_off = self.findChild(QPushButton,'btn_rank_off')


        self.btn_lkne_reset.clicked.connect(partial(self.ResetCylinder,self.lineEdit_lkne_init_f,self.lineEdit_lkne_impact_imp,self.lineEdit_lkne_restore_imp,LKRA*2+FORCE_CON_KNE_EXT))
        self.btn_lank_reset.clicked.connect(partial(self.ResetCylinder,self.lineEdit_lank_init_f,self.lineEdit_lank_impact_imp,self.lineEdit_lank_restore_imp,RKLA*2+FORCE_CON_ANK_PLANT))
        self.btn_rkne_reset.clicked.connect(partial(self.ResetCylinder,self.lineEdit_rkne_init_f,self.lineEdit_rkne_impact_imp,self.lineEdit_rkne_restore_imp,RKLA*2+FORCE_CON_KNE_EXT))
        self.btn_rank_reset.clicked.connect(partial(self.ResetCylinder,self.lineEdit_rank_init_f,self.lineEdit_rank_impact_imp,self.lineEdit_rank_restore_imp,LKRA*2+FORCE_CON_ANK_PLANT))

        self.btn_lkne_off.clicked.connect(partial(self.ImpactConOff,LKRA*2+FORCE_CON_KNE_EXT))
        self.btn_lank_off.clicked.connect(partial(self.ImpactConOff,RKLA*2+FORCE_CON_ANK_PLANT))
        self.btn_rkne_off.clicked.connect(partial(self.ImpactConOff,RKLA*2+FORCE_CON_KNE_EXT))
        self.btn_rank_off.clicked.connect(partial(self.ImpactConOff,LKRA*2+FORCE_CON_ANK_PLANT))

    def ResetCylinder(self,lineEdit_init_f, lineEdit_impact_imp, lineEdit_restore_imp,joint_idx):
        self.parent().udp_port.udp_cmd_packet.init_force[joint_idx] = TextToFloat(lineEdit_init_f.text())
        self.parent().udp_port.udp_cmd_packet.init_impact_imp[joint_idx] = TextToFloat(lineEdit_impact_imp.text())
        self.parent().udp_port.udp_cmd_packet.restore_imp[joint_idx]=TextToFloat(lineEdit_restore_imp.text())
        self.parent().udp_port.udp_cmd_packet.impact_absorb_flag[joint_idx]=True
    def ImpactConOff(self,joint_idx):

        if joint_idx == LKRA*2+FORCE_CON_KNE_EXT:
            self.parent().udp_port.udp_cmd_packet.pwm_duty_data[LKNE_EXT_PWM]=0
            self.parent().udp_port.udp_cmd_packet.pwm_duty_data[RKNE_LANK_PWM]=0
            self.parent().udp_port.udp_cmd_packet.pwm_duty_data[LTANK_PWM]=0
            self.parent().udp_port.udp_cmd_packet.pwm_duty_flag[LKNE_EXT_PWM]=True
            self.parent().udp_port.udp_cmd_packet.pwm_duty_flag[RKNE_LANK_PWM]=True
            self.parent().udp_port.udp_cmd_packet.pwm_duty_flag[LTANK_PWM]=True
        elif joint_idx == RKLA*2+FORCE_CON_ANK_PLANT:
            self.parent().udp_port.udp_cmd_packet.pwm_duty_data[LANK_EXT_PWM]=0
            self.parent().udp_port.udp_cmd_packet.pwm_duty_data[RKNE_LANK_PWM]=0
            self.parent().udp_port.udp_cmd_packet.pwm_duty_data[LTANK_PWM]=0
            self.parent().udp_port.udp_cmd_packet.pwm_duty_flag[LANK_EXT_PWM]=True
            self.parent().udp_port.udp_cmd_packet.pwm_duty_flag[RKNE_LANK_PWM]=True
            self.parent().udp_port.udp_cmd_packet.pwm_duty_flag[LTANK_PWM]=True
        elif joint_idx == RKLA*2+FORCE_CON_KNE_EXT:
            self.parent().udp_port.udp_cmd_packet.pwm_duty_data[RKNE_EXT_PWM]=0
            self.parent().udp_port.udp_cmd_packet.pwm_duty_data[LKNE_RANK_PWM]=0
            self.parent().udp_port.udp_cmd_packet.pwm_duty_data[RTANK_PWM]=0

            self.parent().udp_port.udp_cmd_packet.pwm_duty_flag[RKNE_EXT_PWM]=True
            self.parent().udp_port.udp_cmd_packet.pwm_duty_flag[LKNE_RANK_PWM]=True
            self.parent().udp_port.udp_cmd_packet.pwm_duty_flag[RTANK_PWM]=True
        elif joint_idx == LKRA*2+FORCE_CON_ANK_PLANT:
            self.parent().udp_port.udp_cmd_packet.pwm_duty_data[RANK_EXT_PWM]=0
            self.parent().udp_port.udp_cmd_packet.pwm_duty_data[LKNE_RANK_PWM]=0
            self.parent().udp_port.udp_cmd_packet.pwm_duty_data[RTANK_PWM]=0

            self.parent().udp_port.udp_cmd_packet.pwm_duty_flag[RANK_EXT_PWM]=True
            self.parent().udp_port.udp_cmd_packet.pwm_duty_flag[LKNE_RANK_PWM]=True
            self.parent().udp_port.udp_cmd_packet.pwm_duty_flag[RTANK_PWM]=True
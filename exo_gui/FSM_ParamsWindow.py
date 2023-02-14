from PyQt5.QtWidgets import QLineEdit,QPushButton,QDialog
from PyQt5 import uic
from PyQt5.QtCore import Qt
from Common import *

class FSM_ParamWindow(QDialog):
    def __init__(self,parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.Window)
        uic.loadUi("UI/FSM_ParamWindow.ui",self)
        
        self.lineEdit_left_swing_left_load_ratio = self.findChild(QLineEdit,'lineEdit_p0_p1_ratio')
        self.lineEdit_right_swing_right_load_ratio = self.findChild(QLineEdit,'lineEdit_p2_p3_ratio')
        self.lineEdit_left_init_f = self.findChild(QLineEdit,'lineEdit_left_knee_initF')
        self.lineEdit_right_init_f = self.findChild(QLineEdit,'lineEdit_right_knee_initF')
        self.lineEdit_left_knee_imp = self.findChild(QLineEdit,'lineEdit_left_knee_imp')
        self.lineEdit_right_knee_imp = self.findChild(QLineEdit,'lineEdit_right_knee_imp')

        self.btn_update_fsm_param = self.findChild(QPushButton,'btn_apply_setting')
        self.btn_cancel = self.findChild(QPushButton,'btn_cancel')


        self.btn_cancel.clicked.connect(self.close)
        self.btn_update_fsm_param.clicked.connect(self.UpdateParam)

    def UpdateParam(self):
        with self.parent().udp_port.lock:
            print('send out update')
            self.parent().udp_port.udp_cmd_packet.fsm_left_swing_left_load_ratio = TextToFloat(self.lineEdit_left_swing_left_load_ratio.text())
            self.parent().udp_port.udp_cmd_packet.fsm_right_swing_right_load_ratio = TextToFloat(self.lineEdit_right_swing_right_load_ratio.text())
            self.parent().udp_port.udp_cmd_packet.fsm_left_knee_init_f = TextToFloat(self.lineEdit_left_init_f.text())
            self.parent().udp_port.udp_cmd_packet.fsm_right_knee_init_f = TextToFloat(self.lineEdit_right_init_f.text())
            self.parent().udp_port.udp_cmd_packet.fsm_left_knee_imp = TextToFloat(self.lineEdit_left_knee_imp.text())
            self.parent().udp_port.udp_cmd_packet.fsm_right_knee_imp = TextToFloat(self.lineEdit_right_knee_imp.text())
            self.parent().udp_port.udp_cmd_packet.fsm_param_change_flag=True
        




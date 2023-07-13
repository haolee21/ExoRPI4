import typing
from PyQt5 import QtCore, uic
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QLineEdit,QPushButton,QDialog, QWidget,QCheckBox,QMessageBox
from functools import partial
from UdpClient import *
from collections import deque

class MPC_TrainingWindow(QDialog):
    def __init__(self, parent: None):
        super().__init__(parent)
        self.setWindowFlags(Qt.Window)
        uic.loadUi('UI/MPC_Train_window.ui',self)

        self.num_checked=0
        self.maintank_check = self.findChild(QCheckBox,'checkBox_main_tank')

        self.lsubtank_check = self.findChild(QCheckBox,'checkBox_left_sub_tank')
        self.lkneext_check = self.findChild(QCheckBox,'checkBox_left_knee_ext')
        self.lankpla_check = self.findChild(QCheckBox,'checkBox_left_ank_pla')

        self.rsubtank_check = self.findChild(QCheckBox,'checkBox_right_sub_tank')
        self.rkneext_check = self.findChild(QCheckBox,'checkBox_right_knee_ext')
        self.rankpla_check = self.findChild(QCheckBox,'checkBox_right_ank_pla')
        
        self.maintank_check.stateChanged.connect(partial(self.CheckBoxChanged,self.maintank_check))
        self.lsubtank_check.stateChanged.connect(partial(self.CheckBoxChanged,self.lsubtank_check))
        self.lkneext_check.stateChanged.connect(partial(self.CheckBoxChanged,self.lkneext_check))
        self.lankpla_check.stateChanged.connect(partial(self.CheckBoxChanged,self.lankpla_check))

        self.rsubtank_check.stateChanged.connect(partial(self.CheckBoxChanged,self.rsubtank_check))
        self.rkneext_check.stateChanged.connect(partial(self.CheckBoxChanged,self.rkneext_check))
        self.rankpla_check.stateChanged.connect(partial(self.CheckBoxChanged,self.rankpla_check))

        self.btn_gen = self.findChild(QPushButton,'btn_gen')
        self.btn_gen.clicked.connect(self.GenTrainingDuty)

    def CheckBoxChanged(self,change_checkBox):
        if change_checkBox.isChecked():
            self.num_checked = self.num_checked+1
        else:
            self.num_checked = self.num_checked-1

    def GenTrainingDuty(self):
        if(self.num_checked!=2):
            print('wrong')
            msg=QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Please only select two chambers")
            msg.exec_()
            return
        
        if self.maintank_check.isChecked() & self.lsubtank_check.isChecked():
            with self.parent().udp_port.lock:
                self.parent().udp_port.udp_cmd_packet.mpc_train_chamber1=int(Chamber.MainTank.value)
                self.parent().udp_port.udp_cmd_packet.mpc_train_chamber2=int(Chamber.SubTank.value)
                self.parent().udp_port.udp_cmd_packet.mpc_train_is_lkra=True
                self.parent().udp_port.udp_cmd_packet.mpc_train_gen_flag=True
        elif self.maintank_check.isChecked() & self.rsubtank_check.isChecked():
            with self.parent().udp_port.lock:
                self.parent().udp_port.udp_cmd_packet.mpc_train_chamber1=int(Chamber.MainTank.value)
                self.parent().udp_port.udp_cmd_packet.mpc_train_chamber2=int(Chamber.SubTank.value)
                self.parent().udp_port.udp_cmd_packet.mpc_train_is_lkra=False
                self.parent().udp_port.udp_cmd_packet.mpc_train_gen_flag=True
        
        elif self.lsubtank_check.isChecked() & self.lkneext_check.isChecked():
            with self.parent().udp_port.lock:
                self.parent().udp_port.udp_cmd_packet.mpc_train_chamber1=int(Chamber.SubTank.value)
                self.parent().udp_port.udp_cmd_packet.mpc_train_chamber2=int(Chamber.KneExt.value)
                self.parent().udp_port.udp_cmd_packet.mpc_train_is_lkra=True
                self.parent().udp_port.udp_cmd_packet.mpc_train_gen_flag=True
        elif self.rsubtank_check.isChecked() & self.rkneext_check.isChecked():
            with self.parent().udp_port.lock:
                self.parent().udp_port.udp_cmd_packet.mpc_train_chamber1=int(Chamber.SubTank.value)
                self.parent().udp_port.udp_cmd_packet.mpc_train_chamber2=int(Chamber.KneExt.value)
                self.parent().udp_port.udp_cmd_packet.mpc_train_is_lkra=False
                self.parent().udp_port.udp_cmd_packet.mpc_train_gen_flag=True

        # left knee->right ankle, right knee -> left ankle
        elif self.lkneext_check.isChecked() & self.rankpla_check.isChecked():
            with self.parent().udp_port.lock:
                self.parent().upd_port.upd_cmd_packet.mpc_train_chamber1=int(Chamber.KneExt.value)
                self.parent().udp_port.udp_cmd_packet.mpc_train_chamber2=int(Chamber.AnkPla.value)
                self.parent().udp_port.udp_cmd_packet.mpc_train_is_lkra=True
                self.parent().udp_port.udp_cmd_packet.mpc_train_gen_flag=True
        elif self.rkneext_check.isChecked() & self.lankpla_check.isChecked():
            with self.parent().udp_port.lock:
                self.parent().upd_port.upd_cmd_packet.mpc_train_chamber1=int(Chamber.KneExt.value)
                self.parent().udp_port.udp_cmd_packet.mpc_train_chamber2=int(Chamber.AnkPla.value)
                self.parent().udp_port.udp_cmd_packet.mpc_train_is_lkra=False
                self.parent().udp_port.udp_cmd_packet.mpc_train_gen_flag=True
        
        elif self.lsubtank_check.isChecked() & self.rankpla_check.isChecked():
            with self.parent().udp_port.lock:
                self.parent().udp_port.udp_cmd_packet.mpc_train_chamber1=int(Chamber.SubTank.value)
                self.parent().udp_port.udp_cmd_packet.mpc_train_chamber2=int(Chamber.AnkPla.value)
                self.parent().udp_port.udp_cmd_packet.mpc_train_is_lkra=True
                self.parent().udp_port.udp_cmd_packet.mpc_train_gen_flag=True
        elif self.rsubtank_check.isChecked() & self.lankpla_check.isChecked():
            with self.parent().udp_port.lock:
                self.parent().udp_port.udp_cmd_packet.mpc_train_chamber1=int(Chamber.SubTank.value)
                self.parent().udp_port.udp_cmd_packet.mpc_train_chamber2=int(Chamber.AnkPla.value)
                self.parent().udp_port.udp_cmd_packet.mpc_train_is_lkra=False
                self.parent().udp_port.udp_cmd_packet.mpc_train_gen_flag=True
        else:
            print('nothing match')
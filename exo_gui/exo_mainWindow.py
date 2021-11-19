from PyQt5.QtWidgets import QAction, QApplication, QCheckBox, QLabel, QLineEdit, QMainWindow,QPushButton, QRadioButton,QWidget,QProgressBar
from PyQt5 import uic,QtCore


import pyqtgraph as pg
from pyqtgraph import GraphicsLayoutWidget

import numpy as np
from TCP_Con import TCP
import pdb
from collections import deque
from ConnectionWindow import *
from PlotJointWindow import *
from PlotPressureWindow import *
from PWM_TestWindow import *
import math
DATALEN=120
SAMPT = 40
class SystemData:
    def __init__(self):
        self.ip_address='127.0.0.1'
        self.port=1234





class MW(QMainWindow):
    def __init__(self,_systemInfo):
        self.app = QApplication([])
        self.systemInfo = _systemInfo
        super().__init__()
        uic.loadUi('exo_gui_mw.ui',self)
        self.setWindowTitle('Bionics LLExo')

        self.tcp_port = TCP()
        self.dataLen=DATALEN
        self.max_pressure=80.0 #max pressure is 80 psi
        #init all windows, otherwise all data will be lost when we close it
        self.con_window = ConnectionWindow(self)
        self.joint_plot_window = PlotJointWindow(self)
        self.pressure_plot_window=PlotPressureWindow(self)
        self.pwm_test_window = PWM_TestWindow(self)
       
        
        # jointUpdateCB = lambda data:self.joint_plot_window.UpdateData(self,data)
        # preUpdateCB = lambda data:self.pressure_plot_window.UpdateData(self,data)
        # self.tcp_port.SetCallBack(jointUpdateCB,preUpdateCB)
        self.tcp_port.SetCallBack(self.joint_plot_window.UpdateData,self.pressure_plot_window.UpdateData,self.update_air_volume,self.found_disconnect)
        # TCP/IP connection
        
        self.cur_ip = self.findChild(QLabel,'cur_ip_label')
        self.cur_ip.setText(self.tcp_port.ip_address+':'+str(self.tcp_port.port))
        self.btn_connect = self.findChild(QPushButton,'btn_connect')
        self.btn_connect.clicked.connect(self.btn_connect_clicked)
        
        # example of open another window with menu action
        self.act_connection = self.findChild(QAction,'actionSet_IP_Address')
        self.act_connection.triggered.connect(self.open_con_window)
        self.act_plotJoint = self.findChild(QAction,'actionGraphJoint')
        self.act_plotJoint.triggered.connect(self.open_joint_plot)
        self.act_plotPressure = self.findChild(QAction,'action_plotPressure')
        self.act_plotPressure.triggered.connect(self.open_pressure_plot)

        self.act_pwm_test =self.findChild(QAction,'act_testPWM_valves')
        self.act_pwm_test.triggered.connect(self.open_pwm_test)

        # air reserivor 
        
        self.air_volume = self.findChild(QProgressBar,'air_volumn')
        
        # connection setting
        self.value = 0
        

        # create exo plot
        self.numJoint=6
        self.model_plot_widget = self.findChild(GraphicsLayoutWidget,'exo_rt_plot')
        self.model_plot_widget.setBackground('w')
        self.model_plot = self.model_plot_widget.addPlot(colspan=1)
        self.model_plot.hideAxis('bottom')
        self.model_plot.hideAxis('left')
        self.model_plot.setYRange(-50,50)
        self.model_plot.setXRange(-50,50)
        self.rtplot_data=[10.0,-10.0,0,10.0,-10.0,0]
        
        self.thigh_len=100*0.245 #these constants are just normal human length ratio*100
        self.tibia_len=100*0.245
        self.foot_len=100*0.152
        self.torso_len=100*0.34
        exo_x,exo_y=self.get_exo_model()
        self.init_exo_model(exo_x,exo_y)


        # reset encoder pos
        self.btn_exo_resetLHipS = self.findChild(QPushButton,'btn_resetLHip')
        self.btn_exo_resetLHipS.clicked.connect(self.btn_resetLHipS_clicked)
        self.btn_exo_resetLKneS = self.findChild(QPushButton,'btn_resetLKne')
        self.btn_exo_resetLKneS.clicked.connect(self.btn_resetLKneS_clicked)
        self.btn_exo_resetLAnkS = self.findChild(QPushButton,'btn_resetLAnk')
        self.btn_exo_resetLAnkS.clicked.connect(self.btn_resetLAnkS_clicked)

        self.btn_exo_resetRHipS = self.findChild(QPushButton,'btn_resetRHip')
        self.btn_exo_resetRHipS.clicked.connect(self.btn_resetRHipS_clicked)
        self.btn_exo_resetRKneS = self.findChild(QPushButton,'btn_resetRKne')
        self.btn_exo_resetRKneS.clicked.connect(self.btn_resetRKneS_clicked)
        self.btn_exo_resetRAnkS = self.findChild(QPushButton,'btn_resetRAnk')
        self.btn_exo_resetRAnkS.clicked.connect(self.btn_resetRAnkS_clicked)
        


        #function checkbox
        self.but_sendCmd = self.findChild(QPushButton,'but_sendCmd')
        self.but_sendCmd.clicked.connect(self.btn_sendCmd_clicked)
        self.relLKne_task = self.findChild(QCheckBox,'checkBox_rel_LKne')
        self.relRKne_task = self.findChild(QCheckBox,'checkBox_rel_RKne')
        self.relLAnk_task = self.findChild(QCheckBox,'checkBox_rel_LAnk')
        self.relRAnk_task = self.findChild(QCheckBox,'checkBox_rel_RAnk')

        self.actLKne_task = self.findChild(QCheckBox,'checkBox_act_LKne')
        self.actRKne_task = self.findChild(QCheckBox,'checkBox_act_RKne')
        self.actLAnk_task = self.findChild(QCheckBox,'checkBox_act_LAnk')
        self.actRAnk_task = self.findChild(QCheckBox,'checkBox_act_RAnk')

        self.walkRec_task = self.findChild(QRadioButton,'radioButton_walkRec')
        self.walkRec_task.toggled.connect(self.radio_walkRec_checked)

        


        self.show()
    def radio_walkRec_checked(self):
        self.relLKne_task.setChecked(False)
        self.relRKne_task.setChecked(False)
        self.relLAnk_task.setChecked(False)
        self.relRAnk_task.setChecked(False)

        self.actLKne_task.setChecked(False)
        self.actRKne_task.setChecked(False)
        self.actLAnk_task.setChecked(False)
        self.actRAnk_task.setChecked(False)

    def btn_sendCmd_clicked(self):
        if self.walkRec_task.isChecked():
            self.tcp_port.SendCmd('STR:REC:ALL',2)
            self.walkRec_task.setChecked(False)
        else:
            if self.relLKne_task.isChecked():
                self.tcp_port.SendCmd('STR:REL:LKNE',2)
            if self.relRKne_task.isChecked():
                self.tcp_port.SendCmd('STR:REL:RKNE',2)
            if self.relLAnk_task.isChecked():
                self.tcp_port.SendCmd('STR:REL:LANK',2)
            if self.relRAnk_task.isChecked():
                self.tcp_port.SendCmd('STR:REL:RANK',2)
            if self.actLKne_task.isChecked():
                self.tcp_port.SendCmd('STR:ACT:LKNE',2)
            if self.actRKne_task.isChecked():
                self.tcp_port.SendCmd('STR:ACT:RKNE',2)
            if self.actLAnk_task.isChecked():
                self.tcp_port.SendCmd('STR:ACT:LANK',2)
            if self.actRAnk_task.isChecked():
                self.tcp_port.SendCmd('STR:ACT:RANK',2)
        self.radio_walkRec_checked()
                
        
        
    def btn_connect_clicked(self):
        
        if not self.tcp_port.flag:
            if self.tcp_port.Connect():
                self.btn_connect.setText('Disconnect')
                self.timer = QtCore.QTimer()
                self.timer.timeout.connect(self.tcp_port.DataUpdate)
                self.timer.start(SAMPT)
                
        else:
            self.timer.stop()
            print('Disconnect   ',self.tcp_port.SendCmd('CON:STOP',2).decode())
            self.tcp_port.Disconnect()
            self.btn_connect.setText('Connect')
    def found_disconnect(self):
        self.timer.stop()
        self.tcp_port.Disconnect()
        self.btn_connect.setText('Connect')
    def update_air_volume(self,volume):
        cur_pre = volume*0.0037147-25
        if cur_pre>self.max_pressure:
            cur_pre=self.max_pressure
        self.air_volume.setValue(int(cur_pre/self.max_pressure*100))
    def open_con_window(self):
        self.con_window.show()
    def open_joint_plot(self):
        self.joint_plot_window.show()
    def open_pressure_plot(self):
        self.pressure_plot_window.show()
    def open_pwm_test(self):
        self.pwm_test_window.show()
        
    def btn_resetLHipS_clicked(self):
        self.tcp_port.SendCmd('CAL:ENC:LHIP_S',1,True)
    def btn_resetLKneS_clicked(self):
        self.tcp_port.SendCmd('CAL:ENC:LKNE_S',1,True)
    def btn_resetLAnkS_clicked(self):
        self.tcp_port.SendCmd('CAL:ENC:LANK_S',1,True)
    def btn_resetRHipS_clicked(self):
        self.tcp_port.SendCmd('CAL:ENC:RHIP_S',1,True)
    def btn_resetRKneS_clicked(self):
        self.tcp_port.SendCmd('CAL:ENC:RKNE_S',1,True)
    def btn_resetRAnkS_clicked(self):
        self.tcp_port.SendCmd('CAL:ENC:RANK_S',1,True)



    def btn_dischargeAll_clicked(self):
        ret = self.tcp_port.SendCmd('STR:REL:ALL',2)
        print('Discharge all air  ',ret.decode())

    def get_exo_model(self):
        #data format = [LHip,LKne,LAnk,RHip,RKne,RAnk]
        
        lkne = [self.thigh_len*math.sin(math.radians(self.rtplot_data[0])),-self.thigh_len*math.cos(math.radians(self.rtplot_data[0]))]
        lank = [self.tibia_len*math.sin(math.radians(self.rtplot_data[0]+self.rtplot_data[1])),-self.tibia_len*math.cos(math.radians(self.rtplot_data[0]+self.rtplot_data[1]))]
        ltoe = [self.foot_len*math.sin(math.radians(self.rtplot_data[0]+self.rtplot_data[1]+self.rtplot_data[2]-90)),self.foot_len*math.cos(math.radians(self.rtplot_data[0]+self.rtplot_data[1]+self.rtplot_data[2]-90))]
        head = [0,self.torso_len]
        rkne = [self.thigh_len*math.sin(math.radians(self.rtplot_data[3])),-self.thigh_len*math.cos(math.radians(self.rtplot_data[3]))]
        rank = [self.tibia_len*math.sin(math.radians(self.rtplot_data[3]+self.rtplot_data[4])),-self.tibia_len*math.cos(math.radians(self.rtplot_data[3]+self.rtplot_data[4]))]
        rtoe = [self.foot_len*math.sin(math.radians(self.rtplot_data[3]+self.rtplot_data[4]+self.rtplot_data[5]-90)),self.foot_len*math.cos(math.radians(self.rtplot_data[3]+self.rtplot_data[4]+self.rtplot_data[5]-90))]
        xpos=[lkne[0],lkne[0]+lank[0],lkne[0]+lank[0]+ltoe[0],head[0],rkne[0],rkne[0]+rank[0],rkne[0]+rank[0]+rtoe[0]]
        ypos=[lkne[1],lkne[1]+lank[1],lkne[1]+lank[1]+ltoe[1],head[1],rkne[1],rkne[1]+rank[1],rkne[1]+rank[1]+rtoe[1]]
        
        
        return xpos,ypos
    def init_exo_model(self,xpos,ypos):
        torso_line = self.model_plot.plot([0.0]+[xpos[3]],[0.0]+[ypos[3]],pen=pg.mkPen('g',width=5),symbol='o')
        self.left_leg_line = self.model_plot.plot([0.0]+xpos[0:3],[0.0]+ypos[0:3],pen=pg.mkPen('r',width=5),symbol='o')
        self.right_leg_line=self.model_plot.plot([0.0]+xpos[4:],[0.0]+ypos[4:],pen=pg.mkPen('b',width=5),symbol='o')
        self.left_leg_line.setSymbolSize(20)
        self.right_leg_line.setSymbolSize(20)
        
    def update_exo_model(self):
        xpos,ypos=self.get_exo_model()
        self.left_leg_line.setData([0.0]+xpos[0:3],[0.0]+ypos[0:3])
        self.right_leg_line.setData([0.0]+xpos[4:],[0.0]+ypos[4:])
    

sysData = SystemData()


window = MW(sysData)

window.app.exec_()
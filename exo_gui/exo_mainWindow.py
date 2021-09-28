from PyQt5.QtWidgets import QAction, QApplication, QCheckBox, QLabel, QLineEdit, QMainWindow,QPushButton, QRadioButton,QWidget,QProgressBar
from PyQt5 import uic,QtCore


import pyqtgraph as pg
from pyqtgraph import GraphicsLayoutWidget
import numpy as np
from TCP_Con import TCP
import pdb
from collections import deque
dataLen=120
class SystemData:
    def __init__(self):
        self.ip_address='0.0.0.0'
        self.port=0
class ConnectionWindow(QWidget):
    def __init__(self,parent=None):
        super().__init__()
        uic.loadUi('Con_window.ui',self)
        btn_ok = self.findChild(QPushButton,'btn_ok')
        btn_ok.clicked.connect(self.btn_ok_clicked)
        btn_cancel = self.findChild(QPushButton,'btn_cancel')
        btn_cancel.clicked.connect(self.btn_cancel_clicked)
        btn_apply = self.findChild(QPushButton,'btn_apply')
        btn_apply.clicked.connect(self.btn_apply_clicked)


        self.parent = parent
        self.ip_input = self.findChild(QLineEdit,'ip_lineEdit')
        self.ip_input.setText(parent.tcp_port.ip_address)
        self.port_input = self.findChild(QLineEdit,'port_lineEdit')
        self.port_input.setText(str(parent.tcp_port.port))

        

        
        
    def btn_ok_clicked(self):
        self.finish_set()
        self.close()
    def btn_cancel_clicked(self):
        self.close()    
    def btn_apply_clicked(self):
        self.finish_set()
    def closeEvent(self,event):
        self.ip_input.setText(self.parent.tcp_port.ip_address)
        self.port_input.setText(str(self.parent.tcp_port.port))

    def finish_set(self):
        self.parent.tcp_port.ip_address=self.ip_input.text()
        self.parent.tcp_port.port = int(self.port_input.text())
        self.parent.cur_ip.setText(self.parent.tcp_port.ip_address+':'+str(self.parent.tcp_port.port))

        
class PlotJointWindow(QWidget):
    def __init__(self,parent=None):
        super().__init__()
        uic.loadUi('two_col_graph.ui',self)
        self.parent = parent
        self.left_plot_widget = self.findChild(GraphicsLayoutWidget,'left_graphicsView')
        self.left_plot_widget.setBackground('w')

        self.left_hip_plot = self.left_plot_widget.addPlot(colspan=1,title='Hip')
        self.left_hip_plot.setYRange(-90,90)
        self.left_hip_plot.setLabel('left','Angle (deg)')
        self.left_hip_plot.setLabel('bottom','Time (sec)')
        self.left_hip_line = self.left_hip_plot.plot(pen=pg.mkPen('b', width=1))
        

        self.left_plot_widget.nextRow()
        self.left_knee_plot = self.left_plot_widget.addPlot(colspan=1,title='Knee')
        self.left_knee_plot.setYRange(-180,0)
        self.left_knee_plot.setLabel('left','Angle (deg)')
        self.left_knee_plot.setLabel('bottom','Time (sec)')
        self.left_knee_line = self.left_knee_plot.plot(pen=pg.mkPen('b', width=1))
        

        self.left_plot_widget.nextRow()
        self.left_ankle_plot = self.left_plot_widget.addPlot(colspan=1,title='Ankle')
        self.left_ankle_plot.setYRange(-90,90)
        self.left_ankle_plot.setLabel('left','Angle (deg)')
        self.left_ankle_plot.setLabel('bottom','Time (sec)')
        self.left_ankle_line = self.left_ankle_plot.plot(pen=pg.mkPen('b', width=1))
        

        self.right_plot_widget = self.findChild(GraphicsLayoutWidget,'right_graphicsView')
        self.right_plot_widget.setBackground('w')
        self.right_hip_plot = self.right_plot_widget.addPlot(colspan=1,title='Hip')
        self.right_hip_plot.setYRange(-90,90)
        self.right_hip_plot.setLabel('left','Angle (deg)')
        self.right_hip_plot.setLabel('bottom','Time (sec)')
        self.right_hip_line = self.right_hip_plot.plot(pen=pg.mkPen('b', width=1))

        self.right_plot_widget.nextRow()
        self.right_knee_plot = self.right_plot_widget.addPlot(colspan=1,title='Knee',color='b')
        self.right_knee_plot.setYRange(0,180)
        self.right_knee_plot.setLabel('left','Angle (deg)')
        self.right_knee_plot.setLabel('bottom','Time (sec)')
        self.right_knee_line = self.right_knee_plot.plot(pen=pg.mkPen('b', width=1))

        self.right_plot_widget.nextRow()
        self.right_ankle_plot = self.right_plot_widget.addPlot(colspan=1,title='Ankle')
        self.right_ankle_plot.setYRange(-90,90)
        self.right_ankle_plot.setLabel('left','Angle (deg)')
        self.right_ankle_plot.setLabel('bottom','Time (sec)')
        self.right_ankle_line = self.right_ankle_plot.plot(pen=pg.mkPen('b', width=1))

        self.setWindowTitle('Joint Angles')

        # init data
        self.l_hipData= deque([0.0]*dataLen)
        self.left_hip_line.setData(self.l_hipData)
        self.l_kneeData = deque([0.0]*dataLen)
        self.left_knee_line.setData(self.l_kneeData)
        self.l_ankData = deque([0.0]*dataLen)
        self.left_ankle_line.setData(self.l_ankData)
        self.r_hipData = deque([0.0]*dataLen)
        self.right_hip_line.setData(self.r_hipData)
        self.r_kneeData = deque([0.0]*dataLen)
        self.right_knee_line.setData(self.r_kneeData)
        self.r_ankData = deque([0.0]*dataLen)
        self.right_ankle_line.setData(self.r_ankData)

    def UpdateData(self,data):
        # the encoder are 12 bits, we use two bytes to send the data, 
        # each joint has two bytes, the order is MSB,LSB
        # [l_hip,l_knee,l_ank,r_hip,r_knee,r_ank]
        # print('joint got update')
        self.l_hipData.popleft()
        self.l_hipData.append(int.from_bytes(data[0:2],'big')*0.087890625)
        self.left_hip_line.setData(self.l_hipData)
        self.l_kneeData.popleft()
        self.l_kneeData.append(int.from_bytes(data[2:4],'big')*0.087890625)
        self.left_knee_line.setData(self.l_kneeData)
        self.l_ankData.popleft()
        self.l_ankData.append(int.from_bytes(data[4:6],'big')*0.087890625)
        self.left_ankle_line.setData(self.l_ankData)
        print('left ankle: ',int.from_bytes(data[4:6],'big'))

        self.r_hipData.popleft()
        self.r_hipData.append(int.from_bytes(data[6:8],'big')*0.087890625)
        self.right_hip_line.setData(self.r_hipData)
        self.r_kneeData.popleft()
        self.r_kneeData.append(int.from_bytes(data[8:10],'big')*0.087890625)
        self.right_knee_line.setData(self.r_kneeData)
        self.r_ankData.popleft()
        self.r_ankData.append(int.from_bytes(data[10:12],'big')*0.087890625)
        self.right_ankle_line.setData(self.r_ankData)

        self.parent.app.processEvents()
class PlotPressureWindow(QWidget):
    def __init__(self,parent=None):
        super().__init__()
        self.parent =parent
        uic.loadUi('two_col_graph.ui',self)
        self.left_plot_widget = self.findChild(GraphicsLayoutWidget,'left_graphicsView')
        self.left_plot_widget.setBackground('w')
        self.left_kneePre_plot = self.left_plot_widget.addPlot(colspan=1,title='Knee Pressure')
        self.left_kneePre_plot.setYRange(0,100)
        self.left_kneePre_plot.setLabel('left','Pressure (psi)')
        self.left_kneePre_plot.setLabel('bottom','Time (sec)')
        self.left_kneePre_line = self.left_kneePre_plot.plot(pen=pg.mkPen('b', width=1))

        self.left_plot_widget.nextRow()
        self.left_anklePre_plot = self.left_plot_widget.addPlot(colspan=1,title='Ankle Pressure')
        self.left_anklePre_plot.setYRange(0,100)
        self.left_anklePre_plot.setLabel('left','Pressure (psi)')
        self.left_anklePre_plot.setLabel('bottom','Time (sec)')
        self.left_anklePre_line = self.left_anklePre_plot.plot(pen=pg.mkPen('b', width=1))

        self.right_plot_widget = self.findChild(GraphicsLayoutWidget,'right_graphicsView')
        self.right_plot_widget.setBackground('w')
        self.right_kneePre_plot = self.right_plot_widget.addPlot(colspan=1,title='Knee Pressure')
        self.right_kneePre_plot.setYRange(0,100)
        self.right_kneePre_plot.setLabel('left','Pressure (psi)')
        self.right_kneePre_plot.setLabel('bottom','Time (sec)')
        self.right_kneePre_line = self.right_kneePre_plot.plot(pen=pg.mkPen('b', width=1))

        self.right_plot_widget.nextRow()
        self.right_anklePre_plot = self.right_plot_widget.addPlot(colspan=1,title='Ankle Pressure')
        self.right_anklePre_plot.setYRange(0,100)
        self.right_anklePre_plot.setLabel('left','Pressure (psi)')
        self.right_anklePre_plot.setLabel('bottom','Time (sec)')
        self.right_anklePre_line = self.right_anklePre_plot.plot(pen=pg.mkPen('b', width=1))

        self.setWindowTitle('Pressure')


        # init data
        self.l_knePreData = deque([0]*dataLen)
        self.l_ankPreData = deque([0]*dataLen)
        self.r_knePreData = deque([0]*dataLen)
        self.r_ankPreData = deque([0]*dataLen)
    def UpdateData(self,data):
        # the results are from 16 bits ADC, MSB, LSB
        # print('pressure got update')
        self.l_knePreData.popleft()
        self.l_knePreData.append(int.from_bytes(data[0:2],'big'))
        self.left_kneePre_line.setData(self.l_knePreData)
        self.l_ankPreData.popleft()
        self.l_ankPreData.append(int.from_bytes(data[2:4],'big'))
        self.left_anklePre_line.setData(self.l_ankPreData)
        self.r_knePreData.popleft()
        self.r_knePreData.append(int.from_bytes(data[4:6],'big'))
        self.right_kneePre_line.setData(self.r_knePreData)
        self.r_ankPreData.popleft()
        self.r_ankPreData.append(int.from_bytes(data[6:8],'big'))
        self.right_anklePre_line.setData(self.r_ankPreData)

        self.parent.app.processEvents()


class MW(QMainWindow):
    def __init__(self,_systemInfo):
        self.app = QApplication([])
        self.systemInfo = _systemInfo
        super().__init__()
        uic.loadUi('exo_gui_mw.ui',self)
        self.setWindowTitle('Bionics LLExo')

        self.tcp_port = TCP()
        #init all windows, otherwise all data will be lost when we close it
        self.con_window = ConnectionWindow(self)
        self.joint_plot_window = PlotJointWindow(self)
        self.pressure_plot_window=PlotPressureWindow(self)
        
        # jointUpdateCB = lambda data:self.joint_plot_window.UpdateData(self,data)
        # preUpdateCB = lambda data:self.pressure_plot_window.UpdateData(self,data)
        # self.tcp_port.SetCallBack(jointUpdateCB,preUpdateCB)
        self.tcp_port.SetCallBack(self.joint_plot_window.UpdateData,self.pressure_plot_window.UpdateData,self.update_air_volume)
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

        # air reserivor 
        self.air_volume = self.findChild(QProgressBar,'air_volumn')
        
        # connection setting
        self.value = 0
        

        # create exo plot
        self.model_plot_widget = self.findChild(GraphicsLayoutWidget,'exo_rt_plot')
        self.model_plot_widget.setBackground('w')
        self.model_plot = self.model_plot_widget.addPlot(colspan=1)
        self.model_plot.hideAxis('bottom')
        self.model_plot.hideAxis('left')

        self.btn_exo_resetHip = self.findChild(QPushButton,'but_resetLHip')
        self.btn_exo_resetHip.clicked.connect(self.btn_resetHip_clicked)


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
                self.timer.start(20)
                
        else:
            self.timer.stop()
            print('Disconnect   ',self.tcp_port.SendCmd('CON:STOP',2).decode())
            self.tcp_port.Disconnect()
            self.btn_connect.setText('Connect')
    def update_air_volume(self,volume):
        self.air_volume.setValue(volume)
    def open_con_window(self):
        self.con_window.show()
    def open_joint_plot(self):
        self.joint_plot_window.show()
    def open_pressure_plot(self):
        self.pressure_plot_window.show()
    def btn_resetHip_clicked(self):
        # self.tcp_port.Update_test(self.joint_plot_window.UpdateData,self.pressure_plot_window.UpdateData)
        ret = self.tcp_port.SendCmd('RESET:HIP',2)
        print('return is ',ret.decode())
    def btn_dischargeAll_clicked(self):
        ret = self.tcp_port.SendCmd('STR:REL:ALL',2)
        print('Discharge all air  ',ret.decode())

sysData = SystemData()


window = MW(sysData)

window.app.exec_()
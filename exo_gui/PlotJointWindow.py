import struct
from PyQt5.QtWidgets import QWidget
from PyQt5 import uic
from numpy import double
from pyqtgraph import GraphicsLayoutWidget
import pyqtgraph as pg
from collections import deque

from TCP_Con import DOUBLE_SIZE
class PlotJointWindow(QWidget):
    def __init__(self,parent=None):
        super().__init__()
        uic.loadUi('two_col_graph.ui',self)
        self.parent = parent
        self.left_plot_widget = self.findChild(GraphicsLayoutWidget,'left_graphicsView')
        self.left_plot_widget.setBackground('w')

        self.left_hip_plot = self.left_plot_widget.addPlot(colspan=1,title='Hip')
        self.left_hip_plot.setYRange(0,360)
        self.left_hip_plot.setLabel('left','Angle (deg)')
        self.left_hip_plot.setLabel('bottom','Time (sec)')
        self.left_hip_line = self.left_hip_plot.plot(pen=pg.mkPen('b', width=1))
        

        self.left_plot_widget.nextRow()
        self.left_knee_plot = self.left_plot_widget.addPlot(colspan=1,title='Knee')
        self.left_knee_plot.setYRange(0,360)
        self.left_knee_plot.setLabel('left','Angle (deg)')
        self.left_knee_plot.setLabel('bottom','Time (sec)')
        self.left_knee_line = self.left_knee_plot.plot(pen=pg.mkPen('b', width=1))
        

        self.left_plot_widget.nextRow()
        self.left_ankle_plot = self.left_plot_widget.addPlot(colspan=1,title='Ankle')
        self.left_ankle_plot.setYRange(0,360)
        self.left_ankle_plot.setLabel('left','Angle (deg)')
        self.left_ankle_plot.setLabel('bottom','Time (sec)')
        self.left_ankle_line = self.left_ankle_plot.plot(pen=pg.mkPen('b', width=1))
        

        self.right_plot_widget = self.findChild(GraphicsLayoutWidget,'right_graphicsView')
        self.right_plot_widget.setBackground('w')
        self.right_hip_plot = self.right_plot_widget.addPlot(colspan=1,title='Hip')
        self.right_hip_plot.setYRange(0,360)
        self.right_hip_plot.setLabel('left','Angle (deg)')
        self.right_hip_plot.setLabel('bottom','Time (sec)')
        self.right_hip_line = self.right_hip_plot.plot(pen=pg.mkPen('b', width=1))

        self.right_plot_widget.nextRow()
        self.right_knee_plot = self.right_plot_widget.addPlot(colspan=1,title='Knee',color='b')
        self.right_knee_plot.setYRange(0,360)
        self.right_knee_plot.setLabel('left','Angle (deg)')
        self.right_knee_plot.setLabel('bottom','Time (sec)')
        self.right_knee_line = self.right_knee_plot.plot(pen=pg.mkPen('b', width=1))

        self.right_plot_widget.nextRow()
        self.right_ankle_plot = self.right_plot_widget.addPlot(colspan=1,title='Ankle')
        self.right_ankle_plot.setYRange(0,360)
        self.right_ankle_plot.setLabel('left','Angle (deg)')
        self.right_ankle_plot.setLabel('bottom','Time (sec)')
        self.right_ankle_line = self.right_ankle_plot.plot(pen=pg.mkPen('b', width=1))
        

        self.setWindowTitle('Joint Angles')

        # init data
        self.l_hipData= deque([0.0]*parent.dataLen)
        self.left_hip_line.setData(self.l_hipData)
        self.l_kneeData = deque([0.0]*parent.dataLen)
        self.left_knee_line.setData(self.l_kneeData)
        self.l_ankData = deque([0.0]*parent.dataLen)
        self.left_ankle_line.setData(self.l_ankData)
        self.r_hipData = deque([0.0]*parent.dataLen)
        self.right_hip_line.setData(self.r_hipData)
        self.r_kneeData = deque([0.0]*parent.dataLen)
        self.right_knee_line.setData(self.r_kneeData)
        self.r_ankData = deque([0.0]*parent.dataLen)
        self.right_ankle_line.setData(self.r_ankData)
        
    def UpdateData(self,data):
        # the encoder are 12 bits, we use two bytes to send the data, 
        # each joint has two bytes, the order is MSB,LSB
        # [l_hip,l_knee,l_ank,r_hip,r_knee,r_ank]
        # print('joint got update')

        # using parent.rt_data is already initialized since we have to manually pressed connect to establish connection
        self.l_hipData.popleft()
        # self.parent.rtplot_data[0]=int.from_bytes(data[0:2],'little')*0.087890625
        

        self.parent.rtplot_data[0]=struct.unpack("d",data[0:DOUBLE_SIZE])[0]*0.087890625
        self.l_hipData.append(self.parent.rtplot_data[0])
        self.left_hip_line.setData(self.l_hipData)

        self.l_kneeData.popleft()
        # self.parent.rtplot_data[1]=int.from_bytes(data[2:4],'little')*0.087890625
        self.parent.rtplot_data[1]=struct.unpack("d",data[DOUBLE_SIZE*1:DOUBLE_SIZE*2])[0]*0.087890625
        self.l_kneeData.append(self.parent.rtplot_data[1])
        self.left_knee_line.setData(self.l_kneeData)

        self.l_ankData.popleft()
        # self.parent.rtplot_data[2]=int.from_bytes(data[4:6],'little')*0.087890625
        self.parent.rtplot_data[2]=struct.unpack("d",data[DOUBLE_SIZE*2:DOUBLE_SIZE*3])[0]*0.087890625
        self.l_ankData.append(self.parent.rtplot_data[2])
        self.left_ankle_line.setData(self.l_ankData)
        

        self.r_hipData.popleft()
        # self.parent.rtplot_data[3]=int.from_bytes(data[6:8],'little')*0.087890625
        self.parent.rtplot_data[3]=struct.unpack("d",data[DOUBLE_SIZE*3:DOUBLE_SIZE*4])[0]*0.087890625
        self.r_hipData.append(self.parent.rtplot_data[3])
        self.right_hip_line.setData(self.r_hipData)
        
        self.r_kneeData.popleft()
        # self.parent.rtplot_data[4]=int.from_bytes(data[8:10],'little')*0.087890625
        self.parent.rtplot_data[4]=struct.unpack("d",data[DOUBLE_SIZE*4:DOUBLE_SIZE*5])[0]*0.087890625
        self.r_kneeData.append(self.parent.rtplot_data[4])
        self.right_knee_line.setData(self.r_kneeData)

        self.r_ankData.popleft()
        # self.parent.rtplot_data[5]=int.from_bytes(data[10:12],'little')*0.087890625
        self.parent.rtplot_data[5]=struct.unpack("d",data[DOUBLE_SIZE*5:DOUBLE_SIZE*6])[0]*0.087890625
        self.r_ankData.append(self.parent.rtplot_data[5])
        self.right_ankle_line.setData(self.r_ankData)

        self.parent.update_exo_model()


        self.parent.app.processEvents()

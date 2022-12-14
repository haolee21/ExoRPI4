from ctypes.wintypes import DOUBLE
from PyQt5.QtWidgets import QWidget
from PyQt5 import uic
from pyqtgraph import GraphicsLayoutWidget
import pyqtgraph as pg
from collections import deque
import struct

from ExoDataStruct import *


class PlotPressureWindow(QWidget):
    def __init__(self,parent=None):
        super().__init__()
        self.parent =parent
        uic.loadUi('two_col_graph.ui',self)
        self.left_plot_widget = self.findChild(GraphicsLayoutWidget,'left_graphicsView')
        self.left_plot_widget.setBackground('w')


        self.left_tank_plot = self.left_plot_widget.addPlot(colspan=1,title='Left Tank Pressure')
        self.left_tank_plot.setYRange(-1,self.parent.max_pressure)
        self.left_tank_plot.setLabel('left','Pressure (psi)')
        self.left_tank_plot.setLabel('bottom','Time (sec)')
        self.left_tank_line = self.left_tank_plot.plot(pen=pg.mkPen('b', width=1))
        self.left_tank_yaxis = self.left_tank_plot.getAxis('left')
        self.left_tank_yaxis.setTickSpacing(20,10)

        self.left_plot_widget.nextRow()
        self.left_knee_ext_plot = self.left_plot_widget.addPlot(colspan=1,title='Knee Extension Pressure')
        self.left_knee_ext_plot.setYRange(-1,self.parent.max_pressure)
        self.left_knee_ext_plot.setLabel('left','Pressure (psi)')
        self.left_knee_ext_plot.setLabel('bottom','Time (sec)')
        self.left_knee_ext_line = self.left_knee_ext_plot.plot(pen=pg.mkPen('b', width=1))
        self.left_knee_ext_yaxis = self.left_knee_ext_plot.getAxis('left')
        self.left_knee_ext_yaxis.setTickSpacing(20,10)

        self.left_plot_widget.nextRow()
        self.left_knee_flex_plot = self.left_plot_widget.addPlot(colspan=1,title='Knee Flexion Pressure')
        self.left_knee_flex_plot.setYRange(-1,self.parent.max_pressure)
        self.left_knee_flex_plot.setLabel('left','Pressure (psi)')
        self.left_knee_flex_plot.setLabel('bottom','Time (sec)')
        self.left_knee_flex_line = self.left_knee_flex_plot.plot(pen=pg.mkPen('b', width=1))
        self.left_knee_flex_yaxis = self.left_knee_flex_plot.getAxis('left')
        self.left_knee_flex_yaxis.setTickSpacing(20,10)
        

        self.left_plot_widget.nextRow()
        self.left_ankle_ext_plot = self.left_plot_widget.addPlot(colspan=1,title='Ankle Extension Pressure')
        self.left_ankle_ext_plot.setYRange(-1,self.parent.max_pressure)
        self.left_ankle_ext_plot.setLabel('left','Pressure (psi)')
        self.left_ankle_ext_plot.setLabel('bottom','Time (sec)')
        self.left_ankle_ext_line = self.left_ankle_ext_plot.plot(pen=pg.mkPen('b', width=1))
        self.left_ankle_ext_yaxis = self.left_ankle_ext_plot.getAxis('left')
        self.left_ankle_ext_yaxis.setTickSpacing(20,10)

        self.left_plot_widget.nextRow()
        self.left_ankle_flex_plot = self.left_plot_widget.addPlot(colspan=1,title='Ankle Flexion Pressure')
        self.left_ankle_flex_plot.setYRange(-1,self.parent.max_pressure)
        self.left_ankle_flex_plot.setLabel('left','Pressure (psi)')
        self.left_ankle_flex_plot.setLabel('bottom','Time (sec)')
        self.left_ankle_flex_line = self.left_ankle_flex_plot.plot(pen=pg.mkPen('b', width=1))
        self.left_ankle_flex_yaxis = self.left_ankle_flex_plot.getAxis('left')
        self.left_ankle_flex_yaxis.setTickSpacing(20,10)




        self.right_plot_widget = self.findChild(GraphicsLayoutWidget,'right_graphicsView')
        self.right_plot_widget.setBackground('w')
        # self.right_kneePre_plot.setYRange(-30,self.parent.max_pressure) #TODO: change the unit back to psi when fully integrated
        self.right_tank_plot = self.right_plot_widget.addPlot(colspan=1,title='Right Tank Pressure')
        self.right_tank_plot.setYRange(-1,self.parent.max_pressure)
        self.right_tank_plot.setLabel('left','Pressure (psi)')
        self.right_tank_plot.setLabel('bottom','Time (sec)')
        self.right_tank_line = self.right_tank_plot.plot(pen=pg.mkPen('b', width=1))
        self.right_tank_yaxis = self.right_tank_plot.getAxis('left')
        self.right_tank_yaxis.setTickSpacing(20,10)

        self.right_plot_widget.nextRow()
        self.right_knee_ext_plot = self.right_plot_widget.addPlot(colspan=1,title='Knee Extension Pressure')
        self.right_knee_ext_plot.setYRange(-1,self.parent.max_pressure)
        self.right_knee_ext_plot.setLabel('left','Pressure (psi)')
        self.right_knee_ext_plot.setLabel('bottom','Time (sec)')
        self.right_knee_ext_line = self.right_knee_ext_plot.plot(pen=pg.mkPen('b', width=1))
        self.right_knee_ext_yaxis = self.right_knee_ext_plot.getAxis('left')
        self.right_knee_ext_yaxis.setTickSpacing(20,10)


        self.right_plot_widget.nextRow()
        self.right_knee_flex_plot = self.right_plot_widget.addPlot(colspan=1,title='Knee Flexion Pressure')
        self.right_knee_flex_plot.setYRange(-1,self.parent.max_pressure)
        self.right_knee_flex_plot.setLabel('left','Pressure (psi)')
        self.right_knee_flex_plot.setLabel('bottom','Time (sec)')
        self.right_knee_flex_line = self.right_knee_flex_plot.plot(pen=pg.mkPen('b', width=1))
        self.right_knee_flex_yaxis = self.right_knee_flex_plot.getAxis('left')
        self.right_knee_flex_yaxis.setTickSpacing(20,10)

        self.right_plot_widget.nextRow()
        self.right_ankle_ext_plot = self.right_plot_widget.addPlot(colspan=1,title='Ankle Extension Pressure')
        self.right_ankle_ext_plot.setYRange(-1,self.parent.max_pressure)
        self.right_ankle_ext_plot.setLabel('left','Pressure (psi)')
        self.right_ankle_ext_plot.setLabel('bottom','Time (sec)')
        self.right_ankle_ext_line = self.right_ankle_ext_plot.plot(pen=pg.mkPen('b', width=1))
        self.right_ankle_ext_yaxis = self.right_ankle_ext_plot.getAxis('left')
        self.right_ankle_ext_yaxis.setTickSpacing(20,10)


        self.right_plot_widget.nextRow()
        self.right_ankle_flex_plot = self.right_plot_widget.addPlot(colspan=1,title='Ankle Flexion Pressure')
        self.right_ankle_flex_plot.setYRange(-1,self.parent.max_pressure)
        self.right_ankle_flex_plot.setLabel('left','Pressure (psi)')
        self.right_ankle_flex_plot.setLabel('bottom','Time (sec)')
        self.right_ankle_flex_line = self.right_ankle_flex_plot.plot(pen=pg.mkPen('b', width=1))
        self.right_ankle_flex_yaxis = self.right_ankle_flex_plot.getAxis('left')
        self.right_ankle_flex_yaxis.setTickSpacing(20,10)

        self.setWindowTitle('Pressure')


        # init data
        self.l_tank_data = deque([0.0]*parent.dataLen)
        self.l_kne_ext_data = deque([0.0]*parent.dataLen)
        self.l_kne_flex_data = deque([0.0]*parent.dataLen)
        self.l_ank_ext_data = deque([0.0]*parent.dataLen)
        self.l_ank_flex_data = deque([0.0]*parent.dataLen)

        self.r_tank_data = deque([0.0]*parent.dataLen)
        self.r_kne_ext_data = deque([0.0]*parent.dataLen)
        self.r_kne_flex_data = deque([0.0]*parent.dataLen)
        self.r_ank_ext_data = deque([0.0]*parent.dataLen)
        self.r_ank_flex_data = deque([0.0]*parent.dataLen)


    def UpdateData(self,data):
        
        self.l_tank_data.popleft()
        self.l_tank_data.append(data[LTANK_ADC]*0.003125-25)
        self.left_tank_line.setData(self.l_tank_data)
        
        self.l_kne_ext_data.popleft()
        self.l_kne_ext_data.append(data[LKNE_EXT_ADC]*0.003125-25)
        self.left_knee_ext_line.setData(self.l_kne_ext_data)
        
        self.l_kne_flex_data.popleft()
        self.l_kne_flex_data.append(data[LKNE_FLEX_ADC]*0.003125-25)
        self.left_knee_flex_line.setData(self.l_kne_flex_data)

        self.l_ank_ext_data.popleft()
        self.l_ank_ext_data.append(data[LANK_EXT_ADC]*0.003125-25)
        self.left_ankle_ext_line.setData(self.l_ank_ext_data)

        # self.l_ank_flex_data.popleft()
        # self.l_ank_flex_data.append(data[LANK_FLEX_ADC]*0.003125-25)
        # self.left_ankle_flex_line.setData(self.l_ank_flex_data)

        #right, 
        # TODO: use the correct unit and data

        self.r_tank_data.popleft()
        self.r_tank_data.append(data[RTANK_ADC]*0.003125-25)
        self.right_tank_line.setData(self.r_tank_data)
        
        self.r_kne_ext_data.popleft()
        self.r_kne_ext_data.append(data[RKNE_EXT_ADC]*0.003125-25)
        self.right_knee_ext_line.setData(self.r_kne_ext_data)
        
        self.r_kne_flex_data.popleft()
        self.r_kne_flex_data.append(data[RKNE_FLEX_ADC]*0.003125-25)
        self.right_knee_flex_line.setData(self.r_kne_flex_data)

        self.r_ank_ext_data.popleft()
        self.r_ank_ext_data.append(data[RANK_EXT_ADC]*0.003125-25)
        self.right_ankle_ext_line.setData(self.r_ank_ext_data)

        # self.l_ank_flex_data.popleft()
        # self.l_ank_flex_data.append(data[LANK_FLEX]*0.003125-25)
        # self.left_ankle_flex_line.setData(self.l_ank_flex_data)
        

        self.parent.app.processEvents()
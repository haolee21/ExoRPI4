from PyQt5.QtWidgets import QWidget
from PyQt5 import uic
from pyqtgraph import GraphicsLayoutWidget
import pyqtgraph as pg
from collections import deque
class PlotPressureWindow(QWidget):
    def __init__(self,parent=None):
        super().__init__()
        self.parent =parent
        uic.loadUi('two_col_graph.ui',self)
        self.left_plot_widget = self.findChild(GraphicsLayoutWidget,'left_graphicsView')
        self.left_plot_widget.setBackground('w')
        self.left_kneePre_plot = self.left_plot_widget.addPlot(colspan=1,title='Knee Pressure')
        self.left_kneePre_plot.setYRange(-30,self.parent.max_pressure)
        self.left_kneePre_plot.setLabel('left','Pressure (psi)')
        self.left_kneePre_plot.setLabel('bottom','Time (sec)')
        self.left_kneePre_line = self.left_kneePre_plot.plot(pen=pg.mkPen('b', width=1))

        self.left_plot_widget.nextRow()
        self.left_anklePre_plot = self.left_plot_widget.addPlot(colspan=1,title='Ankle Pressure')
        self.left_anklePre_plot.setYRange(-30,self.parent.max_pressure)
        self.left_anklePre_plot.setLabel('left','Pressure (psi)')
        self.left_anklePre_plot.setLabel('bottom','Time (sec)')
        self.left_anklePre_line = self.left_anklePre_plot.plot(pen=pg.mkPen('b', width=1))

        self.right_plot_widget = self.findChild(GraphicsLayoutWidget,'right_graphicsView')
        self.right_plot_widget.setBackground('w')
        self.right_kneePre_plot = self.right_plot_widget.addPlot(colspan=1,title='Knee Pressure')
        # self.right_kneePre_plot.setYRange(-30,self.parent.max_pressure) #TODO: change the unit back to psi when fully integrated
        self.right_kneePre_plot.setYRange(0,2**16)
        self.right_kneePre_plot.setLabel('left','Pressure (psi)')
        self.right_kneePre_plot.setLabel('bottom','Time (sec)')
        self.right_kneePre_line = self.right_kneePre_plot.plot(pen=pg.mkPen('b', width=1))

        self.right_plot_widget.nextRow()
        self.right_anklePre_plot = self.right_plot_widget.addPlot(colspan=1,title='Ankle Pressure')
        # self.right_anklePre_plot.setYRange(-30,self.parent.max_pressure)
        self.right_anklePre_plot.setYRange(0,2**16)
        self.right_anklePre_plot.setLabel('left','Pressure (psi)')
        self.right_anklePre_plot.setLabel('bottom','Time (sec)')
        self.right_anklePre_line = self.right_anklePre_plot.plot(pen=pg.mkPen('b', width=1))

        self.setWindowTitle('Pressure')


        # init data
        self.l_knePreData = deque([0]*parent.dataLen)
        self.l_ankPreData = deque([0]*parent.dataLen)
        self.r_knePreData = deque([0]*parent.dataLen)
        self.r_ankPreData = deque([0]*parent.dataLen)
    def UpdateData(self,data):
        # the results are from 16 bits ADC, MSB, LSB
        # print('pressure got update')
        self.l_knePreData.popleft()
        self.l_knePreData.append(int.from_bytes(data[0:2],'little')*0.0038147-25)
        self.left_kneePre_line.setData(self.l_knePreData)
        self.l_ankPreData.popleft()
        self.l_ankPreData.append(int.from_bytes(data[2:4],'little')*0.0038147-25)
        self.left_anklePre_line.setData(self.l_ankPreData)
        self.r_knePreData.popleft()
        self.r_knePreData.append(int.from_bytes(data[4:6],'little')) #*0.0038147-25) #TODO: change them back to pressure when force sensor test is done
        self.right_kneePre_line.setData(self.r_knePreData)
        self.r_ankPreData.popleft()
        self.r_ankPreData.append(int.from_bytes(data[6:8],'little')) #*0.0038147-25)
        self.right_anklePre_line.setData(self.r_ankPreData)

        self.parent.app.processEvents()
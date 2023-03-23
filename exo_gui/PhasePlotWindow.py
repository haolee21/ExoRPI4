import pyqtgraph as pg
from pyqtgraph import GraphicsLayoutWidget
from PyQt5 import uic
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QGraphicsScene,QDialog
from collections import deque
from ExoDataStruct import *
class PhasePlot(QDialog):
    def __init__(self,parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.Window)
        uic.loadUi('UI/Phase.ui',self)
        self.scene = QGraphicsScene(self)
        self.plot = pg.PlotItem()
        self.phase_plot_widget = self.findChild(GraphicsLayoutWidget,'FSM_Phase')
        self.phase_plot_widget.addItem(self.plot)

        self.scatter = pg.ScatterPlotItem()
        self.plot.addItem(self.scatter)
        self.phase_plot_widget.setBackground('w')

        self.plot.setXRange(-90,90)
        self.plot.setYRange(-90,90)

        self.x_data = deque([0.0]*1000)
        self.y_data = deque([0.0]*1000)
        self.scatter.setData(self.x_data, self.y_data)
    def Update(self,data):
        self.x_data.popleft()
        self.y_data.popleft()
        self.x_data.append(data[ENC_LHIP_S]-data[ENC_LKNE_S]+data[ENC_LANK_S])
        self.y_data.append(data[ENC_RHIP_S]-data[ENC_RKNE_S]+data[ENC_RANK_S])
        self.scatter.setData(self.x_data, self.y_data)
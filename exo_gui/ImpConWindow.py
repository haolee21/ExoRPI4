from PyQt5 import uic
from PyQt5.QtWidgets import QWidget,QLabel,QLineEdit,QPushButton
from PyQt5.QtGui import QPalette
from PyQt5.QtGui import QColor
from PyQt5.QtCore import Qt
from functools import partial
from TCP_Con import TextToFloat
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
        self.btn_lkne_set.clicked.connect(partial(self.ImpSet,self.lineEdit_lkne,'LKNE'))
        self.btn_lkne_stop.clicked.connect(partial(self.ImpStop,'LKNE'))

        self.lineEdit_lank.textChanged.connect(partial(self.TextChange,self.lineEdit_lank))
        self.btn_lank_set.clicked.connect(partial(self.ImpSet,self.lineEdit_lank,'LANK'))
        self.btn_lank_stop.clicked.connect(partial(self.ImpStop,'LANK'))

        self.lineEdit_rkne.textChanged.connect(partial(self.TextChange,self.lineEdit_rkne))
        self.btn_rkne_set.clicked.connect(partial(self.ImpSet,self.lineEdit_rkne,'RKNE'))
        self.btn_rkne_stop.clicked.connect(partial(self.ImpStop,'RKNE'))

        self.lineEdit_rank.textChanged.connect(partial(self.TextChange,self.lineEdit_rank))
        self.btn_rank_set.clicked.connect(partial(self.ImpSet,self.lineEdit_rank,'RANK'))
        self.btn_rank_stop.clicked.connect(partial(self.ImpStop,'RANK'))
       

    def TextChange(self,line_edit):
        palette = QPalette()
        palette.setColor(QPalette.Text,Qt.blue)
        line_edit.setPalette(palette)
    def ImpSet(self,line_edit,joint_name):
        palette = QPalette()
        palette.setColor(QPalette.Text,Qt.black)
        line_edit.setPalette(palette)
        self.parent.tcp_port.SendCmd('SET:IMP:'+joint_name+':'+str(TextToFloat(line_edit.text())),byte_to_read=1,print_response=True)
        self.parent.tcp_port.SendCmd('ACT:IMP:'+joint_name+':1',byte_to_read=1,print_response=True)
        self.parent.tcp_port.SendCmd('ACT:MPC:'+joint_name+':1',byte_to_read=1,print_response=True)
    def ImpStop(self,joint_name):
        self.parent.tcp_port.SendCmd('ACT:IMP:'+joint_name+':0',byte_to_read=1,print_response=True)
        self.parent.tcp_port.SendCmd('ACT:MPC:'+joint_name+':0',byte_to_read=1,print_response=True)


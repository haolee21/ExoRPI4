from PyQt5 import uic
from PyQt5.QtWidgets import QWidget,QLabel,QLineEdit,QPushButton
from PyQt5.QtGui import QPalette
from PyQt5.QtGui import QColor
from PyQt5.QtCore import Qt
from functools import partial

class ImpWindow(QWidget):
    def __init__(self, parent= None):
        super().__init__()
        self.parent = parent
        uic.loadUi("Imp_window.ui",self)
        self.btn_lkne_set = self.findChild(QPushButton,'btn_lkne_set')
        self.btn_lkne_cancel = self.findChild(QPushButton,'btn_lkne_cancel')
        self.btn_lank_set = self.findChild(QPushButton,'btn_lank_set')
        self.btn_lank_cancel = self.findChild(QPushButton,'btn_lank_cancel')
        self.btn_rkne_set = self.findChild(QPushButton,'btn_rkne_set')
        self.btn_rkne_cancel = self.findChild(QPushButton,'btn_rkne_cancel')
        self.btn_rank_set = self.findChild(QPushButton,'btn_rank_set')
        self.btn_rank_cancel = self.findChild(QPushButton,'btn_rank_cancel')

        self.lineEdit_lkne = self.findChild(QLineEdit,'lineEdit_lkne')
        self.lineEdit_lank = self.findChild(QLineEdit,'lineEdit_lank')
        self.lineEdit_rkne = self.findChild(QLineEdit,'lineEdit_rkne')
        self.lineEdit_rank = self.findChild(QLineEdit,'lineEdit_rank')
        
        self.lineEdit_lkne.textChanged.connect(partial(self.TextChange,self.lineEdit_lkne))
        self.btn_lkne_set.clicked.connect(partial(self.ImpSet,self.lineEdit_lkne))
        self.lineEdit_lank.textChanged.connect(partial(self.TextChange,self.lineEdit_lank))
        self.btn_lank_set.clicked.connect(partial(self.ImpSet,self.lineEdit_lank))

        self.lineEdit_rkne.textChanged.connect(partial(self.TextChange,self.lineEdit_rkne))
        self.btn_rkne_set.clicked.connect(partial(self.ImpSet,self.lineEdit_rkne))
        self.lineEdit_rank.textChanged.connect(partial(self.TextChange,self.lineEdit_rank))
        self.btn_rank_set.clicked.connect(partial(self.ImpSet,self.lineEdit_rank))
       

    def TextChange(self,line_edit):
        palette = QPalette()
        palette.setColor(QPalette.Text,Qt.blue)
        line_edit.setPalette(palette)
    def ImpSet(self,line_edit):
        palette = QPalette()
        palette.setColor(QPalette.Text,Qt.black)
        line_edit.setPalette(palette)

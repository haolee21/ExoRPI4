from PyQt5.QtWidgets import QPushButton,QLabel,QLineEdit,QWidget,QDialog
from PyQt5.QtCore import Qt
from PyQt5 import uic
class ConnectionWindow(QDialog):
    def __init__(self,parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.Window) # if you don't set it, it will not show up in alt-tab (for QDialog)
        uic.loadUi('UI/Con_window.ui',self)
        btn_ok = self.findChild(QPushButton,'btn_ok')
        btn_ok.clicked.connect(self.btn_ok_clicked)
        btn_cancel = self.findChild(QPushButton,'btn_cancel')
        btn_cancel.clicked.connect(self.btn_cancel_clicked)
        btn_apply = self.findChild(QPushButton,'btn_apply')
        btn_apply.clicked.connect(self.btn_apply_clicked)

    
        self.ip_input = self.findChild(QLineEdit,'ip_lineEdit')
        self.ip_input.setText(self.parent().udp_port.ip_address)
        

    def btn_ok_clicked(self):
        self.finish_set()
        self.close()
    def btn_cancel_clicked(self):
        self.close()    
    def btn_apply_clicked(self):
        self.finish_set()
    def closeEvent(self,event):
        self.ip_input.setText(self.parent().udp_port.ip_address)
       

    def finish_set(self):
        self.parent().udp_port.ip_address=self.ip_input.text()
        self.parent().cur_ip.setText(self.parent().udp_port.ip_address)

from PyQt5.QtWidgets import QPushButton,QLabel,QLineEdit,QWidget
from PyQt5 import uic
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

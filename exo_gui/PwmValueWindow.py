from PyQt5.QtWidgets import QWidget,QLCDNumber
from PyQt5 import uic

from ExoDataStruct import *
class PwmValueWindow(QWidget):
    def __init__(self,parent=None):
        super().__init__()
        self.parent = parent
        uic.loadUi('UI/PWM_value_window.ui',self)
        self.lcd_ltank = self.findChild(QLCDNumber,'lcd_ltank')
        self.lcd_lkne_ext = self.findChild(QLCDNumber,'lcd_lkne_ext')
        self.lcd_lkne_flex = self.findChild(QLCDNumber,'lcd_lkne_flex')
        self.lcd_lank_ext = self.findChild(QLCDNumber,'lcd_lank_ext')
        self.lcd_lkne_ank = self.findChild(QLCDNumber,'lcd_lkne_ank')
        self.lcd_lkne_exut = self.findChild(QLCDNumber,'lcd_lkne_exut')
        self.lcd_lank_exut = self.findChild(QLCDNumber,'lcd_lank_exut')
        
        self.lcd_rtank = self.findChild(QLCDNumber,'lcd_rtank')
        self.lcd_rkne_ext = self.findChild(QLCDNumber,'lcd_rkne_ext')
        self.lcd_rkne_flex = self.findChild(QLCDNumber,'lcd_rkne_flex')
        self.lcd_rank_ext = self.findChild(QLCDNumber,'lcd_rank_ext')
        self.lcd_rkne_ank = self.findChild(QLCDNumber,'lcd_rkne_ank')
        self.lcd_rkne_exut = self.findChild(QLCDNumber,'lcd_rkne_exut')
        self.lcd_rank_exut = self.findChild(QLCDNumber,'lcd_rank_exut')


    def UpdateLCD(self,data):
        
        self.lcd_ltank.display(data[LTANK_PWM])
        self.lcd_lkne_ext.display(data[LKNE_EXT_PWM])
        
        self.lcd_lkne_flex.display(data[LKNE_FLEX_PWM])
        self.lcd_lank_ext.display(data[LANK_EXT_PWM])
        self.lcd_lkne_ank.display(data[LKNE_ANK_PWM])
        self.lcd_lkne_exut.display(data[LKNE_EXUT_PWM])
        self.lcd_lank_exut.display(data[LANK_EXUT_PWM])


        self.lcd_rtank.display(data[RTANK_PWM])
        self.lcd_rkne_ext.display(data[RKNE_EXT_PWM])
        self.lcd_rkne_flex.display(data[RKNE_FLEX_PWM])
        self.lcd_rank_ext.display(data[RANK_EXT_PWM])
        self.lcd_rkne_ank.display(data[RKNE_ANK_PWM])
        self.lcd_rkne_exut.display(data[RKNE_EXUT_PWM])
        self.lcd_rank_exut.display(data[RANK_EXUT_PWM])
      
        


          

from Calibration import *
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget,QLabel,QLineEdit,QPushButton,QRadioButton,QLCDNumber,QDialog,QFileDialog
from ExoDataStruct import *
from Common import *
from PyQt5.QtCore import Qt
import pdb
class CylnCalibWindow(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent) 
        uic.loadUi('UI/CylnCalib.ui',self)
        self.setWindowFlags(Qt.Window) # if you don't set it, it will not show up in alt-tab (for QDialog)

        self.lineEdit_lineEdit_l1 = self.findChild(QLineEdit,'lineEdit_l1')
        self.lineEdit_lineEdit_l2 = self.findChild(QLineEdit,'lineEdit_l2')
        self.lineEdit_lineEdit_l3 = self.findChild(QLineEdit,'lineEdit_l3')

        self.btn_set_l1 = self.findChild(QPushButton,'btn_set_1')
        self.btn_set_l2 = self.findChild(QPushButton,'btn_set_2')
        self.btn_set_l3 = self.findChild(QPushButton,'btn_set_3')

        self.btn_calibrate = self.findChild(QPushButton,'btn_calibrate')
        self.btn_reset = self.findChild(QPushButton,'btn_reset')

        self.radio_lknee = self.findChild(QRadioButton,'radio_left_knee')
        self.radio_lank = self.findChild(QRadioButton,'radio_left_ankle')
        self.radio_rknee = self.findChild(QRadioButton,'radio_right_knee')
        self.radio_rank = self.findChild(QRadioButton,'radio_right_ankle')

        self.radio_joint1=self.findChild(QRadioButton,'radio_joint1')
        self.radio_joint2=self.findChild(QRadioButton,'radio_joint2')
        self.radio_joint3=self.findChild(QRadioButton,'radio_joint3')

        self.lcd_ang_1 = self.findChild(QLCDNumber,'lcd_angle_1')
        self.lcd_ang_2 = self.findChild(QLCDNumber,'lcd_angle_2')
        self.lcd_ang_3 = self.findChild(QLCDNumber,'lcd_angle_3')
        self.lcd_verify_ang = self.findChild(QLCDNumber,'lcd_verify_angle')
        self.lcd_verify_length = self.findChild(QLCDNumber,'lcd_verify_length')
        self.lcd_moment_arm = self.findChild(QLCDNumber,'lcd_moment_arm')

        self.btn_update_exo = self.findChild(QPushButton,'btn_update_exo')
        self.btn_update_exo.clicked.connect(self.UpdateConfig)


        self.lkne_cyln_eqn = self.parent().config_loader.phy_param['LeftKnee']['CylnEqn']
        self.lank_cyln_eqn = self.parent().config_loader.phy_param['LeftAnkle']['CylnEqn']
        self.rkne_cyln_eqn = self.parent().config_loader.phy_param['RightKnee']['CylnEqn']
        self.rank_cyln_eqn = self.parent().config_loader.phy_param['RightAnkle']['CylnEqn']

        self.radio_joint_lock=True # set False when retrieve
        self.radio_joint1.toggled.connect(lambda:self.Joint1Select())
        self.radio_joint2.toggled.connect(lambda:self.Joint2Select())
        self.radio_joint3.toggled.connect(lambda:self.Joint3Select())

        #flags that locks the degree
        self.joint_angle_update=[True]*3

        #saved joint valves/cylinder length
        self.joint_val_list=[0,0,0]
        self.cyln_len_list=[0,0,0]

        self.btn_set_l1.clicked.connect(lambda:self.JointSet(0,self.btn_set_l1))
        self.btn_set_l2.clicked.connect(lambda:self.JointSet(1,self.btn_set_l2))
        self.btn_set_l3.clicked.connect(lambda:self.JointSet(2,self.btn_set_l3))

        # calculate beta0, beta1
        self.btn_calibrate.clicked.connect(self.CalibratingCylnEqn)

        #total reset
        self.btn_reset.clicked.connect(self.ResetCalibAll)
        self.radio_lknee.toggled.connect(self.ResetCalibAll) #reset all when calibrating different joint
        self.radio_lank.toggled.connect(self.ResetCalibAll)
        self.radio_rknee.toggled.connect(self.ResetCalibAll)
        self.radio_rank.toggled.connect(self.ResetCalibAll)
    def UpdateReading(self,enc_data):
        
        joint_angle=0
        cyln_eqn = None
        if self.radio_lknee.isChecked():
            joint_angle=180-enc_data[ENC_LKNE_S]
            cyln_eqn = self.lkne_cyln_eqn

            
            # self.lcd_verify_length.display(math.sqrt(self.beta0_lkne-self.beta1_lkne*math.cos(math.radians(joint_angle-self.alpha_lkne))))
            # self.lcd_verify_length.display(math.sqrt(self.lkne_cyln_eqn[0]-self.lkne_cyln_eqn[1]*math.cos(math.radians(joint_angle-self.lkne_cyln_eqn[2]))))
            # self.lcd_moment_arm.display(0.5*self.lkne)


        elif self.radio_lank.isChecked():
            joint_angle=enc_data[ENC_LANK_S]+180
            # self.lcd_verify_ang.display(joint_angle)
            cyln_eqn = self.lank_cyln_eqn
            # self.lcd_verify_length.display(math.sqrt(self.beta0_lank-self.beta1_lank*math.cos(math.radians(joint_angle-self.alpha_lank))))
            # self.lcd_verify_length.display(math.sqrt(self.lank_cyln_eqn[0]-self.lank_cyln_eqn[1]*math.cos(math.radians(joint_angle-self.lank_cyln_eqn[2]))))
        elif self.radio_rknee.isChecked():
            # if enc_data[ENC_RKNE_S]>180:
            #     joint_angle = 180-enc_data[ENC_RKNE_S]
            # else:
            #     joint_angle=180-enc_data[ENC_RKNE_S]
            joint_angle = 180-enc_data[ENC_RKNE_S]
            cyln_eqn=self.rkne_cyln_eqn
            # self.lcd_verify_ang.display(joint_angle)
            # self.lcd_verify_length.display(math.sqrt(self.beta0_rkne-self.beta1_rkne*math.cos(math.radians(joint_angle-self.alpha_rkne))))
            # self.lcd_verify_length.display(math.sqrt(self.rkne_cyln_eqn[0]-self.rkne_cyln_eqn[1]*math.cos(math.radians(joint_angle-self.rkne_cyln_eqn[2]))))
        elif self.radio_rank.isChecked():
            joint_angle=enc_data[ENC_RANK_S]+180
            cyln_eqn = self.rank_cyln_eqn
            # self.lcd_verify_ang.display(joint_angle)
            # self.lcd_verify_length.display(math.sqrt(self.beta0_rank-self.beta1_rank*math.cos(math.radians(joint_angle-self.alpha_rank))))
            # self.lcd_verify_length.display(math.sqrt(self.rank_cyln_eqn[0]-self.rank_cyln_eqn[1]*math.cos(math.radians(joint_angle-self.rank_cyln_eqn[2]))))
        
        self.lcd_verify_ang.display(joint_angle)
        
        try:
            cyln_len = math.sqrt(cyln_eqn[0]-cyln_eqn[1]*math.cos(math.radians(joint_angle-cyln_eqn[2])))
            self.lcd_verify_length.display(math.sqrt(cyln_len))
            self.lcd_moment_arm.display(0.5*cyln_eqn[1]*math.sin(math.radians(joint_angle-cyln_eqn[2]))/cyln_len)
        except:
            pass
        self.UpdateJointLCD(joint_angle)
        
        
    def UpdateJointLCD(self,joint_val):

        if self.radio_joint1.isChecked() & self.joint_angle_update[0]:
            self.lcd_ang_1.display(joint_val)
        elif self.radio_joint2.isChecked() & self.joint_angle_update[1]:
            self.lcd_ang_2.display(joint_val)
        elif self.radio_joint3.isChecked() & self.joint_angle_update[2]:  
            self.lcd_ang_3.display(joint_val)
        
    def Joint1Select(self):
        if self.radio_joint_lock:
            self.radio_joint_lock = False
            if self.radio_joint2.isChecked():
                self.radio_joint2.setChecked(False)
            if self.radio_joint3.isChecked():
                self.radio_joint3.setChecked(False)
            self.radio_joint_lock=True
    def Joint2Select(self):
        if self.radio_joint_lock:
            self.radio_joint_lock=False
            if self.radio_joint1.isChecked():
                self.radio_joint1.setChecked(False)
            if self.radio_joint3.isChecked():
                self.radio_joint3.setChecked(False)
            self.radio_joint_lock=True
    def Joint3Select(self):
        if self.radio_joint_lock:
            self.radio_joint_lock=False
            if self.radio_joint1.isChecked():
                self.radio_joint1.setChecked(False)
            if self.radio_joint2.isChecked():
                self.radio_joint2.setChecked(False)
            self.radio_joint_lock=True
    def JointSet(self,joint_val_idx,btn_set):
        if self.joint_angle_update[joint_val_idx]:
            self.joint_angle_update[joint_val_idx]=False
            btn_set.setText('Reset')
        else:
            self.joint_angle_update[joint_val_idx]=True
            btn_set.setText('Set')
        
    def CalibratingCylnEqn(self):
        if sum(self.joint_angle_update)!=0:
            print(self.joint_angle_update)
            print('please set 3 measurements')
        else:
            ang_list = [self.lcd_ang_1.value(),self.lcd_ang_2.value(),self.lcd_ang_3.value()]
            len_list = [TextToFloat(self.lineEdit_lineEdit_l1.text()), TextToFloat(self.lineEdit_lineEdit_l2.text()),TextToFloat(self.lineEdit_lineEdit_l3.text())]
            beta0,beta1,alpha = GetJointLenEqn(ang_list,len_list)
            print("beta0=",beta0,', beta1=',beta1, ', alpha=',alpha)
            if self.radio_lknee.isChecked():
                self.lkne_cyln_eqn = [beta0,beta1,alpha].copy()
                # self.beta0_lkne = beta0
                # self.beta1_lkne=beta1
                # self.alpha_lkne = alpha
            elif self.radio_lank.isChecked():
                self.lank_cyln_eqn = [beta0,beta1,alpha].copy()
                # self.beta0_lank = beta0
                # self.beta1_lank = beta1
                # self.alpha_lank = alpha
            elif self.radio_rknee.isChecked():
                self.rkne_cyln_eqn = [beta0,beta1,alpha].copy()
                # self.beta0_rkne = beta0
                # self.beta1_rkne = beta1
                # self.alpha_rkne = alpha
            elif self.radio_rank.isChecked():
                self.rank_cyln_eqn = [beta0,beta1,alpha].copy()
                # self.beta0_rank = beta0
                # self.beta1_rank = beta1
                # self.alpha_rank = alpha

    def ResetCalibAll(self):
        for i in range(len(self.joint_angle_update)):
            self.joint_angle_update[i]=True
        self.btn_set_l1.setText('Set')
        self.btn_set_l2.setText('Set')
        self.btn_set_l3.setText('Set')

    def UpdateConfig(self):
        # load the config file you want to update

        old_config,_ = QFileDialog.getOpenFileName(caption='Config to update',filter="Config file (*.json)",directory=self.parent().config_loader.phy_config_path)
        self.parent().config_loader.LoadPhyConfig(old_config)

        if self.radio_lknee.isChecked():
            self.parent().config_loader.UpdatePhyParam("LeftKnee","CylnEqn",self.lkne_cyln_eqn) #TODO: check if the offset is correct
        elif self.radio_lank.isChecked():
            self.parent().config_loader.UpdatePhyParam("LeftAnkle","CylnEqn",self.lank_cyln_eqn)
        elif self.radio_rknee.isChecked():
            self.parent().config_loader.UpdatePhyParam("RightKnee","CylnEqn",self.rkne_cyln_eqn)
        elif self.radio_rank.isChecked():
            self.parent().config_loader.UpdatePhyParam("RightAnkle","CylnEqn",self.rank_cyln_eqn)

        # save new config
        new_config,_ = QFileDialog.getSaveFileName(caption='Config to update',filter="Config file (*.json)",directory=self.parent().config_loader.phy_config_path)
        if(new_config.find(".json")<0):
            new_config = new_config+".json"
        self.parent().config_loader.SavePhyConfig(new_config)


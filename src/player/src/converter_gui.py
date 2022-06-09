#!/usr/bin/env python3
import sys
import os
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import QCoreApplication, Qt
import subprocess
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Imu
import rospy
import imu_processing as mu
import raw2bag
import open3d
import time

class Main(QDialog):
    def __init__(self):
        super().__init__()
        self.init_open_widgets()
        self.init_ui()
        
    def init_open_widgets(self):
        
        # Global Widgets----------------------------
        self.bag_locate_01 = QLineEdit('', self)
        self.slider_s = QSlider(Qt.Horizontal, self)
        self.slider_g = QSlider(Qt.Horizontal, self)
        self.lidar_cnt = QLineEdit('0', self)
        self.imu_data1 = QLineEdit('', self)
        self.imu_data2 = QLineEdit('', self)
        self.imu_data3 = QLineEdit('', self)
        self.s_sec = QLineEdit('0.0', self)
        self.g_sec = QLineEdit('1.0', self)
        
        # Parameters--------------------------
        self.dt = 0.1
        self.msgs_ind = 10
        self.start_sec = 0
        self.end_sec = 1.0
        
        # Process
        self.rosbag_process = None
        self.rviz_process = None
        self.waits = None
    
    def set_gb(self, function, w, h):
        gb = function
        gb.setMaximumSize(w, h)
        gb.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Preferred)
        return gb
    
    def init_ui(self):
        main_layout = QVBoxLayout()
        
        # [main] Tab
        tab = QTabWidget()
        main_layout.addWidget(tab)
        
        # [Sub] Tab1 -> bag viewer
        widget1 = QWidget()
        lay1 = QVBoxLayout()
        lay1.setSizeConstraint(QLayout.SetMaximumSize)
        widget1.setLayout(lay1)
        
        tab.addTab(widget1, "Bag Viewer")
        lay1.addWidget(self.set_gb(self.read_bag(), 500, 100))
        lay1.addWidget(self.set_gb(self.set_play(), 500, 200))
        lay1.addWidget(self.set_gb(self.set_button(), 500, 130))
        lay1.addWidget(self.set_gb(self.view_datas(), 500, 400))
        
        # [Sub] Tab2 -> bag viewer
        widget2 = QWidget()
        lay2 = QVBoxLayout()
        lay2.setSizeConstraint(QLayout.SetMaximumSize)
        widget2.setLayout(lay2)
        
        tab.addTab(widget2, "Bag Extractor")
        lay2.addWidget(self.set_gb(self.read_and_write_bag(), 500, 300))
        lay2.addWidget(self.set_gb(self.read_and_write_pcd(), 500, 300))
        
        # [Sub] Tab3 -> bag viewer
        widget3 = QWidget()
        lay3 = QVBoxLayout()
        lay3.setSizeConstraint(QLayout.SetMaximumSize)
        widget3.setLayout(lay3)
        
        tab.addTab(widget3, "Raw2bag")
        lay3.addWidget(self.set_gb(self.read_pcd_imu(), 500, 300))
        lay3.addWidget(self.set_gb(self.raw2bag(), 500, 300))

        self.setWindowTitle('Bag Customizer')
        self.setWindowIcon(QIcon('../img/icon.png'))
        self.setLayout(main_layout)
        self.resize(550, 700)
        self.show()
    
    def labels(self):
        lists = []
        lists.append(QLabel("Start Index: "))
        lists.append(QLabel("Start Time(sec): "))
        lists.append(QLabel("Start ->"))
        lists.append(QLabel("Goal Index: "))
        lists.append(QLabel("Goal Time(sec): "))
        lists.append(QLabel("Goal ->"))
        return lists
    
    def add_open(self, bag_locate):
        file_open = QFileDialog.getOpenFileName(self, 'Open file', bag_locate.text())
        bag_locate.setText(file_open[0])
    
    def add_find(self, label):
        file_open = QFileDialog.getExistingDirectory(self, 'Find Folder')
        label.setText(file_open)
    
    def cloud_loader(self, msg):
        self.lidar_cnt.setText(str(len(pc2.read_points_list(msg))))
        
    def imu_loader(self, msg):
        orientation = "%.6f, %.6f, %.6f, %.6f"%(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.imu_data1.setText(orientation)
        angular_velocity = "%.7f, %.7f, %.7f"%(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        self.imu_data2.setText(angular_velocity)
        linear_acceleration = "%.7f, %.7f, %.7f"%(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        self.imu_data3.setText(linear_acceleration)
        self.waits.sleep()
        
    
    def start_view(self, start_btn, delay_check, delays, loop_check, defaults, t_lidar, t_imu, rvizs):
        if self.slider_s.value() > self.slider_g.value():
            QMessageBox.warning(self, 'Warning', '시작 시간이 끝 시간보다 큽니다.')
        else:
            if start_btn.text() == 'Start':
                start_btn.setText('Stop')
                
                dt_s = str(float(self.s_sec.text()) - float(self.start_sec))
                dur = str(float(self.g_sec.text()) - float(self.s_sec.text()))
                
                if delay_check.isChecked():
                    self.rosbag_process = subprocess.Popen(('gnome-terminal', '--', 'rosbag', 'play', '-s', dt_s, '-u', dur,
                                                            '-d', delays.text(), self.bag_locate_01.text()), stdout=subprocess.PIPE)
                
                if loop_check.isChecked():
                    self.rosbag_process = subprocess.Popen(('gnome-terminal', '--', 'rosbag', 'play', '-l', '-s', dt_s, '-u', dur,
                                                            self.bag_locate_01.text()), stdout=subprocess.PIPE)
                
                if defaults.isChecked():
                    self.rosbag_process = subprocess.Popen(('gnome-terminal', '--', 'rosbag', 'play', '-s', dt_s, '-u', dur, 
                                                            self.bag_locate_01.text()), stdout=subprocess.PIPE)
                
                # Read Data in real time
                self.waits = rospy.Rate(10)
                rospy.Subscriber(t_lidar, PointCloud2, self.cloud_loader, queue_size=1)
                rospy.Subscriber(t_imu, Imu, self.imu_loader, queue_size=1)
                
                os.popen('rosrun rviz rviz -d `rospack find viewer`' + rvizs)
                
            else:
                os.system('killall -9 play')
                os.system('killall -9 rviz')
                
                start_btn.setText('Start')
        
    def rosbag_info(self, bag_path, s_sec=None, g_sec=None, g_ind=None, slider_s=None, slider_g=None, progress_label=None):
        self.msgs_ind = int(os.popen("rosbag info " + bag_path + " | grep messages: | cut -d ':' -f 2").read())
        self.start_sec = os.popen("rosbag info " + bag_path + " | grep start: | cut -d '(' -f 2- | cut -d ')' -f 1").read()
        self.end_sec = os.popen("rosbag info " + bag_path + " | grep end: | cut -d '(' -f 2- | cut -d ')' -f 1").read()
        
        if g_sec != None:
            g_ind.setText(str(self.msgs_ind))
            s_sec.setText(self.start_sec)
            g_sec.setText(self.end_sec)
            self.dt = (float(self.end_sec) - float(self.start_sec)) / self.msgs_ind
        
        if progress_label != None:
            progress_label.setText('Ready')
        
        if slider_s != None:
            try:
                slider_s.setRange(0, self.msgs_ind)
                slider_g.setRange(0, self.msgs_ind)
                slider_g.setValue(self.msgs_ind)
            except Exception as e:
                pass

# TAB1-------------------------------------------------------
    def read_bag(self):
        
        groupbox = QGroupBox("Read Bag File")
        
        # Button
        open_btn = QPushButton('Open .bag', self)
        open_btn.setFixedSize(80, 25)
        
        # Functions------------------------------------------------
        open_btn.clicked.connect(lambda: self.add_open(self.bag_locate_01))
        
        # Widget Locate---------------------------
        hbox = QHBoxLayout()
        hbox.addWidget(self.bag_locate_01)
        hbox.addWidget(open_btn)
        
        groupbox.setLayout(hbox)
        
        return groupbox

    def set_play(self):
        groupbox = QGroupBox("Play Setting")
        
        # Widgets---------------------------------------------
        # Labels
        s_ind = QLineEdit('0', self)
        g_ind = QLineEdit('0', self)
    
        # Configuration
        self.slider_s.setRange(0, 10)
        self.slider_g.setRange(0, 10)
        
        # Functions------------------------------------------------
        self.bag_locate_01.textChanged.connect(lambda: self.rosbag_info(self.bag_locate_01.text(), self.s_sec, self.g_sec, g_ind, self.slider_s, self.slider_g,))
        self.slider_s.valueChanged.connect(lambda: self.s_value_change( s_ind, self.s_sec))
        self.slider_g.valueChanged.connect(lambda: self.g_value_change( g_ind, self.g_sec))
        s_ind.textChanged.connect(lambda: self.s_sec_change(s_ind))
        g_ind.textChanged.connect(lambda: self.g_sec_change(g_ind))
        
        # Labels----------------
        l_lists = self.labels()
        
        # Widget Locate---------------------------
        vbox = QVBoxLayout()
        hbox = QHBoxLayout()
        hbox.addWidget(l_lists[0])
        hbox.addWidget(s_ind)
        hbox.addWidget(l_lists[1])
        hbox.addWidget(self.s_sec)
        vbox.addLayout(hbox)
        hbox = QHBoxLayout()
        hbox.addWidget(l_lists[2])
        hbox.addWidget(self.slider_s)
        vbox.addLayout(hbox)
        hbox = QHBoxLayout()
        hbox.addWidget(l_lists[3])
        hbox.addWidget(g_ind)
        hbox.addWidget(l_lists[4])
        hbox.addWidget(self.g_sec)
        vbox.addLayout(hbox)
        hbox = QHBoxLayout()
        hbox.addWidget(l_lists[5])
        hbox.addWidget(self.slider_g)
        vbox.addLayout(hbox)
        groupbox.setLayout(vbox)
        
        return groupbox
    
    def set_button(self):
        groupbox = QGroupBox("Setting Topic and Play Button")
        
        # CheckBox
        topic_pcd = QLabel("Lidar:")
        sub_pcd = QLineEdit('/hesai/pandar', self)
        topic_imu = QLabel("IMU:")
        sub_imu = QLineEdit('/imu/data', self)
        rvizs = QLabel("Rviz:")
        r_view = QLineEdit('/rviz/show.rviz', self)
        delay_check = QRadioButton('Delay:', self)
        loop_check = QRadioButton('Loop', self)
        defaults = QRadioButton('Default', self)
        defaults.setChecked(True)
        
        # Button
        start_btn = QPushButton('Start', self)
        start_btn.setFixedSize(80, 25)
        
        # labels = QLabel('1. "Spacebar" -> Pause&Go\n2. "s" -> View next data\n  - while Terminal Pause', self)
        exit_btn = QPushButton('Exit', self)
        exit_btn.setFixedSize(80, 25)
        
        delays = QLineEdit('0.0', self)
        delays.setMaximumWidth(100)
        
        # Functions------------------------------------------------
        start_btn.clicked.connect(lambda: self.start_view(start_btn, delay_check, delays, loop_check, defaults, sub_pcd.text(), sub_imu.text(), r_view.text()))
        exit_btn.clicked.connect(QCoreApplication.instance().quit)
        
        vbox = QVBoxLayout()
        hbox = QHBoxLayout()
        hbox.addWidget(topic_pcd)
        hbox.addWidget(sub_pcd)
        hbox.addWidget(topic_imu)
        hbox.addWidget(sub_imu)
        hbox.addWidget(rvizs)
        hbox.addWidget(r_view)
        vbox.addLayout(hbox)
        hbox = QHBoxLayout()
        hbox.addWidget(defaults)
        hbox.addWidget(loop_check)
        hbox.addStretch(8)
        hbox.addWidget(delay_check)
        hbox.addWidget(delays)
        vbox.addLayout(hbox)
        hbox = QHBoxLayout()
        # hbox.addWidget(labels)
        hbox.addWidget(start_btn)
        hbox.addWidget(exit_btn)
        vbox.addLayout(hbox)
        groupbox.setLayout(vbox)
        
        return groupbox
    
    def view_datas(self):
        groupbox = QGroupBox("Data Viewer")

        label1 = QLabel("LiDAR Count:")
        label2 = QLabel("IMU: Orientation(x, y, z, w)")
        label3 = QLabel("IMU: Angular Velocity(x, y, z)")
        label4 = QLabel("IMU: Linear Acceleration(x, y, z)")
        
        self.lidar_cnt.setMaximumWidth(80)
        
        hbox = QHBoxLayout()
        vbox = QVBoxLayout()
        hbox.addWidget(label1)
        hbox.addWidget(self.lidar_cnt)
        vbox.addLayout(hbox)
        hbox = QHBoxLayout()
        hbox.addWidget(label2)
        hbox.addWidget(self.imu_data1)
        vbox.addLayout(hbox)
        hbox = QHBoxLayout()
        hbox.addWidget(label3)
        hbox.addWidget(self.imu_data2)
        vbox.addLayout(hbox)
        hbox = QHBoxLayout()
        hbox.addWidget(label4)
        hbox.addWidget(self.imu_data3)
        vbox.addLayout(hbox)
        groupbox.setLayout(vbox)
        
        return groupbox

    def s_value_change(self, s_ind, s_sec):
        s_ind.setText(str(self.slider_s.value()))
        s_sec.setText(str(round(float(self.start_sec) + (self.dt * self.slider_s.value()), 3)))
        
    def g_value_change(self, g_ind, g_sec):
        g_ind.setText(str(self.slider_g.value()))
        try:
            g_sec.setText(str(round(float(self.start_sec) + (self.dt * self.slider_g.value()), 3)))
        except Exception as e:
            pass
    
    def s_sec_change(self, s_ind):
        try:
            self.slider_s.setValue(int(s_ind.text()))
        except ValueError as e:
            self.slider_s.setValue(0)
    
    def g_sec_change(self, g_ind):
        try:
            self.slider_g.setValue(int(g_ind.text()))
        except ValueError as e:
            self.slider_g.setValue(0)   
    
# TAB2-------------------------------------------------------------
    def index_change(self, ind, sec):
        if ind.text() == '': 
            ind.setText(str(0))
            index = 0
        else:
            index = int(ind.text())
        
        if int(ind.text()) > self.msgs_ind:
            ind.setText(str(self.msgs_ind))
            index = self.msgs_ind
            
        sec.setText(str(round(float(self.start_sec) + (self.dt * index), 3)))
    
    def pre_extractor(self,bag_locate_02, bag_locate_03, progress_label, num, s_sec=0, g_sec=0, bag_name='', pcd_name='', imu_name=''):
        if s_sec > g_sec:
            QMessageBox.warning(self, 'Warning', '시작 시간이 끝 시간보다 큽니다.')
            return
        if bag_locate_02 == '':
            QMessageBox.warning(self, 'Warning', 'Bag 파일을 불러오십시오.')
            return
        if bag_locate_03 == '':
            QMessageBox.warning(self, 'Warning', '저장할 폴더 경로를 불러오십시오.')
            return
        
        if num == 1:
            if bag_name == '':
                QMessageBox.warning(self, 'Warning', '저장할 파일 이름을 입력하세요.')
                return
        elif num == 2:
            if pcd_name == '':
                QMessageBox.warning(self, 'Warning', 'PCD Topic을 입력하세요')
                return        
            if imu_name == '':
                QMessageBox.warning(self, 'Warning', 'IMU Topic을 입력하세요')
                return
        progress_label.setText("Extracting..")
            
    def post_extractor(self, bag_locate_02, bag_locate_03, store_name, progress_label, num, sub_pcd='', sub_imu='', s_sec=0, g_sec=0):
        if progress_label.text() == 'Extracting..':
            if num == 1:
                time_range = '"t.to_sec() >= ' + str(s_sec) + ' and t.to_sec() <= ' + str(g_sec) + '"' 
                os.system('rosbag filter ' + bag_locate_02 + ' ' + bag_locate_03 + '/' +
                        store_name + '.bag ' + time_range)
            elif num == 2:
                storing = bag_locate_03 + '/' + store_name
                
                if not os.path.isdir(storing):
                    os.mkdir(storing)
                
                
                os.system('rosrun pcl_ros bag_to_pcd ' + bag_locate_02 +
                          ' ' + sub_pcd + ' ' + (storing + '/pcd'))
                print("-----Finish PCD-----")
                
                n = len(bag_locate_02.split('/')[-1])
                bag_folder = bag_locate_02[:-n]
                mu.imu_extract(bag_folder, bag_locate_02.split('/')[-1], storing, sub_imu)
            
            progress_label.setText("Finish!")
        
    def read_and_write_pcd(self):
        groupbox = QGroupBox("Read Bag and Extract [PCD/IMU]")
        
        # Widgets---------------------------------------------
        bag_locate_02 = QLineEdit('', self)
        bag_locate_03 = QLineEdit('', self)
        topic_pcd = QLabel("PCD Topic:")
        sub_pcd = QLineEdit('/hesai/pandar', self)
        topic_imu = QLabel("IMU Topic:")
        sub_imu = QLineEdit('/imu/data', self)
        progress_label = QLineEdit("", self)
        store_name = QLineEdit('extracted_data', self)
        file_name = QLabel("Store Folder Name:")
        num = 2
        
        btn1 = QPushButton('Open .bag', self)
        btn1.setFixedSize(80, 25)
        
        btn2 = QPushButton('Save Folder', self)
        btn2.setFixedSize(80, 25)
        
        start_btn = QPushButton('Extract', self)
        start_btn.setFixedSize(80, 25)
        
        exit_btn = QPushButton('Exit', self)
        exit_btn.setFixedSize(80, 25)        
         # Widgets---------------------------------------------
        
        # Functions---------------------------------------
        bag_locate_02.textChanged.connect(lambda: self.rosbag_info(bag_locate_02.text(), progress_label=progress_label))
        btn1.clicked.connect(lambda: self.add_open(bag_locate_02))
        btn2.clicked.connect(lambda: self.add_find(bag_locate_03))
        start_btn.clicked.connect(lambda: self.pre_extractor(bag_locate_02.text(), bag_locate_03.text(), progress_label, num, 
                                                             pcd_name=sub_pcd.text(), imu_name=sub_imu.text()))
        progress_label.textChanged.connect(lambda: self.post_extractor(bag_locate_02.text(), bag_locate_03.text(), store_name.text(), 
                                                                       progress_label, num, sub_pcd=sub_pcd.text(),sub_imu=sub_imu.text()))
        exit_btn.clicked.connect(QCoreApplication.instance().quit)
        # Functions---------------------------------------
        
        # Located Widgets------------------------------------
        hbox = QHBoxLayout()
        hbox.addWidget(bag_locate_02)
        hbox.addWidget(btn1)
        vbox = QVBoxLayout()
        vbox.addLayout(hbox)
        hbox = QHBoxLayout()
        hbox.addWidget(bag_locate_03)
        hbox.addWidget(btn2)
        vbox.addLayout(hbox)
        hbox = QHBoxLayout()
        hbox.addWidget(topic_pcd)
        hbox.addWidget(sub_pcd)
        hbox.addStretch(10)
        hbox.addWidget(topic_imu)
        hbox.addWidget(sub_imu)
        vbox.addLayout(hbox)      
        hbox = QHBoxLayout()
        hbox.addWidget(file_name)
        hbox.addWidget(store_name)
        hbox.addStretch(8)
        hbox.addWidget(start_btn)
        hbox.addWidget(exit_btn)
        vbox.addLayout(hbox) 
        hbox = QHBoxLayout()
        hbox.addWidget(progress_label)
        vbox.addLayout(hbox)
        groupbox.setLayout(vbox)
        # Located Widgets------------------------------------
        
        return groupbox
        
    def read_and_write_bag(self):
        groupbox = QGroupBox("Read Bag and Extract Bag")
        
        # Widgets---------------------------------------------
        bag_locate_02 = QLineEdit('', self)
        bag_locate_03 = QLineEdit('', self)
        s_ind = QLineEdit('0', self)
        s_sec = QLineEdit('0.0', self)
        g_ind = QLineEdit('0', self)
        g_sec = QLineEdit('0.0', self)
        bag_store_name = QLineEdit('default', self)
        file_name = QLabel("Bag File Name:")
        progress_label = QLineEdit("", self)
        
        btn1 = QPushButton('Open .bag', self)
        btn1.setFixedSize(80, 25)
        
        btn2 = QPushButton('Save .bag', self)
        btn2.setFixedSize(80, 25)
        
        start_btn = QPushButton('Extract', self)
        start_btn.setFixedSize(80, 25)
        
        exit_btn = QPushButton('Exit', self)
        exit_btn.setFixedSize(80, 25)

        num = 1
        # Widgets---------------------------------------------
        
        # Functions---------------------------------------
        bag_locate_02.textChanged.connect(lambda: self.rosbag_info(bag_locate_02.text(), s_sec, g_sec, g_ind, progress_label=progress_label))
        s_ind.textChanged.connect(lambda: self.index_change(s_ind, s_sec))
        g_ind.textChanged.connect(lambda: self.index_change(g_ind, g_sec))
        btn1.clicked.connect(lambda: self.add_open(bag_locate_02))
        btn2.clicked.connect(lambda: self.add_find(bag_locate_03))
        start_btn.clicked.connect(lambda: self.pre_extractor(bag_locate_02.text(), bag_locate_03.text(), progress_label, num, s_sec=float(s_sec.text()),
                                                             g_sec=float(g_sec.text()), bag_name=bag_store_name.text()))
        progress_label.textChanged.connect(lambda: self.post_extractor(bag_locate_02.text(), bag_locate_03.text(), bag_store_name.text(),
                                                                       progress_label, num, s_sec=float(s_sec.text()), g_sec=float(g_sec.text())))
        exit_btn.clicked.connect(QCoreApplication.instance().quit)
        # Functions---------------------------------------
        
        # Located Widgets------------------------------------
        hbox = QHBoxLayout()
        hbox.addWidget(bag_locate_02)
        hbox.addWidget(btn1)
        vbox = QVBoxLayout()
        vbox.addLayout(hbox)
        hbox = QHBoxLayout()
        hbox.addWidget(bag_locate_03)
        hbox.addWidget(btn2)
        vbox.addLayout(hbox)
        l_lists = self.labels()
        hbox = QHBoxLayout()
        hbox.addWidget(l_lists[0])
        hbox.addWidget(s_ind)
        hbox.addWidget(l_lists[1])
        hbox.addWidget(s_sec)
        vbox.addLayout(hbox)
        hbox = QHBoxLayout()
        hbox.addWidget(l_lists[3])
        hbox.addWidget(g_ind)
        hbox.addWidget(l_lists[4])
        hbox.addWidget(g_sec)
        vbox.addLayout(hbox)
        hbox = QHBoxLayout()
        hbox.addWidget(file_name)
        hbox.addWidget(bag_store_name)
        hbox.addStretch(10)
        hbox.addWidget(start_btn)
        hbox.addWidget(exit_btn)
        vbox.addLayout(hbox)       
        hbox = QHBoxLayout()
        hbox.addWidget(progress_label)
        vbox.addLayout(hbox) 
        groupbox.setLayout(vbox)
        # Located Widgets------------------------------------
        
        return groupbox

# TAB3-------------------------------------------------------------
    def open_pcd(self, pcd_locate):
        pcds = open3d.io.read_point_cloud(pcd_locate.text())
        open3d.visualization.draw_geometries([pcds], zoom=0.3412, front=[0.1257, 0, 2.8795],
                                  lookat=[1.6172, 2.0475, 0.532],
                                  up=[0.0694, 1.0, 0.2024])
    
    def raw2bag(self):
        groupbox = QGroupBox("[PCD/IMU]to Bag File")
        # Widgets---------------------------------------------
        pcd_locate = QLineEdit('', self)
        imu_locate = QLineEdit('', self)
        LIDAR = QPushButton('Lidar', self)
        LIDAR.setFixedSize(80, 25)
        IMU = QPushButton('IMU', self)
        IMU.setFixedSize(80, 25)
        save_locate = QLineEdit('', self)  
        SAVE = QPushButton('SAVE', self)     
        SAVE.setFixedSize(80, 25)
        topic_pcd = QLabel("PCD Topic:")
        sub_pcd = QLineEdit('/hesai/pandar', self)
        topic_imu = QLabel("IMU Topic:")
        sub_imu = QLineEdit('/imu/data', self)
        frames = QLabel("Frame ID:")
        frame_id = QLineEdit('PandarXT-32', self)
        file_name = QLabel("Save File Name:")
        store_name = QLineEdit('Sample', self)
        progress_label = QLineEdit("", self)        
        start_btn = QPushButton('Extract', self)
        start_btn.setFixedSize(80, 25)
        exit_btn = QPushButton('Exit', self)
        exit_btn.setFixedSize(80, 25)
        
        # Functions--------------------------------------------------
        LIDAR.clicked.connect(lambda: self.add_find(pcd_locate))
        IMU.clicked.connect(lambda: self.add_open(imu_locate))
        SAVE.clicked.connect(lambda: self.add_find(save_locate))
        start_btn.clicked.connect(lambda: raw2bag.r2b(pcd_locate.text(), imu_locate.text(), save_locate.text() + '/' + store_name.text(), 
                                                      frame_id.text(), sub_pcd.text(), sub_imu.text(), progress_label))
        exit_btn.clicked.connect(QCoreApplication.instance().quit)
        
        # Located Widgets------------------------------------
        hbox = QHBoxLayout()
        hbox.addWidget(pcd_locate)
        hbox.addWidget(LIDAR)
        vbox = QVBoxLayout()
        vbox.addLayout(hbox)
        hbox = QHBoxLayout()
        hbox.addWidget(imu_locate)
        hbox.addWidget(IMU)
        vbox.addLayout(hbox)
        hbox = QHBoxLayout()
        hbox.addWidget(save_locate)
        hbox.addWidget(SAVE)
        vbox.addLayout(hbox)
        hbox = QHBoxLayout()
        hbox.addWidget(topic_pcd)
        hbox.addWidget(sub_pcd)
        hbox.addWidget(topic_imu)
        hbox.addWidget(sub_imu)
        vbox.addLayout(hbox)
        hbox = QHBoxLayout()
        hbox.addWidget(frames)
        hbox.addWidget(frame_id)
        hbox.addWidget(file_name)
        hbox.addWidget(store_name)
        vbox.addLayout(hbox)
        hbox = QHBoxLayout()
        hbox.addWidget(start_btn)
        hbox.addWidget(exit_btn)
        hbox.addWidget(progress_label)
        vbox.addLayout(hbox)
        groupbox.setLayout(vbox)
        # Located Widgets------------------------------------
        
        return groupbox
    
    def read_pcd_imu(self):
        groupbox = QGroupBox("View PCD or IMU Data")
        # Widgets---------------------------------------------
        pcd_locate = QLineEdit('', self)
        imu_locate = QLineEdit('', self)
        btn1 = QPushButton('Open .pcd', self)
        btn1.setFixedSize(80, 25)
        btn2 = QPushButton('Open .csv', self)
        btn2.setFixedSize(80, 25)
        view_pcd = QPushButton('View', self)
        view_pcd.setFixedSize(80, 25)
        view_imu = QPushButton('View', self)
        view_imu.setFixedSize(80, 25)                
        exit_btn = QPushButton('Exit', self)
        exit_btn.setFixedSize(80, 25)
        # Widgets---------------------------------------------
        
        # Functions---------------------------------------
        btn1.clicked.connect(lambda: self.add_open(pcd_locate))
        btn2.clicked.connect(lambda: self.add_open(imu_locate))
        view_pcd.clicked.connect(lambda: self.open_pcd(pcd_locate))
        view_imu.clicked.connect(lambda: mu.imu_plot(imu_locate.text()))
        exit_btn.clicked.connect(QCoreApplication.instance().quit)
        # Functions---------------------------------------
        
        # Located Widgets------------------------------------
        hbox = QHBoxLayout()
        hbox.addWidget(pcd_locate)
        hbox.addWidget(btn1)
        vbox = QVBoxLayout()
        vbox.addLayout(hbox)
        vbox.addWidget(view_pcd)
        hbox = QHBoxLayout()
        hbox.addWidget(imu_locate)
        hbox.addWidget(btn2)
        vbox.addLayout(hbox)
        vbox.addWidget(view_imu)
        vbox.addWidget(exit_btn)
        groupbox.setLayout(vbox)
        # Located Widgets------------------------------------
        
        return groupbox

if __name__ == '__main__':
    # os.system('killall -9 rosmaster')
    
    # rosmasters = subprocess.Popen(('roscore'), stdout=subprocess.PIPE)
    # time.sleep(1)
    
    rospy.init_node('Data_cache', anonymous=True)
    app = QApplication(sys.argv)
    main = Main()
    app.exec_()
    # rosmasters.terminate()
    # os.system('killall -9 roslaunch')
    # os.system('killall -9 rosmaster')
    sys.exit()

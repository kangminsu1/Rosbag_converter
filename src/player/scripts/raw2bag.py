import os
import numpy as np
import rosbag
import rospy
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import Imu, PointField
import csv

def r2b(l_pcd, l_imu, l_save, frame_id, t_lidar, t_imu, progress):
    
    compression = rosbag.Compression.NONE
    bag = rosbag.Bag(l_save + ".bag", 'w', compression=compression)
    
    print("**Exporting Lidar Data**")
    velo_filenames = sorted(os.listdir(l_pcd))
    
    for f_name in velo_filenames:
        dt = f_name[:-4]

        scan = (np.fromfile(l_pcd+'/'+ f_name, dtype=np.float32)).reshape(-1, 4)
        
        header = Header()
        header.frame_id = frame_id
        header.stamp = rospy.Time.from_sec(float(dt))
        
        # fill pcd messages
        field = [PointField('x', 0, PointField.FLOAT32, 1),
                 PointField('y', 4, PointField.FLOAT32, 1),
                 PointField('z', 8, PointField.FLOAT32, 1),
                 PointField('intensity', 16, PointField.FLOAT32, 1)]
        pcd_msg = pcl2.create_cloud(header, field, scan)

        bag.write(t_lidar, pcd_msg, t=pcd_msg.header.stamp)
        
    
    print("**Exporting IMU Data**")

    f = open(l_imu, 'r', encoding='utf-8')
    datas = csv.reader(f)

    for line in datas:
        if line[0] != 'Time':
            dt = float(line[0]) / pow(10, 9)
            imu = Imu()
            imu.header.frame_id = frame_id
            imu.header.stamp = rospy.Time.from_sec(dt)
            imu.orientation.x = float(line[1])
            imu.orientation.y = float(line[2])
            imu.orientation.z = float(line[3])
            imu.orientation.w = float(line[4])
            imu.angular_velocity.x = float(line[5])
            imu.angular_velocity.y = float(line[6])
            imu.angular_velocity.z = float(line[7])
            imu.linear_acceleration.x = float(line[8])
            imu.linear_acceleration.y = float(line[9])
            imu.linear_acceleration.z = float(line[10])

            bag.write(t_imu, imu, t=imu.header.stamp)

    bag.close()

    progress.setText("Finish!!")

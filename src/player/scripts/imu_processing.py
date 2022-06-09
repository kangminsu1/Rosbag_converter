#!/usr/bin/env python3
import numpy as np
import rospy
import csv
import rosbag
import os
import matplotlib.pyplot as plt


def imu_extract(bag_folder, bag_name, store_name, imu_topic):
    # Parameters
    data_text = '/imu.csv'
    
    t0 = rospy.get_time()
    
    bag_name = bag_folder + '/' + bag_name
    i_msgs = os.popen("rosbag info " + bag_name + " | grep " + imu_topic + " | cut -d ':' -f 2").read()
    
    bagfile = rospy.get_param(bag_folder, bag_name)
    topic = rospy.get_param(i_msgs[1:], imu_topic)
    bag = rosbag.Bag(bagfile)
    n = bag.get_message_count(topic)
    
    data = np.zeros((10, n))
    time_sample = np.zeros((2, n))
    print("Total IMU Data = ", n)
    
    cnt = 0
    avgSampleRate = 0
    for topic, msg, t in bag.read_messages(topics=[topic]):
        data[0,cnt] = msg.orientation.x
        data[1,cnt] = msg.orientation.y
        data[2,cnt] = msg.orientation.z
        data[3,cnt] = msg.orientation.w
        data[4,cnt] = msg.angular_velocity.x
        data[5,cnt] = msg.angular_velocity.y
        data[6,cnt] = msg.angular_velocity.z
        data[7,cnt] = msg.linear_acceleration.x
        data[8,cnt] = msg.linear_acceleration.y
        data[9,cnt] = msg.linear_acceleration.z

        time_sample[0,cnt] = msg.header.stamp.secs*pow(10,9) + msg.header.stamp.nsecs;
        if cnt > 1:
            time_sample[1,cnt] = pow(10,9) / (time_sample[0,cnt] - time_sample[0,cnt-1])
            avgSampleRate = avgSampleRate + time_sample[1,cnt]
        cnt = cnt + 1

    # sampleRate = avgSampleRate/(cnt-3)
    bag.close()

    print("[%0.2f seconds] bagfile parsed\n"%(rospy.get_time() - t0))

    f = open(store_name + data_text, 'wt')
    
    try:
        writer = csv.writer(f)
        writer.writerow( ('Time', 'Ox', 'Oy', 'Oz', 'Ow',
                          'Gx', 'Gy', 'Gz', 'Ax', 'Ay', 'Az') )

        for i in range(data[1,:].size):
            writer.writerow( (time_sample[0,i], data[0,i], data[1,i], data[2,i],
                              data[3,i], data[4,i], data[5,i], data[6,i],
                              data[7,i], data[8,i], data[9,i]))
    finally:
        f.close()

    print("---------Finish IMU---------")
    
def imu_plot(file):
    f = open(file, 'r', encoding='utf-8')
    datas = csv.reader(f)

    times = []
    orientation = []
    angular_v = []
    linear_accel = []
    for line in datas:
        if line[0] != 'Time':
            dt = round((float(line[0]) / pow(10, 9) % 10000), 3)
            times.append(dt)
            orientation.append([line[1], line[2], line[3], line[4]])
            angular_v.append([line[5], line[6], line[7]])
            linear_accel.append([line[1], line[2], line[3]])

    times = np.array(times)
    orientation = np.array(orientation).astype(float)
    angular_v = np.array(angular_v).astype(float)
    linear_accel = np.array(linear_accel).astype(float)

    fig, axs = plt.subplots(3, 1, figsize=(13, 8))
    fig.subplots_adjust(hspace=0.5)
    for i in range(3):
        axs[i].grid(axis='x')
        axs[i].grid(axis='y')
        
    axs[0].plot(times, orientation[:, 0], '-', label='x')
    axs[0].plot(times, orientation[:, 1], '-', label='y')
    axs[0].plot(times, orientation[:, 2], '-', label='z')
    axs[0].plot(times, orientation[:, 3], '-', label='w')
    axs[0].set_title('Orientation')
    axs[0].set_xlabel('sec.nsec')
    axs[0].set_ylabel('Radian')
    axs[0].legend()

    axs[1].plot(times, angular_v[:, 0], '-', label='x')
    axs[1].plot(times, angular_v[:, 1], '-', label='y')
    axs[1].plot(times, angular_v[:, 2], '-', label='z')
    axs[1].set_title('Angular Velocity')
    axs[1].set_xlabel('sec.nsec')
    axs[1].set_ylabel('rad/s')
    axs[1].legend()

    axs[2].plot(times, linear_accel[:, 0], '-', label='x')
    axs[2].plot(times, linear_accel[:, 1], '-', label='y')
    axs[2].plot(times, linear_accel[:, 2], '-', label='z')
    axs[2].set_title('Linear Acceleration')
    axs[2].set_xlabel('sec.nsec')
    axs[2].set_ylabel('m/s^2')
    axs[2].legend()
    plt.show()


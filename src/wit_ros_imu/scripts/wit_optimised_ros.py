#!/usr/bin/env python3
# -*- coding:utf-8 -*-
from serial import Serial
import rospy
import signal
from math import pi
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Time


exit_ros = False

# Parsing serial Port Data
def handleSerialData(raw_data: bytearray):
    values = [int(val) for val in raw_data[1:-1].decode("ascii").split(";")]

    if len(values) != 12:
        return
    msg_time = rospy.rostime.Time((values[0] * 60 + values[1]) * 60 + values[2], values[3] * 1000000)

    if values[10]: # LiDAR
        lidar_pub.publish(Time(msg_time))
    if values[11]: # Camera
        camera_pub.publish(Time(msg_time))
        rospy.loginfo("Sent " + str(msg_time.nsecs))
    
    imu_msg = Imu()

    imu_msg.header.stamp = msg_time
    imu_msg.header.frame_id = "base_link"

    imu_msg.linear_acceleration = Vector3(*[val / 32768.0 * 16 for val in values[4:7]])
    imu_msg.angular_velocity = Vector3(*[val / 32768.0 * 2000 * pi / 180 for val in values[7:10]])

    imu_pub.publish(imu_msg)

def signal_handler(sig, frame):
    global exit_ros
    exit_ros = True


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node("imu")
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baudrate = rospy.get_param("~baud", 9600)
    print("IMU Type: Normal Port:%s baud:%d" %(port,baudrate))
    try:
        wt_imu = Serial(port=port, baudrate=baudrate, timeout=0.5)
        if wt_imu.isOpen():
            rospy.loginfo("\033[32mSerial port opened successfully...\033[0m")
        else:
            wt_imu.open()
            rospy.loginfo("\033[32mSerial port opened successfully...\033[0m")
    except Exception as e:
        print(e)
        rospy.loginfo("\033[31mSerial port opening failure\033[0m")
        exit(0)
    else:
        imu_pub = rospy.Publisher("wit/imu", Imu, queue_size=40)
        lidar_pub = rospy.Publisher("wit/lidar", Time, queue_size=1)
        camera_pub = rospy.Publisher("wit/camera", Time, queue_size=1)

        while not rospy.is_shutdown() and not exit_ros:
            try:
                buff_data = wt_imu.read_until(b'\x03')
                print(buff_data)
                if buff_data.startswith(b"\x02"):
                    handleSerialData(buff_data)
            except Exception as e:
                print("exception:" + str(e))
                print("imu disconnect")
                exit_ros = True
        
        rospy.loginfo("Exiting cleanly")
        wt_imu.cancel_read()
        wt_imu.close()


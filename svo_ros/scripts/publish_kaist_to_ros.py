#!/usr/bin/env python2
from __future__ import print_function
import os
import sys
import csv
import cv2 as cv
import rospy
import cv_bridge
from sensor_msgs.msg import Image, Imu

KAIST_PATH_PRE = "/home/marcus/Documents/VisualInertialDatasets/KAIST/"
KAIST_PATH_POST = "/image/stereo_left/"
DATASET_NAME = "urban18-highway"
IMG_TOPIC_NAME = "image0"
IMU_TOPIC_NAME = "imu0"
FPS = 10

def imread_debayer(img_path):
    image = cv.imread(img_path, cv.IMREAD_UNCHANGED)
    if image.size == 0:
        print("Could not read image at {}".format(img_path))
    return cv.cvtColor(image, cv.COLOR_BayerRG2GRAY)

def get_filenames_at(path, sort=True):
    for _, _, files in os.walk(path): filenames = files
    if sort: filenames = sorted(filenames)
    return filenames

def publish_images(filenames, publisher, framerate):
    rate = rospy.Rate(framerate)
    bridge = cv_bridge.CvBridge()
    try:
        for filename in filenames:
            print("Reading image {}".format(filename))
            img = imread_debayer(IMG_PATH + filename)
            img_message = bridge.cv2_to_imgmsg(img)
            publisher.publish(img_message)
            rate.sleep()
            if rospy.is_shutdown(): return
    except rospy.ROSInterruptException:
        print("Exited because of ROS Interrupt.")
        return

def publish_data(dataset_path, img_pub, imu_pub):
    with open(dataset_path + 'sensor_data/data_stamp.csv') as data_stamps, open(dataset_path + 'sensor_data/xsens_imu.csv') as imu_stamps:
        stamp_reader = csv.reader(data_stamps)
        imu_reader = csv.reader(imu_stamps)
        bridge = cv_bridge.CvBridge()
        time_diff = None
        last_imu_msg = None
        curr_img_msg = None
        try:
            for timestamp, sensor_name in stamp_reader:
                if rospy.is_shutdown(): break
                if time_diff == None:
                    first_stamp = timestamp_str_to_ros(timestamp)
                    time_diff = first_stamp - rospy.Time.now()
                    new_image = False

                if sensor_name == "stereo":
                    img = imread_debayer(IMG_PATH + timestamp + ".png")
                    img_message = bridge.cv2_to_imgmsg(img)

                    img_message.header.stamp = timestamp_str_to_ros(timestamp) - time_diff
                    curr_img_msg = img_message
                    # while (rospy.Time.now() < img_message.header.stamp) and not rospy.is_shutdown(): pass
                    # 
                    new_image = True

                elif sensor_name == "imu":
                    imu_line = imu_reader.next()
                    imu_message = Imu()
                    imu_message.header.stamp = timestamp_str_to_ros(timestamp) - time_diff
                    imu_message.orientation.x = float(imu_line[1])
                    imu_message.orientation.y = float(imu_line[2])
                    imu_message.orientation.z = float(imu_line[3])
                    imu_message.orientation.w = float(imu_line[4])
                    imu_message.angular_velocity.x = float(imu_line[5])
                    imu_message.angular_velocity.y = float(imu_line[6])
                    imu_message.angular_velocity.z = float(imu_line[7])
                    imu_message.linear_acceleration.x = float(imu_line[11])
                    imu_message.linear_acceleration.y = float(imu_line[12])
                    imu_message.linear_acceleration.z = float(imu_line[13])
                    
                    # Pseudo synchronization of image to closest IMU message
                    if new_image and last_imu_msg is not None:
                        diff_to_last = abs(curr_img_msg.header.stamp - last_imu_msg.header.stamp)
                        diff_to_curr = abs(curr_img_msg.header.stamp - imu_message.header.stamp)
                        if diff_to_last < diff_to_curr: imu_pub_msg = last_imu_msg
                        else: imu_pub_msg = imu_message
                        curr_img_msg.header.stamp = imu_pub_msg.header.stamp
                        img_pub.publish(curr_img_msg)
                        new_image = False

                    while (rospy.Time.now() < imu_message.header.stamp) and not rospy.is_shutdown(): pass
                    imu_pub.publish(imu_message)
                    last_imu_msg = imu_message
                else:
                    pass
        except rospy.ROSInterruptException:
            print( "Exited because of ROS Interrupt.")
            return


def timestamp_str_to_ros(timestamp):
    secs = int(timestamp[0:10])
    nsecs = int(timestamp[10:19])
    return rospy.Time(secs, nsecs)

if __name__ == '__main__':
    if len(sys.argv) > 1: DATASET_NAME = sys.argv[1]
    if len(sys.argv) > 2: IMG_TOPIC_NAME = sys.argv[2]
    if len(sys.argv) > 3: IMU_TOPIC_NAME = sys.argv[3]
    DATASET_PATH = KAIST_PATH_PRE + DATASET_NAME + '/'
    IMG_PATH = KAIST_PATH_PRE + DATASET_NAME + KAIST_PATH_POST
    
    file_locations = get_filenames_at(IMG_PATH)
    
    img_pub = rospy.Publisher(IMG_TOPIC_NAME, Image, queue_size=5)
    imu_pub = rospy.Publisher(IMU_TOPIC_NAME, Imu, queue_size=5)
    rospy.init_node('kaist_publisher', anonymous=True)
    # publish_images(file_locations, img_pub, FPS)
    publish_data(DATASET_PATH, img_pub, imu_pub)

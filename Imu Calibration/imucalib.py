#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

imu_raw_data = Imu()
magnet_raw_data = MagneticField()

def imucallback(msg):
    global imu_raw_data
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg)
    msg.orientation_covariance =[0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
    magnet_raw_data.magnetic_field = msg.orientation
    magnet_raw_data.magnetic_field_covariance =[0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
    magnet_raw_data.header = msg.header
    imu_raw_data.header = msg.header
    imu_raw_data.linear_acceleration = msg.linear_acceleration
    imu_raw_data.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
    imu_raw_data.angular_velocity = msg.angular_velocity
    imu_raw_data.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
   
    
    
def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/imu", Imu, imucallback)
    pub = rospy.Publisher("imu/data_raw", Imu , queue_size=10 )
    pub_magnet = rospy.Publisher("imu/mag", MagneticField , queue_size=10 )
    rate = rospy.Rate(200)  #200Hz
    while not rospy.is_shutdown():
        pub.publish(imu_raw_data)
        pub_magnet.publish(magnet_raw_data)
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

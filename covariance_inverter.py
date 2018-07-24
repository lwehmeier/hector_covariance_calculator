#!/usr/bin/python
import math
from math import sin, cos, pi, radians
import rospy
import tf
from std_msgs.msg import Float32, Int32, Int16, Float64, Header
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped
import struct
import numpy as np
def callbackPose(poseCV):
    covariance = []
    for i in range(0,6):
        covariance.append([])
        for j in range(0,6):
            covariance[i].append(poseCV.pose.covariance[6*i+j])
    covariance = np.array(covariance)
    covariance = np.linalg.pinv(covariance)
    msg = PoseWithCovarianceStamped()
    msg.header = Header(poseCV.header.seq, poseCV.header.stamp, poseCV.header.frame_id)
    msg.pose = poseCV.pose
    data = []
    for i in covariance:
        for j in i:
            data.append(float(j))
    msg.pose.covariance = data
    ppub.publish(msg)
    print("(x,y,yaw): (" + str(covariance[0,0]) + ", " + str(covariance[1,1]) + ", " + str(covariance[5,5]) + ")")
rospy.init_node('hector_covariance_calculator')
ppub = rospy.Publisher("/poseupdate_fixed", PoseWithCovarianceStamped, queue_size=5)
rospy.Subscriber("/poseupdate", PoseWithCovarianceStamped, callbackPose, queue_size=5)
r = rospy.Rate(10) # 10hz
rospy.spin()

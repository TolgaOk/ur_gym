# THIS IS FOR TEST

from environment import ur10_env
import os
import numpy as np

import rospy
from controllers import control

joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
arm_control = control("velocity", joint_names=joint_names)

joint_names = ["joint_{}".format(i) for i in range(16)]
hand_control = control("effort", joint_names=joint_names)


data_arm = np.zeros(6, dtype=np.float64)
data_hand = np.ones(16, dtype=np.float64)*1

rospy.init_node("try_node")
r = rospy.Rate(10)
try:
    while True:
        arm_control.publish_all(data_arm)
        hand_control.publish_all(data_hand)
        r.sleep()
except KeyboardInterrupt:
    print "done!"

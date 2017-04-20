import rospy
from std_srvs.srv import Empty
import time
from gazebo_msgs.srv import DeleteModel
import roslaunch
from environment import ur10_env
import numpy as np


rospy.init_node("basic_node", anonymous=True)
env = ur10_env("velocity", 5)
for j in range(4):
    for i in range(50):
        a = env._step(np.random.normal(0., 1, size=(6)))
        if i == 40:
            print a[1]
    env._reset()

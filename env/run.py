
import numpy as np
import time


import rospy
from sensor_msgs.msg import Image
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelConfiguration
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from pick_and_lift import PickAndLift

env = PickAndLift(5)
for j in range(20):
    image, joint_states = env.reset()
    for i in range(50):
        env.step(np.random.normal(size=22))
time.sleep(15)

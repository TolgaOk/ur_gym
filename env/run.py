import rospy
import time, os
from environment import ur10_env
import numpy as np

# env = ur10_env("velocity", 10)
# for j in range(400):
#     for i in range(50):
#         a = env._step(np.clip(np.random.normal(size=(6)))
#     env._reset()

print "/".join(os.path.realpath(__file__).split("/")[:-2] + ["launch", "ur10_gym.launch"])

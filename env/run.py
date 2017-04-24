# import rospy
# import time, os
# from environment import ur10_env
import numpy as np

# env = ur10_env("velocity", 10)
# for j in range(400):
#     for i in range(50):
#         a = env._step(np.clip(np.random.normal(size=(6)))
#     env._reset()

def generate_target():
    pi = np.pi
    phi = np.random.uniform(pi/6, pi/3)
    theta = np.random.uniform(pi/3, 2*pi/3)
    r = np.random.uniform(0.7, 1.4)

    z = np.cos(phi)*r
    x = np.sin(phi)*np.cos(theta)*r
    y = np.sin(phi)*np.sin(theta)*r

    return ((x, y, z), (r, theta, phi))

print generate_target()[0]

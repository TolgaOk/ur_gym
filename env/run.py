import rospy
from std_srvs.srv import Empty
import time
from gazebo_msgs.srv import DeleteModel
import roslaunch
from environment import ur10_env
import numpy as np


# rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
# rospy.wait_for_service("/gazebo/unpause_physic", timeout=0.01)

# rospy.ServiceProxy("/gazebo/pause_physics", Empty)()
# rospy.ServiceProxy("/gazebo/unpause_physics", Empty)()

# rospy.ServiceProxy("/gazebo/pause_physics", Empty)()
# rospy.ServiceProxy("/gazebo/unpause_physics", Empty)()
# rospy.ServiceProxy("/gazebo/reset_world", Empty)()
rospy.init_node("basic_node", anonymous=True)
env = ur10_env("velocity", 5)
for j in range(4):
    for i in range(50):
        a = env._step(np.random.normal(0., 1, size=(6)))
    env._reset()




# time.sleep(50)
# rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)("robot")
# uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
# roslaunch.configure_logging(uuid)
# l = roslaunch.parent.ROSLaunchParent(uuid, ["/home/tolga/gazebo_tutorial/ur10_gym/src/ur_gym/launch/ur10_gym.launch"])
# l.start()
# # rospy.init_node("node_gym")
# time.sleep(50)
# l.shutdown()


# rospy.ServiceProxy("/gazebo/unpause_physics", Empty)()
# from gazebo_msgs.srv import SetModelConfiguration
# rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)("robot", "", ["shoulder_lift_joint"], [0.0])

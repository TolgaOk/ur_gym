import roslaunch
import rospy
from gazebo_msgs.srv import DeleteModel


# del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel) # Handle to model spawner
#     # rospy.wait_for_service('gazebo/delete_model') # Wait for the model loader to be ready
#     # FREEZES EITHER WAY
# del_model_prox("robot")



uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/tolga/gazebo_tutorial/ur10_gym/src/ur_gym/launch/ur10_gym.launch"])
launch.start()

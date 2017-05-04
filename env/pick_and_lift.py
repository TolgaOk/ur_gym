from environment import ur10_env
from controllers import control
import rospy
from sensor_msgs.msg import Image
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelConfiguration
import os, time
import numpy as np
from cv_bridge import CvBridge

class PickAndLift(ur10_env):
    """
    """
    def __init__(self, frequency):
        launch_path = "/".join(os.path.realpath(__file__).split("/")[:-2] + ["launch", "PickAndLift.launch"])
        super(PickAndLift, self).__init__("velocity", frequency, launch_path)

        self.action_dimension = 22
        self.observation_dimension = ((160, 160), 44)
        self.object_position = lambda : rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)("cylindir", "").pose.position
        self.hand_initial_joint_angle = np.zeros(16, dtype=np.float32)

        #Task parameter.          # UNIT
        self.limit_distance = 0.1 # meter
        self.reach_goal = 1.5     # reward
        self.pick_goal = 5.0      # reward

        joint_names = ["joint_{}".format(i) for i in range(16)]
        self.hand_control = control("effort", joint_names=joint_names)
        self.reset_hand = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)

    def _publish_function(self, action):
        self.action_control.publish_all(action[0:6])
        self.hand_control.publish_all(action[6:])

    def _camera_callback(self, data):
        self.serialized_image = data

    def step(self, action):
        self.camera_subscriber = rospy.Subscriber("/ur10/camera/image_raw", Image, callback=self._camera_callback, queue_size=1)
        joint_states, tip_position = self._step(action)
        pose = self.object_position()
        target = (pose.x, pose.y, pose.z)
        distance = self._distance_to_target(tip_position, target)
        reward = self._reward(distance, pose.z)
        image = CvBridge().imgmsg_to_cv2(self.serialized_image, "rgb8")
        return ((image, joint_states), reward, False, {})


    def reset(self):

        self.set_random_object_position("cylindir")
        self.reset_hand("robot", "", self.hand_control.joint_names, self.hand_initial_joint_angle)
        joint_states = self._reset()
        self.unpause()
        raw_image = rospy.wait_for_message("/ur10/camera/image_raw", Image, 10.0)
        print "--------------------------------------------------------"
        self.pause()
        image = CvBridge().imgmsg_to_cv2(raw_image, "rgb8")
        return (image, joint_states)



    def set_random_object_position(self, name):
        """ ################################
            ## CC ##########################
            ## CC ##++++++++++++############
            ########++++++++++++#### UR10 ##
            ########++++++++++++#### UR10 ##
            ########++++++++++++#### UR10 ##
            ########++++++++++++#### UR10 ##
            ########++++++++++++############
            ################################
            ################################
            Position of the object is randomly chosen from a uniform distribution.
            Object can be spawn anywhere on the '+' area.
        """
        set_model = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        model_state = ModelState()
        model_state.model_name = name
        model_state.pose.position.x = np.random.uniform(low=0.4, high=1.0)
        model_state.pose.position.y = np.random.uniform(low=-0.6, high=0.6)
        model_state.reference_frame = "world"
        set_model(model_state)

    def _reward(self, distance, z):
        r1 = self.reach_goal if  distance < self.limit_distance else np.exp(-distance)
        r2 = self.pick_goal if z > 0.1 else 0.0
        return r1 + r2

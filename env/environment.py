import rospy
import numpy as np
# import shlex, subprocess
import roslaunch
import time
import sys
import os
import tf
import numpy as np
import threading
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import DeleteModel
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from controllers import position_contol, velocity_contol, acceleration_contol, effort_contol

class ur10_env(object):
    def __init__(self, action_type, frequency, transmission_path=None, launch_path=None):

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        self.main_launch_path = "/home/tolga/gazebo_tutorial/ur10_gym/src/ur_gym/launch/ur10_gym.launch"
        self.robot_launch_path = "/home/tolga/gazebo_tutorial/ur10_gym/src/ur_gym/launch/ur10_spawn.launch"
        self.launch = lambda path: roslaunch.parent.ROSLaunchParent(uuid, [path])

        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.joints_initializer = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)

        self.action_dimension = 6
        self.observation_dimension = 14

        self.target = [1, 1, 1]
        self.task_limit = 0.01
        self.initial_angle = np.array([1, 1, 0, 1, 0, 0], dtype=np.float32)*-1*np.pi/2

        action_controllers = {name: obj for name, obj in zip(["postion", "velocity", "acceleration", "effort"],
                                        [position_contol, velocity_contol, acceleration_contol, effort_contol])}


        transmission_path = "/".join(os.path.realpath(__file__).split("/")[:-3]) + "/universal_robot/ur_description/urdf" if False else None
        launch_path = "/".join(os.path.realpath(__file__).split("/")[:-2]) + "/launch" if False else None
        kwargs = {"transmission_path": transmission_path, "launch_path": launch_path}
        self.action_control = action_controllers[action_type](**kwargs)

        self.joint_names = self.action_control.joint_names
        self.frequency = frequency
        self.period = rospy.Duration(1./frequency)




        try:
            rospy.wait_for_service("/gazebo/pause_physics", timeout=0.01)
            print "gazebo on!"
            # rospy.init_node("node_gym")
        except rospy.exceptions.ROSException:
            self.launch(self.main_launch_path).start(auto_terminate=True)



        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform("base_link", "ee_link", rospy.Time(0), rospy.Duration(5.0));
        self._reset()


        #TODO: Write a state refresher
        #TODO: Write a safety mechanism for unexpected break of ros

    def _step(self, action):
        """Initilizations"""

        self._state = None
        is_done = False
        self._transform_msg = [None, None]
        self.position = None


        def update_state_callback(data):
            self._state = data
            try:
                position, quaternion = self.tf_listener.lookupTransform("/base_link", "/ee_link", rospy.Time(0))
                self.position = position
            except KeyError:
                pass

        def update_tf_callback(data):
            if data:
                if len(data.transforms) == 6:
                    self._transform_msg[0] = data
                else:
                    self._transform_msg[1] = data

        state_subscriber = rospy.Subscriber("/joint_states", JointState, callback=update_state_callback, queue_size=1)
        rospy.wait_for_service("/gazebo/unpause_physics", timeout=None)

        self.unpause()

        start_ros_time = rospy.Time.now()
        while True:
            self.action_control.publish_all(action)
            elapsed_time = rospy.Time.now() - start_ros_time
            if elapsed_time > self.period*(4.0/5):
                if elapsed_time > self.period:
                    self.pause()
                    break
                else:
                    rospy.sleep(self.period-elapsed_time)
                    self.pause()
                    break
            else:
                rospy.sleep(self.period/5.0)

        # ros_joint_names = {name: index for name, index in zip(self._state.name, range(len(self._state.name)))}

        #TODO: Make simple state that is also syncronized with actions and joint names
        self._state = np.concatenate([self._state.position, self._state.velocity], axis=0)

        distance_to_target = np.sqrt(np.sum([(x-y)**2 for x, y in zip(self.position, self.target)]))
        if distance_to_target < self.task_limit:
            is_done = True
            reward = 2.
        else:
            reward = -distance_to_target

        return (self._state, reward, is_done, dict())

    def _reset_model(self):

        try:
            rospy.wait_for_service("/gazebo/unpause_physics", timeout=0.2)
            self.unpause()
        except:
            print "No model found! When resetting."
            return None

        try:
            rospy.wait_for_service("/gazebo/delete_model", timeout=0.2)
            rospy.ServiceProxy("/controller_manager/unload_controller", Empty)
            rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)("robot")
        except:
            print "No model found! When resetting."
            return None

        robot = self.launch(self.robot_launch_path)
        robot.start(auto_terminate=False)

        while True:
            try:
                rospy.wait_for_message("/joint_states", JointState, timeout=1.0)
                break
            except:
                print "Wainting for robot topics (/joint_states) to republish again!"

        self.pause()
        robot.shutdown()

    def _reset(self):


        rospy.wait_for_service("/gazebo/unpause_physics", timeout=2)
        self.unpause()
        self.joints_initializer("robot", "", self.joint_names, self.initial_angle)
        _state = rospy.wait_for_message("/joint_states", JointState, timeout=0.2)
        _state = np.concatenate([_state.position, _state.velocity], axis=0)
        return _state

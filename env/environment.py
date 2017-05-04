# MIT License
#
# Copyright (c) 2017 Tolga Ok
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rospy
import numpy as np
import roslaunch
import time
import os
import tf
import numpy as np
from gazebo_msgs.srv import SetModelConfiguration
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from controllers import position_contol, velocity_contol, acceleration_contol, effort_contol

class ur10_env(object):
    """ This is a base class for ur10 manupulation tasks. It can be used to initialize
        launch file, controller type, tf listener and ros node.
        Methods:
        _step:
            This method returns position of the tip of the ur10 robot and next state.
            It publishes given action for 1/5f seconds. Note that, actions are considered
            as at the same order with the initial joint name list.

            Args:
                action: A length 6 iterable corresponding to each joint action.
            Return:
                tip_position: X, Y, Z coordinate of the tip of the ur10.
                next_state: Last values for the joint postion and their time derivatives.

        _reset:
            This method pause the gazebo and then resets the ur10 joint angles to the
            initial joint angles. After that it listens joint states for a single message
            to return it. It does not refreshs the ros time.

            Return:
                state: Joint position and time derivaties of the last state.

    """
    def __init__(self, action_type, frequency, launch_file_path):

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        self.main_launch_path = launch_file_path
        self.launch = lambda path: roslaunch.parent.ROSLaunchParent(uuid, [path])

        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.joints_initializer = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)

        # self.target = np.array([0.8, 0.8, 0.8], dtype=np.float32)
        # self.task_limit = 0.1
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

        rospy.init_node("ur_gym_node", anonymous=True)

        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform("base_link", "ee_link", rospy.Time(0), rospy.Duration(5.0));
        self._reset()

    def _step(self, action):
        """Initilizations"""

        self._state = None
        self.tip_position = None

        def update_state_callback(data):
            self._state = data
            position, quaternion = self.tf_listener.lookupTransform("/base_link", "/ee_link", rospy.Time(0))
            self.tip_position = position
            self._additional_state_callback()

        state_subscriber = rospy.Subscriber("/joint_states", JointState, callback=update_state_callback, queue_size=1)

        rospy.wait_for_service("/gazebo/unpause_physics", timeout=None)
        self.unpause()

        start_ros_time = rospy.Time.now()
        while True:
            self._publish_function(action)
            elapsed_time = rospy.Time.now() - start_ros_time
            if elapsed_time > self.period*(4.0/5):
                if elapsed_time > self.period:
                    break
                else:
                    rospy.sleep(self.period-elapsed_time)
                    break
            else:
                rospy.sleep(self.period/5.0)
        rospy.wait_for_service("/gazebo/pause_physics", timeout=0.1)
        self.pause()

        state = np.concatenate([map(lambda i: i%np.pi, self._state.position), self._state.velocity], axis=0)
        return (state, self.tip_position)

    def _reset(self):

        rospy.wait_for_service("/gazebo/unpause_physics", timeout=2)
        self.joints_initializer("robot", "", self.joint_names, self.initial_angle)
        self.unpause()
        _state = rospy.wait_for_message("/joint_states", JointState, timeout=1.0)
        self.pause()
        _state = np.concatenate([map(lambda i: i%(2*np.pi), _state.position), _state.velocity], axis=0)
        return _state

    def _additional_state_callback(self):
        """This method is an empthy method for additional sensor informations that child
        classes may add.
        """
        pass

    def _publish_function(self, action):
        self.action_control.publish_all(action)


    def _distance_to_target(self, link, target):
        return np.sqrt(np.sum([(x-y)**2 for x, y in zip(link, target)]))

    def _render(self):
        print "In terminal, write; gzclient "

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
from std_msgs.msg import Float64
import xml.etree.ElementTree as ET
import sys

class control(object):
    """ Initializing this will generate publishers for all joints in ur10.
        It can be used to publish the data to all joints.
        This is a base class for controller classes in ur10.
        Methods:
        -publish_all
            Publishs the argument "data" for all six joints.
        -modifier
            This method modifies launch and transmission files in order to
            control ur10 with a given control method. It modifies by default
            file name and file path, so the file name and the file path of
            both transmission file in the ur_description package(catkin package)
            and launch file "ur10_gym.launch" must not be changed.
    """
    def __init__(self, control_method_string, joint_names=None):
        self.control_method_string = control_method_string
        if not joint_names:
            self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        else:
            self.joint_names = joint_names
        topic_name = lambda name: "/{}_controller/{}_controller/command".format(control_method_string, name)
        self.publishers = {name: rospy.Publisher(topic_name(name), Float64, queue_size=1) for name in self.joint_names}


    def publish_all(self, data):
        try:
            for joint in self.joint_names:
                self.publishers[joint].publish(data[joint])
        except (TypeError, IndexError):
            for i, joint in enumerate(self.joint_names):
                self.publishers[joint].publish(data[i])

    def modifier(self, transmission_path, launch_path):
        transmission_modifier(transmission_path, self.control_method_string)
        launch_modifier(launch_path, self.control_method_string)

class position_contol(control):

    def __init__(self, transmission_path=None, launch_path=None):
        super(self.__class__, self).__init__("position")
        if transmission_path and launch_path:
            self.modifier(transmission_path, launch_path)

class velocity_contol(control):

    def __init__(self, transmission_path=None, launch_path=None):
        super(self.__class__, self).__init__("velocity")
        if transmission_path and launch_path:
            self.modifier(transmission_path, launch_path)

class acceleration_contol(control):

    def __init__(self, transmission_path=None, launch_path=None):
        super(self.__class__, self).__init__("acceleration")
        if transmission_path and launch_path:
            self.modifier(transmission_path, launch_path)

class effort_contol(control):

    def __init__(self, transmission_path=None, launch_path=None):
        super(self.__class__, self).__init__("effort")
        if transmission_path and launch_path:
            self.modifier(transmission_path, launch_path)

def transmission_modifier(xacro_path, control):
    """ This function changes the transmission.xacro file according to the control argument.
        Xacro path is the path for ur.transmission.xacro
    """
    ET.register_namespace("xacro", "http://ros.org/wiki/xacro")
    if not xacro_path.split("/")[-1].split(".")[-1] == "xacro":
        xacro_path = xacro_path + "/ur.transmission.xacro"
    _method = control_method(control)

    parser = ET.parse(xacro_path)
    root = parser.getroot()
    for interface in root.iter("hardwareInterface"):
        interface.text = _method["interface"]

    parser.write(xacro_path)

def launch_modifier(launch_path, control):
    """ This function changes the .launch file according to the control argument.
        Launch path is the path for ur10.launch. Name of the launch file must be ->ur10.launch<-
    """
    if not len(launch_path.split("/")[-1].split(".")) > 1:
        launch_path = launch_path + "/ur10_controller.launch"
    _method = control_method(control)

    parser = ET.parse(launch_path)
    root = parser.getroot()
    for node in root.iter("node"):
        _arg = "/".join([_method["spawner"]] + [node.get("args").split("/")[-1]])
        node.set('args', _arg)

    param = root.iter("rosparam").next()
    param.set("file", "/".join(param.get("file").split("/")[:-1] + [_method["controller_file"]]))

    parser.write(launch_path)

def control_method(control):

    if control == "position":
        return {"interface": "PositionJointInterface", "spawner": "spawn postion_controller", "controller_file": "position_control.yaml"}
    elif control == "velocity":
        return {"interface": "VelocityJointInterface", "spawner": "spawn velocity_controller", "controller_file": "velocity_control.yaml"}
    elif control == "acceleration":
        return {"interface": "AccelerationJointInterface", "spawner": "spawn acceleration_controller", "controller_file": "acceleration_control.yaml"}
    elif control == "effort":
        return {"interface": "EffortJointInterface", "spawner": "spawn effort_controller", "controller_file": "effort_control.yaml"}
    else:
        raise ValueError("Argument for function ->control_method<- is invalid.")
    return None

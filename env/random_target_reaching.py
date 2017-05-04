# MIT License
#
# Copyright (c) 2016 Tolga Ok
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

from environment import ur10_env
import os
import numpy as np

class RandomTargetReaching(ur10_env):
    """ This task is a minimal robot manupulation task. Agent can be trained to
        learn robot's tip to the target position. At the training, target position
        is randomly generated in order to generalize the reaching process.
        Parameters of the task:
            limit_distance: This parameter is the radius of the goal state which's
                center is the randomly generated target point.
            goal_reward: The reward for the goal state, which is defined by the
                limit_distance parameter.
            single_target: This parameter can be used to define a target point
                by user. This can be useful after learning. If None, each episode
                will has it's own randomly generated target point.
    """
    def __init__(self):
        launch_path = "/".join(os.path.realpath(__file__).split("/")[:-2] + ["launch", "RandomTargetReaching.launch"])
        super(RandomTargetReaching, self).__init__("velocity", 10, launch_path)
        self.action_dimension = 6
        self.observation_dimension = 15

        #Task parameter.
        self.limit_distance = 0.1 # meter
        self.goal_reward = 5.0
        self.single_target = None


    def step(self, action):
        robot_state, tip_position = self._step(action)

        try:
            distance = self._distance_to_target(tip_position, self.target)
        except AttributeError:
            self.target = self.generate_target()[1] if not self.single_target else self.single_target
            distance = self._distance_to_target(tip_position, self.target)

        state = np.concatenate([robot_state, self.target], 0)
        reward = self.reward_from_distance(distance) if distance > self.limit_distance else self.goal_reward
        return (state, reward, False, {})

    def reset(self):
        self.target = self.generate_target()[0] if not self.single_target else self.single_target
        robot_state = self._reset()
        state = np.concatenate([robot_state, self.target])
        return state

    def generate_target(self):
        pi = np.pi
        phi = np.random.uniform(pi/6, pi/3)
        theta = np.random.uniform(120*pi/180, 80*pi/180)
        r = np.random.uniform(0.95, 1.2)

        z = np.cos(phi)*r
        x = np.sin(phi)*np.cos(theta)*r
        y = np.sin(phi)*np.sin(theta)*r

        return ((x, y, z), (r, theta, phi))

    def reward_from_distance(self, distance):
        return np.exp(-distance)

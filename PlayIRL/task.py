import rospy
import numpy as np


class VRGraspTask:
    def __init__(self, name, state_dim=9, action_dim=3):
        self.name = name
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.sub = rospy.Subscriber('')

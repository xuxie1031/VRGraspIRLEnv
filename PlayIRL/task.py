import rospy
from std_msgs.msg import String

import numpy as np
import time
import json


class VRGraspTask:
    def __init__(self, name, state_dim=9, action_dim=3):
        self.name = name
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.pub = rospy.Publisher('/irl_trainer_pub', String, queue_size=1)
        self.sub = rospy.Subscriber('/irl_trainer_sub', String, callback=self.step_callback, queue_size=1)
        self.json_msg = ''
        self.callback_flag = False


    def step_callback(self, msg):
        self.json_msg = msg.data
        self.callback_flag = True


    def wait_execution(self):
        while not self.callback_flag:   # timeout variable needed
            time.sleep(.05)
            pass

        self.callback_flag = False         


    def reset(self):
        msg_dict = {'cmd': 'reset'}
        msg = json.dumps(msg_dict)
        self.pub.publish(msg)

        # wait for execution
        self.wait_execution()

        obj = json.loads(self.json_msg)
        assert 'cmd' in obj
        assert obj['cmd'] == 'reset'
        assert 'state' in obj

        return np.asarray(obj['state'])


    def step(self, state, action):
        msg_dict = {'cmd':'step', 'state':state.tolist(), 'action':action.tolist()}
        msg = json.dumps(msg_dict)
        self.pub.publish(msg)

        # wait for execution
        self.wait_execution()

        obj = json.loads(self.json_msg)
        assert 'cmd' in obj
        assert obj['cmd'] == 'step'
        assert 'state' in obj
        assert 'action' in obj
        assert 'next_state' in obj
        assert 'terminal' in obj

        return np.asarray(obj['state']), np.asarray(obj['action']), np.asarray(obj['next_state']), int(obj['terminal'])        
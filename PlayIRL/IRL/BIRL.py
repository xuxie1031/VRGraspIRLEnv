import threading
import numpy as np

from ..IRLPolicy import *

class BIOIRL(threading.Thread):
    def __init__(self, irl_config, policy_config):
        self.irlconfig = irl_config
        self.policy_config = policy_config

        self.rl_model = DDPGModel(self.policy_config)
        self.omega = np.zeros(self.irlconfig.feature_dim)


    def run(self):
        for irl_iter in range(self.irlconfig.episodes_num):
            self.rl_model.policy_iteration(self.policy_config.episodes_num)


    def 
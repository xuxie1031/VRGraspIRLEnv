import os
import threading
from copy import deepcopy
import numpy as np

from ..IRLPolicy import *
from .sampleUtils import *

class BIOIRL(threading.Thread):
    def __init__(self, irl_config, policy_config):
        self.irl_config = irl_config
        self.policy_config = policy_config

        self.rl_model = DDPGModel(self.policy_config)
        self.omega = np.zeros(self.irl_config.feature_dim)

        self.demos_D = None
        self.demos_F = None

    def run(self):
        # policy iteration only
        for irl_iter in range(self.irl_config.episodes_num):
            self.rl_model.policy_iteration(irl_iter, self.omega)


        # with mcmc sample on omega
        qvalues_D = self.calc_qvalue(self.rl_model, self.omega, flag='demo')
        log_posterior = self.calc_posterior(qvalues_D)
        for irl_iter in range(self.irl_config.episodes_num):
            new_rl_model = deepcopy(self.rl_model)
            new_omega = random_sample_omega(self.omega)
            new_rl_model.qvalue_iteration(irl_iter, new_omega)
            new_qvalues_pi = self.calc_qvalue(new_rl_model, new_omega, flag='policy')
            new_qvalues_D = self.calc_qvalue(new_rl_model, new_omega, flag='demo')

            if self.assert_policy_diff(new_qvalues_pi, new_qvalues_D):
                new_rl_model.policy_iteration(irl_iter, new_omega)
                new_qvalues_D = self.calc_qvalue(new_rl_model, new_omega, flag='demo')
                new_log_posterior = self.calc_posterior(new_qvalues_D)
                if sample_prob(min(1, np.exp(new_log_posterior-log_posterior))):
                    self.rl_model, self.omega, log_posterior = deepcopy(new_rl_model), new_omega, new_log_posterior
            else:
                new_log_posterior = self.calc_posterior(new_qvalues_D)
                if sample_prob(min(1, np.exp(new_log_posterior-log_posterior))):
                    self.rl_model, self.omega, log_posterior = deepcopy(new_rl_model), new_omega, new_log_posterior
            

    def calc_qvalue(self, rl_model, omega, flag='demo'):
        qvalues = []
        pos = 0
        while pos < len(self.demos_D):
            states = self.demos_D[pos:(min(pos+self.irl_config.batch_size_D, len(self.demos_D))), 0]
            actions = self.demos_D[pos:(min(pos+self.irl_config.batch_size_D, len(self.demos_D))), 1]
            if flag == 'demo':
                qs = rl_model.network.predict_qvalue(np.stack([states]), np.stack([actions]), omega, to_numpy=True).flatten()
            elif flag == 'policy':
                qs = rl_model.network.predict_policy_qvalue(np.stack([states]), omega, to_numpy=True).flatten()
            qvalues.extend(qs)
            pos = min(pos+self.irl_config.batch_size_D, len(self.demos_D))

        return qvalues
    

    def infogap_from_demos(self):
        pass


    def assert_policy_diff(self, qvalues_pi, qvalues_D):
        q_pi = np.asarray(qvalues_pi)
        q_D = np.asarray(qvalues_D)

        return np.prod((q_pi-q_D) > 0) < 1.0


    def calc_posterior(self, qvalues):
        log_posterior = 0.0

        # qvalue_D likelihood only
        log_likelihood = self.irl_config.gamma*sum(qvalues)
        log_posterior = log_likelihood

        # qvalue_D & infogap likelihood

        return log_posterior


    def load_demos_set(self):
        demos_path = os.path.join('..','DemoData')
        self.demos_D = np.genfromtxt(os.path.join(demos_path, 'demos_D.csv'), delimiter=',')
        self.demos_F = np.genfromtxt(os.path.join(demos_path, 'demos_F.csv'), delimiter=',')

        self.rl_model.replay_D.feed_batch(self.demos_D.tolist())
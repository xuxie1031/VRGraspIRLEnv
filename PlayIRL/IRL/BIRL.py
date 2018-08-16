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
            self.rl_model.policy_iteration(irl_iter, self.omega, self.irl_config.bound_r)


        # with mcmc sample on omega hard-constraint transition
        mus_D = self.calc_mu(self.rl_model, self.demos_D, flag='demo')
        qvalues_D = self.calc_qvalue(mus_D, self.omega)
        log_posterior = self.calc_posterior(qvalues_D)
        for irl_iter in range(self.irl_config.episodes_num):
            new_rl_model = deepcopy(self.rl_model)
            new_omega = random_sample_omega(self.omega)
            new_rl_model.qvalue_iteration(irl_iter, new_omega, self.irl_config.bound_r)
            new_mus_pi = self.calc_mu(new_rl_model, self.demos_D, flag='policy')
            new_mus_D = self.calc_mu(new_rl_model, self.demos_D, flag='demo')
            new_qvalues_pi, new_qvalues_D = self.calc_qvalue(new_mus_pi, new_omega), self.calc_qvalue(new_mus_D, new_omega)

            if self.assert_policy_diff(new_qvalues_pi, new_qvalues_D):
                new_rl_model.policy_iteration(irl_iter, new_omega, self.irl_config.bound_r)
                new_mus_D = self.calc_mu(new_rl_model, self.demos_D, flag='demo')
                new_qvalues_D = self.calc_qvalue(new_mus_D, new_omega)
                new_log_posterior = self.calc_posterior(new_qvalues_D)
                if sample_prob(min(1, np.exp(new_log_posterior-log_posterior))):
                    self.rl_model, self.omega, log_posterior = deepcopy(new_rl_model), new_omega, new_log_posterior
            else:
                new_log_posterior = self.calc_posterior(new_qvalues_D)
                if sample_prob(min(1, np.exp(new_log_posterior-log_posterior))):
                    self.rl_model, self.omega, log_posterior = deepcopy(new_rl_model), new_omega, new_log_posterior
            

    def calc_mu(self, rl_model, demos, flag='demo'):
        mus = []
        pos = 0
        while pos < len(demos):
            states = demos[0][pos:(min(pos+self.irl_config.batch_size_demos, len(demos))), :]
            actions = demos[1][pos:(min(pos+self.irl_config.batch_size_demos, len(demos))), :]
            if flag == 'demo':
                batch_mu = rl_model.network.predict_mu(np.stack(states), np.stack(actions), to_numpy=True)
            elif flag == 'policy':
                batch_mu = rl_model.network.predict_policy_mu(np.stack(states), to_numpy=True)
            mus.append(batch_mu)
            pos = min(pos+self.irl_config.batch_size_demos, len(demos))

        mus = np.asarray(mus)
        return mus


    def calc_qvalue(self, mus, omega):
        return mus.dot(omega)


    def generate_halfplane_normals_pi(self, rl_model):
        sampled_action_num = 16
        sampled_states = random_sample_states_around_demos(self.demos_D)
        sampled_mus = []
        for i in range(len(sampled_states)):
            batch_states = np.stack([sampled_states[i, :]]).repeat(sampled_action_num, axis=0)
            batch_actions = rl_model.networkt.predict(batch_states, to_numpy=True)
            batch_mu_pi = rl_model.network.predict_mu(batch_states, batch_actions, to_numpy=True)

            action_norm = np.linalg.norm(batch_actions[0, :])
            sampled_actions = random_sample_actions_by_norm(self.policy_config.action_dim, action_norm)
            batch_mu_other = rl_model.network.predict_mu(batch_states, sampled_actions, to_numpy=True)

            sampled_mus.append(batch_mu_pi-batch_mu_other)
        
        sampled_mus = np.concatenate(sampled_mus, axis=0)
        sampled_mus /= np.linalg.norm(sampled_mus, axis=1, keepdims=True)

        return sampled_mus


    def generate_halfplane_normals_demos(self, rl_model, demos):
        pass

    def infogap_from_demos(self):
        pass


    def assert_policy_diff(self, qvalues_pi, qvalues_D):
        q_pi = np.asarray(qvalues_pi)
        q_D = np.asarray(qvalues_D)

        return np.prod((q_pi-q_D) > 0) < 1.0


    def calc_posterior(self, qvalues):
        log_posterior = 0.0

        # qvalue_D likelihood only
        log_likelihood = self.irl_config.gamma*np.sum(qvalues)
        log_posterior = log_likelihood

        # qvalue_D & infogap likelihood

        return log_posterior


    def load_demos_set(self):
        demos_path = os.path.join('..','DemoData')
        demos_seg = [self.policy_config.state_dim, self.policy_config.action_dim, self.policy_config.state_dim, self.policy_config.terminal_dim]
        self.demos_D = np.genfromtxt(os.path.join(demos_path, 'demos_D.csv'), delimiter=',')
        self.demos_F = np.genfromtxt(os.path.join(demos_path, 'demos_F.csv'), delimiter=',')

        self.demos_D = np.split(self.demos_D, np.cumsum(demos_seg), axis=1)[:-1]
        self.demos_F = np.split(self.demos_F, np.cumsum(demos_seg), axis=1)[:-1]

        experience = [x.tolist() for x in self.demos_D]
        self.rl_model.replay_D.feed_batch(experience)
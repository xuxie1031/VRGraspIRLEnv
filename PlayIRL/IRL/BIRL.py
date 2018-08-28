import os
import threading
import pickle
from copy import deepcopy
import numpy as np

from ..IRLPolicy import *
from .sampleUtils import *

class BIOIRL(threading.Thread):
    def __init__(self, irl_config, policy_config):
        threading.Thread.__init__(self)
        self.irl_config = irl_config
        self.policy_config = policy_config

        self.rl_model = DDPGModel(self.policy_config)
        self.omega = np.ones(self.irl_config.feature_dim)
        self.log_posterior = 0

        if self.irl_config.b_load:
            with open('env_irl_model{0}.pkl'.format(self.irl_config.save_flag), 'rb') as fi:
                self.irl_config, self.policy_config, self.rl_model, self.omega, self.log_posterior = pickle.load(fi)
                # load halfplane if hard constraint

        self.demos_D = None
        self.demos_F = None

        # self.load_demos_set()


    def soft_transition(self, new_rl_model, new_omega, new_qvalues_D):
        new_log_posterior = self.calc_posterior(new_rl_model, new_omega, new_qvalues_D, None)
        if sample_prob(min(1, np.exp(new_log_posterior-self.log_posterior))):
            return deepcopy(new_rl_model), new_omega, new_log_posterior
        return self.rl_model, self.omega, self.log_posterior


    def hard_transition(self, new_rl_model, new_omega, new_qvalues_D, new_halfplane_normals):
        if not is_omega_in_halfspace(new_omega, new_halfplane_normals)[0]:
            return self.rl_model, self.omega, self.log_posterior
        new_log_posterior = self.calc_posterior(new_rl_model, new_omega, new_qvalues_D, new_halfplane_normals)
        if not is_omega_in_halfspace(self.omega, new_halfplane_normals)[0]:
            return deepcopy(new_rl_model), new_omega, new_log_posterior
        if sample_prob(min(1, np.exp(new_log_posterior-self.log_posterior))):
            return deepcopy(new_rl_model), new_omega, new_log_posterior


    def run(self):
        # policy iteration only
        for irl_iter in range(self.irl_config.episodes_num):
            if self.irl_config.evalT > 0 and irl_iter % self.irl_config.evalT == 0:
                with open('env_irl_model{0}.pkl'.format(self.irl_config.save_flag), 'wb') as fo:
                    data = [self.irl_config, self.policy_config, self.rl_model, self.omega, self.log_posterior]
                    pickle.dump(data, fo, pickle.HIGHEST_PROTOCOL)
                fo.close()

                eval_reward = self.rl_model.qvalue_iteration(irl_iter, self.omega, self.irl_config.bound_r, deterministic=True)
                print('itr %d evaluation reward %f' % (irl_iter, eval_reward))
            self.rl_model.policy_iteration(irl_iter, self.omega, self.irl_config.bound_r)


        # # with mcmc sample transition without omega constraint
        # mus_D = self.calc_mu(self.rl_model, self.demos_D, flag='demo')
        # qvalues_D = self.calc_qvalue(mus_D, self.omega)
        # self.log_posterior = self.calc_posterior(self.rl_model, self.omega, qvalues_D, None)
        # for irl_iter in range(self.irl_config.episodes_num):
        #     new_rl_model = deepcopy(self.rl_model)
        #     new_omega = random_sample_omega(self.omega)
        #     new_rl_model.qvalue_iteration(irl_iter, new_omega, self.irl_config.bound_r)
        #     new_mus_pi = self.calc_mu(new_rl_model, self.demos_D, flag='policy')
        #     new_mus_D = self.calc_mu(new_rl_model, self.demos_D, flag='demo')
        #     new_qvalues_pi, new_qvalues_D = self.calc_qvalue(new_mus_pi, new_omega), self.calc_qvalue(new_mus_D, new_omega)

        #     if self.assert_policy_diff(new_qvalues_pi, new_qvalues_D):
        #         new_rl_model.policy_iteration(irl_iter, new_omega, self.irl_config.bound_r)
        #         new_mus_D = self.calc_mu(new_rl_model, self.demos_D, flag='demo')
        #         new_qvalues_D = self.calc_qvalue(new_mus_D, new_omega)
        #         self.rl_model, self.omega, self.log_posterior = self.soft_transition(new_rl_model, new_omega, new_qvalues_D)
        #     else:
        #         self.rl_model, self.omega, self.log_posterior = self.soft_transition(new_rl_model, new_omega, new_qvalues_D)
            

        # with mcmc sample transition with hard constraint
        # mus_D = self.calc_mu(self.rl_model, self.demos_D, flag='demo')
        # qvalues_D = self.calc_qvalue(mus_D, self.omega)
        # halfplane_normals = self.generate_halfplane_normals_demos(self.rl_model, self.demos_D)
        # self.log_posterior = self.calc_posterior(self.rl_model, self.omega, qvalues_D, halfplane_normals)
        # for irl_iter in range(self.irl_config.episodes_num):
        #     new_rl_model = deepcopy(self.rl_model)
        #     new_omega = random_sample_omega_hyperplane_constraint(self.omega, halfplane_normals)
        #     if new_omega is None:
        #         print('exploration ended. Abort ...')
        #         break
        #     new_rl_model.qvalue_iteration(irl_iter, new_omega, self.irl_config.bound_r)
        #     new_mus_pi = self.calc_mu(new_rl_model, self.demos_D, flag='policy')
        #     new_mus_D = self.calc_mu(new_rl_model, self.demos_D, flag='demo')
        #     new_qvalues_pi, new_qvalues_D = self.calc_qvalue(new_mus_pi, new_omega), self.calc_qvalue(new_mus_D, new_omega)
        #     new_halfplane_normals = self.generate_halfplane_normals_demos(new_rl_model, self.demos)

        #     if self.assert_policy_diff(new_qvalues_pi, new_qvalues_D):
        #         new_rl_model.policy_iteration(irl_iter, new_omega, self.irl_config.bound_r)
        #         new_mus_D = self.calc_mu(new_rl_model, self.demos_D, flag='demo')
        #         new_qvalues_D = self.calc_qvalue(new_mus_D, new_omega)
        #         new_halfplane_normals = self.generate_halfplane_normals_demos(new_rl_model, self.demos_D)
        #         self.rl_model, self.omega, self.log_posterior = self.hard_transition(new_rl_model, new_omega, new_qvalues_D, new_halfplane_normals)
        #     else:
        #         self.rl_model, self.omega, self.log_posterior = self.hard_transition(new_rl_model, new_omega, new_qvalues_D, new_halfplane_normals)
        #     halfplane_normals = new_halfplane_normals


    def calc_mu(self, rl_model, demos, flag='demo'):
        mus = []
        pos = 0
        while pos < len(demos[0]):
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
        sampled_action_num = 8
        sampled_states = random_sample_states_around_demos(self.demos_D)
        sampled_delta_mus = []
        for i in range(len(sampled_states)):
            batch_states = np.stack([sampled_states[i, :]]).repeat(sampled_action_num, axis=0)
            batch_mu_pi = rl_model.network.predict_policy_mu(batch_states, to_numpy=True)

            sampled_actions = random_sample_actions_by_norm(self.policy_config.action_dim, sampled_action_num)
            batch_mu_other = rl_model.network.predict_mu(batch_states, sampled_actions, to_numpy=True)

            sampled_delta_mus.append(batch_mu_pi-batch_mu_other)
        
        sampled_delta_mus = np.concatenate(sampled_delta_mus, axis=0)
        sampled_delta_mus /= np.linalg.norm(sampled_delta_mus, axis=1, keepdims=True)

        return sampled_delta_mus


    def generate_halfplane_normals_demos(self, rl_model, demos):
        sampled_action_num = 8
        demo_states = demos[0]
        demo_actions = demos[1]
        demo_delta_mus = []
        for i in range(len(demo_states)):
            batch_states = np.stack([demo_states[i, :]]).repeat(sampled_action_num, axis=0)
            batch_actions = np.stack([demo_actions[i, :]]).repeat(sampled_action_num, axis=0)
            batch_mu_demo = rl_model.network.predict_mu(batch_states, batch_actions, to_numpy=True)

            sampled_actions = random_sample_actions_by_norm(self.policy_config.action_dim, sampled_action_num)
            batch_mu_other = rl_model.network.predict_mu(batch_states, sampled_actions, to_numpy=True)

            demo_delta_mus.append(batch_mu_demo-batch_mu_other)
        
        demo_delta_mus = np.concatenate(demo_delta_mus, axis=0)
        demo_delta_mus /= np.linalg.norm(demo_delta_mus, axis=1, keepdims=True)

        return demo_delta_mus


    def infogap_from_demos(self, halfplane_normals_pi, halfplane_normals_demo, omega):
        halfplane_pair_count = len(halfplane_normals_pi)
        idx_pair = np.array(np.meshgrid(range(len(halfplane_normals_pi)), range(len(halfplane_normals_demo)))).T.reshape(-1, 2)

        idx_occupy_pi = np.zeros(len(halfplane_normals_pi))
        idx_occupy_demo = np.zeros(len(halfplane_normals_demo))

        idx_normals_pi = halfplane_normals_pi[idx_pair[:, 0], :]
        idx_normals_demo = halfplane_normals_demo[idx_pair[:, 1], :]
        full_angular_sim = 1-np.arccos(np.sum(idx_normals_pi*idx_normals_demo, axis=1))/np.pi
        sorted_idx = (-full_angular_sim).argsort()  # descending order

        idx = 0
        accu_count = 0
        sum_angular_sim = 0.0
        while accu_count < halfplane_pair_count:
            curr_idx = sorted_idx[idx]
            if idx_occupy_pi[idx_pair[curr_idx, 0]] < 1.0 and idx_occupy_demo[idx_pair[curr_idx, 1]] < 1.0:
                sum_angular_sim += full_angular_sim[curr_idx]
                idx_occupy_pi[idx_pair[curr_idx, 0]] = 1.0
                idx_occupy_demo[idx_pair[curr_idx, 1]] = 1.0
                accu_count += 1
            idx += 1

        return sum_angular_sim/halfplane_pair_count


    def assert_policy_diff(self, qvalues_pi, qvalues_D):
        q_pi = np.asarray(qvalues_pi)
        q_D = np.asarray(qvalues_D)

        return np.prod((q_pi-q_D) > 0) < 1.0


    def calc_posterior(self, rl_model, omega, qvalues, halfplane_normals_demo):
        log_posterior = 0.0

        # qvalue_D likelihood only
        log_likelihood = self.irl_config.gamma*np.sum(qvalues)
        log_posterior = log_likelihood

        # # qvalue_D & infogap_D likelihood
        # if halfplane_normals_demo is None:
        #     halfplane_normals_demo = self.generate_halfplane_normals_demos(rl_model, self.demos_D)
        # halfplane_normals_pi = self.generate_halfplane_normals_pi(rl_model)
        # infogap_D = self.infogap_from_demos(halfplane_normals_pi, halfplane_normals_demo, omega)
        # log_likelihood = self.irl_config.gamma*np.sum(qvalues)+self.irl_config.gamma_infogap*infogap_D
        # log_posterior = log_likelihood

        return log_posterior


    def load_demos_set(self):
        demos_path = os.path.join('..','DemoData')
        demos_seg = [self.policy_config.state_dim, self.policy_config.action_dim, self.policy_config.state_dim, self.policy_config.terminal_dim, self.policy_config.flag_dim]
        self.demos_D = np.genfromtxt(os.path.join(demos_path, 'demos_D.csv'), delimiter=',')
        self.demos_F = np.genfromtxt(os.path.join(demos_path, 'demos_F.csv'), delimiter=',')

        self.demos_D = np.split(self.demos_D, np.cumsum(demos_seg), axis=1)[:-1]
        self.demos_F = np.split(self.demos_F, np.cumsum(demos_seg), axis=1)[:-1]

        experience = [x.tolist() for x in self.demos_D]
        self.rl_model.replay_D.feed_batch(experience)
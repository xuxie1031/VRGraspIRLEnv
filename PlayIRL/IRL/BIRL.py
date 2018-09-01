import os
import threading
import copy
import numpy as np

from ..IRLPolicy import *
from .sampleUtils import *

class BIOIRL(threading.Thread):
    def __init__(self, irl_config, policy_config):
        threading.Thread.__init__(self)
        self.irl_config = irl_config
        self.policy_config = policy_config

        self.rl_model = DDPGModel(self.policy_config)
        self.omega = np.random.uniform(-1.0, 1.0, self.irl_config.feature_dim)
        self.log_posterior = 0

        if self.irl_config.b_load:
            filename = 'env_irl_model{0}.pth.tar'.format(self.irl_config.save_flag)
            self.load_checkpoint(filename)


        self.demos_D = None
        self.demos_F = None

        self.load_demos_set()


    def save_checkpoint(self, filename):
        state = {}
        
        state['omega'] = self.omega
        state['log_posterior'] = self.log_posterior
        state['network_dict'] = self.rl_model.network.state_dict()
        state['target_network_dict'] = self.rl_model.target_network.state_dict()
        state['actor_opt_dict'] = self.rl_model.network.actor_opt.state_dict()
        state['critic_opt_dict'] = self.rl_model.network.critic_opt.state_dict()
        state['replay_D'] = self.rl_model.replay_D.get_params()
        state['replay_M'] = self.rl_model.replay_M.get_params()
        state['total_step'] = self.rl_model.total_step
        state['p_episode_rewards'] = self.rl_model.p_episode_rewards
        state['q_episode_rewards'] = self.rl_model.q_episode_rewards
        state['eval_episode_rewards'] = self.rl_model.eval_episode_rewards

        torch.save(state, filename)


    def load_checkpoint(self, filename):
        assert os.path.isfile(filename)
        state = torch.load(filename)

        self.omega = state['omega']
        self.log_posterior = state['log_posterior']
        self.rl_model.network.load_state_dict(state['network_dict'])
        self.rl_model.target_network.load_state_dict(state['target_network_dict'])
        self.rl_model.network.actor_opt.load_state_dict(state['actor_opt_dict'])
        self.rl_model.network.critic_opt.load_state_dict(state['critic_opt_dict'])
        self.rl_model.replay_D.set_params(*state['replay_D'])
        self.rl_model.replay_M.set_params(*state['replay_M'])
        self.rl_model.total_step = state['total_step']
        self.rl_model.p_episode_rewards = state['p_episode_rewards']
        self.rl_model.q_episode_rewards = state['q_episode_rewards']
        self.rl_model.eval_episode_rewards = state['eval_episode_rewards']


    def copy_to_new_model(self, rl_model):
        new_rl_model = DDPGModel(self.policy_config)

        # network
        new_rl_model.network.load_state_dict(rl_model.network.state_dict())
        new_rl_model.target_network.load_state_dict(rl_model.target_network.state_dict())

        # optimizer
        new_rl_model.network.actor_opt.load_state_dict(rl_model.network.actor_opt.state_dict())
        new_rl_model.network.critic_opt.load_state_dict(rl_model.network.critic_opt.state_dict())

        # replay
        new_rl_model.replay_D.set_params(*rl_model.replay_D.get_params())
        new_rl_model.replay_M.set_params(*rl_model.replay_M.get_params())

        # other params
        new_rl_model.total_step = rl_model.total_step
        new_rl_model.p_episode_rewards = copy.copy(rl_model.p_episode_rewards)
        new_rl_model.q_episode_rewards = copy.copy(rl_model.q_episode_rewards)

        return new_rl_model


    def soft_transition(self, new_rl_model, new_omega, new_qvalues_D):
        new_log_posterior = self.calc_posterior(new_rl_model, new_omega, new_qvalues_D, None)
        if sample_prob(min(1, np.exp(new_log_posterior-self.log_posterior))):
            print('transition to new model')
            return new_rl_model, new_omega, new_log_posterior
        print('remain old model')
        return self.copy_to_new_model(self.rl_model), self.omega, self.log_posterior


    def hard_transition(self, new_rl_model, new_omega, new_qvalues_D, new_halfplane_normals):
        if not is_omega_in_halfspace(new_omega, new_halfplane_normals)[0]:
            return self.copy_to_new_model(self.rl_model), self.omega, self.log_posterior
        new_log_posterior = self.calc_posterior(new_rl_model, new_omega, new_qvalues_D, new_halfplane_normals)
        if not is_omega_in_halfspace(self.omega, new_halfplane_normals)[0]:
            return new_rl_model, new_omega, new_log_posterior
        if sample_prob(min(1, np.exp(new_log_posterior-self.log_posterior))):
            return new_rl_model, new_omega, new_log_posterior


    def run(self):
        # # policy iteration only
        # for irl_iter in range(self.irl_config.episodes_num):
        #     if self.irl_config.evalT > 0 and irl_iter % self.irl_config.evalT == 0:
        #         filename = 'env_irl_model{0}.pth.tar'.format(self.irl_config.save_flag)
        #         self.save_checkpoint(filename)

        #         eval_reward = self.rl_model.policy_evaluation(irl_iter, self.omega, self.irl_config.bound_r)
        #         print('itr %d evaluation reward %f' % (irl_iter, eval_reward))
        #     self.rl_model.policy_iteration(irl_iter, self.omega, self.irl_config.bound_r)


        # with mcmc sample transition without omega constraint
        self.rl_model.policy_iteration(0, self.omega, self.irl_config.bound_r)
        mus_D = self.calc_mu(self.rl_model, self.demos_D, flag='demo')
        qvalues_D = self.calc_qvalue(mus_D, self.omega)
        self.log_posterior = self.calc_posterior(self.rl_model, self.omega, qvalues_D, None)
        for irl_iter in range(1, self.irl_config.episodes_num):
            if self.irl_config.evalT > 0 and irl_iter % self.irl_config.evalT == 0:
                filename = 'env_irl_model{0}.pth.tar'.format(self.irl_config.save_flag)
                self.save_checkpoint(filename)
                eval_reward = self.rl_model.policy_evaluation(irl_iter, self.omega, self.irl_config.bound_r)
                print('itr %d evaluation reward %f' % (irl_iter, eval_reward))

            new_rl_model = self.copy_to_new_model(self.rl_model)
            new_omega = random_sample_omega(self.omega)
            new_rl_model.qvalue_iteration(irl_iter, new_omega, self.irl_config.bound_r)
            new_mus_pi = self.calc_mu(new_rl_model, self.demos_D, flag='policy')
            new_mus_D = self.calc_mu(new_rl_model, self.demos_D, flag='demo')
            new_qvalues_pi, new_qvalues_D = self.calc_qvalue(new_mus_pi, new_omega), self.calc_qvalue(new_mus_D, new_omega)

            if self.assert_policy_diff(new_qvalues_pi, new_qvalues_D):
                new_rl_model.policy_iteration(irl_iter, new_omega, self.irl_config.bound_r)
                new_mus_D = self.calc_mu(new_rl_model, self.demos_D, flag='demo')
                new_qvalues_D = self.calc_qvalue(new_mus_D, new_omega)
                self.rl_model, self.omega, self.log_posterior = self.soft_transition(new_rl_model, new_omega, new_qvalues_D)
            else:
                self.rl_model, self.omega, self.log_posterior = self.soft_transition(new_rl_model, new_omega, new_qvalues_D)
            

        # with mcmc sample transition with hard constraint
        # mus_D = self.calc_mu(self.rl_model, self.demos_D, flag='demo')
        # qvalues_D = self.calc_qvalue(mus_D, self.omega)
        # halfplane_normals = self.generate_halfplane_normals_demos(self.rl_model, self.demos_D)
        # self.log_posterior = self.calc_posterior(self.rl_model, self.omega, qvalues_D, halfplane_normals)
        # for irl_iter in range(self.irl_config.episodes_num):
        #     new_rl_model = self.copy_to_new_model(self.rl_model)
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
        # print('calculating mu values ...')
        mus = []
        pos = 0
        while pos < len(demos[0]):
            states = demos[0][pos:(min(pos+self.irl_config.batch_size_demos, len(demos[0]))), :]
            actions = demos[1][pos:(min(pos+self.irl_config.batch_size_demos, len(demos[1]))), :]
            if flag == 'demo':
                batch_mu = rl_model.network.predict_mu(np.stack(states), np.stack(actions), to_numpy=True)
            elif flag == 'policy':
                batch_mu = rl_model.network.predict_policy_mu(np.stack(states), to_numpy=True)
            mus.append(batch_mu)
            pos = min(pos+self.irl_config.batch_size_demos, len(demos[0]))

        mus = np.concatenate(mus, axis=0)
        return mus


    def calc_qvalue(self, mus, omega):
        return mus.dot(omega)


    def generate_halfplane_normals_pi(self, rl_model):
        # print('generating halfplane normals pi ...')
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
        # print('generating halfplane normals demos ...')
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
        print('calculating infogap ...')
        halfplane_pair_count = len(halfplane_normals_pi)
        idx_pair = np.array(np.meshgrid(range(len(halfplane_normals_pi)), range(len(halfplane_normals_demo)))).T.reshape(-1, 2)

        idx_occupy_pi = np.zeros(len(halfplane_normals_pi))
        idx_occupy_demo = np.zeros(len(halfplane_normals_demo))

        idx_normals_pi = halfplane_normals_pi[idx_pair[:, 0], :]
        idx_normals_demo = halfplane_normals_demo[idx_pair[:, 1], :]
        full_angular_sim = 1-np.arccos(np.clip(np.sum(idx_normals_pi*idx_normals_demo, axis=1), -1.0, 1.0))/np.pi
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

        # # qvalue_D likelihood only
        # log_likelihood = self.irl_config.gamma*np.sum(qvalues)
        # log_posterior = log_likelihood

        # qvalue_D & infogap_D likelihood
        if halfplane_normals_demo is None:
            halfplane_normals_demo = self.generate_halfplane_normals_demos(rl_model, self.demos_D)
        halfplane_normals_pi = self.generate_halfplane_normals_pi(rl_model)
        infogap_D = self.infogap_from_demos(halfplane_normals_pi, halfplane_normals_demo, omega)
        print('data log_likelihood {0}, infogap {1}'.format(np.sum(qvalues), infogap_D))
        print('old log_likelihood {0}'.format(self.log_posterior))
        log_likelihood = self.irl_config.gamma*np.sum(qvalues)+self.irl_config.gamma_infogap*infogap_D
        log_posterior = log_likelihood

        return np.maximum(log_posterior, 1e-6)


    def load_demos_set(self):
        print('load demos set ...')
        base_path = 'src/PlayIRL'
        demos_path = os.path.join(base_path,'DemosData')
        demos_seg = [self.policy_config.state_dim, self.policy_config.action_dim, self.policy_config.state_dim, self.policy_config.terminal_dim, self.policy_config.flag_dim]
        self.demos_D = np.genfromtxt(os.path.join(demos_path, 'demos_D.csv'), delimiter=',')
        self.demos_F = np.genfromtxt(os.path.join(demos_path, 'demos_F.csv'), delimiter=',')

        self.demos_D = np.split(self.demos_D, np.cumsum(demos_seg), axis=1)[:-1]
        self.demos_F = np.split(self.demos_F, np.cumsum(demos_seg), axis=1)[:-1]

        experience = [x.tolist() for x in self.demos_D]
        self.rl_model.replay_D.feed_batch(experience)
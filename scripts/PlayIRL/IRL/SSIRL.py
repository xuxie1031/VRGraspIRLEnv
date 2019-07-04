import os
import threading
from copy import copy
import numpy as np

from ..IRLPolicy import *
from .sampleUtils import *

class SSIRL(threading.Thread):
    def __init__(self, irl_config, policy_config):
        threading.Thread.__init__(self)
        self.irl_config = irl_config
        self.policy_config = policy_config

        self.rl_model = DDPGModel(self.policy_config)
        self.omega = np.random.uniform(-1.0, 1.0, self.irl_config.feature_dim)
        self.policy_mus = np.zeros((self.irl_config.episodes_num, self.irl_config.feature_dim))

        if self.irl_config.b_load:
            filename = 'env_irl_model{0}.pth.tar'.format(self.irl_config.save_flag)
            self.load_checkpoint(filename)

        self.demos_D = None
        self.demos_F = None

        self.load_demos_set()


    def save_checkpoint(self, filename):
        state = {}
        state['omega'] = self.omega
        state['network_dict'] = self.rl_model.network.state_dict()
        state['target_network_dict'] = self.rl_model.target_network.state_dict()
        state['actor_opt_dict'] = self.rl_model.network.actor_opt.state_dict()
        state['critic_opt_dict'] = self.rl_model.network.critic_opt.state_dict()
        state['replay_D'] = self.rl_model.replay_D.get_params()
        state['replay_M'] = self.rl_model.replay_M.get_params()
        state['total_step'] = self.rl_model.total_step
        state['p_episode_rewards'] = self.rl_model.p_episode_rewards
        state['q_episode_rewards'] = self.rl_model.q_episode_rewards

        torch.save(state, filename)


    def load_checkpoint(self, filename):
        assert os.path.isfile(filename)
        state = torch.load(filename)

        self.omega = state['omega']
        self.rl_model.network.load_state_dict(state['network_dict'])
        self.rl_model.target_network.load_state_dict(state['target_network_dict'])
        self.rl_model.network.actor_opt.load_state_dict(state['actor_opt_dict'])
        self.rl_model.network.critic_opt.load_state_dict(state['critic_opt_dict'])
        self.rl_model.replay_D.set_params(*state['replay_D'])
        self.rl_model.replay_M.set_params(*state['replay_M'])
        self.rl_model.total_step = state['total_step']
        self.rl_model.p_episode_rewards = state['p_episode_rewards']
        self.rl_model.q_episode_rewards = state['q_episode_rewards']


    def run(self):
        for irl_iter in range(1, self.irl_config.episodes_num):
            if self.irl_config.evalT > 0 and irl_iter % self.irl_config.evalT == 0:
                filename = 'env_irl_model{0}.pth.tar'.format(self.irl_config.save_flag)
                self.save_checkpoint(filename)
                eval_reward, eval_traj = self.rl_model.policy_evaluation(irl_iter, self.irl_config.bound_r, save_traj=True, name='linear', rname='linear_reward', omega=self.omega)
                print('itr %d evaluation reward %f' % (irl_iter, eval_reward))
            
            self.calc_omega()
            self.rl_model.policy_iteration(irl_iter, self.irl_config.bound_r, name='linear', rname='linear_reward', omega=self.omega)
            self.calc_mu_pi(irl_iter, eval_traj)


    def calc_mu_pi(self, irl_iter, eval_traj):
        mus = []

        steps = 0
        accu_phi = np.zeros(self.irl_config.feature_dim)
        for i in len(eval_traj):
            state, terminal = eval_traj[i]
            phi = self.rl_model.network.feature(np.stack[state], to_numpy=True).flatten()
            accu_phi = accu_phi+self.policy_config.discount**steps*phi
            if terminal > 0:
                mus.append(accu_phi)
                steps = 0
                accu_phi = np.zeros(self.irl_config.feature_dim)
        mus = np.asarray(mus)
        self.policy_mus[irl_iter, :] = np.mean(mus, axis=0)

            
    def calc_mu_demos(self, demos):
        mus = []
        states = demos[0]
        terminals = demos[3]

        steps = 0
        accu_phi = np.zeros(self.irl_config.feature_dim)
        for i in len(states):
            state = states[i, :]
            phi = self.rl_model.network.feature(np.stack([state]), to_numpy=True).flatten()
            accu_phi = accu_phi+self.policy_config.discount**steps*phi
            if terminals[i] > 0:
                mus.append(accu_phi)
                steps = 0
                accu_phi = np.zeros(self.irl_config.feature_dim)
        mus = np.asarray(mus)

        return np.mean(mus, axis=0)


    def calc_omega(self):
        mu_D = self.calc_mu_demos(self.demos_D)
        mu_F = self.calc_mu_demos(self.demos_F)

        dim = np.random.randint(self.irl_config.feature_dim, size=1).item()
        min_obj = np.inf
        arg_omega = np.zeros(self.omega.shape)
        for e in np.linspace(-1.0, 1.0, num=100):
            omega = np.copy(self.omega)
            omega[dim] = e
            obj = max(1-mu_D.dot(omega), 0.0)+self.policy_config.discount*np.linalg.norm(omega)+ \
                np.sum(np.maximum(1+self.policy_mus.dot(omega), 0.0))+self.policy_config.discount*max(1-abs(mu_F.dot(omega)), 0)
            if obj < min_obj:
                min_obj = obj
                arg_omega = omega
        self.omega = arg_omega/np.linalg.norm(arg_omega)

    
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
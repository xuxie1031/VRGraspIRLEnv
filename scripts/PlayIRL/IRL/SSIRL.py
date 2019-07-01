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
                eval_reward = self.rl_model.policy_evaluation(irl_iter, self.irl_config.bound_r, name='linear', rname='linear_reward', omega=self.omega)
                print('itr %d evaluation reward %f' % (irl_iter, eval_reward))

            
    def calc_mu(self, demos, flag='demo'):
        

        mus = []
        pos = 0
        while pos < len(demos[0]):
            states = demos[0][pos:(min(pos+self.irl_config.batch_size_demos, len(demos[0]))), :]
            actions = demos[1][pos:(min(pos+self.irl_config.batch_size_demos, len(demos[1]))), :]
            if flag == 'demo':
                batch_mu = self.rl_model.network.predict_mu(np.stack(states), np.stack(actions), to_numpy=True)
            elif flag == 'policy':
                batch_mu = self.rl_model.network.predict_policy_mu(np.stack(states), to_numpy=True)
            mus.append(batch_mu)
            pos = min(pos+self.irl_config.batch_size_demos, len(demos[0]))

        mus = np.concatenate(mus, axis=0)
        return mus


    def calc_omega(self):
        mus_demos_D, mus_demos_F = self.calc_mu(self.demos_D, flag='demo'), self.calc_mu(self.demos_F, flag='demo')


    
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
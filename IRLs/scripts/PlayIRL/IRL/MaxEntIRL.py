import os
import threading
from copy import copy
import numpy as np

from ..IRLPolicy import *
from .sampleUtils import *

class MaxEntIRL(threading.Thread):
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

        self.demos = None
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
        state['eval_episode_rewards'] = self.rl_model.eval_episode_rewards

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
        self.rl_model.eval_episode_rewards=  state['eval_episode_rewards']


    def run(self):
        for irl_iter in range(1, self.irl_config.episodes_num):
            if self.irl_config.evalT > 0 and irl_iter % self.irl_config.evalT == 0:
                filename = 'env_irl_model{0}.pth.tar'.format(self.irl_config.save_flag)
                self.save_checkpoint(filename)
                eval_reward = self.rl_model.policy_evaluation(irl_iter, self.irl_config.bound_r, name='linear', rname='linear_reward', omega=self.omega)
                print('itr %d evaluation reward %f' % (irl_iter, eval_reward))

            self.rl_model.policy_iteration(irl_iter, self.irl_config.bound_r, name='linear', rname='linear_reward', omega=self.omega)
            self.omega += self.irl_config.maxent_lr*self.calc_maxent_gradient()


    def phi_demos(self):
        demo_phis = []
        pos = 0
        while pos < len(self.demos[0]):
            states = self.demos[0][pos:(min(pos+self.irl_config.batch_size_demos, len(self.demos[0]))), :]
            batch_phi = self.rl_model.network.feature(np.stack(states), to_numpy=True)
            demo_phis.append(batch_phi)
            pos = min(pos+self.irl_config.batch_size_demos, len(self.demos[0]))

        demo_phis = np.concatenate(demo_phis, axis=0)
        return demo_phis


    def phi_eval_demos(self):
        eval_phis = []
        pos = 0
        while pos < self.rl_model.replay_M.size():
            experiences = self.rl_model.replay_M.sample_segment(pos, min(pos+self.irl_config.batch_size_demos, self.rl_model.replay_M.size()))
            states, _, _, _, _ = experiences
            batch_phi = self.rl_model.network.feature(np.stack(states), to_numpy=True)
            eval_phis.append(batch_phi)
            pos = min(pos+self.irl_config.batch_size_demos, self.rl_model.replay_M.size())

        eval_phis = np.concatenate(eval_phis, axis=0)
        return eval_phis


    def calc_maxent_gradient(self):
        demo_phis = self.phi_demos()
        eval_phis = self.phi_eval_demos()

        return np.mean(demo_phis, axis=0) + np.mean(eval_phis, axis=0)


    def load_demos_set(self):
        print('load demos set ...')
        base_path = 'scripts/PlayIRL'
        demos_path = os.path.join(base_path,'DemosData')
        demos_seg = [self.policy_config.state_dim, self.policy_config.action_dim, self.policy_config.state_dim, self.policy_config.terminal_dim, self.policy_config.flag_dim]
        self.demos = np.genfromtxt(os.path.join(demos_path, 'demos.csv'), delimiter=',')
        self.demos = np.split(self.demos, np.cumsum(demos_seg), axis=1)[:-1]

        experience = [x.tolist() for x in self.demos]
        self.rl_model.replay_D.feed_batch(experience)

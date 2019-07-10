import os
import threading
import copy
import numpy as np
from collections import defaultdict

from ..IRLPolicy import *
from .kernelUtils import *

class GPIRL(threading.Thread):
    def __init__(self, irl_config, policy_config):
        threading.Thread.__init__(self)
        self.irl_config = irl_config
        self.policy_config = policy_config

        self.rl_model = DDPGModel(self.policy_config)

        if self.irl_config.b_load:
            filename = 'env_irl_model{0}.pth.tar'.format(self.irl_config.save_flag)
            self.load_checkpoint(filename)
        
        self.demos = None
        self.load_demos_set()

        self.state_th_ubound = 0.0
        self.action_th_lbound = 0.0
        self.calc_state_action_th()

        self.policy_savf = defaultdict(lambda:0, {})
        self.demos_savf = defaultdict(lambda:0, {})
        self.build_state_action_dict_demos()

        self.u = np.ones((len(self.demos[0]), 1))
        self.sigma_sq = self.irl_config.sigma_sq
        self.beta = 0.5
        self.lambd = np.ones((self.irl_config.feature_dim, 1))
        self.Kuu_inv = None
        self.Xu = None


    def save_checkpoint(self, filename):
        state = {}

        # include neccessary gp params
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

        # include neccessary gp params
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


    def run(self):
        for irl_iter in range(0, self.irl_config.episodes_num):
            if irl_iter > 0 and self.irl_config.evalT > 0 and irl_iter % self.irl_config.evalT == 0:
                filename = 'env_irl_model{0}.pth.tar'.format(self.irl_config.save_flag)
                self.save_checkpoint(filename)
                eval_reward = self.rl_model.policy_evaluation(irl_iter, self.irl_config.bound_r, name='gp', rname='gp_reward', Xu=self.Xu, Kuu_inv=self.Kuu_inv, u=self.u, lambd=self.lambd, beta=self.beta, sigma_sq=self.sigma_sq, device=self.rl_model.network.device)
                print('iter %d evaluation reward %f' % (irl_iter, eval_reward))

            self.Xu = self.calc_Xu()                        
            n, m = self.Xu.shape[0], self.Xu.shape[1]
            Kuu = kernel(self.Xu, self.Xu, lambd=self.lambd, beta=self.beta, sigma_sq=self.sigma_sq)
            self.Kuu_inv = np.linalg.inv(Kuu)

            print('variables iter %d' % irl_iter)
            print(self.Xu)
            print(self.Kuu_inv)
            print(self.u)
            print(self.lambd)
            print(self.beta)
            self.rl_model.policy_iteration(irl_iter, self.irl_config.bound_r, name='gp', rname='gp_reward', Xu=self.Xu, Kuu_inv=self.Kuu_inv, u=self.u, lambd=self.lambd, beta=self.beta, sigma_sq=self.sigma_sq, device=self.rl_model.network.device)

            self.build_state_action_dict_policy()
            grad_u_LG = -self.Kuu_inv.dot(self.u).T
            grad_r_LD = np.array([[]])
            grad_u_r = np.empty(shape=(0, len(self.demos[0])))
            grad_theta_r = []
            coeffs_uu = grad_lambda_K_coeffs(self.Xu, self.Xu, self.sigma_sq)
            coeffs_uu.append(1./self.beta)
            grad_theta_Kuu = []
            alpha = self.Kuu_inv*self.u
            grad_theta_LG = np.array([])
            grad_theta_LH = np.array([])

            for i in range(len(coeffs_uu)):
                grad_theta_Kuu.append(np.multiply(coeffs_uu[i], Kuu))
                grad_theta_LG = np.append(grad_theta_LG, 0.5*np.trace(np.dot(alpha.dot(alpha.T)-self.Kuu_inv, grad_theta_Kuu[-1])))
                grad_H = np.trace(self.Kuu_inv.dot(self.Kuu_inv).dot(self.Kuu_inv).dot(grad_theta_Kuu[-1]))
                if i != len(coeffs_uu)-1: grad_H -= 1./(1+sum(self.lambd))
                grad_theta_LH = np.append(grad_theta_LH, grad_H)
            
            pts = list(self.demos_savf.keys())
            for pt in self.policy_savf.keys():
                if pt not in self.demos_savf.keys():
                    pts.append(pt)

            for pt in pts:
                state = np.asarray(pt[0])
                feature = self.rl_model.network.feature(np.stack([state]), to_numpy=True)
                K_r_u = kernel(feature, self.Xu, lambd=self.lambd, beta=self.beta, sigma_sq=self.sigma_sq)
                coeffs_ru = grad_lambda_K_coeffs(feature, self.Xu, self.sigma_sq)
                coeffs_ru.append(1./self.beta)
                drdu_vec = []
                for i in range(len(coeffs_ru)):
                    drdu_vec.append((np.multiply(coeffs_ru[i], K_r_u)-K_r_u.dot(self.Kuu_inv).dot(grad_theta_Kuu[i])).dot(self.Kuu_inv).dot(self.u)[0][0])
                grad_r_LD = np.append(grad_r_LD, [[self.demos_savf[pt]-self.policy_savf[pt]]], axis=1)
                grad_u_r = np.append(grad_u_r, K_r_u.dot(self.Kuu_inv), axis=0)
                grad_theta_r.append(drdu_vec)
            
            grad_theta_r = np.array(grad_theta_r)
            grad_u_obj = grad_r_LD.dot(grad_u_r)+grad_u_LG
            grad_theta_obj = grad_r_LD.dot(grad_theta_r)+grad_theta_LG.reshape(1, -1)+grad_theta_LH.reshape(1, -1)

            self.u += self.irl_config.gp_lr*grad_u_obj.T
            self.lambd += self.irl_config.gp_lr*grad_theta_obj.T[:-1]
            self.beta += self.irl_config.gp_lr*grad_theta_obj[0, -1]
            
            
    def calc_Xu(self):
        demo_states = self.demos[0]
        Xu = []
        pos = 0
        while pos < len(demo_states):
            states = demo_states[pos:(min(pos+self.irl_config.batch_size_demos, len(demo_states)))]
            batch_phi = self.rl_model.network.feature(np.stack(states), to_numpy=True)
            Xu.append(batch_phi)
            pos = min(pos+self.irl_config.batch_size_demos, len(demo_states))
        Xu = np.concatenate(Xu, axis=0)
        return Xu


    def build_state_action_dict_demos(self):
        print('build savf demos ...')
        sa_demos = {}
        demo_states = self.demos[0]
        demo_actions = self.demos[1]
        for i in range(len(demo_states)):
            pt = (tuple(demo_states[i]), tuple(demo_actions[i]))
            if pt not in sa_demos.keys(): sa_demos[pt] = 0
            sa_demos[pt] += 1.0/len(demo_states)
        self.demos_savf = defaultdict(lambda:0, sa_demos)


    def build_state_action_dict_policy(self):
        print('build savf policy ...')
        sample_size = self.rl_model.replay_M.size()/10
        sa_policy = {}
        experiences = self.rl_model.replay_M.sample(batch_size=sample_size)
        states, actions, _, _, _ = experiences
        for i in range(len(states)):
            policy_pt = (tuple(states[i]), tuple(actions[i]))
            flag, demo_pt = self.state_action_policy_in_demos(policy_pt)
            if flag:
                if demo_pt not in sa_policy.keys(): sa_policy[demo_pt] = 0
                sa_policy[demo_pt] += 1.0/sample_size
            else: sa_policy[policy_pt] = 1.0/sample_size
        self.policy_savf = defaultdict(lambda:0, sa_policy)


    def state_action_policy_in_demos(self, policy_pt):
        policy_state, policy_action = policy_pt[0], policy_pt[1]
        for demo_pt in self.demos_savf.keys():
            demo_state, demo_action = np.asarray(demo_pt[0]), np.asarray(demo_pt[1])
            if np.linalg.norm(demo_state-policy_state) < self.state_th_ubound and np.dot(demo_action, policy_action)/(np.linalg.norm(demo_action)*np.linalg.norm(policy_action)) > self.action_th_lbound:
                return True, demo_pt
        return False, None    


    def calc_state_action_th(self):
        demo_states = self.demos[0]
        minimum_gap = np.inf
        for i in range(len(demo_states)-1):
            gap = np.linalg.norm(demo_states[i]-demo_states[i+1])
            if minimum_gap > gap: minimum_gap = gap
        self.state_th_ubound = minimum_gap
        self.action_th_lbound = 0.9


    def load_demos_set(self):
        print('load demos set ...')
        base_path = 'scripts/PlayIRL'
        demos_path = os.path.join(base_path, 'DemosData')
        demos_seg = [self.policy_config.state_dim, self.policy_config.action_dim, self.policy_config.state_dim, self.policy_config.terminal_dim, self.policy_config.flag_dim]
        self.demos = np.genfromtxt(os.path.join(demos_path, 'demos.csv'), delimiter=',')
        self.demos = np.split(self.demos, np.cumsum(demos_seg), axis=1)[:-1]

        experience = [x.tolist() for x in self.demos]
        self.rl_model.replay_D.feed_batch(experience)

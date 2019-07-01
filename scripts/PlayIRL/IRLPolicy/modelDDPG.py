import torch
import numpy as np

from ..IRL.kernelUtils import *

class DDPGModel:
    def __init__(self, config):
        self.config = config
        self.task = config.task_fn(config.task_name, config.state_dim, config.action_dim)
        self.network = config.network_fn(self.task.state_dim, self.task.action_dim, config.critic_dim)
        self.target_network = config.network_fn(self.task.state_dim, self.task.action_dim, config.critic_dim)
        self.target_copy(self.target_network, self.network)
        self.replay_D = config.replay_fn()
        self.replay_M = config.replay_fn()
        self.random_process = config.random_process_fn(self.task.action_dim)
        self.total_step = 0
        self.p_episode_rewards = []
        self.q_episode_rewards = []
        self.eval_episode_rewards = []
    

    def target_copy(self, target, src):
        for target_param, param in zip(target.parameters(), src.parameters()):
            target_param.detach_()
            target_param.copy_(param)


    def soft_update(self, target, src):
        for target_param, param in zip(target.parameters(), src.parameters()):
            target_param.detach_()
            target_param.copy_(target_param*(1.0-self.config.target_network_mix)+param*self.config.target_network_mix)
    

    def critic_training(self, states, actions, next_states, terminals, flags, bound_r, **kwargs):
        omega_t = self.network.tensor(omega)
        omega_t.requires_grad = False
        
        phi_next = self.target_network.feature(next_states)
        a_next = self.target_network.actor(phi_next)
        mu_next = self.target_network.critic(phi_next, a_next)
        q_next = mu_next.mm(omega_t.unsqueeze(-1))
        rewards = self.network.predict_reward(states, **kwargs)
        rewards = torch.clamp(rewards, bound_r[0], bound_r[1])

        terminals = self.network.tensor(terminals).unsqueeze(-1)
        flags = self.network.tensor(flags)

        rewards[(flags == 2.0).nonzero()] = 100.0
        rewards[(flags == 1.0).nonzero()] = 10.0
        rewards[(flags < 0.0).nonzero()] = bound_r[0]

        q_next = self.config.discount * q_next * (1 - terminals)
        q_next.add_(rewards)
        q_next = q_next.detach()

        phi = self.network.feature(states)
        mu = self.network.critic(phi, self.network.tensor(actions))
        q = mu.mm(omega_t.unsqueeze(-1))
        critic_loss = (q - q_next).pow(2).mul(0.5).sum(-1).mean()

        self.network.zero_grad()
        critic_loss.backward()
        self.network.critic_opt.step()


    def actor_training(self, states, **kwargs):
        phi = self.network.feature(states)
        action = self.network.actor(phi)

        if kwargs['name'] == 'linear':
            ometa_t = self.network.tensor(kwargs['omega'])
            omega_t.requires_grad = False
            policy_loss = -self.network.critic(phi.detach(), action).mm(omega_t.unsqueeze(-1)).mean()

        if kwargs['name'] == 'gp':
            Xu_t = self.tensor(kwargs['Xu'])
            Kuu_inv_t = self.tensor(kwargs['Kuu_inv'])
            u_t = self.tensor(kwargs['u'])
            Xu_t.requires_grad, Kuu_inv_t.requires_grad, u_t.requires_grad = False, False, False
            kernel_critic = kernel(self.network.critic(phi.detach(), action), kwargs['Xu'], lambd=kwargs['lambd'], beta=kwargs['beta']).mm(kwargs['Kuu_inv']).mm(kwargs['u'])
            policy_loss = -self.network.tensor(kernel_critic)

        self.network.zero_grad()
        policy_loss.backward()
        self.network.actor_opt.step()


    def policy_iteration(self, itr, bound_r, **kwargs):
        # play through demonstration D
        print('policy iter from demonstration ...')
        self.target_copy(self.target_network, self.network)
        for _ in range(self.config.D_p_episodes_num):
            if self.replay_D.size() >= self.config.min_replay_size:
                experiences = self.replay_D.sample()
                states, actions, next_states, terminals, flags = experiences

                self.critic_training(states, actions, next_states, terminals, flags, bound_r, **kwargs)
                self.actor_training(states, **kwargs)

                self.soft_update(self.target_network, self.network)

        # play through replay samples M
        # self.replay_M.reset()
        for p_episode in range(self.config.p_episodes_num):
            rewards = 0.0
            self.random_process.reset_states()
            state = self.task.reset()
            
            while True:
                reward = self.network.predict_reward(np.stack([state]), True, **kwargs).item()
                action = self.network.predict(np.stack([state]), True).flatten()
                # action += self.random_process.sample()
                next_state, terminal, flag = self.task.step(state, action)
                
                rewards += reward
                self.total_step += 1

                if terminal:
                    flag = 2 if self.task.grasp_check() > 0 else -1
                if flag == 2:
                    print('grasp success')
                # flag = self.task.grasp_check() if terminal else flag
                # flag = self.task.grasp_check() if terminal else 0
                self.replay_M.feed([state, action, next_state, int(terminal), int(flag)])

                state = next_state

                if self.replay_M.size() >= self.config.min_replay_size:
                    experiences = self.replay_M.sample()
                    states, actions, next_states, terminals, flags = experiences

                    self.critic_training(states, actions, next_states, terminals, flags, bound_r, **kwargs)
                    self.actor_training(states, **kwargs)

                    self.soft_update(self.target_network, self.network)
                
                if terminal: break
            
            print('policy reward {0}'.format(rewards))
            self.p_episode_rewards.append(rewards)
            print('policy iter %d episode %d total step %d avg reward %f' % (itr, p_episode, self.total_step, np.mean(np.array(self.p_episode_rewards[-100:]))))


    def qvalue_iteration(self, itr, bound_r, **kwargs):
        # play through demonstration D
        print('qvalue iter from demonstration ...')
        self.target_copy(self.target_network, self.network)
        for _ in range(self.config.D_q_episodes_num):
            if self.replay_D.size() >= self.config.min_replay_size:
                experiences = self.replay_D.sample()
                states, actions, next_states, terminals, flags = experiences

                self.critic_training(states, actions, next_states, terminals, flags, bound_r, **kwargs)

                self.soft_update(self.target_network, self.network)

        # play through replay samples M
        # self.replay_M.reset()
        for q_episode in range(self.config.q_episodes_num):
            rewards = 0.0
            self.random_process.reset_states()
            state = self.task.reset()

            while True:
                reward = self.network.predict_reward(np.stack([state]), True, **kwargs).item()
                action = self.network.predict(np.stack([state]), True).flatten()
                # action += self.random_process.sample()
                next_state, terminal, flag = self.task.step(state, action)
                
                rewards += reward
                # self.total_step += 1

                flag = self.task.grasp_check() if terminal else flag
                # flag = self.task.grasp_check() if terminal else 0
                self.replay_M.feed([state, action, next_state, int(terminal), int(flag)])

                state = next_state

                if self.replay_M.size() >= self.config.min_replay_size:
                    experiences = self.replay_M.sample()
                    states, actions, next_states, terminals, flags = experiences

                    self.critic_training(states, actions, next_states, terminals, flags, bound_r, **kwargs)

                    self.soft_update(self.target_network, self.network)
                
                if terminal: break
            
            self.q_episode_rewards.append(rewards)
            print('qvalue iter %d episode %d total step %d avg reward %f' % (itr, q_episode, self.total_step, np.mean(np.array(self.q_episode_rewards[-100:]))))

        
    def policy_evaluation(self, itr, bound_r, save_traj=False, **kwargs):
        # evaluation deterministic action
        print('evaluation iter from demonstration ...')
        eval_traj = []
        rewards = 0.0
        for _ in range(self.config.e_episodes_num):
            state = self.task.reset()
            eval_traj.append(state)
            steps = 0

            while steps < 1000:
                reward = self.network.predict_reward(np.stack([state]), True, **kwargs).item()
                action = self.network.predict(np.stack([state]), True).flatten()
                next_state, terminal, flag = self.task.step(state, action)
                
                rewards += reward
                steps += 1
                self.task.grasp_check()
                _ = self.task.grasp_check() if terminal else flag
                state = next_state
                eval_traj.append(state)
                
                if terminal: break

        self.eval_episode_rewards.append(rewards/self.config.e_episodes_num)

        if save_traj:
            return self.eval_episode_rewards[-1], np.asarray(eval_traj)
        return self.eval_episode_rewards[-1]

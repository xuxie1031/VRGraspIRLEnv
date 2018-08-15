import numpy as np

class DDPGModel:
    def __init__(self, config):
        super(DDPGModel, self).__init__()
        self.config = config
        self.task = config.task_fn()
        self.network = config.network_fn(self.task.state_dim, self.task.action_dim, self.task.feature_dim)
        self.target_network = config.network_fn(self.task.state_dim, self.task.action_dim, self.task.feature_dim)
        self.replay_D = config.replay_fn()
        self.replay_M = config.replay_fn()
        self.random_process = config.random_process_fn(self.task.action_dim)
        self.total_step = 0
        self.episodes_num = config.episodes_num
        self.episode_rewards = []
    

    def target_copy(self, target, src):
        for target_param, param in zip(target.parameters(), src.parameters()):
            target_param.detach_()
            target_param.copy_(param)


    def soft_update(self, target, src):
        for target_param, param in zip(target.parameters(), src.parameters()):
            target_param.detach_()
            target_param.copy_(target_param*(1.0-self.config.target_network_mix)+param*self.config.target_network_mix)
    

    def critic_training(self, states, actions, next_states, terminals, omega):
        omega_t = self.network.tensor(omega)
        omega_t.requires_grad = False
        
        phi_next = self.target_network.feature(next_states)
        a_next = self.target_network.actor(phi_next)
        mu_next = self.target_network.critic(phi_next, a_next)
        q_next = mu_next.mm(omega_t.unsqueeze(-1))
        rewards = self.network.predict_reward(states, omega_t)
        terminals = self.network.tensor(terminals).unsqueeze(-1)
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


    def actor_training(self, states, omega):
        omega_t = self.network.tensor(omega)
        omega_t.requires_grad = False

        phi = self.network.feature(states)
        action = self.network.actor(phi)
        policy_loss = -self.network.critic(phi.detach(), action).mm(omega_t.unsqueeze(-1)).mean()

        self.network.zero_grad()
        policy_loss.backward()
        self.network.actor_opt.step()


    def policy_iteration(self, itr, omega):
        # play through demonstration D
        print('policy iter from demonstration ...')
        self.target_copy(self.target_network, self.network)
        for _ in range(self.config.D_p_episodes_num):
            rewards = 0.0
            experiences = self.replay_D.sample()
            states, actions, next_states, terminals = experiences

            self.critic_training(states, actions, next_states, terminals, omega)
            self.actor_training(states, omega)

            self.soft_update(self.target_network, self.network)

        # play through replay samples M
        self.replay_M.reset()
        for p_episode in range(self.config.p_episodes_num):
            rewards = 0.0
            self.random_process.reset_states()
            state = self.task.reset()
            
            while True:
                reward = self.network.predict_reward(np.stack([state]), omega, True).item()
                action = self.network.predict(np.stack([state]), True).flatten()
                next_state, terminal = self.task.step(action)
                
                rewards += reward
                self.total_step += 1
                self.replay_M.feed([state, action, next_state, int(terminal)])

                state = next_state

                if self.replay_M.size() >= self.config.min_replay_M_size:
                    experiences = self.replay_M.sample()
                    states, actions, next_states, terminals = experiences

                    self.critic_training(states, actions, next_states, terminals, omega)
                    self.actor_training(states, omega)

                    self.soft_update(self.target_network, self.network)
                
                if terminal: break
            
            self.episode_rewards.append(rewards)
            print('policy iter %d episode %d total step %d avg reward %f' % (itr, p_episode, self.total_step, np.mean(np.array(self.episode_rewards[-100:]))))


    def qvalue_iteration(self, itr, omega):
        # play through demonstration D
        print('qvalue iter from demonstration ...')
        self.target_copy(self.target_network, self.network)
        for _ in range(self.config.D_q_episodes_num):
            rewards = 0.0
            experiences = self.replay_D.sample()
            states, actions, next_states, terminals = experiences

            self.critic_training(states, actions, next_states, terminals, omega)

            self.soft_update(self.target_network, self.network)

        # play through replay samples M
        self.replay_M.reset()
        for q_episode in range(self.config.q_episodes_num):
            rewards = 0.0
            self.random_process.reset_states()
            state = self.task.reset()

            while True:
                reward = self.network.predict_reward(np.stack([state]), omega, True).item()
                action = self.network.predict(np.stack([state]), True).flatten()
                next_state, terminal = self.task.step(action)
                
                rewards += reward
                self.total_step += 1
                self.replay_M.feed([state, action, next_state, int(terminal)])

                state = next_state

                if self.replay_M.size() >= self.config.min_replay_M_size:
                    experiences = self.replay_M.sample()
                    states, actions, next_states, terminals = experiences

                    self.critic_training(states, actions, next_states, terminals, omega)

                    self.soft_update(self.target_network, self.network)
                
                if terminal: break
            
            self.episode_rewards.append(rewards)
            print('qvalue iter %d episode %d total step %d avg reward %f' % (itr, q_episode, self.total_step, np.mean(np.array(self.episode_rewards[-100:]))))
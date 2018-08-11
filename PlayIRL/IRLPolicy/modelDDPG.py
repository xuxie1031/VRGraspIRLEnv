import numpy as np

class DDPGModel:
    def __init__(self, config):
        super(DDPGModel, self).__init__()
        self.config = config
        self.task = config.task_fn()
        self.network = config.network_fn(self.task.state_dim, self.task.action_dim)
        self.target_network = config.network_fn(self.task.state_dim, self.task.action_dim)
        self.replay = config.replay_fn()
        self.random_process = config.random_process_fn(self.task.action_dim)
        self.total_step = 0
        self.episodes_num = config.episodes_num
        self.episode_rewards = []
    

    def soft_update(self, target, src):
        for target_param, param in zip(target.parameters(), src.parameters()):
            target_param.detach_()
            target_param.copy_(target_param*(1.0-self.config.target_network_mix)+param*self.config.target_network_mix)
    

    def policy_iteration(self, itr):
        for episode in range(self.episodes_num):
            rewards = 0.0
            self.random_process.reset_states()
            state = self.task.reset()
            
            while True:
                action = self.network.predict(np.stack([state]), True).flatten()
                next_state, reward, terminal, _ = self.task.step(action)
                
                rewards += reward
                self.total_step += 1
                self.replay.feed([state, action, reward, next_state, int(terminal)])

                state = next_state

                if self.replay.size() >= self.config.min_replay_size:
                    experiences = self.replay.sample()
                    states, actions, rewards, next_states, terminals = experiences

                    phi_next = self.target_network.feature(next_states)
                    a_next = self.target_network.actor(phi_next)
                    q_next = self.target_network.critic(phi_next, a_next)
                    terminals = self.network.tensor(terminals).unsqueeze(1)
                    rewards = self.network.tensor(rewards).unsqueeze(1)
                    q_next = self.config.discount * q_next * (1 - terminals)
                    q_next.add_(rewards)
                    q_next = q_next.detach()
                    phi = self.network.feature(states)
                    q = self.network.critic(phi, self.network.tensor(actions))
                    critic_loss = (q - q_next).pow(2).mul(0.5).sum(-1).mean()

                    self.network.zero_grad()
                    critic_loss.backward()
                    self.network.critic_opt.step()

                    phi = self.network.feature(states)
                    action = self.network.actor(phi)
                    policy_loss = -self.network.critic(phi.detach(), action).mean()

                    self.network.zero_grad()
                    policy_loss.backward()
                    self.network.actor_opt.step()

                    self.soft_update(self.target_network, self.network)
                
                if terminal: break
            
            self.episode_rewards.append(rewards)
            print('IRL iter %d episode %d total step %d avg reward %f' % (itr, episode, self.total_step, np.mean(np.array(self.episode_rewards[-100:]))))
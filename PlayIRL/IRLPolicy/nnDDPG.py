from .nnUtils import *

class DDPGNet(nn.Module, BaseNet):
    def __init__(
            self, 
            state_dim, 
            action_dim,
            critic_dim, 
            phi_body=None, 
            actor_body=None, 
            critic_body=None,
            actor_opt_fn=None,
            critic_opt_fn=None,
            gpu=-1):
        super(DDPGNet, self).__init__()
        if phi_body is None: phi_body = DummyBody(state_dim)
        if actor_body is None: actor_body = DummyBody(phi_body.feature_dim)
        if critic_body is None: critic_body = DummyBody(phi_body.feature_dim)
        self.phi_body = phi_body
        self.actor_body = actor_body
        self.critic_body = critic_body
        self.fc_action = layer_init(nn.Linear(actor_body.feature_dim, action_dim), 1e-3)
        self.fc_critic = layer_init(nn.Linear(critic_body.feature_dim, critic_dim), 1e-3)

        self.actor_params = list(self.actor_body.parameters())+list(self.fc_action.parameters())
        self.critic_params = list(self.critic_body.parameters())+list(self.fc_critic.parameters())
        self.phi_params = list(self.phi_body.parameters())
    
        if actor_opt_fn is not None and critic_opt_fn is not None:
            self.actor_opt = actor_opt_fn(self.actor_params+self.phi_params)
            self.critic_opt = critic_opt_fn(self.critic_params+self.phi_params)

        self.set_gpu(gpu)


    def feature(self, state, to_numpy=False):
        state = self.tensor(state)
        phi = self.phi_body(state)
        if to_numpy:
            return phi.cpu().detach().numpy()
        return phi


    def actor(self, phi):
        return torch.tanh(self.fc_action(self.actor_body(phi)))


    def critic(self, phi, a):
        return self.fc_critic(self.critic_body(phi, a))


    def predict(self, state, to_numpy=False):
        phi = self.feature(state)
        action = self.actor(phi)
        if to_numpy:
            return action.cpu().detach().numpy()
        return action


    def predict_reward(self, state, omega, to_numpy=False):
        omega_t = self.tensor(omega).unsqueeze(-1)
        phi = self.feature(state)
        reward = phi.mm(omega_t)
        if to_numpy:
            reward = reward.cpu().detach().numpy()

        return reward


    def predict_mu(self, state, action, to_numpy=False):
        action = self.tensor(action)
        phi = self.feature(state)
        mu = self.critic(phi, action)
        if to_numpy:
            mu = mu.cpu().detach().numpy()

        return mu


    def predict_policy_mu(self, state, to_numpy=False):
        phi = self.feature(state)
        action = self.actor(phi)
        mu = self.critic(phi, action)
        if to_numpy:
            mu = mu.cpu().detach().numpy()

        return mu


    def predict_qvalue(self, state, action, omega, to_numpy=False):
        action = self.tensor(action)
        omega_t = self.tensor(omega).unsqueeze(-1)
        phi = self.feature(state)
        mu = self.critic(phi, action)
        q = mu.mm(omega_t)
        if to_numpy:
            q = q.cpu().detach().numpy()

        return q


    def predict_policy_qvalue(self, state, omega, to_numpy=False):
        omega_t = self.tensor(omega).unsqueeze(-1)
        phi = self.feature(state)
        action = self.actor(phi)
        mu = self.critic(phi, action)
        q = mu.mm(omega_t)
        if to_numpy:
            q = q.cpu().detach().numpy()
        
        return q
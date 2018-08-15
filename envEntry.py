# /usr/bin/env python

import rospy
import torch

from PlayIRL import *

def run_playground():
    irl_config = IRLConfig()
    irl_config.episodes_num = 10

    policy_config = PolicyConfig()
    policy_config.episodes_num = 1000

    policy_config.network_fn = lambda state_dim, action_dim, critic_dim: DDPGNet(
        state_dim, action_dim, critic_dim,
        actor_body=FCBody(state_dim, hidden_units=(300, 200), gate=torch.tanh),
        critic_body=FCBodyWithAction(state_dim, action_dim, hidden_state_dim=400, hidden_units=(300, ), gate=torch.tanh),
        actor_opt_fn=lambda params: torch.optim.Adam(params, lr=1e-4),
        critic_opt_fn=lambda params: torch.optim.Adam(params, lr=1e-3)
    )
    policy_config.replay_fn = lambda : Replay(memory_size=1000000, batch_size=64)
    policy_config.random_process_fn = lambda action_dim: OrnsteinUhlenbeckProcess(size=(action_dim, ), std=LinearSchedule(.2))
    policy_config.discount = .99
    policy_config.min_replay_size = 64
    policy_config.target_network_mix = 1e-3
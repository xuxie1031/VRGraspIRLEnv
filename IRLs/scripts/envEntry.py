#! /usr/bin/env python

import rospy
import torch
import argparse

import os
import sys
sys.path.append(os.path.join(os.getcwd()))

from PlayIRL import *

def run_playground():
    rospy.init_node('playground', anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument('--irl_name', type=str, default='BIRLF')
    args = parser.parse_args()

    ''' BIRLF Setup '''
    if args.irl_name == 'BIRLF':
        irl_config = IRLConfig()
        irl_config.feature_dim = 64
        irl_config.episodes_num = 10000
        irl_config.batch_size_demos = 128
        irl_config.bound_r = (-1.0, 1.0)
        irl_config.gamma = 1.0
        irl_config.gamma_infogap = 1e4
        irl_config.evalT = 1
        # irl_config.b_load = True
        irl_config.save_flag = '_BIRLF_reward'

    ''' MaxEnt Setup '''
    if args.irl_name == 'MaxEntIRL':
        irl_config = IRLConfig()
        irl_config.maxent_lr = 1e-2
        irl_config.feature_dim = 64
        irl_config.episodes_num = 10000
        irl_config.batch_size_demos = 128
        irl_config.bound_r = (-1.0, 1.0)
        irl_config.evalT = 1
        # irl_config.b_load = True
        irl_config.save_flag = '_MaxEnt_reward'

    ''' SSIRL Setup '''
    if args.irl_name == 'SSIRL':
        irl_config = IRLConfig()
        irl_config.feature_dim = 64
        irl_config.episodes_num = 10000
        irl_config.batch_size_demos = 128
        irl_config.bound_r = (-1.0, 1.0)
        irl_config.evalT = 1
        # irl_config.b_load = True
        irl_config.save_flag = '_SSIRL_reward'

    ''' GPIRL Setup '''
    if args.irl_name == 'GPIRL':
        irl_config = IRLConfig()
        irl_config.sigma_sq = 1e-4
        irl_config.gp_lr = 1e-3
        irl_config.feature_dim = 64
        irl_config.episodes_num = 10000
        irl_config.batch_size_demos = 128
        irl_config.bound_r = (-1.0, 1.0)
        irl_config.evalT = 1
        # irl_config.b_load = True
        irl_config.save_flag = '_GPIRL_reward'
    

    ''' Policy Config '''
    policy_config = PolicyConfig()
    policy_config.D_p_episodes_num = 1#5
    policy_config.D_q_episodes_num = 5
    policy_config.p_episodes_num = 20#50
    policy_config.q_episodes_num = 20
    policy_config.e_episodes_num = 1#3

    policy_config.task_name = 'VRGrasp'
    policy_config.state_dim = 9
    policy_config.action_dim = 4
    policy_config.terminal_dim = 1
    policy_config.flag_dim = 1

    # critic dim as feature dim
    policy_config.critic_dim = irl_config.feature_dim
    policy_config.task_fn = lambda task_name, state_dim, action_dim: VRGraspTask(
        task_name, state_dim, action_dim
    )

    policy_config.network_fn = lambda state_dim, action_dim, critic_dim: DDPGNet(
        state_dim, action_dim, critic_dim,
        phi_body=FCBody(state_dim, hidden_units=(critic_dim, ), gate=torch.tanh),
        actor_body=FCBody(critic_dim, hidden_units=(300, 200), gate=torch.tanh),
        critic_body=FCBodyWithAction(critic_dim, action_dim, hidden_state_dim=400, hidden_units=(300, ), gate=torch.tanh),
        actor_opt_fn=lambda params: torch.optim.Adam(params, lr=1e-4),
        critic_opt_fn=lambda params: torch.optim.Adam(params, lr=1e-3),
        gpu=0
    )
    policy_config.replay_fn = lambda : Replay(memory_size=1000000, batch_size=64)
    policy_config.random_process_fn = lambda action_dim: OrnsteinUhlenbeckProcess(size=(action_dim, ), std=LinearSchedule(.2))
    policy_config.discount = .99
    policy_config.min_replay_size = 64
    policy_config.target_network_mix = 1e-3

    if args.irl_name == 'BIRLF':
        irl_thread = BIRLF(irl_config, policy_config)
    if args.irl_name == 'MaxEntIRL':
        irl_thread = MaxEntIRL(irl_config, policy_config)
    if args.irl_name == 'SSIRL':
        irl_thread = SSIRL(irl_config, policy_config)
    if args.irl_name == 'GPIRL':
        irl_thread = GPIRL(irl_config, policy_config)
        
    irl_thread.daemon = True
    irl_thread.start()
    rospy.spin()


if __name__ == '__main__':
    run_playground()
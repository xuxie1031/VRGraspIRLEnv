class PolicyConfig:
    def __init__(self):
        self.task_fn = None
        self.network_fn = None
        self.replay_fn = None
        self.random_process_fn = None

        self.D_p_episodes_num = 0
        self.D_q_episodes_num = 0
        self.p_episodes_num = 0
        self.q_episodes_num = 0

        self.target_network_mix = 0
        self.min_replay_size = 0
        self.discount = 0

        self.bound_r = (0, 0)

        self.state_dim = 0
        self.action_dim = 0
        self.task_name = ''

class IRLConfig:
    def __init__(self):
        self.episodes_num = 0
        self.batch_size_demos = 0
        self.gamma = 0
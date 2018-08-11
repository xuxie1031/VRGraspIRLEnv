class PolicyConfig:
    def __init__(self):
        self.task_fn = None
        self.network_fn = None
        self.replay_fn = None
        self.random_process_fn = None
        self.episodes_num = 0
        self.target_network_mix = 0
        self.min_replay_size = 0
        self.discount = 0

class IRLConfig:
    def __init__(self):
        self.episodes_num = 0
import torch
import torch.nn as nn
import torch.nn.functional as F


class BaseNet:
    def set_gpu(self, gpu):
        if gpu >= 0 and torch.cuda.is_available():
            gpu = gpu % torch.cuda.device_count()
            self.device = torch.device('cuda:%d' % (gpu))
        else:
            self.device = torch.device('cpu')
        self.to(self.device)

    def tensor(self, x):
        if isinstance(x, torch.Tensor):
            return x
        x = torch.tensor(x, device=self.device, dtype=torch.float32)
        return x


def layer_init(layer, w_scale=1.0):
    nn.init.orthogonal_(layer.weight.data)
    layer.weight.data.mul_(w_scale)
    nn.init.constant_(layer.bias.data, 0)
    return layer


class DummyBody(nn.Module):
    def __init__(self, state_dim):
        super(DummyBody, self).__init__()
        self.feature_dim = state_dim
    
    def forward(self, x):
        return x


class FCBody(nn.Module):
    def __init__(self, state_dim, hidden_units=(64, 64), gate=F.relu):
        super(FCBody, self).__init__()
        dims = (state_dim, )+hidden_units
        self.layers = nn.ModuleList([layer_init(nn.Linear(dim_in, dim_out)) for dim_in, dim_out in zip(dims[:-1], dims[1:])])
        self.gate = gate
        self.feature_dim = dims[-1]

    def forward(self, x):
        for layer in self.layers:
            x = self.gate(layer(x))
        return x


class FCBodyWithAction(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_state_dim, hidden_units=(64, 64), gate=F.relu):
        super(FCBodyWithAction, self).__init__()
        self.hidden_state_fc = layer_init(nn.Linear(state_dim, hidden_state_dim))
        dims = (hidden_state_dim+action_dim, )+hidden_units
        self.layers = nn.ModuleList([layer_init(nn.Linear(dim_in, dim_out)) for dim_in, dim_out in zip(dims[:-1], dims[1:])])
        self.gate = gate
        self.feature_dim = dims[-1]

    def forward(self, x, action):
        x = self.gate(self.hidden_state_fc(x))
        x = torch.cat([x, action], dim=1)
        for layer in self.layers:
            x = self.gate(layer(x))
        return x
from random import *
import numpy as np

# sample in omega neighboring space w.r.t one random dim
def random_sample_omega(omega, step_size=.05):
    num_dim = len(omega)
    direction = randint(0, 1)*2-1
    dim = randint(0, num_dim-1)
    omega[dim] += direction*step_size

    return omega


def random_sample_omega_hyperplane_constraint(omega, normals, step_size=.05):
    pass


def random_sample_states_around_demos(demos, step_size=.1):
    states = demos[0]
    states_step = np.random.randn(*states.shape)*step_size
    states_step /= np.linalg.norm(states_step, axis=1, keepdims=True)

    return states + states_step


def random_sample_actions_by_norm(action_dim, action_norm, action_num=16):
    actions = np.random.randn(action_num, *action_dim)*action_norm

    return actions


# sample probability
def sample_prob(p):
    return p > uniform(0.0, 1.0)
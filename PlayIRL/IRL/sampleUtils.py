from random import *
import numpy as np


def is_omega_in_halfspace(omega, normals):
    res = normals.dot(omega)
    if res[res < 0.0].shape[0] == 0:
        return True, 0.0
    else:
        return False, max(abs(res[res < 0.0]))


# sample in omega neighboring space w.r.t one random dim
def random_sample_omega(omega, step_size=.05):
    num_dim = len(omega)
    direction = randint(0, 1)*2-1
    dim = randint(0, num_dim-1)
    omega[dim] += direction*step_size

    return omega


def random_sample_omega_hyperplane_constraint(omega, normals, step_size=.05, search_itr=5, trial_num_per_itr=8):
    for _ in range(search_itr):
        for _ in range(trial_num_per_itr):
            delta_omega = np.random.randn(*omega.shape)*step_size
            if is_omega_in_halfspace(omega+delta_omega, normals)[0]:
                return omega+delta_omega
        step_size *= .1
    
    return None
            

def random_sample_states_around_demos(demos, step_size=.1):
    states = demos[0]
    states_step = np.random.randn(*states.shape)*step_size
    states_step /= np.linalg.norm(states_step, axis=1, keepdims=True)

    return states + states_step


def random_sample_actions_by_norm(action_dim, action_num=8):
    actions = np.random.randn(action_num, *action_dim)

    return actions


# sample probability
def sample_prob(p):
    return p > uniform(0.0, 1.0)
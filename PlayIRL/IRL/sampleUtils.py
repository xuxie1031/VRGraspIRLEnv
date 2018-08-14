from random import *


# sample in omega neighboring space w.r.t one random dim
def random_sample_omega(omega, idx, step_size=.05):
    num_dim = len(omega)
    direction = randint(0, 1)*2-1
    dim = randint(0, num_dim-1)
    omega[dim] += direction*step_size

    return omega
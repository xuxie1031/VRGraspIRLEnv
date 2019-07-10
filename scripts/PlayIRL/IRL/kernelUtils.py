import torch
import numpy as np

def kernel_calc(xi, xj, k_name='ARD', **kwargs):
    if k_name == 'ARD':
        assert len(xi) == len(kwargs['lambd'])
        trace = sum(kwargs['lambd'])[0]
        s = np.sum(kwargs['lambd'].flatten()*(xi-xj)**2+2*kwargs['i_neq_j']*kwargs['sigma_sq']*trace, axis=1)
        return kwargs['beta']*np.exp(-s/2.)


def kernel_calc_tensor(xi, xj, k_name='ARD', **kwargs):
    if k_name == 'ARD':
        assert len(xi) == len(kwargs['lambd'])
        trace = sum(kwargs['lambd'])[0]
        lambd_t = torch.from_numpy(kwargs['lambd'].flatten()).float().to(kwargs['device']).detach()
        i_neq_j_t = torch.from_numpy(kwargs['i_neq_j']).float().to(kwargs['device']).detach()
        s = torch.sum(lambd_t*(xi-xj)**2+2*i_neq_j_t*kwargs['sigma_sq']*trace, dim=1)
        return kwargs['beta']*torch.exp(-s/2.)


def kernel(x1, x2, k_name='ARD', **kwargs):
    is_not_diagnol = len(x1) != len(x2)
    i_neq_j = np.ones((len(x2), 1))

    K = np.zeros((len(x1), len(x2)))
    for i in range(len(x1)):
        xi = x1[i, :]
        if not is_not_diagnol:
            i_neq_j = np.asarray(i != np.arange(len(x2)), dtype=np.float).reshape(-1, 1)
        K[i, :] = kernel_calc(xi, x2, k_name='ARD', lambd=kwargs['lambd'], beta=kwargs['beta'], i_neq_j=i_neq_j, sigma_sq=kwargs['sigma_sq'])

    return K


def kernel_tensor(x1, x2, k_name='ARD', **kwargs):
    is_not_diagnol = len(x1) != len(x2)
    i_neq_j = np.ones((len(x2), 1))

    K = torch.zeros(len(x1), len(x2)).to(kwargs['device'])
    for i in range(len(x1)):
        xi = x1[i, :]
        if not is_not_diagnol:
            i_neq_j = np.asarray(i != np.arange(len(x2)), dtype=np.float).reshape(-1, 1)
        K[i, :] = kernel_calc_tensor(xi, x2, k_name='ARD', lambd=kwargs['lambd'], beta=kwargs['beta'], i_neq_j=i_neq_j, sigma_sq=kwargs['sigma_sq'], device=kwargs['device'])
    
    return K


def grad_lambda_K_coeffs(x1, x2, sigma_sq):
    is_not_diagnol = len(x1) != len(x2)
    i_neq_j = np.ones((len(x2), ))

    m = x1.shape[1]
    coeff_matrice = []
    for k in range(m):
        K = []
        for i in range(len(x1)):
            xi = x1[i]
            if not is_not_diagnol:
                i_neq_j = np.asarray(i != np.arange(len(x2)), dtype=np.float)
            K.append(-0.5*(xi[k]-x2[:, k])**2-i_neq_j*sigma_sq)
        coeff_matrice.append(np.array(K))
    return coeff_matrice
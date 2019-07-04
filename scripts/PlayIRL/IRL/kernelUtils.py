import torch
import numpy as np

def kernel_calc(xi, xj, k_name='RBF', **kwargs):
    assert len(xi) == len(xj)

    if name == 'RBF':
        s = np.sum((xi-xj)**2)
        return kwargs['sigma_f']**2*np.exp(-1. / (2*kwargs['l']**2)*s)+kwargs['sigma_n']**2*(not kwargs['i_neq_j'])
    
    if name == 'ARD':
        assert len(xi) == len(kwargs['lambd'])
        trace = sum(kwargs['lambd'])
        s = np.sum(kwargs['lambd']*(xi-xj)**2+2*kwargs['i_neq_j']*kwargs['sigma_sq']*trace)
        return kwargs['beta']*np.exp(-s/2.)


def kernel_calc_tensor(xi, xj, k_name='RBF', **kwargs):
    assert len(xi) == len(xj)

    if name == 'RBF':
        s = torch.sum((xi-xj)**2)
        return kwargs['sigma_f']**2*torch.exp(-1. / (2*kwargs['l']**2)*s)+kwargs['sigma_n']**2*(not kwargs['i_neq_j'])
    
    if name == 'ARD':
        assert len(xi) == len(kwargs['lambd'])
        trace = sum(kwargs['lambd'])
        s = torch.sum(kwargs['lambd']*(xi-xj)**2+2*kwargs['i_neq_j']*kwargs['sigma_sq']*trace)
        return kwargs['beta']*torch.exp(-s/2.)


def kernel(x1, x2, **kwargs):
    is_not_diagnol = len(x1) != len(x2)

    K = np.zeros((len(x1), len(x2)))
    for i in range(len(x1)):
        xi = x1[i, :]
        for j in range(len(x2)):
            xj = x2[j, :]
            K[i, j] = kernel_calc(xi, xj, k_name='ARD', lambd=kwargs['lambd'], beta=kwargs['beta'], i_neq_j=((i!=j) or is_not_diagnol), sigma_sq=kwargs['sigma_sq'])

    return K


def kernel_tensor(x1, x2, **kwargs):
    is_not_diagnol = len(x1) != len(x2)

    K = torch.zeros(len(x1), len(x2)).to(kwargs['device'])
    for i in range(len(x1)):
        xi = x1[i, :]
        for j in range(len(x2)):
            xj = x2[j, :]
            K[i, j] = kernel_calc_tensor(xi, xj, k_name='ARD', lambd=kwargs['lambd'], beta=kwargs['beta'], i_neq_j=((i!=j) or is_not_diagnol), sigma_sq=kwargs['sigma_sq'])

    return K


def grad_lambda_K_coeffs(self, x1, x2, sigma_sq):
    is_not_diagnol = len(x1) != len(x2)
    m = x1.shape[1]
    coeff_matrice = []
    for k in range(m):
        K = []
        for i in range(len(x1)):
            row = []
            xi = x1[i]
            for j in range(len(x2)):
                xj = x2[j]
                row.append(-0.5*(xi[k]-xj[k])**2-(is_not_diagnol or (i!=j))*sigma_sq)
            K.append(row)
        coeff_matrice.append(np.array(K))
    return coeff_matrice
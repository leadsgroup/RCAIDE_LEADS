# src.py
# (c) Copyright 2024 Aerospace Research Community LLC

# Created:  Oct 2024 M. Clarke
# Modified: Apr 2026, E. Botero

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------  

import RNUMPY as rp

j   = rp.jax_handle
np  = rp.numpy_handle
sp  = rp.scipy_handle
tr  = rp.torch_handle

if j is not None:
    try:
        import jax.scipy.linalg
    except ImportError:
        pass
    
jnp = j.numpy if j else None
jl  = j.scipy.linalg if j else None
sl  = sp.linalg  if sp is not None else None
tl  = tr.linalg  if tr else None
 
def block_diag(*arrs): 
    if rp.use_jax: return j.scipy.linalg.block_diag(*arrs)
    elif rp.use_torch: raise NotImplementedError('PyTorch does not support block_diag directly')
    else: return sp.linalg.block_diag(*arrs)   
                  
def cho_factor(a, lower=False, overwrite_a=False, check_finite=True): 
    if rp.use_jax: return jl.cho_factor(a, lower=lower, overwrite_a=overwrite_a, check_finite=check_finite)
    elif rp.use_torch:
        # torch.linalg.cholesky returns the factor
        return rp.TorchArray(tr.linalg.cholesky(a, upper=not lower)), lower
    else: return sl.cho_factor(a, lower=lower, overwrite_a=overwrite_a, check_finite=check_finite)
 
def cho_solve(c_and_lower, b, overwrite_b=False, check_finite=True): 
    if rp.use_jax: return jl.cho_solve(c_and_lower, b, overwrite_b=overwrite_b, check_finite=check_finite)
    elif rp.use_torch:
        c, lower = c_and_lower
        return rp.TorchArray(tr.linalg.cholesky_solve(b, c, upper=not lower))
    else: return sl.cho_solve(c_and_lower, b, overwrite_b=overwrite_b, check_finite=check_finite) 


def eigh_tridiagonal(d, e, *, eigvals_only=False, select='a', select_range=None, tol=None): 
    if rp.use_jax: return jnp.eigh_tridiagonal(d, e, eigvals_only=eigvals_only, select=select, select_range=select_range, tol=tol)
    elif rp.use_torch: raise NotImplementedError('PyTorch does not support eigh_tridiagonal directly')
    else: return np.eigh_tridiagonal(d, e, eigvals_only=eigvals_only, select=select, select_range=select_range, tol=tol)
           

def det(a, overwrite_a=False, check_finite=True): 
    if rp.use_jax: return jl.det(a, overwrite_a=overwrite_a, check_finite=check_finite)
    elif rp.use_torch: return rp.TorchArray(tr.linalg.det(a))
    else: return sl.det(a, overwrite_a=overwrite_a, check_finite=check_finite)


def eigh(a, b = None, lower = True, eigvals_only = False, overwrite_a= False, overwrite_b = False, turbo = True,
         eigvals = None, type = 1, check_finite = True): 
    if rp.use_jax: return jl.eigh(a, b, lower = lower, eigvals_only = eigvals_only, overwrite_a= overwrite_a,
                          overwrite_b =overwrite_b, turbo = turbo, eigvals= eigvals, type = type,
                          check_finite= check_finite)
    elif rp.use_torch:
        if b is not None: raise NotImplementedError('Generalized eigh not supported for Torch here')
        vals, vecs = tr.linalg.eigh(a, UPLO='L' if lower else 'U')
        if eigvals_only: return rp.TorchArray(vals)
        return rp.TorchArray(vals), rp.TorchArray(vecs)
    else: return sl.eigh(a, b, lower = lower, eigvals_only = eigvals_only, overwrite_a= overwrite_a,
                          overwrite_b =overwrite_b, type = type, check_finite= check_finite)

def expm(A, *, upper_triangular=False, max_squarings=16): 
    if rp.use_jax: return jl.expm(A, upper_triangular=upper_triangular, max_squarings=max_squarings)
    elif rp.use_torch: return rp.TorchArray(tr.matrix_exp(A))
    else: return sl.expm(A)
 
def expm_frechet(A, E, *, method = None, compute_expm= True): 
    if rp.use_jax: return jnp.expm_frechet(A, E,method = method, compute_expm= compute_expm)
    elif rp.use_torch: raise NotImplementedError('expm_frechet not supported for Torch')
    else: return np.expm_frechet(A, E, method=method, compute_expm=compute_expm )
           
def hessenberg(a, calc_q= False, overwrite_a = False, check_finite= True): 
    if rp.use_jax: return jnp.hessenberg(a, calc_q= calc_q, overwrite_a = overwrite_a, check_finite= check_finite )
    elif rp.use_torch: raise NotImplementedError('hessenberg not supported for Torch')
    else: return np.hessenberg(a, calc_q= calc_q, overwrite_a = overwrite_a, check_finite= check_finite )

def hilbert(n): 
    if rp.use_jax: return jnp.hilbert(n)
    elif rp.use_torch: raise NotImplementedError('hilbert not supported for Torch')
    else: return np.hilbert(n)

def inv(a, overwrite_a=False, check_finite=True ): 
    if rp.use_jax: return jnp.inv(a, overwrite_a=overwrite_a, check_finite=check_finite )
    elif rp.use_torch: return rp.TorchArray(tr.linalg.inv(a))
    else: return np.inv(a, overwrite_a=overwrite_a, check_finite=check_finite)

def lu_factor(a, overwrite_a=False, check_finite=True): 
    if rp.use_jax: return jl.lu_factor(a, overwrite_a=overwrite_a, check_finite=check_finite )
    elif rp.use_torch:
        P, L, U = tr.linalg.lu(a)
        return (rp.TorchArray(L), rp.TorchArray(U)), rp.TorchArray(P) # not exactly same format
    else: return sl.lu_factor(a, overwrite_a=overwrite_a, check_finite=check_finite )
    
def lu(a, permute_l=False, overwrite_a=False, check_finite=True, p_indices=False ): 
    if rp.use_jax: return jl.lu(a, permute_l=permute_l, overwrite_a=overwrite_a, check_finite=check_finite, p_indices=p_indices )
    elif rp.use_torch:
        P, L, U = tr.linalg.lu(a)
        if permute_l: return rp.TorchArray(tr.matmul(P, L)), rp.TorchArray(U)
        return rp.TorchArray(P), rp.TorchArray(L), rp.TorchArray(U)
    else: return sl.lu(a, permute_l=permute_l, overwrite_a=overwrite_a, check_finite=check_finite, p_indices=p_indices)

def lu_solve(lu_and_piv, b, trans=0, overwrite_b=False, check_finite=True): 
    if rp.use_jax: return jl.lu_solve(lu_and_piv, b, trans=trans, overwrite_b=overwrite_b, check_finite=check_finite )
    elif rp.use_torch: raise NotImplementedError('lu_solve not supported for Torch easily')
    else: return sl.lu_solve(lu_and_piv, b, trans=trans, overwrite_b=overwrite_b, check_finite=check_finite )

def polar(a, side='right', *, method='qdwh', eps=None, max_iterations=None): 
    if rp.use_jax: return jl.polar(a, side=side, method=method, eps=eps, max_iterations=max_iterations)
    elif rp.use_torch:
        U, S, Vh = tr.linalg.svd(a, full_matrices=False)
        P = tr.matmul(Vh.mH, tr.matmul(tr.diag_embed(S), Vh))
        U_polar = tr.matmul(U, Vh)
        return rp.TorchArray(U_polar), rp.TorchArray(P)
    else: return sl.polar(a, side=side)
                
def qr(a, overwrite_a = False, lwork  = None, mode = 'full', pivoting = False, check_finite = True): 
    if rp.use_jax: return jl.qr(a, overwrite_a = overwrite_a, lwork  = lwork, mode = mode, pivoting = pivoting, check_finite = check_finite)      
    elif rp.use_torch:
        Q, R = tr.linalg.qr(a, mode='reduced' if mode == 'economic' else 'complete')
        return rp.TorchArray(Q), rp.TorchArray(R)
    else: return sl.qr(a, overwrite_a = overwrite_a, lwork  = lwork, mode = mode, pivoting = pivoting, check_finite = check_finite)   
 
def rsf2csf(T, Z, check_finite=True): 
    if rp.use_jax: return jnp.rsf2csf(T, Z, check_finite=check_finite )
    elif rp.use_torch: raise NotImplementedError('rsf2csf not supported for Torch')
    else: return np.rsf2csf(T, Z, check_finite=check_finite)

def schur(a, output='real'): 
    if rp.use_jax: return jl.schur(a, output=output)
    elif rp.use_torch: raise NotImplementedError('schur not supported for Torch')
    else: return sl.schur(a, output=output)

def solve(a, b, lower=False, overwrite_a=False, overwrite_b=False, debug=False, check_finite=True, assume_a='gen'): 
    if rp.use_jax: return jl.solve(a, b, lower=lower, overwrite_a=overwrite_a, overwrite_b=overwrite_b, debug=debug, check_finite=check_finite, assume_a=assume_a)
    elif rp.use_torch: return rp.TorchArray(tr.linalg.solve(a, b))
    else: return sl.solve(a, b, lower=lower, overwrite_a=overwrite_a, overwrite_b=overwrite_b, check_finite=check_finite, assume_a=assume_a)

def solve_triangular(*args, **kwargs): 
    if rp.use_jax: return jl.solve_triangular(*args, **kwargs)
    elif rp.use_torch: return rp.TorchArray(tr.linalg.solve_triangular(*args, **kwargs))
    else: return sl.solve_triangular(*args, **kwargs)

def sqrtm(A, blocksize=1): 
    if rp.use_jax: return jnp.sqrt(A, blocksize=blocksize)
    elif rp.use_torch: raise NotImplementedError('sqrtm not supported for Torch')
    else: return np.sqrt(A, blocksize=blocksize)
 
def svd(a, full_matrices= True, compute_uv = True, overwrite_a = False, check_finite= True, lapack_driver= 'gesdd'): 
    if rp.use_jax: return jl.svd(a, full_matrices= full_matrices, compute_uv = compute_uv, overwrite_a =overwrite_a, check_finite= check_finite, lapack_driver= lapack_driver)
    elif rp.use_torch:
        res = tr.linalg.svd(a, full_matrices=full_matrices)
        return tuple(rp.TorchArray(r) for r in res)
    else: return sl.svd(a, full_matrices= full_matrices, compute_uv = compute_uv, overwrite_a =overwrite_a, check_finite= check_finite, lapack_driver= lapack_driver)

def toeplitz(c, r=None): 
    if rp.use_jax: return jnp.toeplitz(c, r=r)
    elif rp.use_torch: raise NotImplementedError('toeplitz not supported for Torch')
    else: return np.toeplitz(c, r=r)

def funm(A, func, disp=True): 
    if rp.use_jax: return jnp.funm(A, func, disp=disp)
    elif rp.use_torch: raise NotImplementedError('funm not supported for Torch')
    else: return np.funm(A, func, disp=disp)        
      
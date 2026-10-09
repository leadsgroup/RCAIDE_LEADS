# src.py
# (c) Copyright 2024 Aerospace Research Community LLC

# Created:  Aug 2024 E. Botero
# Modified: Apr 2026, E. Botero

import RNUMPY as rp

j   = rp.jax_handle
np  = rp.numpy_handle
tr  = rp.torch_handle
jnp = j.numpy if j else None
jl = jnp.linalg if jnp else None
nl = np.linalg
tl = tr.linalg if tr else None

def multi_dot(arrays, *, precision=None):
    if rp.use_jax: return jl.multi_dot(arrays, precision=precision)
    elif rp.use_torch:
        # torch doesn't have multi_dot directly in linalg, but we can implement sequentially
        res = arrays[0]
        for a in arrays[1:]:
            res = tr.matmul(res, a)
        return rp.TorchArray(res)
    else: return rp.NumpyArray(nl.multi_dot(arrays, out=precision))
    
def cross(x1, x2, /, *, axis=-1):
    if rp.use_jax: return jl.cross(x1, x2, axis=axis)
    elif rp.use_torch: return rp.TorchArray(tr.cross(x1, x2, dim=axis))
    else: return rp.NumpyArray(nl.cross(x1, x2, axis=axis))
    
def cholesky(a, *, upper=False):
    if rp.use_jax: return jl.cholesky(a, upper=upper)
    elif rp.use_torch: return rp.TorchArray(tr.linalg.cholesky(a, upper=upper))
    else: return rp.NumpyArray(nl.cholesky(a, upper=upper))
    
def outer(x1, x2, /):
    if rp.use_jax: return jl.outer(x1, x2)
    elif rp.use_torch: return rp.TorchArray(tr.outer(x1, x2))
    else: return rp.NumpyArray(nl.outer(x1, x2))
    
def qr(a, mode='reduced'):
    if rp.use_jax: return jl.qr(a, mode=mode)
    elif rp.use_torch:
        Q, R = tr.linalg.qr(a, mode='reduced' if mode == 'reduced' else 'complete')
        return rp.TorchArray(Q), rp.TorchArray(R)
    else: return rp.NumpyArray(nl.qr(a, mode=mode))
    
def svd(a, full_matrices=True, *, compute_uv=True, hermitian=False, subset_by_index=None):
    if rp.use_jax: return jl.svd(a, full_matrices=full_matrices, compute_uv=compute_uv,
                        hermitian=hermitian, subset_by_index=subset_by_index)
    elif rp.use_torch:
        res = tr.linalg.svd(a, full_matrices=full_matrices)
        return tuple(rp.TorchArray(r) for r in res)
    else: return rp.NumpyArray(nl.svd(a, full_matrices=full_matrices, compute_uv=compute_uv))
    
def svdvals(x, /):
    if rp.use_jax: return jl.svdvals(x)
    elif rp.use_torch: return rp.TorchArray(tr.linalg.svdvals(x))
    else: return rp.NumpyArray(nl.svdvals(x))
    
def eig(a):
    if rp.use_jax: return jl.eig(a)
    elif rp.use_torch:
        vals, vecs = tr.linalg.eig(a)
        return rp.TorchArray(vals), rp.TorchArray(vecs)
    else:
        vals, vecs = nl.eig(a)
        return rp.NumpyArray(vals), rp.NumpyArray(vecs)
    
def eigh(a, UPLO=None, symmetrize_input=True):
    if rp.use_jax: return jl.eigh(a, UPLO=UPLO, symmetrize_input=symmetrize_input)
    elif rp.use_torch:
        vals, vecs = tr.linalg.eigh(a, UPLO=UPLO if UPLO else 'L')
        return rp.TorchArray(vals), rp.TorchArray(vecs)
    else: return rp.NumpyArray(nl.eigh(a, UPLO=UPLO))
    
def eigvals(a):
    if rp.use_jax: return jl.eigvals(a)
    elif rp.use_torch: return rp.TorchArray(tr.linalg.eigvals(a))
    else: return rp.NumpyArray(nl.eigvals(a))
    
def eigvalsh(a, UPLO='L'):
    if rp.use_jax: return jl.eigvalsh(a, UPLO=UPLO)
    elif rp.use_torch: return rp.TorchArray(tr.linalg.eigvalsh(a, UPLO=UPLO))
    else: return rp.NumpyArray(nl.eigvalsh(a, UPLO=UPLO))
    
def norm(x, ord=None, axis=None, keepdims=False):
    if rp.use_jax: return jl.norm(x, ord=ord, axis=axis, keepdims=keepdims)
    elif rp.use_torch: return rp.TorchArray(tr.linalg.norm(x, ord=ord, dim=axis, keepdim=keepdims))
    else: return rp.NumpyArray(nl.norm(x, ord=ord, axis=axis, keepdims=keepdims))
    
def matrix_norm(x, /, *, keepdims=False, ord='fro'):
    if rp.use_jax: return jl.matrix_norm(x, keepdims=keepdims, ord=ord)
    elif rp.use_torch: return rp.TorchArray(tr.linalg.matrix_norm(x, ord=ord, keepdim=keepdims))
    else: return rp.NumpyArray(nl.matrix_norm(x, keepdims=keepdims, ord=ord))
    
def vector_norm(x, /, *, axis=None, keepdims=False, ord=2):
    if rp.use_jax: return jl.vector_norm(x, axis=axis, keepdims=keepdims, ord=ord)
    elif rp.use_torch: return rp.TorchArray(tr.linalg.vector_norm(x, ord=ord, dim=axis, keepdim=keepdims))
    else: return rp.NumpyArray(nl.vector_norm(x, axis=axis, keepdims=keepdims, ord=ord))
    
def cond(x, p=None):
    if rp.use_jax: return jl.cond(x, p=p)
    elif rp.use_torch: return rp.TorchArray(tr.linalg.cond(x, p=p))
    else: return rp.NumpyArray(nl.cond(x, p=p))
    
def det(a):
    if rp.use_jax: return jl.det(a)
    elif rp.use_torch: return rp.TorchArray(tr.linalg.det(a))
    else: return rp.NumpyArray(nl.det(a))
    
def matrix_rank(M, rtol=None, *, tol=None):
    if rp.use_jax: return jl.matrix_rank(M, rtol=rtol, tol=tol)
    elif rp.use_torch: return rp.TorchArray(tr.linalg.matrix_rank(M, tol=tol if tol is not None else rtol))
    else: return rp.NumpyArray(nl.matrix_rank(M, tol=tol, hermitian=False, rtol=rtol))
    
def slogdet(a, *, method=None):
    if rp.use_jax: return jl.slogdet(a, method=method)
    elif rp.use_torch:
        sign, logdet = tr.linalg.slogdet(a)
        return rp.TorchArray(sign), rp.TorchArray(logdet)
    else: return rp.NumpyArray(nl.slogdet(a))
    
def matrix_power(a, n):
    if rp.use_jax: return jl.matrix_power(a, n)
    elif rp.use_torch: return rp.TorchArray(tr.linalg.matrix_power(a, n))
    else: return rp.NumpyArray(nl.matrix_power(a, n))
    
def tensordot(x1, x2, /, *, axes=2, precision=None, preferred_element_type=None):
    if rp.use_jax: return jl.tensordot(x1, x2, axes=axes, precision=precision, preferred_element_type=preferred_element_type)
    elif rp.use_torch: return rp.TorchArray(tr.tensordot(x1, x2, dims=axes))
    else: return rp.NumpyArray(nl.tensordot(x1, x2, axes=axes))
    
def matmul(x1, x2, /, *, precision=None, preferred_element_type=None):
    if rp.use_jax: return jl.matmul(x1, x2, precision=precision, preferred_element_type=preferred_element_type)
    elif rp.use_torch: return rp.TorchArray(tr.matmul(x1, x2))
    else: return rp.NumpyArray(nl.matmul(x1, x2))
    
def trace(x, /, *, offset=0, dtype=None):
    if rp.use_jax: return jl.trace(x, offset=offset, dtype=dtype)
    elif rp.use_torch: return rp.TorchArray(tr.trace(x)) # Note: torch.trace only for 2D, offset not supported easily
    else: return rp.NumpyArray(np.trace(x, offset=offset, dtype=dtype))
    
def solve(a, b):
    if rp.use_jax: return jl.solve(a, b)
    elif rp.use_torch: return rp.TorchArray(tr.linalg.solve(a, b))
    else: return rp.NumpyArray(nl.solve(a, b))
    
def tensorsolve(a, b, axes=None):
    if rp.use_jax: return jl.tensorsolve(a, b, axes=axes)
    elif rp.use_torch: return rp.TorchArray(tr.linalg.tensorsolve(a, b, dims=axes))
    else: return rp.NumpyArray(nl.tensorsolve(a, b, axes=axes))
    
def lstsq(a, b, rcond=None, *, numpy_resid=False):
    if rp.use_jax: return jl.lstsq(a, b, rcond=rcond, numpy_resid=numpy_resid)
    elif rp.use_torch:
        res = tr.linalg.lstsq(a, b, rcond=rcond)
        return tuple(rp.TorchArray(r) for r in res)
    else: return rp.NumpyArray(nl.lstsq(a, b, rcond=rcond))
    
def inv(a):
    if rp.use_jax: return jl.inv(a)
    elif rp.use_torch: return rp.TorchArray(tr.linalg.inv(a))
    else: return rp.NumpyArray(nl.inv(a))
    
def pinv(a, rtol=None, hermitian=False, *, rcond=None):
    if rp.use_jax: return jl.pinv(a, rtol=rtol, hermitian=hermitian, rcond=rcond)
    elif rp.use_torch: return rp.TorchArray(tr.linalg.pinv(a, rcond=rcond if rcond is not None else rtol, hermitian=hermitian))
    else: return rp.NumpyArray(nl.pinv(a, rcond=rcond, hermitian=hermitian, rtol=rtol))
    
def tensorinv(a, ind=2):
    if rp.use_jax: return jl.tensorinv(a, ind=ind)
    elif rp.use_torch: return rp.TorchArray(tr.linalg.tensorinv(a, ind=ind))
    else: return rp.NumpyArray(nl.tensorinv(a, ind=ind))
    
def diagonal(x, /, *, offset=0):
    if rp.use_jax: return jl.diagonal(x, offset=offset)
    elif rp.use_torch: return rp.TorchArray(tr.diagonal(x, offset=offset))
    else: return rp.NumpyArray(np.diagonal(x, offset=offset))
    
def matrix_transpose(x, /):
    if rp.use_jax: return jl.matrix_transpose(x)
    elif rp.use_torch: return rp.TorchArray(tr.transpose(x, -2, -1))
    else: return rp.NumpyArray(nl.matrix_transpose(x))
    
def LinAlgError(): raise NotImplementedError
    
def vecdot(x1, x2, /, *, axis=-1, precision=None, preferred_element_type=None):
    if rp.use_jax: return jl.vecdot(x1, x2, axis=axis, precision=precision, preferred_element_type=preferred_element_type)
    elif rp.use_torch:
        if hasattr(tr.linalg, 'vecdot'):
            return rp.TorchArray(tr.linalg.vecdot(x1, x2, dim=axis))
        else:
            # Fallback for older torch versions
            return rp.TorchArray((x1 * x2).sum(dim=axis))
    else: return rp.NumpyArray(nl.vecdot(x1, x2, axis=axis))
    
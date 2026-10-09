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
        import jax.scipy.special
    except ImportError:
        pass

js  = j.scipy.special if j else None
ss  = sp.special if sp is not None else None
ts  = tr.special if tr else None

def fresnel(x): 
    if rp.use_jax: return js.fresnel(x)
    elif rp.use_torch: raise NotImplementedError('fresnel not supported for Torch')
    else: return ss.fresnel(x)
    
def factorial(n, exact=False): 
    if rp.use_jax: return js.factorial(n, exact=exact)
    elif rp.use_torch: return rp.TorchArray(tr.exp(tr.lgamma(n + 1)))
    else: return ss.factorial(n, exact=exact)
    
def gamma(x): 
    if rp.use_jax: return js.gamma(x)
    elif rp.use_torch: return rp.TorchArray(ts.gamma(x))
    else: return ss.gamma(x) 

def bernoulli(n):
    if rp.use_jax: return js.bernoulli(n)
    else: return ss.bernoulli(n)

def beta(p, q):
    if rp.use_jax: return js.beta(p, q)
    elif rp.use_torch: return rp.TorchArray(ts.beta(p, q))
    else: return ss.beta(p, q)

def betainc(a, b, x):
    if rp.use_jax: return js.betainc(a, b, x)
    elif rp.use_torch: return rp.TorchArray(ts.betainc(a, b, x))
    else: return ss.betainc(a, b, x)

def betaln(a, b):
    if rp.use_jax: return js.betaln(a, b)
    elif rp.use_torch: return rp.TorchArray(ts.betaln(a, b))
    else: return ss.betaln(a, b)

def digamma(x):
    if rp.use_jax: return js.digamma(x)
    elif rp.use_torch: return rp.TorchArray(ts.digamma(x))
    else: return ss.digamma(x)

def entr(x):
    if rp.use_jax: return js.entr(x)
    elif rp.use_torch: return rp.TorchArray(ts.entr(x))
    else: return ss.entr(x)

def erf(x):
    if rp.use_jax: return js.erf(x)
    elif rp.use_torch: return rp.TorchArray(tr.erf(x))
    else: return ss.erf(x)

def erfc(x):
    if rp.use_jax: return js.erfc(x)
    elif rp.use_torch: return rp.TorchArray(tr.erfc(x))
    else: return ss.erfc(x)

def erfinv(x):
    if rp.use_jax: return js.erfinv(x)
    elif rp.use_torch: return rp.TorchArray(tr.erfinv(x))
    else: return ss.erfinv(x)

def exp1(x):
    if rp.use_jax: return js.exp1(x)
    elif rp.use_torch: return rp.TorchArray(ts.exp1(x))
    else: return ss.exp1(x)

def expi(x):
    if rp.use_jax: return js.expi(x)
    elif rp.use_torch: return rp.TorchArray(ts.expi(x))
    else: return ss.expi(x)

def expit(x):
    if rp.use_jax: return js.expit(x)
    elif rp.use_torch: return rp.TorchArray(tr.sigmoid(x))
    else: return ss.expit(x)

def expn(n, x):
    if rp.use_jax: return js.expn(n, x)
    else: return ss.expn(n, x)

def gammainc(a, x):
    if rp.use_jax: return js.gammainc(a, x)
    elif rp.use_torch: return rp.TorchArray(ts.gammainc(a, x))
    else: return ss.gammainc(a, x)

def gammaincc(a, x):
    if rp.use_jax: return js.gammaincc(a, x)
    elif rp.use_torch: return rp.TorchArray(ts.gammaincc(a, x))
    else: return ss.gammaincc(a, x)

def gammaln(x):
    if rp.use_jax: return js.gammaln(x)
    elif rp.use_torch: return rp.TorchArray(tr.lgamma(x))
    else: return ss.gammaln(x)

def i0(x):
    if rp.use_jax: return js.i0(x)
    elif rp.use_torch: return rp.TorchArray(ts.i0(x))
    else: return ss.i0(x)

def i0e(x):
    if rp.use_jax: return js.i0e(x)
    elif rp.use_torch: return rp.TorchArray(ts.i0e(x))
    else: return ss.i0e(x)

def i1(x):
    if rp.use_jax: return js.i1(x)
    elif rp.use_torch: return rp.TorchArray(ts.i1(x))
    else: return ss.i1(x)

def i1e(x):
    if rp.use_jax: return js.i1e(x)
    elif rp.use_torch: return rp.TorchArray(ts.i1e(x))
    else: return ss.i1e(x)

def jv(v, z):
    if rp.use_jax:
        if hasattr(js, 'jv'): return js.jv(v, z)
        if isinstance(v, (int, np.integer)):
             # JAX's bessel_jn(z, v) returns orders 0, 1, ..., v
             # v must be a non-negative integer for this to work as expected
             if v >= 0:
                 return js.bessel_jn(z, v=int(v))[int(v)]
             else:
                 # J_{-n}(z) = (-1)^n J_n(z)
                 return ((-1)**int(-v)) * js.bessel_jn(z, v=int(-v))[int(-v)]
        raise NotImplementedError('jv with real order not natively supported in JAX. Use an integer order.')
    elif rp.use_torch:
        class JV(tr.autograd.Function):
            @staticmethod
            def forward(v, z):
                v_np   = v.detach().cpu().numpy()
                z_np   = z.detach().cpu().numpy()
                res_np = ss.jv(v_np, z_np)
                return tr.as_tensor(res_np, dtype=z.dtype, device=z.device)

            @staticmethod
            def setup_context(ctx, inputs, output):
                v, z = inputs
                ctx.save_for_backward(v, z)

            @staticmethod
            def backward(ctx, grad_output):
                v, z = ctx.saved_tensors
                v_np = v.detach().cpu().numpy()
                z_np = z.detach().cpu().numpy()
                dj_dz_np = 0.5 * (ss.jv(v_np - 1, z_np) - ss.jv(v_np + 1, z_np))
                grad_z = grad_output * tr.as_tensor(dj_dz_np, dtype=z.dtype, device=z.device)
                
                # Handle broadcasting for grad_z
                while grad_z.ndim > z.ndim:
                    grad_z = grad_z.sum(0)
                for i, dim in enumerate(z.shape):
                    if dim == 1:
                        grad_z = grad_z.sum(i, keepdim=True)
                
                return None, grad_z

        return rp.TorchArray(JV.apply(tr.as_tensor(v), tr.as_tensor(z)))
    else: return ss.jv(v, z)

def log_ndtr(x):
    if rp.use_jax: return js.log_ndtr(x)
    elif rp.use_torch: return rp.TorchArray(ts.log_ndtr(x))
    else: return ss.log_ndtr(x)

def log_softmax(x, axis=None):
    if rp.use_jax: return js.log_softmax(x, axis=axis)
    elif rp.use_torch: return rp.TorchArray(tr.log_softmax(x, dim=axis))
    else: return ss.log_softmax(x, axis=axis)

def logit(x):
    if rp.use_jax: return js.logit(x)
    elif rp.use_torch: return rp.TorchArray(tr.logit(x))
    else: return ss.logit(x)

def logsumexp(a, axis=None, b=None, keepdims=False, return_sign=False):
    if rp.use_jax: return js.logsumexp(a, axis=axis, b=b, keepdims=keepdims, return_sign=return_sign)
    else: return ss.logsumexp(a, axis=axis, b=b, keepdims=keepdims, return_sign=return_sign)

def multigammaln(a, d):
    if rp.use_jax: return js.multigammaln(a, d)
    elif rp.use_torch: return rp.TorchArray(ts.multigammaln(a, d))
    else: return ss.multigammaln(a, d)

def ndtr(x):
    if rp.use_jax: return js.ndtr(x)
    elif rp.use_torch: return rp.TorchArray(ts.ndtr(x))
    else: return ss.ndtr(x)

def ndtri(x):
    if rp.use_jax: return js.ndtri(x)
    elif rp.use_torch: return rp.TorchArray(ts.ndtri(x))
    else: return ss.ndtri(x)

def poch(z, m):
    if rp.use_jax: return js.poch(z, m)
    else: return ss.poch(z, m)

def polygamma(n, x):
    if rp.use_jax: return js.polygamma(n, x)
    elif rp.use_torch: return rp.TorchArray(ts.polygamma(n, x))
    else: return ss.polygamma(n, x)

def softmax(x, axis=None):
    if rp.use_jax: return js.softmax(x, axis=axis)
    elif rp.use_torch: return rp.TorchArray(tr.softmax(x, dim=axis))
    else: return ss.softmax(x, axis=axis)

def xlog1py(x, y):
    if rp.use_jax: return js.xlog1py(x, y)
    elif rp.use_torch: return rp.TorchArray(ts.xlog1py(x, y))
    else: return ss.xlog1py(x, y)

def xlogy(x, y):
    if rp.use_jax: return js.xlogy(x, y)
    elif rp.use_torch: return rp.TorchArray(ts.xlogy(x, y))
    else: return ss.xlogy(x, y)

def zeta(x, q=None):
    if rp.use_jax: return js.zeta(x, q)
    elif rp.use_torch: return rp.TorchArray(ts.zeta(x, q))
    else: return ss.zeta(x, q)









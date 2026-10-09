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
jnp = j.numpy if j else None
ji  = j.scipy.integrate if j else None
si  = sp.integrate if sp is not None else None
# ti  = tr.integrate if tr else None

def _jax_cumulative_trapezoid(y, x=None, dx=1.0, axis=-1, initial=None):
    """JAX implementation matching scipy.integrate.cumulative_trapezoid."""
    y = jnp.asarray(y)
    ndim = y.ndim
    axis = axis if axis >= 0 else ndim + axis
    y_m = jnp.moveaxis(y, axis, -1)

    if x is not None:
        x_m = jnp.moveaxis(jnp.asarray(x), axis, -1)
        dt = x_m[..., 1:] - x_m[..., :-1]
    else:
        dt = dx

    areas = 0.5 * (y_m[..., 1:] + y_m[..., :-1]) * dt
    cum = jnp.cumsum(areas, axis=-1)
    out = jnp.moveaxis(cum, -1, axis)

    if initial is not None:
        init_shape = list(out.shape)
        init_shape[axis] = 1
        init_arr = jnp.full(init_shape, initial, dtype=out.dtype)
        out = jnp.concatenate([init_arr, out], axis=axis)
    return out

def trapezoid(y, x=None, dx=1.0, axis=-1): 
    if rp.use_jax: return jnp.trapezoid(y=y, x=x, dx=dx, axis=axis)
    elif rp.use_torch:
        y_t = tr.as_tensor(y)
        if x is not None:
            x_t = tr.as_tensor(x)
            return rp.TorchArray(tr.trapezoid(y_t, x=x_t, dim=axis))
        else:
            return rp.TorchArray(tr.trapezoid(y_t, dx=dx, dim=axis))
    else: return np.trapezoid(y=y, x=x, dx=dx, axis=axis)

def cumulative_trapezoid(y, x=None, dx=1.0, axis=-1, initial=None):
    if rp.use_jax:
        return _jax_cumulative_trapezoid(y, x=x, dx=dx, axis=axis, initial=initial)
    elif rp.use_torch:
        y_t = tr.as_tensor(y)
        if x is not None:
            x_t = tr.as_tensor(x)
            res = tr.cumulative_trapezoid(y_t, x=x_t, dim=axis)
        else:
            res = tr.cumulative_trapezoid(y_t, dx=dx, dim=axis)
            
        if initial is not None:
            # Need to prepend the initial value along the given axis
            init_shape = list(res.shape)
            init_shape[axis] = 1
            init_tensor = tr.full(init_shape, initial, dtype=res.dtype, device=res.device)
            res = tr.cat([init_tensor, res], dim=axis)
            
        return rp.TorchArray(res)
    else:
        import scipy.integrate as spi
        return rp.NumpyArray(spi.cumulative_trapezoid(y, x=x, dx=dx, axis=axis, initial=initial))
# src.py
# (c) Copyright 2024 Aerospace Research Community LLC
# Created:  Aug 2024 E. Botero
# Modified: Apr 2026, E. Botero

# ----------------------------------------------------------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------------------------------------------------------  

import RNUMPY as rp
import warnings
import builtins
from typing import cast, Union

j = rp.jax_handle
np = rp.numpy_handle
tr = rp.torch_handle
jnp = j.numpy if j is not None else None
JaxArray = rp.JaxArray
NumpyArray = rp.NumpyArray
TorchArray = rp.TorchArray

# ----------------------------------------------------------------------------------------------------------------------
#  Debug Print Function
# ----------------------------------------------------------------------------------------------------------------------  


def debugprint(fmt, *args, ordered=False, **kwargs):
    if not rp.use_jax:
        print(fmt.format(*args, **kwargs))
    else:
        j.debug.print(fmt, *args, ordered=ordered, **kwargs)

# ----------------------------------------------------------------------------------------------------------------------
#  ndarray
# ----------------------------------------------------------------------------------------------------------------------  

_ndarray_types = [np.ndarray]
if jnp is not None:
    _ndarray_types.append(jnp.ndarray)
if tr is not None:
    _ndarray_types.append(tr.Tensor)

if len(_ndarray_types) > 1:
    ndarray = Union[tuple(_ndarray_types)]
else:
    ndarray = _ndarray_types[0]

def _to_torch_dtype(dtype):
    if dtype is None:
        return None
    if isinstance(dtype, tr.dtype):
        return dtype
    
    # Check if it is a numpy object dtype or the builtin object type
    if dtype is object or (builtins.hasattr(dtype, 'type') and dtype.type is np.object_):
        return None # Torch doesn't have an object dtype
    
    _map = {
        np.float16: tr.float16,
        np.float32: tr.float32,
        np.float64: tr.float64,
        np.double: tr.float64,
        float: tr.float32,
        np.int8: tr.int8,
        np.int16: tr.int16,
        np.int32: tr.int32,
        np.int64: tr.int64,
        int: tr.int64,
        np.uint8: tr.uint8,
        np.bool_: tr.bool,
        bool: tr.bool,
        np.complex64: tr.complex64,
        np.complex128: tr.complex128,
        complex: tr.complex64
    }
    if dtype in _map:
        return _map[dtype]
    if hasattr(dtype, 'type') and dtype.type in _map:
        return _map[dtype.type]
    return dtype

def _to_numpy_dtype(dtype):
    if dtype is None:
        return None
    
    # If it's already a numpy dtype or builtin type, return it
    # We check if it's NOT a torch/jax-specific object that happens to be a 'type'
    if isinstance(dtype, (np.dtype, type)):
        if tr is not None and isinstance(dtype, tr.dtype):
             pass # continue to torch mapping
        else:
             return dtype
    
    if tr is not None and isinstance(dtype, tr.dtype):
        _rev_torch_map = {
            tr.float16: np.float16,
            tr.float32: np.float32,
            tr.float64: np.float64,
            tr.int8: np.int8,
            tr.int16: np.int16,
            tr.int32: np.int32,
            tr.int64: np.int64,
            tr.uint8: np.uint8,
            tr.bool: np.bool_,
            tr.complex64: np.complex64,
            tr.complex128: np.complex128
        }
        return _rev_torch_map.get(dtype, dtype)
    
    return dtype

def _seq_to_base(item):
    if type(item) in (list, tuple):
        return [_seq_to_base(i) for i in item]
    if isinstance(item, tr.Tensor):
        return item.as_subclass(tr.Tensor)
    return item

# ----------------------------------------------------------------------------------------------------------------------
#  Numpy Functions
# ----------------------------------------------------------------------------------------------------------------------  


def dot(x, y):
    if rp.use_jax:
        return jnp.dot(x, y)
    elif rp.use_torch:
        x_t = tr.as_tensor(x)
        y_t = tr.as_tensor(y)
        if x_t.ndim == 0 or y_t.ndim == 0:
             return TorchArray(x_t * y_t)
        if x_t.ndim == 1 and y_t.ndim == 1:
             return TorchArray(tr.dot(x_t, y_t))
        if y_t.ndim == 1:
             return TorchArray(tr.matmul(x_t, y_t))
        return TorchArray(tr.tensordot(x_t, y_t, dims=([-1], [-2])))
    else:
        return NumpyArray(np.dot(x, y))


def sin(x, /):
    if rp.use_jax:
        return jnp.sin(x)
    elif rp.use_torch:
        return TorchArray(tr.sin(tr.as_tensor(x)))
    else:
        return NumpyArray(np.sin(x))


def cos(x, /):
    if rp.use_jax:
        return jnp.cos(x)
    elif rp.use_torch:
        return TorchArray(tr.cos(tr.as_tensor(x)))
    else:
        return NumpyArray(np.cos(x))


def tan(x, /):
    if rp.use_jax:
        return jnp.tan(x)
    elif rp.use_torch:
        return TorchArray(tr.tan(tr.as_tensor(x)))
    else:
        return NumpyArray(np.tan(x))


def arcsin(x, /):
    if rp.use_jax:
        return jnp.arcsin(x)
    elif rp.use_torch:
        return TorchArray(tr.asin(tr.as_tensor(x)))
    else:
        return NumpyArray(np.arcsin(x))


def asin(x, /):
    if rp.use_jax:
        return jnp.asin(x)
    elif rp.use_torch:
        return TorchArray(tr.asin(tr.as_tensor(x)))
    else:
        return NumpyArray(np.asin(x))


def arccos(x, /):
    if rp.use_jax:
        return jnp.arccos(x)
    elif rp.use_torch:
        return TorchArray(tr.acos(tr.as_tensor(x)))
    else:
        return NumpyArray(np.arccos(x))


def acos(x, /):
    if rp.use_jax:
        return jnp.acos(x)
    elif rp.use_torch:
        return TorchArray(tr.acos(tr.as_tensor(x)))
    else:
        return NumpyArray(np.acos(x))


def arctan(x, /):
    if rp.use_jax:
        return jnp.arctan(x)
    elif rp.use_torch:
        return TorchArray(tr.atan(tr.as_tensor(x)))
    else:
        return NumpyArray(np.arctan(x))


def atan(x, /):
    if rp.use_jax:
        return jnp.atan(x)
    elif rp.use_torch:
        return TorchArray(tr.atan(tr.as_tensor(x)))
    else:
        return NumpyArray(np.atan(x))


def hypot(x1, x2, /):
    if rp.use_jax:
        return jnp.hypot(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.hypot(tr.as_tensor(x1), tr.as_tensor(x2)))
    else:
        return NumpyArray(np.hypot(x1, x2))


def arctan2(x1, x2, /):
    if rp.use_jax:
        return jnp.arctan2(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.atan2(tr.as_tensor(x1), tr.as_tensor(x2)))
    else:
        return NumpyArray(np.arctan2(x1, x2))


def atan2(x1, x2, /):
    if rp.use_jax:
        return jnp.atan2(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.atan2(tr.as_tensor(x1), tr.as_tensor(x2)))
    else:
        return NumpyArray(np.atan2(x1, x2))


def degrees(x, /):
    if rp.use_jax:
        return jnp.degrees(x)
    elif rp.use_torch:
        return TorchArray(tr.rad2deg(tr.as_tensor(x)))
    else:
        return NumpyArray(np.degrees(x))


def radians(x, /):
    if rp.use_jax:
        return jnp.radians(x)
    elif rp.use_torch:
        return TorchArray(tr.deg2rad(tr.as_tensor(x)))
    else:
        return NumpyArray(np.radians(x))


def unwrap(p, discont=None, axis=-1, period=6.283185307179586):
    if not rp.use_jax:
        return NumpyArray(np.radians(p, discont=discont, axis=axis, period=period))
    else:
        return jnp.unwrap(p, discont=discont, axis=axis, period=period)


def deg2rad(x, /):
    if rp.use_jax:
        return jnp.deg2rad(x)
    elif rp.use_torch:
        return TorchArray(tr.deg2rad(tr.as_tensor(x)))
    else:
        return NumpyArray(np.deg2rad(x))


def rad2deg(x, /):
    if rp.use_jax:
        return jnp.rad2deg(x)
    elif rp.use_torch:
        return TorchArray(tr.rad2deg(tr.as_tensor(x)))
    else:
        return NumpyArray(np.rad2deg(x))


def sinh(x, /):
    if rp.use_jax:
        return jnp.sinh(x)
    elif rp.use_torch:
        return TorchArray(tr.sinh(tr.as_tensor(x)))
    else:
        return NumpyArray(np.sinh(x))


def cosh(x, /):
    if rp.use_jax:
        return jnp.cosh(x)
    elif rp.use_torch:
        return TorchArray(tr.cosh(tr.as_tensor(x)))
    else:
        return NumpyArray(np.cosh(x))


def tanh(x, /):
    if rp.use_jax:
        return jnp.tanh(x)
    elif rp.use_torch:
        return TorchArray(tr.tanh(tr.as_tensor(x)))
    else:
        return NumpyArray(np.tanh(x))


def arcsinh(x, /):
    if rp.use_jax:
        return jnp.arcsinh(x)
    elif rp.use_torch:
        return TorchArray(tr.asinh(tr.as_tensor(x)))
    else:
        return NumpyArray(np.arcsinh(x))


def asinh(x, /):
    if rp.use_jax:
        return jnp.asinh(x)
    elif rp.use_torch:
        return TorchArray(tr.asinh(tr.as_tensor(x)))
    else:
        return NumpyArray(np.asinh(x))


def arccosh(x, /):
    if rp.use_jax:
        return jnp.arccosh(x)
    elif rp.use_torch:
        return TorchArray(tr.acosh(tr.as_tensor(x)))
    else:
        return NumpyArray(np.arccosh(x))


def acosh(x, /):
    if rp.use_jax:
        return jnp.acosh(x)
    elif rp.use_torch:
        return TorchArray(tr.acosh(tr.as_tensor(x)))
    else:
        return NumpyArray(np.acosh(x))


def arctanh(x, /):
    if rp.use_jax:
        return jnp.arctanh(x)
    elif rp.use_torch:
        return TorchArray(tr.atanh(tr.as_tensor(x)))
    else:
        return NumpyArray(np.arctanh(x))


def atanh(x, /):
    if rp.use_jax:
        return jnp.atanh(x)
    elif rp.use_torch:
        return TorchArray(tr.atanh(tr.as_tensor(x)))
    else:
        return NumpyArray(np.atanh(x))


def round(a, decimals=0, out=None):
    if rp.use_jax:
        return jnp.round(a, decimals=decimals, out=out)
    elif rp.use_torch:
        return TorchArray(tr.round(a, decimals=decimals, out=out))
    else:
        return NumpyArray(np.round(a, decimals=decimals, out=out))


def around(a, decimals=0, out=None):
    if rp.use_jax:
        return jnp.around(a, decimals=decimals, out=out)
    elif rp.use_torch:
        return TorchArray(tr.round(a, decimals=decimals, out=out))
    else:
        return NumpyArray(np.around(a, decimals=decimals, out=out))


def rint(x, /):
    if rp.use_jax:
        return jnp.rint(x)
    elif rp.use_torch:
        return TorchArray(tr.round(tr.as_tensor(x)))
    else:
        return NumpyArray(np.rint(x))


def fix(x, out=None):
    if rp.use_jax:
        return jnp.fix(x, out=out)
    elif rp.use_torch:
        res = tr.fix(x)
        if out is not None: out.copy_(res)
        return TorchArray(res)
    else:
        return NumpyArray(np.fix(x, out=out))


def floor(x, /):
    if rp.use_jax:
        return jnp.floor(x)
    elif rp.use_torch:
        return TorchArray(tr.floor(tr.as_tensor(x)))
    else:
        return NumpyArray(np.floor(x))


def ceil(x, /):
    if rp.use_jax:
        return jnp.ceil(x)
    elif rp.use_torch:
        return TorchArray(tr.ceil(tr.as_tensor(x)))
    else:
        return NumpyArray(np.ceil(x))


def trunc(x):
    if rp.use_jax:
        return jnp.trunc(x)
    elif rp.use_torch:
        return TorchArray(tr.trunc(tr.as_tensor(x)))
    else:
        return NumpyArray(np.trunc(x))


def prod(a, axis=None, dtype=None, out=None, keepdims=False, initial=None, where=None, promote_integers=True):
    if rp.use_jax:
        return jnp.prod(a, axis=axis, dtype=dtype, out=out, keepdims=keepdims, initial=initial, where=where,
                        promote_integers=promote_integers)
    elif rp.use_torch:
        if where is not None: raise NotImplementedError('PyTorch does not support "where" in prod')
        if initial is not None: raise NotImplementedError('PyTorch does not support "initial" in prod')
        a_t = tr.as_tensor(a)
        kwargs = {}
        if dtype is not None: kwargs['dtype'] = dtype
        if axis is not None:
            return TorchArray(tr.prod(a_t, dim=axis, keepdim=bool(keepdims), **kwargs))
        else:
            res = tr.prod(a_t, **kwargs)
            if keepdims:
                res = res.view(*([1]*a_t.ndim))
            return TorchArray(res)
    else:
        kwargs = {}
        if initial is not None: kwargs['initial'] = initial
        if where is not None: kwargs['where'] = where
        return NumpyArray(np.prod(a, axis=axis, dtype=dtype, out=out, keepdims=keepdims, **kwargs))


def sum(a, axis=None, dtype=None, out=None, keepdims=False, initial=None, where=None, promote_integers=True):
    if rp.use_jax:
        return jnp.sum(a, axis=axis, dtype=dtype, out=out, keepdims=keepdims, initial=initial, where=where,
                       promote_integers=promote_integers)
    elif rp.use_torch:
        if where is not None: raise NotImplementedError('PyTorch does not support "where" in sum')
        if initial is not None: raise NotImplementedError('PyTorch does not support "initial" in sum')
        a_t = tr.as_tensor(a)
        kwargs = {}
        if dtype is not None: kwargs['dtype'] = dtype
        if axis is not None:
            return TorchArray(tr.sum(a_t, dim=axis, keepdim=bool(keepdims), **kwargs))
        else:
            res = tr.sum(a_t, **kwargs)
            if keepdims:
                res = res.view(*([1]*a_t.ndim))
            return TorchArray(res)
    else:
        kwargs = {}
        if initial is not None: kwargs['initial'] = initial
        if where is not None: kwargs['where'] = where
        return NumpyArray(np.sum(a, axis=axis, dtype=dtype, out=out, keepdims=keepdims, **kwargs))


def nanprod(a, axis=None, dtype=None, out=None, keepdims=False, initial=None, where=None, promote_integers=True):
    if rp.use_jax:
        return jnp.nanprod(a, axis=axis, dtype=dtype, out=out, keepdims=keepdims, initial=initial, where=where,
                           promote_integers=promote_integers)
    elif rp.use_torch:
        if where is not None: raise NotImplementedError('PyTorch does not support "where" in nanprod')
        if initial is not None: raise NotImplementedError('PyTorch does not support "initial" in nanprod')
        a_t = tr.as_tensor(a)
        # manual nanprod: fill NaNs with 1.0
        a_filled = tr.where(tr.isnan(a_t), tr.tensor(1.0, dtype=a_t.dtype, device=a_t.device), a_t)
        kwargs = {}
        if dtype is not None: kwargs['dtype'] = dtype
        if axis is not None:
            return TorchArray(tr.prod(a_filled, dim=axis, keepdim=bool(keepdims), **kwargs))
        else:
            res = tr.prod(a_filled, **kwargs)
            if keepdims:
                res = res.view(*([1]*a_t.ndim))
            return TorchArray(res)
    else:
        kwargs = {}
        if initial is not None: kwargs['initial'] = initial
        if where is not None: kwargs['where'] = where
        return NumpyArray(
            np.nanprod(a, axis=axis, dtype=dtype, out=out, keepdims=keepdims, **kwargs))


def nansum(a, axis=None, dtype=None, out=None, keepdims=False, initial=None, where=None, promote_integers=True):
    if rp.use_jax:
        return jnp.nansum(a, axis=axis, dtype=dtype, out=out, keepdims=keepdims, initial=initial, where=where,
                          promote_integers=promote_integers)
    elif rp.use_torch:
        if where is not None: raise NotImplementedError('PyTorch does not support "where" in nansum')
        if initial is not None: raise NotImplementedError('PyTorch does not support "initial" in nansum')
        a_t = tr.as_tensor(a)
        kwargs = {}
        if dtype is not None: kwargs['dtype'] = dtype
        if axis is not None:
            return TorchArray(tr.nansum(a_t, dim=axis, keepdim=bool(keepdims), **kwargs))
        else:
            res = tr.nansum(a_t, **kwargs)
            if keepdims:
                res = res.view(*([1]*a_t.ndim))
            return TorchArray(res)
    else:
        kwargs = {}
        if initial is not None: kwargs['initial'] = initial
        if where is not None: kwargs['where'] = where
        return NumpyArray(
            np.nansum(a, axis=axis, dtype=dtype, out=out, keepdims=keepdims, **kwargs))


def cumprod(a, axis=None, dtype=None, out=None):
    if rp.use_jax:
        return jnp.cumprod(a, axis=axis, dtype=dtype, out=out)
    elif rp.use_torch:
        kwargs = {}
        if dtype is not None: kwargs['dtype'] = dtype
        if axis is not None:
            return TorchArray(tr.cumprod(a, dim=axis, **kwargs))
        else:
            return TorchArray(tr.cumprod(a.flatten(), dim=0, **kwargs))
    else:
        return NumpyArray(np.cumprod(a, axis=axis, dtype=dtype, out=out))


def cumsum(a, axis=None, dtype=None, out=None):
    if rp.use_jax:
        return jnp.cumsum(a, axis=axis, dtype=dtype, out=out)
    elif rp.use_torch:
        kwargs = {}
        if dtype is not None: kwargs['dtype'] = dtype
        if axis is not None:
            return TorchArray(tr.cumsum(a, dim=axis, **kwargs))
        else:
            return TorchArray(tr.cumsum(a.flatten(), dim=0, **kwargs))
    else:
        return NumpyArray(np.cumsum(a, axis=axis, dtype=dtype, out=out))


def nancumprod(a, axis=None, dtype=None, out=None):
    if rp.use_jax:
        return jnp.nancumprod(a, axis=axis, dtype=dtype, out=out)
    elif rp.use_torch:
         # Torch doesn't have nancumprod directly, might need to mask
        mask = tr.isnan(a)
        a_filled = tr.where(mask, tr.tensor(1.0, dtype=a.dtype, device=a.device), a)
        kwargs = {}
        if dtype is not None: kwargs['dtype'] = dtype
        if axis is not None:
            return TorchArray(tr.cumprod(a_filled, dim=axis, **kwargs))
        else:
            return TorchArray(tr.cumprod(a_filled.flatten(), dim=0, **kwargs))
    else:
        return NumpyArray(np.nancumprod(a, axis=axis, dtype=dtype, out=out))


def nancumsum(a, axis=None, dtype=None, out=None):
    if rp.use_jax:
        return jnp.nancumsum(a, axis=axis, dtype=dtype, out=out)
    elif rp.use_torch:
        mask = tr.isnan(a)
        a_filled = tr.where(mask, tr.tensor(0.0, dtype=a.dtype, device=a.device), a)
        kwargs = {}
        if dtype is not None: kwargs['dtype'] = dtype
        if axis is not None:
            return TorchArray(tr.cumsum(a_filled, dim=axis, **kwargs))
        else:
            return TorchArray(tr.cumsum(a_filled.flatten(), dim=0, **kwargs))
    else:
        return NumpyArray(np.nancumsum(a, axis=axis, dtype=dtype, out=out))


def diff(a, n=1, axis=-1, prepend=None, append=None):
    kwargs = {}
    if prepend is not None: kwargs['prepend'] = prepend
    if append is not None: kwargs['append'] = append
    
    if rp.use_jax:
        return jnp.diff(a, n=n, axis=axis, **kwargs)
    elif rp.use_torch:
        # PyTorch diff: torch.diff(input, n=1, dim=-1, prepend=None, append=None)
        # Using kwargs is safe.
        a_t = tr.as_tensor(a)
        if 'prepend' in kwargs: kwargs['prepend'] = tr.as_tensor(kwargs['prepend'])
        if 'append' in kwargs: kwargs['append'] = tr.as_tensor(kwargs['append'])
        return TorchArray(tr.diff(a_t, n=n, dim=axis, **kwargs))
    else:
        return NumpyArray(np.diff(a, n=n, axis=axis, **kwargs))


def ediff1d(ary, to_end=None, to_begin=None):
    if rp.use_jax:
        return jnp.ediff1d(ary, to_end=to_end, to_begin=to_begin)
    elif rp.use_torch:
        res = tr.diff(ary.flatten())
        if to_begin is not None: res = tr.cat([tr.as_tensor(to_begin).flatten(), res])
        if to_end is not None: res = tr.cat([res, tr.as_tensor(to_end).flatten()])
        return TorchArray(res)
    else:
        return NumpyArray(np.ediff1d(ary, to_end=to_end, to_begin=to_begin))


def gradient(f, *varargs, axis=None, edge_order=None):
    if rp.use_jax:
        return jnp.gradient(f, *varargs, axis=axis, edge_order=edge_order)
    elif rp.use_torch:
        if edge_order is None: edge_order = 1
        f_t = tr.as_tensor(f)
        if varargs:
             spacing = _seq_to_base(varargs)
             return tr.gradient(f_t, spacing=spacing, dim=axis, edge_order=edge_order)[0]
        return tr.gradient(f_t, dim=axis, edge_order=edge_order)[0]
    else:
        if edge_order is None: edge_order = 1
        return NumpyArray(np.gradient(f, *varargs, axis=axis, edge_order=edge_order))


def cross(a, b, axisa=-1, axisb=-1, axisc=-1, axis=None):
    if rp.use_jax:
        return jnp.cross(a, b, axisa=axisa, axisb=axisb, axisc=axisc, axis=axis)
    elif rp.use_torch:
        if axis is not None: axisa = axisb = axisc = axis
        return TorchArray(tr.cross(tr.as_tensor(a), tr.as_tensor(b), dim=axisa))
    else:
        return NumpyArray(np.cross(a, b, axisa=axisa, axisb=axisb, axisc=axisc, axis=axis))


def exp(x, /):
    if rp.use_jax:
        return jnp.exp(x)
    elif rp.use_torch:
        return TorchArray(tr.exp(tr.as_tensor(x)))
    else:
        return NumpyArray(np.exp(x))


def expm1(x, /):
    if rp.use_jax:
        return jnp.expm1(x)
    elif rp.use_torch:
        return TorchArray(tr.expm1(tr.as_tensor(x)))
    else:
        return NumpyArray(np.expm1(x))


def exp2(x, /):
    if rp.use_jax:
        return jnp.exp2(x)
    elif rp.use_torch:
        return TorchArray(tr.exp2(tr.as_tensor(x)))
    else:
        return NumpyArray(np.exp2(x))


def log(x, /):
    if rp.use_jax:
        return jnp.log(x)
    elif rp.use_torch:
        return TorchArray(tr.log(tr.as_tensor(x)))
    else:
        return NumpyArray(np.log(x))


def log10(x, /):
    if rp.use_jax:
        return jnp.log10(x)
    elif rp.use_torch:
        return TorchArray(tr.log10(tr.as_tensor(x)))
    else:
        return NumpyArray(np.log10(x))


def log2(x, /):
    if rp.use_jax:
        return jnp.log2(x)
    elif rp.use_torch:
        return TorchArray(tr.log2(tr.as_tensor(x)))
    else:
        return NumpyArray(np.log2(x))


def log1p(x, /):
    if rp.use_jax:
        return jnp.log1p(x)
    elif rp.use_torch:
        return TorchArray(tr.log1p(tr.as_tensor(x)))
    else:
        return NumpyArray(np.log1p(x))


def logaddexp(x1, x2, /):
    if rp.use_jax:
        return jnp.logaddexp(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.logaddexp(x1, x2))
    else:
        return NumpyArray(np.logaddexp(x1, x2))


def logaddexp2(x1, x2, /):
    if rp.use_jax:
        return jnp.logaddexp2(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.logaddexp2(x1, x2))
    else:
        return NumpyArray(np.logaddexp2(x1, x2))


def i0(x):
    if rp.use_jax:
        return jnp.i0(x)
    elif rp.use_torch:
        return TorchArray(tr.special.i0(x))
    else:
        return NumpyArray(np.i0(x))


def sinc(x, /):
    if rp.use_jax:
        return jnp.sinc(x)
    elif rp.use_torch:
        return TorchArray(tr.sinc(tr.as_tensor(x)))
    else:
        return NumpyArray(np.sinc(x))


def signbitc(x, /):
    if rp.use_jax:
        return jnp.signbit(x)
    elif rp.use_torch:
        return TorchArray(tr.signbit(tr.as_tensor(x)))
    else:
        return NumpyArray(np.signbit(x))


def copysign(x1, x2, /):
    if rp.use_jax:
        return jnp.copysign(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.copysign(x1, x2))
    else:
        return NumpyArray(np.copysign(x1, x2))


def frexp(x, /):
    if rp.use_jax:
        return jnp.frexp(x)
    elif rp.use_torch:
        mantissa, exponent = tr.frexp(x)
        return TorchArray(mantissa), TorchArray(exponent)
    else:
        return NumpyArray(np.frexp(x))


def ldexp(x1, x2, /):
    if rp.use_jax:
        return jnp.ldexp(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.ldexp(x1, x2))
    else:
        return NumpyArray(np.ldexp(x1, x2))


def nextafter(x1, x2, /):
    if rp.use_jax:
        return jnp.nextafter(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.nextafter(x1, x2))
    else:
        return NumpyArray(np.nextafter(x1, x2))


def spacing(x, /):
    if rp.use_jax:
        return jnp.spacing(x)
    elif rp.use_torch:
        # spacing is equivalent to eps at x
        return TorchArray(tr.nextafter(x, tr.tensor(float('inf'), dtype=x.dtype, device=x.device)) - x)
    else:
        return NumpyArray(np.spacing(x))


def lcm(x1, x2):
    if rp.use_jax:
        return jnp.lcm(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.lcm(x1, x2))
    else:
        return NumpyArray(np.lcm(x1, x2))


def gcd(x1, x2):
    if rp.use_jax:
        return jnp.gcd(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.gcd(x1, x2))
    else:
        return NumpyArray(np.gcd(x1, x2))


def add(x1, x2, /):
    if rp.use_jax:
        return jnp.add(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.add(x1, x2))
    else:
        return NumpyArray(np.add(x1, x2))

def _add_reduceat(a, indices, axis=0, dtype=None, out=None):
    if rp.use_jax:
        return jnp.add.reduceat(a, indices, axis=axis, dtype=dtype, out=out)
    elif rp.use_torch:
        a_t = tr.as_tensor(a)
        indices_t = tr.as_tensor(indices, dtype=tr.long)
        results = []
        for i in range(len(indices_t)):
            start = indices_t[i]
            if i + 1 < len(indices_t):
                end = indices_t[i+1]
                if start < end:
                    seg_slice = [slice(None)] * a_t.ndim
                    seg_slice[axis] = slice(start, end)
                    if dtype is not None:
                        reduced = tr.sum(a_t[tuple(seg_slice)], dim=axis, dtype=_to_torch_dtype(dtype))
                    else:
                        reduced = tr.sum(a_t[tuple(seg_slice)], dim=axis)
                else:
                    seg_slice = [slice(None)] * a_t.ndim
                    seg_slice[axis] = start
                    if dtype is not None:
                        reduced = a_t[tuple(seg_slice)].to(_to_torch_dtype(dtype))
                    else:
                        reduced = a_t[tuple(seg_slice)]
            else:
                seg_slice = [slice(None)] * a_t.ndim
                seg_slice[axis] = slice(start, None)
                if dtype is not None:
                    reduced = tr.sum(a_t[tuple(seg_slice)], dim=axis, dtype=_to_torch_dtype(dtype))
                else:
                    reduced = tr.sum(a_t[tuple(seg_slice)], dim=axis)
            results.append(reduced)
        return TorchArray(tr.stack(results, dim=axis))
    else:
        return NumpyArray(np.add.reduceat(a, indices, axis=axis, dtype=dtype, out=out))

add.reduceat = _add_reduceat


def reciprocal(x, /):
    if rp.use_jax:
        return jnp.reciprocal(x)
    elif rp.use_torch:
        return TorchArray(tr.reciprocal(tr.as_tensor(x)))
    else:
        return NumpyArray(np.reciprocal(x))


def positive(x, /):
    if rp.use_jax:
        return jnp.positive(x)
    elif rp.use_torch:
        return TorchArray(+x)
    else:
        return NumpyArray(np.positive(x))


def negative(x, /):
    if rp.use_jax:
        return jnp.negative(x)
    elif rp.use_torch:
        return TorchArray(-x)
    else:
        return NumpyArray(np.negative(x))


def multiply(x1, x2, /):
    if rp.use_jax:
        return jnp.multiply(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.multiply(x1, x2))
    else:
        return NumpyArray(np.multiply(x1, x2))


def divide(x1, x2, /):
    if rp.use_jax:
        return jnp.divide(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.divide(x1, x2))
    else:
        return NumpyArray(np.divide(x1, x2))


def power(x1, x2, /):
    if rp.use_jax:
        return jnp.power(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.power(x1, x2))
    else:
        return NumpyArray(np.power(x1, x2))


def pow(x1, x2, /):
    if rp.use_jax:
        return jnp.pow(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.pow(x1, x2))
    else:
        return NumpyArray(np.pow(x1, x2))


def subtract(x1, x2, /):
    if rp.use_jax:
        return jnp.subtract(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.subtract(x1, x2))
    else:
        return NumpyArray(np.subtract(x1, x2))


def true_divide(x1, x2, /):
    if rp.use_jax:
        return jnp.true_divide(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.true_divide(x1, x2))
    else:
        return NumpyArray(np.true_divide(x1, x2))


def floor_divide(x1, x2, /):
    if rp.use_jax:
        return jnp.floor_divide(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.floor_divide(x1, x2))
    else:
        return NumpyArray(np.floor_divide(x1, x2))


def float_power(x1, x2, /):
    if rp.use_jax:
        return jnp.float_power(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.float_power(x1, x2))
    else:
        return NumpyArray(np.float_power(x1, x2))


def fmod(x1, x2, /):
    if rp.use_jax:
        return jnp.fmod(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.fmod(x1, x2))
    else:
        return NumpyArray(np.fmod(x1, x2))


def mod(x1, x2, /):
    if rp.use_jax:
        return jnp.mod(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.remainder(x1, x2))
    else:
        return NumpyArray(np.mod(x1, x2))


def modf(x, /, out=None):
    if rp.use_jax:
        return jnp.modf(x, out=None)
    elif rp.use_torch:
        fractional, integral = tr.modf(x)
        return TorchArray(fractional), TorchArray(integral)
    else:
        return NumpyArray(np.modf(x, out=out))


def remainder(x1, x2, /):
    if rp.use_jax:
        return jnp.remainder(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.remainder(x1, x2))
    else:
        return NumpyArray(np.remainder(x1, x2))


def divmod(x1, x2, /):
    if rp.use_jax:
        return jnp.divmod(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.div(x1, x2, rounding_mode='floor')), TorchArray(tr.remainder(x1, x2))
    else:
        return NumpyArray(np.divmod(x1, x2))


def angle(z, deg=False):
    if rp.use_jax:
        return jnp.angle(z, deg=deg)
    elif rp.use_torch:
        res = tr.angle(z)
        if deg: res = tr.rad2deg(res)
        return TorchArray(res)
    else:
        return NumpyArray(np.angle(z, deg=deg))


def real(val, /):
    if rp.use_jax:
        return jnp.real(val)
    elif rp.use_torch:
        return TorchArray(tr.real(val))
    else:
        return NumpyArray(np.real(val))


def imag(val, /):
    if rp.use_jax:
        return jnp.imag(val)
    elif rp.use_torch:
        return TorchArray(tr.imag(val))
    else:
        return NumpyArray(np.imag(val))


def conj(x, /):
    if rp.use_jax:
        return jnp.conj(x)
    elif rp.use_torch:
        return TorchArray(tr.conj(tr.as_tensor(x)))
    else:
        return NumpyArray(np.conj(x))


def conjugate(x, /):
    if rp.use_jax:
        return jnp.conjugate(x)
    elif rp.use_torch:
        return TorchArray(tr.conj(tr.as_tensor(x)))
    else:
        return NumpyArray(np.conjugate(x))


def maximum(x, y, /):
    if rp.use_jax:
        return jnp.maximum(x, y)
    elif rp.use_torch:
        return TorchArray(tr.maximum(tr.as_tensor(x), tr.as_tensor(y)))
    else:
        return NumpyArray(np.maximum(x, y))


def max(a, axis=None, out=None, keepdims=False, initial=None, where=None):
    if rp.use_jax:
        return jnp.max(a, axis=axis, out=out, keepdims=keepdims, initial=initial, where=where)
    elif rp.use_torch:
        if where is not None: raise NotImplementedError('PyTorch does not support "where" in max')
        if initial is not None: raise NotImplementedError('PyTorch does not support "initial" in max')
        a_t = tr.as_tensor(a)
        if axis is not None:
            return TorchArray(tr.amax(a_t, dim=axis, keepdim=bool(keepdims)))
        else:
            if keepdims:
                return TorchArray(tr.amax(a_t, dim=tuple(range(a_t.ndim)), keepdim=True))
            else:
                return TorchArray(tr.amax(a_t))
    else:
        kwargs = {}
        if initial is not None: kwargs['initial'] = initial
        if where is not None: kwargs['where'] = where
        return NumpyArray(np.max(a, axis=axis, out=out, keepdims=keepdims, **kwargs))


def amax(a, axis=None, out=None, keepdims=False, initial=None, where=None):
    if rp.use_jax:
        return jnp.amax(a, axis=axis, out=out, keepdims=keepdims, initial=initial, where=where)
    elif rp.use_torch:
        if where is not None: raise NotImplementedError('PyTorch does not support "where" in amax')
        if initial is not None: raise NotImplementedError('PyTorch does not support "initial" in amax')
        a_t = tr.as_tensor(a)
        if axis is not None:
            return TorchArray(tr.amax(a_t, dim=axis, keepdim=bool(keepdims)))
        else:
            if keepdims:
                return TorchArray(tr.amax(a_t, dim=tuple(range(a_t.ndim)), keepdim=True))
            else:
                return TorchArray(tr.amax(a_t))
    else:
        kwargs = {}
        if initial is not None: kwargs['initial'] = initial
        if where is not None: kwargs['where'] = where
        return NumpyArray(np.amax(a, axis=axis, out=out, keepdims=keepdims, **kwargs))


def fmax(x1, x2):
    if rp.use_jax:
        return jnp.fmax(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.fmax(x1, x2))
    else:
        return NumpyArray(np.fmax(x1, x2))


def nanmax(a, axis=None, out=None, keepdims=False, initial=None, where=None):
    if rp.use_jax:
        return jnp.nanmax(a, axis=axis, out=out, keepdims=keepdims, initial=initial, where=where)
    elif rp.use_torch:
        if where is not None: raise NotImplementedError('PyTorch does not support "where" in nanmax')
        if initial is not None: raise NotImplementedError('PyTorch does not support "initial" in nanmax')
        a_t = tr.as_tensor(a)
        # manual nanmax: fill NaNs with -inf
        a_filled = tr.where(tr.isnan(a_t), tr.tensor(-float('inf'), dtype=a_t.dtype, device=a_t.device), a_t)
        if axis is not None:
            return TorchArray(tr.amax(a_filled, dim=axis, keepdim=bool(keepdims)))
        else:
            if keepdims:
                return TorchArray(tr.amax(a_filled, dim=tuple(range(a_t.ndim)), keepdim=True))
            else:
                return TorchArray(tr.amax(a_filled))
    else:
        kwargs = {}
        if initial is not None: kwargs['initial'] = initial
        if where is not None: kwargs['where'] = where
        return NumpyArray(np.nanmax(a, axis=axis, out=out, keepdims=keepdims, **kwargs))


def minimum(x, y, /):
    if rp.use_jax:
        return jnp.minimum(x, y)
    elif rp.use_torch:
        return TorchArray(tr.minimum(tr.as_tensor(x), tr.as_tensor(y)))
    else:
        return NumpyArray(np.minimum(x, y))


def min(a, axis=None, out=None, keepdims=False, initial=None, where=None):
    if rp.use_jax:
        return jnp.min(a, axis=axis, out=out, keepdims=keepdims, initial=initial, where=where)
    elif rp.use_torch:
        if where is not None: raise NotImplementedError('PyTorch does not support "where" in min')
        if initial is not None: raise NotImplementedError('PyTorch does not support "initial" in min')
        a_t = tr.as_tensor(a)
        if axis is not None:
            return TorchArray(tr.amin(a_t, dim=axis, keepdim=bool(keepdims)))
        else:
            if keepdims:
                return TorchArray(tr.amin(a_t, dim=tuple(range(a_t.ndim)), keepdim=True))
            else:
                return TorchArray(tr.amin(a_t))
    else:
        kwargs = {}
        if initial is not None: kwargs['initial'] = initial
        if where is not None: kwargs['where'] = where
        return NumpyArray(np.min(a, axis=axis, out=out, keepdims=keepdims, **kwargs))


def amin(a, axis=None, out=None, keepdims=False, initial=None, where=None):
    if rp.use_jax:
        return jnp.amin(a, axis=axis, out=out, keepdims=keepdims, initial=initial, where=where)
    elif rp.use_torch:
        if where is not None: raise NotImplementedError('PyTorch does not support "where" in amin')
        if initial is not None: raise NotImplementedError('PyTorch does not support "initial" in amin')
        a_t = tr.as_tensor(a)
        if axis is not None:
            return TorchArray(tr.amin(a_t, dim=axis, keepdim=bool(keepdims)))
        else:
            if keepdims:
                return TorchArray(tr.amin(a_t, dim=tuple(range(a_t.ndim)), keepdim=True))
            else:
                return TorchArray(tr.amin(a_t))
    else:
        kwargs = {}
        if initial is not None: kwargs['initial'] = initial
        if where is not None: kwargs['where'] = where
        return NumpyArray(np.amin(a, axis=axis, out=out, keepdims=keepdims, **kwargs))


def fmin(x1, x2):
    if rp.use_jax:
        return jnp.fmin(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.fmin(x1, x2))
    else:
        return NumpyArray(np.fmin(x1, x2))


def nanmin(a, axis=None, out=None, keepdims=False, initial=None, where=None):
    if rp.use_jax:
        return jnp.nanmin(a, axis=axis, out=out, keepdims=keepdims, initial=initial, where=where)
    elif rp.use_torch:
        if where is not None: raise NotImplementedError('PyTorch does not support "where" in nanmin')
        if initial is not None: raise NotImplementedError('PyTorch does not support "initial" in nanmin')
        a_t = tr.as_tensor(a)
        # manual nanmin: fill NaNs with +inf
        a_filled = tr.where(tr.isnan(a_t), tr.tensor(float('inf'), dtype=a_t.dtype, device=a_t.device), a_t)
        if axis is not None:
            return TorchArray(tr.amin(a_filled, dim=axis, keepdim=bool(keepdims)))
        else:
            if keepdims:
                return TorchArray(tr.amin(a_filled, dim=tuple(range(a_t.ndim)), keepdim=True))
            else:
                return TorchArray(tr.amin(a_filled))
    else:
        kwargs = {}
        if initial is not None: kwargs['initial'] = initial
        if where is not None: kwargs['where'] = where
        return NumpyArray(np.nanmin(a, axis=axis, out=out, keepdims=keepdims, **kwargs))


def convolve(a, v, mode='full', *, precision=None, preferred_element_type=None):
    if rp.use_jax:
        return jnp.convolve(a, v, mode=mode, precision=precision, preferred_element_type=preferred_element_type)
    elif rp.use_torch:
        # Basic 1D convolution for vectors
        a_t = tr.as_tensor(a)
        v_t = tr.as_tensor(v)
        # Flip v as per convolution definition
        v_t = tr.flip(v_t, dims=(0,))
        # Pad a_t according to mode
        if mode == 'full':
            padding = v_t.size(0) - 1
        elif mode == 'same':
            padding = v_t.size(0) // 2
        else: # valid
            padding = 0
        
        # Use conv1d with padding
        res = tr.conv1d(a_t.view(1, 1, -1), v_t.view(1, 1, -1), padding=padding)
        return TorchArray(res.view(-1))
    else:
        return NumpyArray(np.convolve(a, v, mode=mode))


def clip(arr=None, /, min=None, max=None, ):
    if rp.use_jax:
        return jnp.clip(arr, min=min, max=max)
    elif rp.use_torch:
        arr = tr.as_tensor(arr) if not isinstance(arr, tr.Tensor) else arr
        return TorchArray(tr.clamp(arr, min=min, max=max))
    else:
        return NumpyArray(np.clip(a=arr, a_min=min, a_max=max))


def sqrt(x, /):
    if rp.use_jax:
        return jnp.sqrt(x)
    elif rp.use_torch:
        return TorchArray(tr.sqrt(tr.as_tensor(x)))
    else:
        return NumpyArray(np.sqrt(x))


def cbrt(x, /):
    if rp.use_jax:
        return jnp.cbrt(x)
    elif rp.use_torch:
        return TorchArray(tr.pow(x, 1/3))
    else:
        return NumpyArray(np.cbrt(x))


def square(x, /):
    if rp.use_jax:
        return jnp.square(x)
    elif rp.use_torch:
        return TorchArray(tr.square(tr.as_tensor(x)))
    else:
        return NumpyArray(np.square(x))


def absolute(x, /):
    if rp.use_jax:
        return jnp.absolute(x)
    elif rp.use_torch:
        return TorchArray(tr.abs(tr.as_tensor(x)))
    else:
        return NumpyArray(np.absolute(x))

abs = absolute


def fab(x, /):
    if rp.use_jax:
        return jnp.fabs(x)
    elif rp.use_torch:
        return TorchArray(tr.abs(tr.as_tensor(x)))
    else:
        return NumpyArray(np.fabs(x))


def sign(x, /):
    if rp.use_jax:
        return jnp.sign(x)
    elif rp.use_torch:
        return TorchArray(tr.sign(tr.as_tensor(x)))
    else:
        return NumpyArray(np.sign(x))


def heaviside(x1, x2, /):
    if rp.use_jax:
        return jnp.heaviside(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.heaviside(x1, x2))
    else:
        return NumpyArray(np.heaviside(x1, x2))


def nan_to_num(x, copy=True, nan=0.0, posinf=None, neginf=None):
    if rp.use_jax:
        return jnp.nan_to_num(x, copy=copy, nan=nan, posinf=posinf, neginf=neginf)
    elif rp.use_torch:
        return TorchArray(tr.nan_to_num(x, nan=nan, posinf=posinf, neginf=neginf))
    else:
        return NumpyArray(np.nan_to_num(x, copy=copy, nan=nan, posinf=posinf, neginf=neginf))


def real_if_close(): raise NotImplementedError  # There is no JAX functionality


def interp(x, xp, fp, left=None, right=None, period=None):
    if rp.use_jax:
        return jnp.interp(x, xp, fp, left=left, right=right, period=period)
    elif rp.use_torch:
        if period is not None:
             raise NotImplementedError("PyTorch interp with period not implemented")
        x_t = tr.as_tensor(x)
        xp_t = tr.as_tensor(xp).contiguous()
        fp_t = tr.as_tensor(fp)
        
        # Ensure xp is sorted (NumPy requires xp to be increasing)
        idx = tr.searchsorted(xp_t, x_t)
        
        # Clamp indices for left/right handling
        idx_clamped = tr.clamp(idx, 1, len(xp_t) - 1)
        
        x_left = xp_t[idx_clamped - 1]
        x_right = xp_t[idx_clamped]
        f_left = fp_t[idx_clamped - 1]
        f_right = fp_t[idx_clamped]
        
        # Linear interpolation formula: f = f_l + (f_r - f_l) * (x - x_l) / (x_r - x_l)
        denom = x_right - x_left
        mask = denom != 0
        slope = tr.where(mask, (f_right - f_left) / denom, tr.zeros_like(f_left))
        res = f_left + slope * (x_t - x_left)
        
        # Handle left/right extrapolation
        if left is None: left_val = fp_t[0]
        else: left_val = tr.as_tensor(left)
        
        if right is None: right_val = fp_t[-1]
        else: right_val = tr.as_tensor(right)
        
        res = tr.where(idx == 0, left_val, res)
        res = tr.where(idx == len(xp_t), right_val, res)
        
        return TorchArray(res)
    else:
        return NumpyArray(np.interp(x, xp, fp, left=left, right=right, period=period))


def bitwise_count(x, /):
    if not rp.use_jax:
        return NumpyArray(np.bitwise_count(x))
    else:
        return jnp.bitwise_count(x)


def vdot(a, b, *, precision=None, preferred_element_type=None):
    if rp.use_jax:
        return jnp.vdot(a, b, precision=precision, preferred_element_type=preferred_element_type)
    elif rp.use_torch:
        return TorchArray(tr.vdot(tr.as_tensor(a), tr.as_tensor(b)))
    else:
        return NumpyArray(np.vdot(a, b))


def vecdot(a, b, *, precision=None, preferred_element_type=None):
    if rp.use_jax:
        return jnp.vecdot(a, b, precision=precision, preferred_element_type=preferred_element_type)
    elif rp.use_torch:
        return TorchArray(tr.vecdot(a, b))
    else:
        return NumpyArray(np.vecdot(a, b))


def inner(a, b, *, precision=None, preferred_element_type=None):
    if rp.use_jax:
        return jnp.inner(a, b, precision=precision, preferred_element_type=preferred_element_type)
    elif rp.use_torch:
        return TorchArray(tr.inner(tr.as_tensor(a), tr.as_tensor(b)))
    else:
        return NumpyArray(np.inner(a, b))


def outer(a, b, out=None):
    if rp.use_jax:
        return jnp.outer(a, b, out=out)
    elif rp.use_torch:
        res = tr.outer(tr.as_tensor(a), tr.as_tensor(b))
        if out is not None: out.copy_(res)
        return TorchArray(res)
    else:
        return NumpyArray(np.outer(a, b, out=out))


def matmul(a, b, *, precision=None, preferred_element_type=None):
    if rp.use_jax:
        return jnp.matmul(a, b, precision=precision, preferred_element_type=preferred_element_type)
    elif rp.use_torch:
        return TorchArray(tr.matmul(tr.as_tensor(a), tr.as_tensor(b)))
    else:
        return NumpyArray(np.matmul(a, b))


def tensordot(a, b, axes=2, *, precision=None, preferred_element_type=None):
    if rp.use_jax:
        return jnp.tensordot(a, b, axes=axes, precision=precision, preferred_element_type=preferred_element_type)
    elif rp.use_torch:
        return TorchArray(tr.tensordot(tr.as_tensor(a), tr.as_tensor(b), dims=axes))
    else:
        return NumpyArray(np.tensordot(a, b, axes=axes))


def einsum(subscript: str, /, *operands, out=None, optimize: str | bool | list[tuple[int, ...]] = 'optimal',
           precision=None, preferred_element_type=None):
    if rp.use_jax:
        return jnp.einsum(subscript, *operands, out=out, optimize=optimize, precision=precision,
                          preferred_element_type=preferred_element_type)
    elif rp.use_torch:
        ops = [tr.as_tensor(o) for o in operands]
        float_ops = [o for o in ops if tr.is_floating_point(o)]
        if float_ops:
            import builtins
            highest = builtins.max(float_ops, key=lambda o: tr.finfo(o.dtype).bits).dtype
            ops = [o.to(highest) if tr.is_floating_point(o) else o for o in ops]
        return TorchArray(tr.einsum(subscript, *ops))
    else:
        return NumpyArray(np.einsum(subscript, *operands, out=out, order='K', casting='safe', optimize=optimize))


def einsum_path(subscripts, /, *operands, optimize="greedy"):
    if not rp.use_jax:
        return NumpyArray(np.einsum_path(subscripts, *operands, optimize=optimize))
    else:
        return jnp.einsum_path(subscripts, *operands, optimize=optimize)


def kron(a, b):
    if rp.use_jax:
        return jnp.kron(a, b)
    elif rp.use_torch:
        return TorchArray(tr.kron(tr.as_tensor(a), tr.as_tensor(b)))
    else:
        return NumpyArray(np.kron(a, b))


def trace(a, offset=0, axis1=0, axis2=1, dtype=None, out=None):
    if rp.use_jax:
        return jnp.trace(a, offset=offset, axis1=axis1, axis2=axis2, dtype=dtype, out=out)
    elif rp.use_torch:
        return TorchArray(tr.trace(a)) # Note: torch.trace only for 2D, might need more for general axis1, axis2
    else:
        return NumpyArray(np.trace(a, offset=offset, axis1=axis1, axis2=axis2, dtype=dtype, out=out))


def diagonal(a, offset=0, axis1=0, axis2=1):
    if rp.use_jax:
        return jnp.diagonal(a, offset=offset, axis1=axis1, axis2=axis2)
    elif rp.use_torch:
        return TorchArray(tr.diagonal(a, offset=offset, dim1=axis1, dim2=axis2))
    else:
        return NumpyArray(np.diagonal(a, offset=offset, axis1=axis1, axis2=axis2))


def load(file, mmap_mode=None, allow_pickle=False, fix_imports=True, encoding='ASCII', *, max_header_size=10000):
    if rp.use_jax:
        return jnp.load(file, mmap_mode=mmap_mode, allow_pickle=allow_pickle, fix_imports=fix_imports,
                        encoding=encoding, max_header_size=max_header_size)
    elif rp.use_torch:
        return TorchArray(tr.load(file))
    else:
        return NumpyArray(
            np.load(file, mmap_mode=mmap_mode, allow_pickle=allow_pickle, fix_imports=fix_imports, encoding=encoding,
                    max_header_size=max_header_size))


def save(file, arr, allow_pickle=True):
    if rp.use_jax:
        return jnp.save(file, arr, allow_pickle=allow_pickle)
    elif rp.use_torch:
        return tr.save(arr, file)
    else:
        return NumpyArray(np.save(file, arr, allow_pickle=allow_pickle))


def savez(file, *args, **kwds):
    if rp.use_jax:
        return jnp.savez(file, *args, **kwds)
    elif rp.use_torch:
        return tr.save(kwds, file)
    else:
        return NumpyArray(np.savez(file, *args, **kwds))


def savez_compressed(): raise NotImplementedError


def loadtxt(): raise NotImplementedError


def savetxt(): raise NotImplementedError


def genfromtxt(): raise NotImplementedError


def fromregex(): raise NotImplementedError


def fromstring(string, dtype=None, count=-1, *, sep):
    if rp.use_jax:
        return jnp.fromstring(string, dtype=dtype, count=-1, sep=sep)
    elif rp.use_torch:
        if dtype is float: dtype = np.float32
        return TorchArray(tr.from_numpy(np.fromstring(string, dtype=_to_torch_dtype(dtype), count=count, sep=sep)))
    else:
        return NumpyArray(np.fromstring(string, dtype=dtype, count=-1, sep=sep))


def array2string(): raise NotImplementedError


def array_repr(arr, max_line_width=None, precision=None, suppress_small=None):
    if rp.use_jax:
        return jnp.array_repr(arr, max_line_width=max_line_width, precision=precision, suppress_small=suppress_small)
    elif rp.use_torch:
        return repr(arr)
    else:
        return NumpyArray(
            np.array_repr(arr, max_line_width=max_line_width, precision=precision, suppress_small=suppress_small))


def array_str(a, max_line_width=None, precision=None, suppress_small=None):
    if rp.use_jax:
        return jnp.array_str(a, max_line_width=max_line_width, precision=precision, suppress_small=suppress_small)
    elif rp.use_torch:
        return str(a)
    else:
        return NumpyArray(
            np.array_str(a, max_line_width=max_line_width, precision=precision, suppress_small=suppress_small))


def format_float_positional(): raise NotImplementedError


def format_float_scientific(): raise NotImplementedError


def memmap(): raise NotImplementedError


def set_printoptions(precision=None, threshold=None, edgeitems=None, linewidth=None, suppress=None, nanstr=None,
                     infstr=None, \
                     formatter=None, sign=None, floatmode=None, *, legacy=None, override_repr=None):
    if rp.use_jax:
        return jnp.set_printoptions(precision=precision, threshold=threshold, edgeitems=edgeitems, linewidth=linewidth, \
                                    suppress=suppress, nanstr=nanstr, infstr=infstr, formatter=formatter, sign=sign,
                                    floatmode=floatmode, legacy=legacy, override_repr=override_repr)
    elif rp.use_torch:
        return tr.set_printoptions(precision=precision, threshold=threshold, edgeitems=edgeitems, linewidth=linewidth, sci_mode=suppress)
    else:
        return NumpyArray(
            np.set_printoptions(precision=precision, threshold=threshold, edgeitems=edgeitems, linewidth=linewidth, \
                                suppress=suppress, nanstr=nanstr, infstr=infstr, formatter=formatter, sign=sign,
                                floatmode=floatmode, legacy=legacy))


def get_printoptions():
    if rp.use_jax:
        return jnp.get_printoptions()
    elif rp.use_torch:
        return tr.get_printoptions()
    else:
        return NumpyArray(np.get_printoptions())


def printoptions(*args, **kwargs):
    if rp.use_jax:
        jnp.printoptions(*args, **kwargs)
    elif rp.use_torch:
        pass # Torch doesn't have a printoptions context manager in the same way
    else:
        np.printoptions(*args, **kwargs)


def binary_repr(): raise NotImplementedError


def base_repr(): raise NotImplementedError


def apply_along_axis(func1d, axis, arr, *args, **kwargs):
    if rp.use_jax:
        return jnp.apply_along_axis(func1d, axis=axis, arr=arr, *args, **kwargs)
    elif rp.use_torch:
        # manual loop for apply_along_axis in torch
        return TorchArray(tr.stack([func1d(x, *args, **kwargs) for x in tr.unbind(arr, dim=axis)], dim=axis))
    else:
        return NumpyArray(np.apply_along_axis(func1d, axis=axis, arr=arr, *args, **kwargs))


def apply_over_axes(func, a, axes):
    if rp.use_jax:
        return jnp.apply_over_axes(func, a, axes)
    elif rp.use_torch:
        res = a
        for axis in axes:
            res = func(res, axis)
        return TorchArray(res)
    else:
        return NumpyArray(np.apply_over_axes(func, a, axes))


def vectorize(pyfunc, *, excluded=frozenset({}), signature=None):
    if rp.use_jax:
        return jnp.vectorize(pyfunc=pyfunc, excluded=excluded, signature=signature)
    elif rp.use_torch:
        return TorchArray(tr.vmap(pyfunc)) # Note: very basic vmap use
    else:
        return NumpyArray(
            np.vectorize(pyfunc=pyfunc, otypes=None, doc=None, excluded=excluded, cache=False, signature=signature))


def frompyfunc(func, /, nin, nout, *, identity=None):
    if rp.use_jax:
        return jnp.frompyfunc(func, nin, nout, identity=identity)
    elif rp.use_torch:
        raise NotImplementedError('PyTorch does not support frompyfunc')
    else:
        return NumpyArray(np.frompyfunc(func, nin, nout, identity=identity))


def piecewise(x, condlist, funclist, *args, **kw):
    if rp.use_jax:
        return jnp.piecewise(x, condlist, funclist, *args, **kw)
    elif rp.use_torch:
        x_t = tr.as_tensor(x)
        res = tr.zeros_like(x_t)
        for cond, func in zip(condlist, funclist):
            cond_t = tr.as_tensor(cond)
            if callable(func):
                res = tr.where(cond_t, tr.as_tensor(func(x_t, *args, **kw)), res)
            else:
                res = tr.where(cond_t, tr.tensor(func, dtype=x_t.dtype, device=x_t.device), res)
        return TorchArray(res)
    else:
        return NumpyArray(np.piecewise(x, condlist, funclist, *args, **kw))


def empty(shape, dtype=None, *, device=None):
    if rp.use_jax:
        return jnp.empty(shape, dtype=dtype, device=device)
    elif rp.use_torch:
        if dtype is float: dtype = tr.float32
        return TorchArray(tr.empty(shape, dtype=_to_torch_dtype(dtype), device=device))
    else:
        return NumpyArray(np.empty(shape, dtype=dtype, device=device))


def empty_like(prototype, dtype=None, shape=None, *, device=None):
    if rp.use_jax:
        return jnp.empty_like(prototype, dtype=dtype, shape=shape, device=device)
    elif rp.use_torch:
        p_t = tr.as_tensor(prototype)
        return TorchArray(tr.empty_like(p_t, dtype=_to_torch_dtype(dtype), device=device))
    else:
        return NumpyArray(np.empty_like(prototype, dtype=dtype, shape=shape, device=device))


def eye(N, M=None, k=0, dtype=None, *, device=None):
    if rp.use_jax:
        return jnp.eye(N, M=M, k=k, dtype=dtype, device=device)
    elif rp.use_torch:
        kwargs = {}
        if dtype is not None: kwargs['dtype'] = _to_torch_dtype(dtype)
        if device is not None: kwargs['device'] = device
        if M is not None:
            return TorchArray(tr.eye(N, m=M, **kwargs))
        else:
            return TorchArray(tr.eye(N, **kwargs))
    else:
        return NumpyArray(np.eye(N, M=M, k=k, dtype=dtype, device=device))


def identity(n, dtype=None):
    if rp.use_jax:
        return jnp.identity(n, dtype=dtype)
    elif rp.use_torch:
        return TorchArray(tr.eye(n, dtype=_to_torch_dtype(dtype)))
    else:
        return NumpyArray(np.identity(n, dtype=dtype))


def ones(shape, dtype=None, *, device=None):
    if rp.use_jax:
        return jnp.ones(shape, dtype=dtype, device=device)
    elif rp.use_torch:
        return TorchArray(tr.ones(shape, dtype=_to_torch_dtype(dtype), device=device))
    else:
        return NumpyArray(np.ones(shape, dtype=dtype, device=device))


def ones_like(a, dtype=None, shape=None, *, device=None):
    if rp.use_jax:
        return jnp.ones_like(a, dtype=dtype, shape=shape, device=device)
    elif rp.use_torch:
        a_t = tr.as_tensor(a)
        return TorchArray(tr.ones_like(a_t, dtype=_to_torch_dtype(dtype), device=device))
    else:
        return NumpyArray(np.ones_like(a, dtype=dtype, shape=shape, device=device))


def zeros(shape, dtype=None, *, device=None):
    if rp.use_jax:
        return jnp.zeros(shape, dtype=dtype, device=device)
    elif rp.use_torch:
        return TorchArray(tr.zeros(shape, dtype=_to_torch_dtype(dtype), device=device))
    else:
        return NumpyArray(np.zeros(shape, dtype=dtype))


def zeros_like(a, dtype=None, shape=None, *, device=None):
    if rp.use_jax:
        return jnp.zeros_like(a, dtype=dtype, shape=shape, device=device)
    elif rp.use_torch:
        a_t = tr.as_tensor(a)
        return TorchArray(tr.zeros_like(a_t, dtype=_to_torch_dtype(dtype), device=device))
    else:
        return NumpyArray(np.zeros_like(a, dtype=dtype, shape=shape))


def full(shape, fill_value, dtype=None, *, device=None):
    if rp.use_jax:
        return jnp.full(shape, fill_value, dtype=dtype, device=device)
    elif rp.use_torch:
        if isinstance(shape, int):
            shape = (shape,)
        return TorchArray(tr.full(shape, fill_value, dtype=_to_torch_dtype(dtype), device=device))
    else:
        return NumpyArray(np.full(shape, fill_value, dtype=dtype, device=device))


def full_like(a, fill_value, dtype=None, shape=None, *, device=None):
    if rp.use_jax:
        return jnp.full_like(a, fill_value, dtype=dtype, shape=shape, device=device)
    elif rp.use_torch:
        a_t = tr.as_tensor(a)
        return TorchArray(tr.full_like(a_t, fill_value, dtype=_to_torch_dtype(dtype), device=device))
    else:
        return NumpyArray(np.full_like(a, fill_value, dtype=dtype, shape=shape, device=device))


def array(object, dtype=None, copy=True, order='K', ndmin=0, *, device=None):
    if rp.use_jax:
        return cast(JaxArray, jnp.array(object, dtype=dtype, copy=copy, order=order, ndmin=ndmin, device=device))
    elif rp.use_torch:
        res = None
        # If it's a sequence, we might have TorchArray subclasses inside, 
        # which confuses torch.as_tensor's shape inference.
        if isinstance(object, (list, tuple)):
             # check if we should use stack (to preserve gradients)
             contains_tensors = any(isinstance(i, tr.Tensor) for i in object)
             if contains_tensors:
                 # Ensure all items are tensors
                 t_list = [tr.as_tensor(i, dtype=_to_torch_dtype(dtype), device=device) for i in object]
                 res = tr.stack(t_list)
             else:
                 object = _seq_to_base(object)

        if res is None:
            try:
                res = tr.as_tensor(object, dtype=_to_torch_dtype(dtype), device=device)
            except (TypeError, RuntimeError, ValueError):
                # Fallback for object arrays or complex types: convert to numpy first
                # Torch doesn't support object arrays.
                np_arr = np.array(object, dtype=_to_numpy_dtype(dtype))
                # If still object type, we can't do much for torch, so return NumpyArray
                if np_arr.dtype == np.dtype('O'):
                     return NumpyArray(np_arr)
                else:
                     res = tr.from_numpy(np_arr).to(device)
                 
        while res.ndim < ndmin:
            res = res.unsqueeze(0)
        return TorchArray(res)
    else:
        return NumpyArray(np.array(object, dtype=dtype, copy=copy, order=order, ndmin=ndmin))


def asarray(a, dtype=None, order=None, *, copy=None, device=None):
    if rp.use_jax:
        return jnp.asarray(a, dtype=dtype, order=order, copy=copy, device=device)
    elif rp.use_torch:
        res = None
        if type(a) in (list, tuple):
             # check if we should use stack (to preserve gradients)
             contains_tensors = any(isinstance(i, tr.Tensor) for i in a)
             if contains_tensors:
                 # Ensure all items are tensors
                 t_list = [tr.as_tensor(i, dtype=_to_torch_dtype(dtype), device=device) for i in a]
                 res = TorchArray(tr.stack(t_list))
             else:
                 a = _seq_to_base(a)
        
        if res is None:
            return TorchArray(tr.as_tensor(a, dtype=_to_torch_dtype(dtype), device=device))
        return res
    else:
        return NumpyArray(np.asanyarray(a, dtype=dtype, order=order, device=device, copy=copy))
# Note: original code used asanyarray in asarray?


def astype(x, dtype, /, *, copy=False, device=None):
    if rp.use_jax:
        return jnp.astype(x, dtype, copy=copy, device=device)
    elif rp.use_torch:
        return TorchArray(x.to(dtype=dtype, device=device, copy=copy))
    else:
        return NumpyArray(np.astype(x, dtype, copy=copy, device=device))


def copy(a, order='K'):
    if rp.use_jax:
        return jnp.copy(a, order=order)
    elif rp.use_torch:
        return TorchArray(a.clone())
    else:
        return NumpyArray(np.copy(a, order=order))


def frombuffer(buffer, dtype=None, count=-1, offset=0):
    if rp.use_jax:
        return jnp.frombuffer(buffer, dtype=dtype, count=count, offset=offset)
    elif rp.use_torch:
        if dtype is float: dtype = np.float32
        return TorchArray(tr.from_numpy(np.frombuffer(buffer, dtype=_to_torch_dtype(dtype), count=count, offset=offset)))
    else:
        return NumpyArray(np.frombuffer(buffer, dtype=dtype, count=count, offset=offset))


def from_dlpack(x, /, *, device=None, copy=None):
    if rp.use_jax:
        return jnp.from_dlpack(x, device=device, copy=copy)
    elif rp.use_torch:
        return TorchArray(tr.from_dlpack(tr.as_tensor(x)))
    else:
        return NumpyArray(np.from_dlpack(x, device=device, copy=copy))


def fromfile(): raise NotImplementedError


def fromfunction(function, shape, *, dtype=None, **kwargs):
    if rp.use_jax:
        return jnp.fromfunction(function, shape, dtype=dtype, **kwargs)
    elif rp.use_torch:
        if dtype is float: dtype = tr.float32
        # manual implementation for fromfunction in torch
        grid = tr.meshgrid(*[tr.arange(s) for s in shape], indexing='ij')
        return TorchArray(function(*grid, **kwargs).to(dtype=dtype))
    else:
        return NumpyArray(np.fromfunction(function, shape, dtype=dtype, like=None, **kwargs))


def fromiter(): raise NotImplementedError


def arange(start, stop=None, step=None, dtype=None, *, device=None):
    if rp.use_jax:
        return jnp.arange(start, stop=stop, step=step, dtype=dtype, device=device)
    elif rp.use_torch:
        # Pytorch is very strict about keyword arguments in arange
        kwargs = {}
        if dtype is not None:  kwargs['dtype'] = _to_torch_dtype(dtype)
        if device is not None: kwargs['device'] = device
        
        if stop is not None:
            if step is not None:
                return TorchArray(tr.arange(start, stop, step, **kwargs))
            else:
                return TorchArray(tr.arange(start, stop, **kwargs))
        else:
            # In this case start is actually the 'stop' value
            if step is not None:
                return TorchArray(tr.arange(0, start, step, **kwargs))
            else:
                return TorchArray(tr.arange(start, **kwargs))
    else:
        if stop is not None:
            return NumpyArray(np.arange(start, stop=stop, step=step, dtype=dtype, device=device))
        else:
            return NumpyArray(np.arange(stop=start, step=step, dtype=dtype, device=device))


def linspace(start, stop, num=50, endpoint=True, retstep=False, dtype=None, axis=0, *, device=None):
    if rp.use_jax:
        return jnp.linspace(start, stop, num=num, endpoint=endpoint, retstep=retstep, dtype=dtype, axis=axis,
                            device=device)
    elif rp.use_torch:
        if retstep: raise NotImplementedError('PyTorch linspace does not support retstep')
        return TorchArray(tr.linspace(start, stop, steps=num, dtype=_to_torch_dtype(dtype), device=device))
    else:
        return NumpyArray(np.linspace(start, stop, num=num, endpoint=endpoint, retstep=retstep, dtype=dtype, axis=axis,
                                      device=device))


def logspace(start, stop, num=50, endpoint=True, base=10.0, dtype=None, axis=0):
    if rp.use_jax:
        return jnp.logspace(start, stop, num=num, endpoint=endpoint, base=base, dtype=dtype, axis=axis)
    elif rp.use_torch:
        return TorchArray(tr.logspace(start, stop, steps=num, base=base, dtype=_to_torch_dtype(dtype)))
    else:
        return NumpyArray(np.logspace(start, stop, num=num, endpoint=endpoint, base=base, dtype=dtype, axis=axis))


def geomspace(start, stop, num=50, endpoint=True, dtype=None, axis=0):
    if rp.use_jax:
        return jnp.geomspace(start, stop, num=num, endpoint=endpoint, dtype=dtype, axis=axis)
    elif rp.use_torch:
        start_t = tr.as_tensor(start)
        stop_t = tr.as_tensor(stop)
        return TorchArray(tr.logspace(tr.log10(start_t), tr.log10(stop_t), steps=num, base=10.0, dtype=_to_torch_dtype(dtype)))
    else:
        return NumpyArray(np.geomspace(start, stop, num=num, endpoint=endpoint, dtype=dtype, axis=axis))


def meshgrid(*xi, copy=True, sparse=False, indexing='xy'):
    if rp.use_jax:
        return jnp.meshgrid(*xi, copy=copy, sparse=sparse, indexing=indexing)
    elif rp.use_torch:
        res = tr.meshgrid(*xi, indexing='ij' if indexing == 'ij' else 'xy')
        return [TorchArray(r) for r in res]
    else:
        return NumpyArray(np.meshgrid(*xi, copy=copy, sparse=sparse, indexing=indexing))


def mgrid(): raise NotImplementedError


def ogrid(): raise NotImplementedError


def diag(v, k=0):
    if rp.use_jax:
        return jnp.diag(v, k=k)
    elif rp.use_torch:
        return TorchArray(tr.diag(v, diagonal=k))
    else:
        return NumpyArray(np.diag(v, k=k))


def diagflat(v, k=0):
    if rp.use_jax:
        return jnp.diagflat(v, k=k)
    elif rp.use_torch:
        return TorchArray(tr.diagflat(v, offset=k))
    else:
        return NumpyArray(np.diagflat(v, k=k))


def tri(N, M=None, k=0, dtype=None):
    if rp.use_jax:
        return jnp.tri(N, M=M, k=k, dtype=dtype)
    elif rp.use_torch:
        if dtype is float: dtype = tr.float32
        if M is None: M = N
        return TorchArray(tr.tril(tr.ones(N, M, dtype=_to_torch_dtype(dtype)), diagonal=k))
    else:
        return NumpyArray(np.tri(N, M=M, k=k, dtype=dtype))


def tril(m, k=0):
    if rp.use_jax:
        return jnp.tril(m, k=k)
    elif rp.use_torch:
        return TorchArray(tr.tril(m, diagonal=k))
    else:
        return NumpyArray(np.tril(m, k=k))


def triu(m, k=0):
    if rp.use_jax:
        return jnp.triu(m, k=k)
    elif rp.use_torch:
        return TorchArray(tr.triu(m, diagonal=k))
    else:
        return NumpyArray(np.triu(m, k=k))


def vander(x, N=None, increasing=False):
    if rp.use_jax:
        return jnp.vander(x, N=N, increasing=increasing)
    elif rp.use_torch:
        return TorchArray(tr.vander(x, N=N, increasing=increasing))
    else:
        return NumpyArray(np.vander(x, N=N, increasing=increasing))


def polyval(p, x, *, unroll=16):
    if rp.use_jax:
        return jnp.polyval(p, x, unroll=unroll)
    elif rp.use_torch:
        # manual implementation for polyval in torch
        res = tr.zeros_like(x)
        for i, coeff in enumerate(p):
            res = res * x + coeff
        return TorchArray(res)
    else:
        return NumpyArray(np.polyval(p, x))
# Note: original code used NumpyArray(np.polyval(p, x)) which is fine.

def polyfit(x, y, deg, rcond=None, full=False, w=None, cov=False):
    if rp.use_jax:
        # JAX polyfit might return a tuple depending on full/cov
        res = jnp.polyfit(x, y, deg, rcond=rcond, full=full, w=w, cov=cov)
        return res
    elif rp.use_torch:
        x_t = tr.as_tensor(x)
        y_t = tr.as_tensor(y)
        
        if x_t.dtype not in (tr.float32, tr.float64, tr.complex64, tr.complex128):
            x_t = x_t.to(tr.float32)
        if y_t.dtype not in (tr.float32, tr.float64, tr.complex64, tr.complex128):
            y_t = y_t.to(tr.float32)
            
        A = tr.vander(x_t, N=deg + 1, increasing=False)
        
        if w is not None:
            w_t = tr.as_tensor(w, dtype=A.dtype, device=A.device)
            A = A * w_t.unsqueeze(-1)
            if y_t.ndim == 2:
                y_t = y_t * w_t.unsqueeze(-1)
            else:
                y_t = y_t * w_t
                
        # torch lstsq
        res = tr.linalg.lstsq(A, y_t, rcond=rcond)
        
        p = res.solution
        
        if full:
            return TorchArray(p), TorchArray(res.residuals), TorchArray(res.rank), TorchArray(res.singular_values), rcond
        elif cov:
            raise NotImplementedError('Covariance matrix calculation is not supported for PyTorch polyfit yet.')
        else:
            return TorchArray(p)
    else:
        # numpy returns tuple or array based on full/cov
        res = np.polyfit(x, y, deg, rcond=rcond, full=full, w=w, cov=cov)
        if isinstance(res, tuple):
            return tuple(NumpyArray(r) if isinstance(r, np.ndarray) else r for r in res)
        else:
            return NumpyArray(res)


def bmat(): raise NotImplementedError


def all(a, axis=None, out=None, keepdims=False, *, where=None):
    if rp.use_jax:
        return jnp.all(a, axis=axis, out=out, keepdims=keepdims, where=where)
    elif rp.use_torch:
        if where is not None: raise NotImplementedError('PyTorch does not support "where" in all')
        
        # Handle generator/iterables
        if not hasattr(a, '__torch_function__') and not isinstance(a, (np.ndarray, tr.Tensor)):
             try:
                  a_t = tr.as_tensor(a)
             except (TypeError, RuntimeError):
                  # Fallback for generators/iterables
                  a = list(a)
                  a_t = tr.as_tensor(a)
        else:
             a_t = tr.as_tensor(a)
             
        if axis is None:
            if keepdims:
                # To keep dims on a global reduction in torch, we specify all dims
                res = tr.all(a_t, dim=builtins.tuple(range(a_t.ndim)), keepdim=True)
            else:
                res = tr.all(a_t)
        else:
            res = tr.all(a_t, dim=axis, keepdim=bool(keepdims))
        
        if out is not None:
             out.copy_(res)
        return TorchArray(res)
    else:
        kwargs = {}
        if where is not None: kwargs['where'] = where
        return NumpyArray(np.all(a, axis=axis, out=out, keepdims=keepdims, **kwargs))
# Note: Fixed original jnp.all argument ordering if needed (already looks okay).


def any(a, axis=None, out=None, keepdims=False, *, where=None):
    if rp.use_jax:
        return jnp.any(a, axis=axis, out=out, keepdims=keepdims, where=where)
    elif rp.use_torch:
        if where is not None: raise NotImplementedError('PyTorch does not support "where" in any')

        # Handle generator/iterables
        if not hasattr(a, '__torch_function__') and not isinstance(a, (np.ndarray, tr.Tensor)):
             try:
                  a_t = tr.as_tensor(a)
             except (TypeError, RuntimeError):
                  # Fallback for generators/iterables
                  a = list(a)
                  a_t = tr.as_tensor(a)
        else:
             a_t = tr.as_tensor(a)

        if axis is None:
            if keepdims:
                res = tr.any(a_t, dim=builtins.tuple(range(a_t.ndim)), keepdim=True)
            else:
                res = tr.any(a_t)
        else:
            res = tr.any(a_t, dim=axis, keepdim=bool(keepdims))
            
        if out is not None:
             out.copy_(res)
        return TorchArray(res)
    else:
        kwargs = {}
        if where is not None: kwargs['where'] = where
        return NumpyArray(np.any(a, axis=axis, out=out, keepdims=keepdims, **kwargs))


def isfinite(x, /):
    if rp.use_jax:
        return jnp.isfinite(x)
    elif rp.use_torch:
        return TorchArray(tr.isfinite(tr.as_tensor(x)))
    else:
        return NumpyArray(np.isfinite(x))


def isinf(x, /):
    if rp.use_jax:
        return jnp.isinf(x)
    elif rp.use_torch:
        return TorchArray(tr.isinf(tr.as_tensor(x)))
    else:
        return NumpyArray(np.isinf(x))


def isnan(x, /):
    if rp.use_jax:
        return jnp.isnan(x)
    elif rp.use_torch:
        return TorchArray(tr.isnan(tr.as_tensor(x)))
    else:
        return NumpyArray(np.isnan(x))


def isnat(): raise NotImplementedError


def isneginf(): raise NotImplementedError


def isposinf(): raise NotImplementedError


def iscomplex(): raise NotImplementedError


def iscomplexobj(): raise NotImplementedError


def isfortran(): raise NotImplementedError


def isreal(): raise NotImplementedError


def isrealobj(): raise NotImplementedError


def isscalar(): raise NotImplementedError


def logical_and(x1, x2, /):
    if rp.use_jax:
        return jnp.logical_and(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.logical_and(tr.as_tensor(x1), tr.as_tensor(x2)))
    else:
        return NumpyArray(np.logical_and(x1, x2))


def logical_or(x1, x2, /):
    if rp.use_jax:
        return jnp.logical_or(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.logical_or(tr.as_tensor(x1), tr.as_tensor(x2)))
    else:
        return NumpyArray(np.logical_or(x1, x2))


def logical_not(x, /):
    if rp.use_jax:
        return jnp.logical_not(x)
    elif rp.use_torch:
        return TorchArray(tr.logical_not(tr.as_tensor(x)))
    else:
        return NumpyArray(np.logical_not(x))


def logical_xor(x1, x2, /):
    if rp.use_jax:
        return jnp.logical_xor(x1, x2)
    elif rp.use_torch:
        return TorchArray(tr.logical_xor(tr.as_tensor(x1), tr.as_tensor(x2)))
    else:
        return NumpyArray(np.logical_xor(x1, x2))


def allclose(a, b, rtol=1e-05, atol=1e-08, equal_nan=False):
    if rp.use_jax:
        return jnp.allclose(a, b, rtol=rtol, atol=atol, equal_nan=equal_nan)
    elif rp.use_torch:
        a_t = tr.as_tensor(a)
        b_t = tr.as_tensor(b)
        return tr.allclose(a_t, b_t, rtol=rtol, atol=atol, equal_nan=equal_nan)
    else:
        return np.allclose(a, b, rtol=rtol, atol=atol, equal_nan=equal_nan)


def isclose(a, b, rtol=1e-05, atol=1e-08, equal_nan=False):
    if rp.use_jax:
        return jnp.isclose(a, b, rtol=rtol, atol=atol, equal_nan=equal_nan)
    elif rp.use_torch:
        a_t = tr.as_tensor(a)
        b_t = tr.as_tensor(b)
        return TorchArray(tr.isclose(a_t, b_t, rtol=rtol, atol=atol, equal_nan=equal_nan))
    else:
        return NumpyArray(np.isclose(a, b, rtol=rtol, atol=atol, equal_nan=equal_nan))


def array_equal(): raise NotImplementedError


def array_equiv(): raise NotImplementedError


def greater(): raise NotImplementedError


def greater_equal(): raise NotImplementedError


def less(): raise NotImplementedError


def less_equal(): raise NotImplementedError


def equal(): raise NotImplementedError


def not_equal(): raise NotImplementedError


def bitwise_and(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.bitwise_and(*args,**kwargs))
# 	else: return jnp.bitwise_and(*args,**kwargs)

def bitwise_or(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.bitwise_or(*args,**kwargs))
# 	else: return jnp.bitwise_or(*args,**kwargs)

def bitwise_xor(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.bitwise_xor(*args,**kwargs))
# 	else: return jnp.bitwise_xor(*args,**kwargs)

def invert(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.invert(*args,**kwargs))
# 	else: return jnp.invert(*args,**kwargs)

def bitwise_invert(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.bitwise_invert(*args,**kwargs))
# 	else: return jnp.bitwise_invert(*args,**kwargs)

def left_shift(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.left_shift(*args,**kwargs))
# 	else: return jnp.left_shift(*args,**kwargs)

def bitwise_left_shift(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.bitwise_left_shift(*args,**kwargs))
# 	else: return jnp.bitwise_left_shift(*args,**kwargs)

def right_shift(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.right_shift(*args,**kwargs))
# 	else: return jnp.right_shift(*args,**kwargs)

def bitwise_right_shift(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.bitwise_right_shift(*args,**kwargs))
# 	else: return jnp.bitwise_right_shift(*args,**kwargs)

def packbits(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.packbits(*args,**kwargs))
# 	else: return jnp.packbits(*args,**kwargs)

def unpackbits(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.unpackbits(*args,**kwargs))
# 	else: return jnp.unpackbits(*args,**kwargs)

def binary_repr(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.binary_repr(*args,**kwargs))
# 	else: return jnp.binary_repr(*args,**kwargs)

def c_(*args,
       **kwargs): raise NotImplementedError  # TODO: "c_", "index_exp", "mgrid", "ogrid", "r_", "s_" all require classes


def r_(*args, **kwargs): raise NotImplementedError


def s_(*args, **kwargs): raise NotImplementedError


def nonzero(a, *, size=None, fill_value=None):
    if rp.use_jax:
        return jnp.nonzero(a, size=size, fill_value=fill_value)
    elif rp.use_torch:
        res = tr.nonzero(a, as_tuple=True)
        return [TorchArray(r) for r in res]
    else:
        warnings.warn("NP and JAX NP nonzero have different behavior, check JAX documentation")
        return NumpyArray(np.nonzero(a))


def where(condition, x=None, y=None, /, *, size=None, fill_value=None):
    if rp.use_jax:
        if x is None:
            return jnp.where(condition, size=size, fill_value=fill_value)
        return jnp.where(condition, x, y, size=size, fill_value=fill_value)
    elif rp.use_torch:
        condition_t = condition if isinstance(condition, tr.Tensor) else tr.as_tensor(condition)
        if x is None:
            return [TorchArray(r) for r in tr.nonzero(condition_t, as_tuple=True)]
        x_t = x if isinstance(x, (tr.Tensor, int, float, bool)) else tr.as_tensor(x)
        y_t = y if isinstance(y, (tr.Tensor, int, float, bool)) else tr.as_tensor(y)
        return TorchArray(tr.where(condition_t, x_t, y_t))
    else:
        if x is None:
            warnings.warn("NP and JAX NP where have different behavior with a single input, check JAX documentation")
            return tuple(NumpyArray(r) for r in np.where(condition))
        return NumpyArray(np.where(condition, x, y))


def indices(dimensions, dtype=None, sparse=False):
    if rp.use_jax:
        return jnp.indices(dimensions, dtype=dtype, sparse=sparse)
    elif rp.use_torch:
        res = tr.meshgrid(*[tr.arange(d) for d in dimensions], indexing='ij')
        if sparse:
            return [TorchArray(r) for r in res]
        return TorchArray(tr.stack(res))
    else:
        return NumpyArray(np.indices(dimensions, dtype=dtype, sparse=sparse))


def ix_(*args):
    if rp.use_jax:
        return jnp.ix_(*args)
    elif rp.use_torch:
        return [TorchArray(x) for x in tr.meshgrid(*args, indexing='ij')]
    else:
        return NumpyArray(np.ix_(*args))


def ravel_multi_index(multi_index, dims, mode='raise', order='C'):
    if rp.use_jax:
        return jnp.ravel_multi_index(multi_index, dims, mode=mode, order=order)
    elif rp.use_torch:
        # manual calculation for ravel_multi_index
        res = tr.zeros_like(multi_index[0])
        # simplified, only C order
        current_stride = 1
        for i in range(len(dims)-1, -1, -1):
            res += multi_index[i] * current_stride
            current_stride *= dims[i]
        return TorchArray(res)
    else:
        return NumpyArray(np.ravel_multi_index(multi_index, dims, mode=mode, order=order))


def unravel_index(indices, shape):
    if rp.use_jax:
        return jnp.unravel_index(indices, shape)
    elif rp.use_torch:
        # manual implementation for unravel_index
        res = []
        indices_t = tr.as_tensor(indices)
        for dim in reversed(shape):
            res.append(indices_t % dim)
            indices_t = indices_t // dim
        return [TorchArray(r) for r in reversed(res)]
    else:
        return NumpyArray(np.unravel_index(indices, shape))


def diag_indices(n, ndim=2):
    if rp.use_jax:
        return jnp.diag_indices(n, ndim=ndim)
    elif rp.use_torch:
        idx = tr.arange(n)
        return tuple(TorchArray(idx) for _ in range(ndim))
    else:
        return NumpyArray(np.diag_indices(n, ndim=ndim))


def diag_indices_from(arr):
    if rp.use_jax:
        return jnp.diag_indices_from(arr)
    elif rp.use_torch:
        n = min(arr.shape)
        return diag_indices(n, ndim=arr.ndim)
    else:
        return NumpyArray(np.diag_indices_from(arr))


def mask_indices(n, mask_func, k=0, *, size=None):
    if rp.use_jax:
        return jnp.mask_indices(n, mask_func, k=k, size=size)
    elif rp.use_torch:
        m = tr.zeros((n, n), dtype=tr.bool)
        m = mask_func(m, k)
        return [TorchArray(r) for r in tr.nonzero(m, as_tuple=True)]
    else:
        if size is not None:
            warnings.warn(
                "NP and JAX NP mask_indices have different behavior when size is specified, check JAX documentation")
        return NumpyArray(np.mask_indices(n, mask_func, k=k))


def tril_indices(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.tril_indices(*args,**kwargs))
# 	else: return jnp.tril_indices(*args,**kwargs)

def tril_indices_from(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.tril_indices_from(*args,**kwargs))
# 	else: return jnp.tril_indices_from(*args,**kwargs)

def triu_indices(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.triu_indices(*args,**kwargs))
# 	else: return jnp.triu_indices(*args,**kwargs)

def triu_indices_from(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.triu_indices_from(*args,**kwargs))
# 	else: return jnp.triu_indices_from(*args,**kwargs)

def take(a, indices, axis=None, out=None, mode=None, unique_indices=False, indices_are_sorted=False,
         fill_value=None):  raise NotImplementedError
# Note: this will take a little time to figure out how we want to handle the default mode
# if not rp.use_jax: return NumpyArray(np.take(a, indices, axis=axis, out=out, mode=mode))
# else: return jnp.take(a,indices, axis=axis, out=out, mode=mode, unique_indices=unique_indices, indices_are_sorted=indices_are_sorted, fill_value=fill_value)

def take_along_axis(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.take_along_axis(*args,**kwargs))
# 	else: return jnp.take_along_axis(*args,**kwargs)


def choose(a, choices, out=None, mode='raise'):
    if rp.use_jax:
        return jnp.choose(a, choices, out=out, mode=mode)
    elif rp.use_torch:
        # manual implementation for choose
        res = tr.zeros_like(choices[0])
        for i, choice in enumerate(choices):
            res = tr.where(a == i, choice, res)
        return TorchArray(res)
    else:
        return NumpyArray(np.choose(a, choices, out=out, mode=mode))


def compress(*args, **kwargs):  raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.compress(*args,**kwargs))
# 	else: return jnp.compress(*args,**kwargs)


def select(condlist, choicelist, default=0):
    if rp.use_jax:
        return jnp.select(condlist, choicelist, default=default)
    elif rp.use_torch:
        choice0 = tr.as_tensor(choicelist[0])
        res = tr.full_like(choice0, default)
        for cond, choice in zip(reversed(condlist), reversed(choicelist)):
            res = tr.where(tr.as_tensor(cond), tr.as_tensor(choice), res)
        return TorchArray(res)
    else:
        return NumpyArray(np.select(condlist, choicelist, default=default))


def place(arr, mask, vals, *, inplace=True): raise NotImplementedError
# if not rp.use_jax: return NumpyArray(np.place(*args,**kwargs))
# else: return jnp.place(*args,**kwargs)


def put(*args, **kwargs): raise NotImplementedError
# if not rp.use_jax: return NumpyArray(np.put(*args,**kwargs))
# else: return jnp.put(*args,**kwargs)


def put_along_axis(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.put_along_axis(*args,**kwargs))
# 	else: return jnp.put_along_axis(*args,**kwargs)


def putmask(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.putmask(*args,**kwargs))
# 	else: return jnp.putmask(*args,**kwargs)


def fill_diagonal(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.fill_diagonal(*args,**kwargs))
# 	else: return jnp.fill_diagonal(*args,**kwargs)


def nditer(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.nditer(*args,**kwargs))
# 	else: return jnp.nditer(*args,**kwargs)


def ndenumerate(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.ndenumerate(*args,**kwargs))
# 	else: return jnp.ndenumerate(*args,**kwargs)


def ndindex(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.ndindex(*args,**kwargs))
# 	else: return jnp.ndindex(*args,**kwargs)


def nested_iters(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.nested_iters(*args,**kwargs))
# 	else: return jnp.nested_iters(*args,**kwargs)


def flatiter(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.flatiter(*args,**kwargs))
# 	else: return jnp.flatiter(*args,**kwargs)


def iterable(y):
    if rp.use_jax:
        return jnp.iterable(y)
    elif rp.use_torch:
        return hasattr(y, '__iter__') or isinstance(y, tr.Tensor)
    else:
        return NumpyArray(np.iterable(y))


def unique(ar, return_index=False, return_inverse=False, return_counts=False, axis=None, **kwargs):
    if rp.use_jax:
        return jnp.unique(ar, return_index=return_index, return_inverse=return_inverse, return_counts=return_counts,
                          axis=axis, **kwargs)
    elif rp.use_torch:
        # PyTorch tr.unique returns (values, [inverse_indices], [counts])
        # It does NOT return return_index.
        if return_index:
            raise NotImplementedError("PyTorch does not support return_index in unique")

        # torch.unique signature: input, sorted=True, return_inverse=False, return_counts=False, dim=None
        # Note: torch.unique 'dim' is 'axis' in numpy
        res = tr.unique(ar, sorted=True, return_inverse=return_inverse, return_counts=return_counts, dim=axis)

        if not return_inverse and not return_counts:
            return TorchArray(res)
        else:
            # res is a tuple
            return tuple(TorchArray(r) for r in res)
    else:
        res = np.unique(ar, return_index=return_index, return_inverse=return_inverse, return_counts=return_counts,
                        axis=axis, **kwargs)
        if isinstance(res, tuple):
            return tuple(NumpyArray(r) for r in res)
        else:
            return NumpyArray(res)


def unique_all(ar, axis=None, **kwargs):
    if rp.use_jax:
        return jnp.unique_all(ar, axis=axis, **kwargs)
    elif rp.use_torch:
        # unique_all returns (values, indices, inverse_indices, counts)
        # torch doesn't have a direct equivalent for indices
        raise NotImplementedError("PyTorch does not support unique_all")
    else:
        # np.unique with all return flags
        return unique(ar, return_index=True, return_inverse=True, return_counts=True, axis=axis, **kwargs)


def unique_counts(ar, axis=None, **kwargs):
    if rp.use_jax:
        return jnp.unique_counts(ar, axis=axis, **kwargs)
    elif rp.use_torch:
        return unique(ar, return_counts=True, axis=axis, **kwargs)
    else:
        return unique(ar, return_counts=True, axis=axis, **kwargs)


def unique_inverse(ar, axis=None, **kwargs):
    if rp.use_jax:
        return jnp.unique_inverse(ar, axis=axis, **kwargs)
    elif rp.use_torch:
        return unique(ar, return_inverse=True, axis=axis, **kwargs)
    else:
        return unique(ar, return_inverse=True, axis=axis, **kwargs)


def unique_values(ar, axis=None, **kwargs):
    if rp.use_jax:
        return jnp.unique_values(ar, axis=axis, **kwargs)
    elif rp.use_torch:
        return unique(ar, axis=axis, **kwargs)
    else:
        return unique(ar, axis=axis, **kwargs)


def in1d(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.in1d(*args,**kwargs))
# 	else: return jnp.in1d(*args,**kwargs)


def intersect1d(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.intersect1d(*args,**kwargs))
# 	else: return jnp.intersect1d(*args,**kwargs)


def isin(*args, **kwargs):  raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.isin(*args,**kwargs))
# 	else: return jnp.isin(*args,**kwargs)


def setdiff1d(*args, **kwargs):  raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.setdiff1d(*args,**kwargs))
# 	else: return jnp.setdiff1d(*args,**kwargs)


def setxor1d(*args, **kwargs):  raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.setxor1d(*args,**kwargs))
# 	else: return jnp.setxor1d(*args,**kwargs)


def union1d(*args, **kwargs):  raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.union1d(*args,**kwargs))
# 	else: return jnp.union1d(*args,**kwargs)


def sort(a, axis=-1, *, kind=None, order=None, stable=True):
    if rp.use_jax:
        return jnp.sort(a, axis=axis, kind=kind, order=order, stable=stable)
    elif rp.use_torch:
        res, _ = tr.sort(a, dim=axis, descending=False, stable=stable)
        return TorchArray(res)
    else:
        return NumpyArray(np.sort(a, axis=axis, kind=kind, order=order, stable=stable))


def lexsort(keys, axis=-1):
    if rp.use_jax:
        return jnp.lexsort(keys, axis=axis)
    elif rp.use_torch:
        raise NotImplementedError('PyTorch does not support lexsort directly')
    else:
        return NumpyArray(np.lexsort(keys, axis=axis))


def argsort(a, axis=-1, *, kind=None, order=None, stable=True):
    if rp.use_jax:
        return jnp.argsort(a, axis=axis, kind=kind, order=order, stable=stable)
    elif rp.use_torch:
        return TorchArray(tr.argsort(a, dim=axis, descending=False, stable=stable))
    else:
        return NumpyArray(np.argsort(a, axis=axis, kind=kind, order=order, stable=stable))


def sort_complex(a):
    if rp.use_jax:
        return jnp.sort_complex(a)
    elif rp.use_torch:
        # Sort by real then imaginary
        real_sort = tr.sort(a.real)
        return TorchArray(a[real_sort.indices])
    else:
        return NumpyArray(np.sort_complex(a))


def partition(a, kth, axis=-1):
    if rp.use_jax:
        return jnp.partition(a, kth, axis=axis)
    elif rp.use_torch:
        raise NotImplementedError('PyTorch does not support partition')
    else:
        return NumpyArray(np.partition(a, kth, axis=axis))


def argpartition(a, kth, axis=-1):
    if rp.use_jax:
        return jnp.argpartition(a, kth, axis=axis)
    elif rp.use_torch:
        raise NotImplementedError('PyTorch does not support argpartition')
    else:
        return NumpyArray(np.argpartition(a, kth, axis=axis))


def argmax(a, axis=None, out=None, keepdims=None):
    if rp.use_jax:
        return jnp.argmax(a, axis=axis, out=out, keepdims=keepdims)
    elif rp.use_torch:
        a_t = tr.as_tensor(a)
        if axis is not None:
            return TorchArray(tr.argmax(a_t, dim=axis, keepdim=bool(keepdims)))
        else:
            if keepdims:
                return TorchArray(tr.argmax(a_t, keepdim=True))
            else:
                return TorchArray(tr.argmax(a_t))
    else:
        return NumpyArray(np.argmax(a, axis=axis, out=out, keepdims=keepdims))


def nanargmax(a, axis=None, out=None, keepdims=None):
    if rp.use_jax:
        return jnp.nanargmax(a, axis=axis, out=out, keepdims=keepdims)
    elif rp.use_torch:
        a_t = tr.as_tensor(a)
        # manual nanargmax for torch
        a_filled = tr.where(tr.isnan(a_t), tr.tensor(-float('inf'), dtype=a_t.dtype, device=a_t.device), a_t)
        if axis is not None:
            return TorchArray(tr.argmax(a_filled, dim=axis, keepdim=bool(keepdims)))
        else:
            if keepdims:
                return TorchArray(tr.argmax(a_filled, keepdim=True))
            else:
                return TorchArray(tr.argmax(a_filled))
    else:
        return NumpyArray(np.nanargmax(a, axis=axis, out=out, keepdims=keepdims))


def argmin(a, axis=None, out=None, keepdims=None):
    if rp.use_jax:
        return jnp.argmin(a, axis=axis, out=out, keepdims=keepdims)
    elif rp.use_torch:
        a_t = tr.as_tensor(a)
        if axis is not None:
            return TorchArray(tr.argmin(a_t, dim=axis, keepdim=bool(keepdims)))
        else:
            if keepdims:
                return TorchArray(tr.argmin(a_t, keepdim=True))
            else:
                return TorchArray(tr.argmin(a_t))
    else:
        return NumpyArray(np.argmin(a, axis=axis, out=out, keepdims=keepdims))


def nanargmin(a, axis=None, out=None, keepdims=None):
    if rp.use_jax:
        return jnp.nanargmin(a, axis=axis, out=out, keepdims=keepdims)
    elif rp.use_torch:
        a_t = tr.as_tensor(a)
        # manual nanargmin for torch
        a_filled = tr.where(tr.isnan(a_t), tr.tensor(float('inf'), dtype=a_t.dtype, device=a_t.device), a_t)
        if axis is not None:
            return TorchArray(tr.argmin(a_filled, dim=axis, keepdim=bool(keepdims)))
        else:
            if keepdims:
                return TorchArray(tr.argmin(a_filled, keepdim=True))
            else:
                return TorchArray(tr.argmin(a_filled))
    else:
        return NumpyArray(np.nanargmin(a, axis=axis, out=out, keepdims=keepdims))


def argwhere(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.argwhere(*args,**kwargs))
# 	else: return jnp.argwhere(*args,**kwargs)

def flatnonzero(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.flatnonzero(*args,**kwargs))
# 	else: return jnp.flatnonzero(*args,**kwargs)

def searchsorted(a, v, side='left', sorter=None, *, method='scan'):
    if rp.use_jax:
        return jnp.searchsorted(a, v, side=side, sorter=sorter, method=method)
    elif rp.use_torch:
        # Convert to base tensors and make contiguous; direct attribute access is safe here
        # since tr.as_tensor guarantees torch.Tensor instances at this point.
        a_base = tr.as_tensor(a).contiguous().as_subclass(tr.Tensor)
        v_base = tr.as_tensor(v).contiguous().as_subclass(tr.Tensor)
        sorter_base = sorter.as_subclass(tr.Tensor) if isinstance(sorter, tr.Tensor) else sorter
        return TorchArray(tr.searchsorted(a_base, v_base, side=side, sorter=sorter_base))
    else:
        return NumpyArray(np.searchsorted(a, v, side=side, sorter=sorter))


def extract(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.extract(*args,**kwargs))
# 	else: return jnp.extract(*args,**kwargs)

def count_nonzero(a, axis=None, keepdims=False):
    if rp.use_jax:
        return jnp.count_nonzero(a, axis=axis, keepdims=keepdims)
    elif rp.use_torch:
        a_t = tr.as_tensor(a)
        if axis is not None:
            return TorchArray(tr.count_nonzero(a_t, dim=axis))
        else:
            res = tr.count_nonzero(a_t)
            if keepdims:
                res = res.view(*([1]*a_t.ndim))
            return TorchArray(res)
    else:
        return NumpyArray(np.count_nonzero(a, axis=axis, keepdims=keepdims))


def ptp(a, axis=None, out=None, keepdims=False):
    if rp.use_jax:
        return jnp.ptp(a, axis=axis, out=out, keepdims=keepdims)
    elif rp.use_torch:
        a_t = tr.as_tensor(a)
        if axis is not None:
            res = tr.amax(a_t, dim=axis, keepdim=bool(keepdims)) - tr.amin(a_t, dim=axis, keepdim=bool(keepdims))
        else:
            res = tr.amax(a_t) - tr.amin(a_t)
            if keepdims:
                res = res.view(*([1]*a_t.ndim))
        return TorchArray(res)
    else:
        return NumpyArray(np.ptp(a, axis=axis, out=out, keepdims=keepdims))


def percentile(a, q, axis=None, out=None, overwrite_input=False, method='linear', keepdims=False):
    if rp.use_jax:
        return jnp.percentile(a, q, axis=axis, out=out, overwrite_input=overwrite_input, method=method,
                              keepdims=keepdims)
    elif rp.use_torch:
        a_t = tr.as_tensor(a)
        q_t = tr.as_tensor(q) / 100.0
        if axis is not None:
            return TorchArray(tr.quantile(a_t, q_t, dim=axis, keepdim=bool(keepdims)))
        else:
            res = tr.quantile(a_t.flatten(), q_t)
            if keepdims:
                res = res.view(*([1]*a_t.ndim))
            return TorchArray(res)
    else:
        return NumpyArray(
            np.percentile(a, q, axis=axis, out=out, overwrite_input=overwrite_input, method=method, keepdims=keepdims))
# Note: Torch quantile uses [0, 1] range, percentile uses [0, 100]


def nanpercentile(a, q, axis=None, out=None, overwrite_input=False, method='linear', keepdims=False):
    if rp.use_jax:
        return jnp.nanpercentile(a, q, axis=axis, out=out, overwrite_input=overwrite_input, method=method,
                                 keepdims=keepdims)
    elif rp.use_torch:
        # manual nanpercentile for torch
        a_t = tr.as_tensor(a)
        q_t = tr.as_tensor(q) / 100.0
        if axis is not None:
            return TorchArray(tr.nanquantile(a_t, q_t, dim=axis, keepdim=bool(keepdims)))
        else:
            res = tr.nanquantile(a_t.flatten(), q_t)
            if keepdims:
                res = res.view(*([1]*a_t.ndim))
            return TorchArray(res)
    else:
        return NumpyArray(np.nanpercentile(a, q, axis=axis, out=out, overwrite_input=overwrite_input, method=method,
                                           keepdims=keepdims))


def quantile(a, q, axis=None, out=None, overwrite_input=False, method='linear', keepdims=False):
    if rp.use_jax:
        return jnp.quantile(a, q, axis=axis, out=out, overwrite_input=overwrite_input, method=method, keepdims=keepdims)
    elif rp.use_torch:
        a_t = tr.as_tensor(a)
        if axis is not None:
            return TorchArray(tr.quantile(a_t, q, dim=axis, keepdim=bool(keepdims)))
        else:
            res = tr.quantile(a_t.flatten(), q)
            if keepdims:
                res = res.view(*([1]*a_t.ndim))
            return TorchArray(res)
    else:
        return NumpyArray(
            np.quantile(a, q, axis=axis, out=out, overwrite_input=overwrite_input, method=method, keepdims=keepdims))


def nanquantile(a, q, axis=None, out=None, overwrite_input=False, method='linear', keepdims=False):
    if rp.use_jax:
        return jnp.nanquantile(a, q, axis=axis, out=out, overwrite_input=overwrite_input, method=method,
                               keepdims=keepdims)
    elif rp.use_torch:
        a_t = tr.as_tensor(a)
        if axis is not None:
            return TorchArray(tr.nanquantile(a_t, q, dim=axis, keepdim=bool(keepdims)))
        else:
            res = tr.nanquantile(a_t.flatten(), q)
            if keepdims:
                res = res.view(*([1]*a_t.ndim))
            return TorchArray(res)
    else:
        return NumpyArray(
            np.nanquantile(a, q, axis=axis, out=out, overwrite_input=overwrite_input, method=method, keepdims=keepdims))


def median(a, axis=None, out=None, overwrite_input=False, keepdims=False):
    if rp.use_jax:
        return jnp.median(a, axis=axis, out=out, overwrite_input=overwrite_input, keepdims=keepdims)
    elif rp.use_torch:
        a_t = tr.as_tensor(a)
        if axis is not None:
            return TorchArray(tr.median(a_t, dim=axis, keepdim=bool(keepdims)).values)
        else:
            res = tr.median(a_t)
            if keepdims:
                res = res.view(*([1]*a_t.ndim))
            return TorchArray(res)
    else:
        return NumpyArray(np.median(a, axis=axis, out=out, overwrite_input=overwrite_input, keepdims=keepdims))


def average(a, axis=None, weights=None, returned=False, *, keepdims=False):
    if rp.use_jax:
        return jnp.average(a, axis=axis, weights=weights, returned=returned, keepdims=keepdims)
    elif rp.use_torch:
        a_t = tr.as_tensor(a)
        if weights is not None:
             w_t = tr.as_tensor(weights)
             res = tr.sum(a_t * w_t, dim=axis, keepdim=bool(keepdims)) / tr.sum(w_t, dim=axis, keepdim=bool(keepdims))
        else:
             if axis is not None:
                 res = tr.mean(a_t, dim=axis, keepdim=bool(keepdims))
             else:
                 res = tr.mean(a_t)
                 if keepdims:
                     res = res.view(*([1]*a_t.ndim))
        if returned:
             return TorchArray(res), TorchArray(tr.sum(tr.as_tensor(weights) if weights is not None else tr.ones_like(a_t), dim=axis, keepdim=bool(keepdims)))
        return TorchArray(res)
    else:
        return NumpyArray(np.average(a, axis=axis, weights=weights, returned=returned, keepdims=keepdims))


def mean(a, axis=None, dtype=None, out=None, keepdims=False, *, where=None):
    if rp.use_jax:
        return jnp.mean(a, axis=axis, dtype=dtype, out=out, keepdims=keepdims, where=where)
    elif rp.use_torch:
        if where is not None: raise NotImplementedError('PyTorch mean does not support where')
        a_t = tr.as_tensor(a)
        if axis is not None:
            return TorchArray(tr.mean(a_t, dim=axis, keepdim=bool(keepdims), dtype=_to_torch_dtype(dtype)))
        else:
            res = tr.mean(a_t, dtype=dtype)
            if keepdims:
                res = res.view(*([1]*a_t.ndim))
            return TorchArray(res)
    else:
        kwargs = {}
        if where is not None: kwargs['where'] = where
        return NumpyArray(np.mean(a, axis=axis, dtype=dtype, out=out, keepdims=keepdims, **kwargs))
# Note: fixed order of jnp.mean arguments if needed.


def std(a, axis=None, dtype=None, out=None, ddof=0, keepdims=False, *, where=None, correction=None):
    if rp.use_jax:
        return jnp.std(a, axis=axis, dtype=dtype, out=out, ddof=ddof, keepdims=keepdims, where=where,
                       correction=correction)
    elif rp.use_torch:
        if where is not None: raise NotImplementedError('PyTorch std does not support where')
        a_t = tr.as_tensor(a)
        if axis is not None:
            return TorchArray(tr.std(a_t, dim=axis, unbiased=(ddof > 0), keepdim=bool(keepdims)))
        else:
            res = tr.std(a_t, unbiased=(ddof > 0))
            if keepdims:
                res = res.view(*([1]*a_t.ndim))
            return TorchArray(res)
    else:
        kwargs = {}
        if where is not None: kwargs['where'] = where
        if correction is not None: kwargs['correction'] = correction
        return NumpyArray(np.std(a, axis=axis, dtype=dtype, out=out, ddof=ddof, keepdims=keepdims, **kwargs))


def var(a, axis=None, dtype=None, out=None, ddof=0, keepdims=False, *, where=None, correction=None):
    if rp.use_jax:
        return jnp.var(a, axis=axis, dtype=dtype, out=out, ddof=ddof, keepdims=keepdims, where=where,
                       correction=correction)
    elif rp.use_torch:
        if where is not None: raise NotImplementedError('PyTorch var does not support where')
        a_t = tr.as_tensor(a)
        if axis is not None:
            return TorchArray(tr.var(a_t, dim=axis, unbiased=(ddof > 0), keepdim=bool(keepdims)))
        else:
            res = tr.var(a_t, unbiased=(ddof > 0))
            if keepdims:
                res = res.view(*([1]*a_t.ndim))
            return TorchArray(res)
    else:
        kwargs = {}
        if where is not None: kwargs['where'] = where
        if correction is not None: kwargs['correction'] = correction
        return NumpyArray(np.var(a, axis=axis, dtype=dtype, out=out, ddof=ddof, keepdims=keepdims, **kwargs))


def nanmedian(a, axis=None, out=None, overwrite_input=False, keepdims=False):
    if rp.use_jax:
        return jnp.nanmedian(a, axis=axis, out=out, overwrite_input=overwrite_input, keepdims=keepdims)
    elif rp.use_torch:
        a_t = tr.as_tensor(a)
        if axis is not None:
            return TorchArray(tr.nanmedian(a_t, dim=axis, keepdim=bool(keepdims)).values)
        else:
            res = tr.nanmedian(a_t)
            if keepdims:
                res = res.view(*([1]*a_t.ndim))
            return TorchArray(res)
    else:
        return NumpyArray(np.nanmedian(a, axis=axis, out=out, overwrite_input=overwrite_input, keepdims=keepdims))


def nanmean(a, axis=None, dtype=None, out=None, keepdims=False, *, where=None):
    if rp.use_jax:
        return jnp.nanmean(a, axis=axis, dtype=dtype, out=out, keepdims=keepdims, where=where)
    elif rp.use_torch:
        if where is not None: raise NotImplementedError('PyTorch nanmean does not support where')
        a_t = tr.as_tensor(a)
        if axis is not None:
            num = tr.nansum(a_t, dim=axis, keepdim=bool(keepdims))
            den = tr.count_nonzero(~tr.isnan(a_t), dim=axis)
        else:
            num = tr.nansum(a_t)
            den = tr.count_nonzero(~tr.isnan(a_t))
            if keepdims:
                num = num.view(*([1]*a_t.ndim))
                den = den.view(*([1]*a_t.ndim))
        return TorchArray(num / den)
    else:
        kwargs = {}
        if where is not None: kwargs['where'] = where
        return NumpyArray(np.nanmean(a, axis=axis, dtype=dtype, out=out, keepdims=keepdims, **kwargs))


def nanstd(a, axis=None, dtype=None, out=None, ddof=0, keepdims=False, *, where=None, correction=None):
    if rp.use_jax:
        return jnp.nanstd(a, axis=axis, dtype=dtype, out=out, ddof=ddof, keepdims=keepdims, where=where,
                          correction=correction)
    elif rp.use_torch:
        if where is not None: raise NotImplementedError('PyTorch nanstd does not support where')
        # manual nanstd for torch
        return TorchArray(tr.sqrt(nanvar(a, axis=axis, ddof=ddof, keepdims=keepdims)))
    else:
        kwargs = {}
        if where is not None: kwargs['where'] = where
        if correction is not None: kwargs['correction'] = correction
        return NumpyArray(np.nanstd(a, axis=axis, dtype=dtype, out=out, ddof=ddof, keepdims=keepdims, **kwargs))


def nanvar(a, axis=None, dtype=None, out=None, ddof=0, keepdims=False, *, where=None, correction=None):
    if rp.use_jax:
        return jnp.nanvar(a, axis=axis, dtype=dtype, out=out, ddof=ddof, keepdims=keepdims, where=where,
                          correction=correction)
    elif rp.use_torch:
        if where is not None: raise NotImplementedError('PyTorch nanvar does not support where')
        # manual nanvar for torch
        m = nanmean(a, axis=axis, keepdims=True)
        a_t = tr.as_tensor(a)
        sq_diff = tr.square(a_t - m)
        return nanmean(sq_diff, axis=axis, keepdims=keepdims)
    else:
        kwargs = {}
        if where is not None: kwargs['where'] = where
        if correction is not None: kwargs['correction'] = correction
        return NumpyArray(np.nanvar(a, axis=axis, dtype=dtype, out=out, ddof=ddof, keepdims=keepdims, **kwargs))


def corrcoef(x, y=None, rowvar=True):
    if rp.use_jax:
        return jnp.corrcoef(x, y=y, rowvar=rowvar)
    elif rp.use_torch:
        return TorchArray(tr.corrcoef(tr.as_tensor(x))) # Note: y is ignored in torch.corrcoef
    else:
        return NumpyArray(np.corrcoef(x, y=y, rowvar=rowvar))


def correlate(a, v, mode='valid', *, precision=None, preferred_element_type=None):
    if rp.use_jax:
        return jnp.correlate(a, v, mode='valid', precision=precision, preferred_element_type=preferred_element_type)
    elif rp.use_torch:
        # Simplified correlate using conv1d
        a_t = tr.as_tensor(a).view(1, 1, -1)
        v_t = tr.as_tensor(v).view(1, 1, -1)
        return TorchArray(tr.conv1d(a_t, v_t).view(-1))
    else:
        return NumpyArray(np.correlate(a, v, mode='valid'))


def cov(m, y=None, rowvar=True, bias=False, ddof=None, fweights=None, aweights=None):
    if rp.use_jax:
        return jnp.cov(m, y=y, rowvar=rowvar, bias=bias, ddof=ddof, fweights=fweights, aweights=aweights)
    elif rp.use_torch:
        return TorchArray(tr.cov(m, correction=ddof, fweights=fweights, aweights=aweights))
    else:
        return NumpyArray(np.cov(m, y=y, rowvar=rowvar, bias=bias, ddof=ddof, fweights=fweights, aweights=aweights))


def histogram(a, bins=10, range=None, weights=None, density=None):
    if rp.use_jax:
        return jnp.histogram(a, bins=bins, range=range, weights=weights, density=density)
    elif rp.use_torch:
        counts, edges = tr.histogram(a, bins=bins, range=range, weight=weights, density=density)
        return TorchArray(counts), TorchArray(edges)
    else:
        return NumpyArray(np.histogram(a, bins=bins, range=range, weights=weights, density=density))


def histogram2d(x, y, bins=10, range=None, weights=None, density=None):
    if rp.use_jax:
        return jnp.histogram2d(x, y, bins=bins, range=range, weights=weights, density=density)
    elif rp.use_torch:
        raise NotImplementedError('PyTorch does not support histogram2d directly')
    else:
        return NumpyArray(np.histogram2d(x, y, bins=bins, range=range, weights=weights, density=density))


def histogramdd(sample, bins=10, range=None, weights=None, density=None):
    if rp.use_jax:
        return jnp.histogramdd(sample, bins=bins, range=range, weights=weights, density=density)
    elif rp.use_torch:
        raise NotImplementedError('PyTorch does not support histogramdd directly')
    else:
        return NumpyArray(np.histogramdd(sample, bins=bins, range=range, weights=weights, density=density))


def bincount(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.bincount(*args,**kwargs))
# 	else: return jnp.bincount(*args,**kwargs)

def histogram_bin_edges(a, bins=10, range=None, weights=None):
    if rp.use_jax:
        return jnp.histogram_bin_edges(a, bins=bins, range=range, weights=weights)
    elif rp.use_torch:
        _, edges = tr.histogram(a, bins=bins, range=range, weight=weights)
        return TorchArray(edges)
    else:
        return NumpyArray(np.histogram_bin_edges(a, bins=bins, range=range, weights=weights))


def digitize(x, bins, right=False, *, method=None):
    if rp.use_jax:
        return jnp.digitize(x, bins, right=right, method=method)
    elif rp.use_torch:
        return TorchArray(tr.bucketize(x, bins, right=right))
    else:
        return NumpyArray(np.digitize(x, bins, right=right, ))


def copyto(*args, **kwargs): raise NotImplementedError
# 	if not rp.use_jax: return NumpyArray(np.copyto(*args,**kwargs))
# 	else: return jnp.copyto(*args,**kwargs)

def ndim(a):
    if rp.use_jax:
        return jnp.ndim(a)
    elif rp.use_torch:
        return a.ndim
    else:
        return NumpyArray(np.ndim(a))


def shape(a):
    if rp.use_jax:
        return jnp.shape(a)
    elif rp.use_torch:
        return a.shape
    else:
        return NumpyArray(np.shape(a))
# Note: fixed return type if needed, but and JAX NP shape return tuples usually.


def size(a, axis=None):
    if rp.use_jax:
        return jnp.size(a, axis=axis)
    elif rp.use_torch:
        if axis is not None:
            return a.size(axis)
        return a.numel()
    else:
        return NumpyArray(np.size(a, axis=axis))


def reshape(a, shape=None, order='C', *, copy=None):
    if rp.use_jax:
        return jnp.reshape(a, shape=shape, order=order, copy=copy)
    elif rp.use_torch:
        if isinstance(shape, (int, np.integer)):
            shape = (int(shape),)
        return TorchArray(tr.reshape(a, shape))
    else:
        return NumpyArray(np.reshape(a, shape=shape, order=order, copy=copy))


def ravel(a, order='C'):
    if rp.use_jax:
        return jnp.ravel(a, order=order)
    elif rp.use_torch:
        return TorchArray(tr.ravel(a))
    else:
        return NumpyArray(np.ravel(a, order=order))


def moveaxis(a, source, destination):
    if rp.use_jax:
        return jnp.moveaxis(a, source, destination)
    elif rp.use_torch:
        return TorchArray(tr.moveaxis(a, source, destination))
    else:
        return NumpyArray(np.moveaxis(a, source, destination))


def rollaxis(a, axis, start=0):
    if rp.use_jax:
        return jnp.rollaxis(a, axis, start=start)
    elif rp.use_torch:
        # manual implementation for rollaxis
        n = a.ndim
        if axis < 0: axis += n
        if start < 0: start += n
        if axis < start:
            # move axis forward
            new_axes = list(range(n))
            new_axes.pop(axis)
            new_axes.insert(start - 1, axis)
        else:
            # move axis backward
            new_axes = list(range(n))
            new_axes.pop(axis)
            new_axes.insert(start, axis)
        return TorchArray(tr.permute(a, tuple(new_axes)))
    else:
        return NumpyArray(np.rollaxis(a, axis, start=start))


def swapaxes(a, axis1, axis2):
    if rp.use_jax:
        return jnp.swapaxes(a, axis1, axis2)
    elif rp.use_torch:
        return TorchArray(tr.swapaxes(a, axis1, axis2))
    else:
        return NumpyArray(np.swapaxes(a, axis1, axis2))


def transpose(a, axes=None):
    if rp.use_jax:
        return jnp.transpose(a, axes=axes)
    elif rp.use_torch:
        if axes is None:
            return TorchArray(tr.transpose(a, -2, -1) if a.ndim >= 2 else a) # wait, torch.T or adjoint?
            # np.transpose for default swaps all axes.
            # but usually it's used for matrices.
            # actually np.transpose(a) returns a.T (reversing all axes)
            return TorchArray(tr.permute(a, tuple(reversed(range(a.ndim)))))
        return TorchArray(tr.permute(a, axes))
    else:
        return NumpyArray(np.transpose(a, axes=axes))


def permute_dims(a, /, axes):
    if rp.use_jax:
        return jnp.permute_dims(a, axes)
    elif rp.use_torch:
        return TorchArray(tr.permute(a, axes))
    else:
        return NumpyArray(np.permute_dims(a, axes=axes))


def matrix_transpose(x, /):
    if rp.use_jax:
        return jnp.matrix_transpose(x)
    elif rp.use_torch:
        return TorchArray(tr.transpose(x, -2, -1))
    else:
        return NumpyArray(np.matrix_transpose(x))


def atleast_1d(*arys):
    if rp.use_jax:
        return jnp.atleast_1d(*arys)
    elif rp.use_torch:
        tensors = [tr.as_tensor(a) for a in arys]
        res = tr.atleast_1d(*tensors)
        if isinstance(res, tuple):
             return tuple(TorchArray(r) for r in res)
        return TorchArray(res)
    else:
        return NumpyArray(np.atleast_1d(*arys))


def atleast_2d(*arys):
    if rp.use_jax:
        return jnp.atleast_2d(*arys)
    elif rp.use_torch:
        tensors = [tr.as_tensor(a) for a in arys]
        res = tr.atleast_2d(*tensors)
        if isinstance(res, tuple):
             return tuple(TorchArray(r) for r in res)
        return TorchArray(res)
    else:
        return NumpyArray(np.atleast_2d(*arys))


def atleast_3d(*arys):
    if rp.use_jax:
        return jnp.atleast_3d(*arys)
    elif rp.use_torch:
        tensors = [tr.as_tensor(a) for a in arys]
        res = tr.atleast_3d(*tensors)
        if isinstance(res, tuple):
             return tuple(TorchArray(r) for r in res)
        return TorchArray(res)
    else:
        return NumpyArray(np.atleast_3d(*arys))


def broadcast(*args, **kwargs): raise NotImplementedError


def broadcast_to(array, shape):
    if rp.use_jax:
        return jnp.broadcast_to(array, shape)
    elif rp.use_torch:
        if isinstance(shape, (int, np.integer)):
            shape = (int(shape),)
        return TorchArray(tr.broadcast_to(array, shape))
    else:
        return NumpyArray(np.broadcast_to(array, shape))


def broadcast_arrays(*args):
    if rp.use_jax:
        return jnp.broadcast_arrays(*args)
    elif rp.use_torch:
        res = tr.broadcast_tensors(*args)
        return [TorchArray(r) for r in res]
    else:
        return NumpyArray(np.broadcast_arrays(*args))


def expand_dims(a, axis):
    if rp.use_jax:
        return jnp.expand_dims(a, axis)
    elif rp.use_torch:
        return TorchArray(tr.unsqueeze(a, axis))
    else:
        return NumpyArray(np.expand_dims(a, axis))


def squeeze(a, axis=None):
    if rp.use_jax:
        return jnp.squeeze(a, axis=axis)
    elif rp.use_torch:
        if axis is not None:
            return TorchArray(tr.squeeze(a, axis))
        return TorchArray(tr.squeeze(a))
    else:
        return NumpyArray(np.squeeze(a, axis=axis))


def asanyarray(*args, **kwargs): raise NotImplementedError


def asmatrix(*args, **kwargs): raise NotImplementedError


def asfortranarray(*args, **kwargs): raise NotImplementedError


def ascontiguousarray(*args, **kwargs): raise NotImplementedError


def asarray_chkfinite(*args, **kwargs): raise NotImplementedError


def require(*args, **kwargs): raise NotImplementedError


def concatenate(arrays, axis=0, dtype=None):
    if rp.use_jax:
        return jnp.concatenate(arrays, axis=axis, dtype=dtype)
    elif rp.use_torch:
        tensors = [tr.as_tensor(a) for a in arrays]
        return TorchArray(tr.cat(tensors, dim=axis))
    else:
        return NumpyArray(np.concatenate(arrays, axis=axis, dtype=dtype))


def concat(arrays, /, *, axis=0):
    if rp.use_jax:
        return jnp.concat(arrays, axis=axis)
    elif rp.use_torch:
        return TorchArray(tr.cat(arrays, dim=axis))
    else:
        return NumpyArray(np.concat(arrays, axis=axis))


def stack(arrays, axis=0, out=None, dtype=None):
    if rp.use_jax:
        return jnp.stack(arrays, axis=axis, out=out, dtype=dtype)
    elif rp.use_torch:
        tensors = [tr.as_tensor(a) for a in arrays]
        return TorchArray(tr.stack(tensors, dim=axis))
    else:
        return NumpyArray(np.stack(arrays, axis=axis, out=out, dtype=dtype))


def block(arrays):
    if rp.use_jax:
        return jnp.block(arrays)
    elif rp.use_torch:
        raise NotImplementedError('PyTorch does not support block directly')
    else:
        return NumpyArray(np.block(arrays))


def vstack(tup, dtype=None):
    if rp.use_jax:
        return jnp.vstack(tup, dtype=dtype)
    elif rp.use_torch:
        tensors = [tr.as_tensor(a) for a in tup]
        return TorchArray(tr.vstack(tensors))
    else:
        return NumpyArray(np.vstack(tup, dtype=dtype))


def hstack(tup, dtype=None):
    if rp.use_jax:
        return jnp.hstack(tup, dtype=dtype)
    elif rp.use_torch:
        tensors = [tr.as_tensor(a) for a in tup]
        return TorchArray(tr.hstack(tensors))
    else:
        return NumpyArray(np.hstack(tup, dtype=dtype))


def dstack(tup, dtype=None):
    if rp.use_jax:
        return jnp.dstack(tup, dtype=dtype)
    elif rp.use_torch:
        tensors = [tr.as_tensor(a) for a in tup]
        return TorchArray(tr.dstack(tensors))
    else:
        return NumpyArray(np.dstack(tup))


def column_stack(tup):
    if rp.use_jax:
        return jnp.column_stack(tup)
    elif rp.use_torch:
        tensors = [tr.as_tensor(a) for a in tup]
        return TorchArray(tr.column_stack(tensors))
    else:
        return NumpyArray(np.column_stack(tup))


def split(ary, indices_or_sections, axis=0):
    if rp.use_jax:
        return jnp.split(ary, indices_or_sections, axis=axis)
    elif rp.use_torch:
        res = tr.tensor_split(ary, indices_or_sections, dim=axis)
        return [TorchArray(r) for r in res]
    else:
        return NumpyArray(np.split(ary, indices_or_sections, axis=axis))


def array_split(ary, indices_or_sections, axis=0):
    if rp.use_jax:
        return jnp.array_split(ary, indices_or_sections, axis=axis)
    elif rp.use_torch:
        res = tr.tensor_split(ary, indices_or_sections, dim=axis)
        return [TorchArray(r) for r in res]
    else:
        return NumpyArray(np.array_split(ary, indices_or_sections, axis=axis))


def dsplit(ary, indices_or_sections):
    if rp.use_jax:
        return jnp.dsplit(ary, indices_or_sections)
    elif rp.use_torch:
        res = tr.tensor_split(ary, indices_or_sections, dim=2)
        return [TorchArray(r) for r in res]
    else:
        return NumpyArray(np.dsplit(ary, indices_or_sections))


def hsplit(ary, indices_or_sections):
    if rp.use_jax:
        return jnp.hsplit(ary, indices_or_sections)
    elif rp.use_torch:
        res = tr.tensor_split(ary, indices_or_sections, dim=1)
        return [TorchArray(r) for r in res]
    else:
        return NumpyArray(np.hsplit(ary, indices_or_sections))


def vsplit(ary, indices_or_sections):
    if rp.use_jax:
        return jnp.vsplit(ary, indices_or_sections)
    elif rp.use_torch:
        res = tr.tensor_split(ary, indices_or_sections, dim=0)
        return [TorchArray(r) for r in res]
    else:
        return NumpyArray(np.vsplit(ary, indices_or_sections))


def unstack(x, /, *, axis=0):
    if rp.use_jax:
        return jnp.unstack(x, axis=axis)
    elif rp.use_torch:
        res = tr.unbind(x, dim=axis)
        return [TorchArray(r) for r in res]
    else:
        return NumpyArray(np.unstack(x, axis=axis))


def tile(A, reps):
    if rp.use_jax:
        return jnp.tile(A, reps)
    elif rp.use_torch:
        if isinstance(reps, (int, np.integer)):
            reps = (int(reps),)
        return TorchArray(tr.tile(A, reps))
    else:
        return NumpyArray(np.tile(A, reps))


def repeat(a, repeats, axis=None):
    if rp.use_jax:
        return jnp.repeat(a, repeats, axis=axis)
    elif rp.use_torch:
        a_t = tr.as_tensor(a)
        # Note: torch.repeat_interleave 'dim' is 'axis' in numpy
        return TorchArray(tr.repeat_interleave(a_t, repeats, dim=axis))
    else:
        return NumpyArray(np.repeat(a, repeats, axis=axis))

def delete(arr, obj, axis=None):
    if rp.use_jax:
        return jnp.delete(arr, obj, axis=axis)
    elif rp.use_torch:
        arr_t = tr.as_tensor(arr)
        if axis is None:
            arr_t = arr_t.flatten()
            axis = 0
            
        N = arr_t.shape[axis]
        
        # Determine indices to remove
        if isinstance(obj, slice):
            remove_indices = tr.arange(N)[obj]
        elif isinstance(obj, (int, np.integer)):
            remove_indices = tr.tensor([int(obj)], device=arr_t.device)
        else:
            obj_tensor = tr.as_tensor(obj, device=arr_t.device)
            if obj_tensor.dtype == tr.bool:
                remove_indices = tr.nonzero(obj_tensor, as_tuple=True)[0]
            else:
                remove_indices = obj_tensor.flatten()
                
        # Create boolean mask for elements to keep
        keep_mask = tr.ones(N, dtype=tr.bool, device=arr_t.device)
        if remove_indices.numel() > 0:
            keep_mask[remove_indices.long()] = False
            
        keep_indices = tr.nonzero(keep_mask, as_tuple=True)[0]
        return TorchArray(tr.index_select(arr_t, axis, keep_indices))
    else:
        return NumpyArray(np.delete(arr, obj, axis=axis))

def insert(arr, obj, values, axis=None):
    if rp.use_jax:
        return jnp.insert(arr, obj, values, axis=axis)
    elif rp.use_torch:
        arr_t = tr.as_tensor(arr)
        values_t = tr.as_tensor(values)
        if axis is None:
            # Flatten everything and insert
            arr_f = arr_t.flatten()
            val_f = values_t.flatten()
            # This is slow but generic. PyTorch doesn't have a direct insert.
            if isinstance(obj, builtins.slice):
                # Handle slice insertion
                pass # Complex
            
            # Simple case: integer index or list of indices
            # We can use tr.cat with slices
            if builtins.isinstance(obj, builtins.int):
                 return TorchArray(tr.cat([arr_f[:obj], val_f, arr_f[obj:]]))
            
            # For more complex cases, fallback to numpy
            np_res = np.insert(arr_t.detach().cpu().numpy(), obj, values_t.detach().cpu().numpy(), axis=axis)
            return TorchArray(np_res).to(arr_t.device)
            
        else:
            # Fallback to numpy for multidimensional insert in torch
            np_res = np.insert(arr_t.detach().cpu().numpy(), obj, values_t.detach().cpu().numpy(), axis=axis)
            return TorchArray(np_res).to(arr_t.device)
    else:
        return NumpyArray(np.insert(arr, obj, values, axis=axis))


def append(arr, values, axis=None):
    if rp.use_jax:
        return jnp.append(arr, values, axis=axis)
    elif rp.use_torch:
        arr_t = tr.as_tensor(arr)
        val_t = tr.as_tensor(values)
        if axis is None:
             return TorchArray(tr.cat((arr_t.flatten(), val_t.flatten())))
        return TorchArray(tr.cat((arr_t, val_t), dim=axis))
    else:
        return NumpyArray(np.append(arr, values, axis=axis))


def resize(a, new_shape):
    if rp.use_jax:
        if isinstance(new_shape, list):
            new_shape = tuple(new_shape)
        return jnp.resize(a, new_shape)
    elif rp.use_torch:
        a_t = tr.as_tensor(a)
        new_size = 1
        for dim in new_shape:
            new_size *= dim
        
        a_flat = a_t.flatten()
        orig_size = a_flat.numel()
        
        if new_size == 0:
            return TorchArray(tr.empty(new_shape, dtype=a_t.dtype, device=a_t.device))
            
        # Repeat the flattened array if the new size is larger
        n_repeats = (new_size + orig_size - 1) // orig_size
        res = a_flat.repeat(n_repeats)[:new_size].reshape(new_shape)
        return TorchArray(res)
    else:
        return NumpyArray(np.resize(a, new_shape))


def trim_zeros(filt, trim='fb'):
    if rp.use_jax:
        return jnp.trim_zeros(filt, trim=trim)
    elif rp.use_torch:
        raise NotImplementedError('PyTorch does not support trim_zeros')
    else:
        return NumpyArray(np.trim_zeros(filt, trim=trim))


def pad(array, pad_width, mode='constant', **kwargs):
    if rp.use_jax:
        return jnp.pad(array, pad_width, mode=mode, **kwargs)
    elif rp.use_torch:
        if isinstance(pad_width, int):
             p = [pad_width] * (2 * array.ndim)
        else:
             p = []
             for b, a in reversed(pad_width):
                  p.extend([b, a])
        return TorchArray(tr.nn.functional.pad(array, tuple(p), mode=mode, **kwargs))
    else:
        return NumpyArray(np.pad(array, pad_width, mode=mode, **kwargs))


def flip(m, axis=None):
    if rp.use_jax:
        return jnp.flip(m, axis=axis)
    elif rp.use_torch:
        if axis is None:
             return TorchArray(tr.flip(m, dims=tuple(range(m.ndim))))
        if isinstance(axis, int): axis = (axis,)
        return TorchArray(tr.flip(m, dims=axis))
    else:
        return NumpyArray(np.flip(m, axis=axis))


def fliplr(m):
    if rp.use_jax:
        return jnp.fliplr(m)
    elif rp.use_torch:
        return TorchArray(tr.fliplr(m))
    else:
        return NumpyArray(np.fliplr(m))


def flipud(m):
    if rp.use_jax:
        return jnp.flipud(m)
    elif rp.use_torch:
        return TorchArray(tr.flipud(m))
    else:
        return NumpyArray(np.flipud(m))


def roll(a, shift, axis=None):
    if rp.use_jax:
        return jnp.roll(a, shift, axis=axis)
    elif rp.use_torch:
        return TorchArray(tr.roll(a, shift, dims=axis))
    else:
        return NumpyArray(np.roll(a, shift, axis=axis))


def rot90(m, k=1, axes=(0, 1)):
    if rp.use_jax:
        return jnp.rot90(m, k=k, axes=axes)
    elif rp.use_torch:
        return TorchArray(tr.rot90(m, k=k, dims=axes))
    else:
        return NumpyArray(np.rot90(m, k=k, axes=axes))


def trapezoid(y, x=None, dx=1.0, axis=-1):
    if rp.use_jax:
        if builtins.hasattr(jnp, 'trapezoid'):
            return jnp.trapezoid(y, x=x, dx=dx, axis=axis)
        else:
            return jnp.trapz(y, x=x, dx=dx, axis=axis)
    elif rp.use_torch:
        y_t = tr.as_tensor(y)
        x_t = tr.as_tensor(x) if x is not None else None
        
        func = builtins.getattr(tr, 'trapezoid', builtins.getattr(tr, 'trapz', None))
        if func is None:
            raise NotImplementedError('trapezoid not found in this version of Torch')
            
        kwargs = {'dim': axis}
        if x_t is not None: kwargs['x'] = x_t
        else: kwargs['dx'] = dx
            
        return TorchArray(func(y_t, **kwargs))
    else:
        if builtins.hasattr(np, 'trapezoid'):
            return NumpyArray(np.trapezoid(y, x=x, dx=dx, axis=axis))
        else:
            return NumpyArray(np.trapz(y, x=x, dx=dx, axis=axis))


def roots(p):
    if rp.use_jax:
        return jnp.roots(p)
    elif rp.use_torch:
        p_t = tr.atleast_1d(tr.as_tensor(p))
        if p_t.numel() == 0:
            return TorchArray(tr.empty(0, device=p_t.device, dtype=tr.complex64))
        
        nonzero_idx = tr.nonzero(p_t, as_tuple=True)[0]
        if nonzero_idx.numel() == 0:
            return TorchArray(tr.empty(0, device=p_t.device, dtype=tr.complex64))
            
        p_t = p_t[nonzero_idx[0]:]
        
        if p_t.numel() < 2:
            return TorchArray(tr.empty(0, device=p_t.device, dtype=tr.complex64))
            
        if not p_t.is_floating_point() and not p_t.is_complex():
            p_t = p_t.to(tr.float64)
            
        N = p_t.numel() - 1
        A = tr.zeros((N, N), dtype=p_t.dtype, device=p_t.device)
        A[1:, :-1] = tr.eye(N - 1, dtype=p_t.dtype, device=p_t.device)
        A[0, :] = -p_t[1:] / p_t[0]
        
        return TorchArray(tr.linalg.eigvals(A))
    else:
        return NumpyArray(np.roots(p))

# def 	lib.npyio.NpzFile	(): raise NotImplementedError
# def 	rec.array	(): raise NotImplementedError
# def 	rec.fromarrays	(): raise NotImplementedError
# def 	rec.fromrecords	(): raise NotImplementedError
# def 	rec.fromstring	(): raise NotImplementedError
# def 	rec.fromfile	(): raise NotImplementedError
# def 	char.array	(): raise NotImplementedError
# def 	char.asarray	(): raise NotImplementedError
# def 	ndarray.sort	(): raise NotImplementedError
# def 	ndarray.flat	(): raise NotImplementedError
# def 	ndarray.flatten	(): raise NotImplementedError
# def 	ndarray.T	(): raise NotImplementedError
# def 	ndarray.tofile	(): raise NotImplementedError
# def 	ndarray.tolist	(): raise NotImplementedError
# def 	lib.format.open_memmap	(): raise NotImplementedError
# def 	lib.npyio.DataSource	(): raise NotImplementedError
# def 	lib.form	(): raise NotImplementedError


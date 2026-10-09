# src.py
# (c) Copyright 2024 Aerospace Research Community LLC

# Created:  Aug 2024 E. Botero
# Modified: 

# ----------------------------------------------------------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------------------------------------------------------  

import RNUMPY as rp
import warnings
from typing import cast

j   = rp.jax_handle
np  = rp.numpy_handle
jnp = j.numpy
JaxArray   = rp.JaxArray 
NumpyArray = rp.NumpyArray

from typing import Union

# ----------------------------------------------------------------------------------------------------------------------
#  Debug Print Function
# ----------------------------------------------------------------------------------------------------------------------  

def debugprint(fmt, *args, ordered=False, **kwargs):
    if not rp.use_jax: print(fmt.format(*args, **kwargs))
    else: j.debug.print(fmt,*args,ordered=ordered,**kwargs)


# ----------------------------------------------------------------------------------------------------------------------
#  ndarray
# ----------------------------------------------------------------------------------------------------------------------  

try:
    ndarray = Union[np.ndarray, jnp.ndarray]
except:
    ndarray = np.ndarray

# ----------------------------------------------------------------------------------------------------------------------
#  Numpy Functions
# ----------------------------------------------------------------------------------------------------------------------  

def dot(x,y):
    if not rp.use_jax: return np.dot(x,y)
    else: return jnp.dot(x,y)

def sin(x,/):
    if not rp.use_jax: return np.sin(x)
    else: return jnp.sin(x)

def cos(x,/): 
    if not rp.use_jax: return np.cos(x)
    else: return jnp.cos(x)

def tan(x,/):
    if not rp.use_jax: return np.tan(x)
    else: return jnp.tan(x)
    
def arcsin(x,/):
    if not rp.use_jax: return np.arcsin(x)
    else: return jnp.arcsin(x)

def asin(x,/):
    if not rp.use_jax: return np.asin(x)
    else: return jnp.asin(x)

def arccos(x,/):
    if not rp.use_jax: return np.arccos(x)
    else: return jnp.arccos(x)

def acos(x,/):
    if not rp.use_jax: return np.acos(x)
    else: return jnp.acos(x)

def arctan(x,/): 
    if not rp.use_jax: return np.arctan(x)
    else: return jnp.arctan(x)

def atan(x,/):
    if not rp.use_jax: return np.atan(x)
    else: return jnp.atan(x)

def hypot(x1,x2,/):
    if not rp.use_jax: return np.hypot(x1,x2)
    else: return jnp.hypot(x1,x2)

def arctan2(x1,x2,/):
    if not rp.use_jax: return np.arctan2(x1,x2)
    else: return jnp.arctan2(x1,x2)

def atan2(x1,x2,/):
    if not rp.use_jax: return np.atan2(x1,x2)
    else: return jnp.atan2(x1,x2)

def degrees(x,/):
    if not rp.use_jax: return np.degrees(x)
    else: return jnp.degrees(x)

def radians(x,/):
    if not rp.use_jax: return np.radians(x)
    else: return jnp.radians(x)

def unwrap(p,discont=None,axis=-1,period=6.283185307179586): 
    if not rp.use_jax: return np.radians(p,discont=discont,axis=axis,period=period)
    else: return jnp.unwrap(p,discont=discont,axis=axis,period=period)

def deg2rad(x,/):
    if not rp.use_jax: return np.deg2rad(x)
    else: return jnp.deg2rad(x)

def rad2deg(x,/): 
    if not rp.use_jax: return np.rad2deg(x)
    else: return jnp.rad2deg(x)

def sinh(x,/):
    if not rp.use_jax: return np.sinh(x)
    else: return jnp.sinh(x)

def cosh(x,/):
    if not rp.use_jax: return np.cosh(x)
    else: return jnp.cosh(x)

def tanh(x,/):
    if not rp.use_jax: return np.tanh(x)
    else: return jnp.tanh(x)

def arcsinh(x,/):
    if not rp.use_jax: return np.arcsinh(x)
    else: return jnp.arcsinh(x)

def asinh(x,/):
    if not rp.use_jax: return np.asinh(x)
    else: return jnp.asinh(x)

def arccosh(x,/):
    if not rp.use_jax: return np.arccosh(x)
    else: return jnp.arccosh(x)

def acosh(x,/):
    if not rp.use_jax: return np.acosh(x)
    else: return jnp.acosh(x)

def arctanh(x,/):
    if not rp.use_jax: return np.arctanh(x)
    else: return jnp.arctanh(x)

def atanh(x,/):
    if not rp.use_jax: return np.atanh(x)
    else: return jnp.atanh(x)

def round(a,decimals=0,out=None):
    if not rp.use_jax: return np.round(a,decimals=decimals,out=out)
    else: return jnp.round(a,decimals=decimals,out=out)

def around(a,decimals=0,out=None):
    if not rp.use_jax: return np.around(a,decimals=decimals,out=out)
    else: return jnp.around(a,decimals=decimals,out=out)

def rint(x,/):
    if not rp.use_jax: return np.rint(x)
    else: return jnp.rint(x)

def fix(x,out=None):
    if not rp.use_jax: return np.fix(x,out=out)
    else: return jnp.fix(x,out=out)

def floor(x,/):
    if not rp.use_jax: return np.floor(x)
    else: return jnp.floor(x)

def ceil(x,/):
    if not rp.use_jax: return np.ceil(x)
    else: return jnp.ceil(x)

def trunc(x):
    if not rp.use_jax: return np.trunc(x)
    else: return jnp.trunc(x)



def prod(a, axis=None, dtype=None, out=None, keepdims=False, initial=None, where=None, promote_integers=True):
    if not rp.use_jax: return np.prod(a,axis=axis,dtype=dtype,out=out,keepdims=keepdims,initial=initial,where=where)
    else: return jnp.prod(a,axis=axis,dtype=dtype,out=out,keepdims=keepdims,initial=initial,where=where,promote_integers=promote_integers)

def sum(a, axis=None, dtype=None, out=None, keepdims=False, initial=None, where=None, promote_integers=True):
    if not rp.use_jax:
         if initial is None: initial=np._NoValue
         return np.sum(a,axis=axis,dtype=dtype,out=out,keepdims=keepdims,initial=initial,where=where)
    else: return jnp.sum(a,axis=axis,dtype=dtype,out=out,keepdims=keepdims,initial=initial,where=where,promote_integers=promote_integers)

def nanprod(a, axis=None, dtype=None, out=None, keepdims=False, initial=None, where=None, promote_integers=True):
    if not rp.use_jax: 
        if initial is None: initial=np._NoValue
        return np.nanprod(a,axis=axis,dtype=dtype,out=out,keepdims=keepdims,initial=initial,where=where)
    else: return jnp.nanprod(a,axis=axis,dtype=dtype,out=out,keepdims=keepdims,initial=initial,where=where,promote_integers=promote_integers)

def nansum(a, axis=None, dtype=None, out=None, keepdims=False, initial=None, where=None, promote_integers=True):
    if not rp.use_jax: 
        if initial is None: initial=np._NoValue
        return np.nansum(a,axis=axis,dtype=dtype,out=out,keepdims=keepdims,initial=initial,where=where)
    else: return jnp.nansum(a,axis=axis,dtype=dtype,out=out,keepdims=keepdims,initial=initial,where=where,promote_integers=promote_integers)

def cumprod(a, axis=None, dtype=None, out=None):
    if not rp.use_jax: return np.cumprod(a,axis=axis,dtype=dtype,out=out)
    else: return jnp.cumprod(a,axis=axis,dtype=dtype,out=out)

def cumsum(a, axis=None, dtype=None, out=None):
    if not rp.use_jax: return np.cumsum(a,axis=axis,dtype=dtype,out=out)
    else: return jnp.cumsum(a,axis=axis,dtype=dtype,out=out)

def nancumprod(a, axis=None, dtype=None, out=None):
    if not rp.use_jax: return np.nancumprod(a,axis=axis,dtype=dtype,out=out)
    else: return jnp.nancumprod(a,axis=axis,dtype=dtype,out=out)

def nancumsum(a, axis=None, dtype=None, out=None):
    if not rp.use_jax: return np.nancumsum(a,axis=axis,dtype=dtype,out=out)
    else: return jnp.nancumsum(a,axis=axis,dtype=dtype,out=out)

def diff(a, n=1, axis=-1, prepend=None, append=None):
    if not rp.use_jax: return np.diff(a,n=n,prepend=prepend,append=append)
    else: return jnp.diff(a,n=n,prepend=prepend,append=append)

def ediff1d(ary, to_end=None, to_begin=None):
    if not rp.use_jax: return np.ediff1d(ary,to_end=to_end,to_begin=to_begin)
    else: return jnp.ediff1d(ary,to_end=to_end,to_begin=to_begin)

def gradient(f, *varargs, axis=None, edge_order=None):
    if not rp.use_jax: return np.gradient(f,*varargs,axis=axis,edge_order=edge_order)
    else: return jnp.gradient(f,*varargs,axis=axis,edge_order=edge_order)

def cross(a, b, axisa=-1, axisb=-1, axisc=-1, axis=None):
    if not rp.use_jax: return np.gradient(a,b,axisa=axisa,axisb=axisb,axisc=axisc,axis=axis)
    else: return jnp.gradient(a,b,axisa=axisa,axisb=axisb,axisc=axisc,axis=axis)

def exp(x,/): 
    if not rp.use_jax: return np.exp(x)
    else: return jnp.exp(x)

def expm1(x,/): 
    if not rp.use_jax: return np.expm1(x)
    else: return jnp.expm1(x)

def exp2(x,/): 
    if not rp.use_jax: return np.exp2(x)
    else: return jnp.exp2(x)

def log(x,/): 
    if not rp.use_jax: return np.log(x)
    else: return jnp.log(x)

def log10(x,/): 
    if not rp.use_jax: return np.log10(x)
    else: return jnp.log10(x)

def log2(x,/): 
    if not rp.use_jax: return np.log2(x)
    else: return jnp.log2(x)

def log1p(x,/): 
    if not rp.use_jax: return np.log1p(x)
    else: return jnp.log1p(x)

def logaddexp(x1,x2,/):
    if not rp.use_jax: return np.logaddexp(x1,x2)
    else: return jnp.logaddexp(x1,x2)

def logaddexp2(x1,x2,/):
    if not rp.use_jax: return np.logaddexp2(x1,x2)
    else: return jnp.logaddexp2(x1,x2)

def i0(x):
    if not rp.use_jax: return np.i0(x)
    else: return jnp.i0(x)

def sinc(x,/):
    if not rp.use_jax: return np.sinc(x)
    else: return jnp.sinc(x)

def signbitc(x,/):
    if not rp.use_jax: return np.signbit(x)
    else: return jnp.signbit(x)

def copysign(x1,x2,/):
    if not rp.use_jax: return np.copysign(x1,x2)
    else: return jnp.copysign(x1,x2)

def frexp(x,/):
    if not rp.use_jax: return np.frexp(x)
    else: return jnp.frexp(x)

def ldexp(x1,x2,/):
    if not rp.use_jax: return np.ldexp(x1,x2)
    else: return jnp.ldexp(x1,x2)

def nextafter(x1,x2,/):
    if not rp.use_jax: return np.nextafter(x1,x2)
    else: return jnp.nextafter(x1,x2)

def spacing(x,/):
    if not rp.use_jax: return np.spacing(x)
    else: return jnp.spacing(x)

def lcm(x1,x2):
    if not rp.use_jax: return np.lcm(x1,x2)
    else: return jnp.lcm(x1,x2)

def gcd(x1,x2):
    if not rp.use_jax: return np.gcd(x1,x2)
    else: return jnp.gcd(x1,x2)

def add(x1,x2,/):
    if not rp.use_jax: return np.add(x1,x2)
    else: return jnp.add(x1,x2)

def reciprocal(x,/):
    if not rp.use_jax: return np.reciprocal(x)
    else: return jnp.reciprocal(x)

def positive(x,/):
    if not rp.use_jax: return np.positive(x)
    else: return jnp.positive(x)
                               
def negative(x,/):
    if not rp.use_jax: return np.negative(x)
    else: return jnp.negative(x)

def multiply(x1,x2,/):
    if not rp.use_jax: return np.multiply(x1,x2)
    else: return jnp.multiply(x1,x2)

def divide(x1,x2,/):
    if not rp.use_jax: return np.divide(x1,x2)
    else: return jnp.divide(x1,x2)

def power(x1,x2,/):
    if not rp.use_jax: return np.power(x1,x2)
    else: return jnp.power(x1,x2)

def pow(x1,x2,/):
    if not rp.use_jax: return np.pow(x1,x2)
    else: return jnp.pow(x1,x2)

def subtract(x1,x2,/):
    if not rp.use_jax: return np.subtract(x1,x2)
    else: return jnp.subtract(x1,x2)

def true_divide(x1,x2,/):
    if not rp.use_jax: return np.true_divide(x1,x2)
    else: return jnp.true_divide(x1,x2)

def floor_divide(x1,x2,/):
    if not rp.use_jax: return np.floor_divide(x1,x2)
    else: return jnp.floor_divide(x1,x2)

def float_power(x1,x2,/):
    if not rp.use_jax: return np.float_power(x1,x2)
    else: return jnp.float_power(x1,x2)

def fmod(x1,x2,/):
    if not rp.use_jax: return np.fmod(x1,x2)
    else: return jnp.fmod(x1,x2)

def mod(x1,x2,/):
    if not rp.use_jax: return np.mod(x1,x2)
    else: return jnp.mod(x1,x2)

def modf(x,/,out=None):
    if not rp.use_jax: return np.modf(x,out=out)
    else: return jnp.modf(x,out=None)

def remainder(x1,x2,/):
    if not rp.use_jax: return np.remainder(x1,x2)
    else: return jnp.remainder(x1,x2)

def divmod(x1,x2,/):
    if not rp.use_jax: return np.divmod(x1,x2)
    else: return jnp.divmod(x1,x2)

def angle(z, deg=False):
    if not rp.use_jax: return np.angle(z,deg=deg)
    else: return jnp.angle(z,deg=deg)

def real(val,/):
    if not rp.use_jax: return np.real(val)
    else: return jnp.real(val)

def imag(val,/):
    if not rp.use_jax: return np.imag(val)
    else: return jnp.imag(val)

def conj(x,/):
    if not rp.use_jax: return np.conj(x)
    else: return jnp.conj(x)

def conjugate(x,/):
    if not rp.use_jax: return np.conjugate(x)
    else: return jnp.conjugate(x)
    
def maximum(x,y,/):
    if not rp.use_jax: return np.maximum(x,y)
    else: return jnp.maximum(x,y)

def max(a, axis=None, out=None, keepdims=False, initial=None, where=None):
    if not rp.use_jax: return np.max(a,axis=axis,out=out,keepdims=keepdims,initial=initial,where=where)
    else: return jnp.max(a,axis=axis,out=out,keepdims=keepdims,initial=initial,where=where)

def amax(a, axis=None, out=None, keepdims=False, initial=None, where=None):
    if not rp.use_jax: return np.amax(a,axis=axis,out=out,keepdims=keepdims,initial=initial,where=where)
    else: return jnp.amax(a,axis=axis,out=out,keepdims=keepdims,initial=initial,where=where)

def fmax(x1,x2):
    if not rp.use_jax: return np.fmax(x1,x2)
    else: return jnp.fmax(x1,x2)

def nanmax(a, axis=None, out=None, keepdims=False, initial=None, where=None):
    if not rp.use_jax: return np.nanmax(a,axis=axis,out=out,keepdims=keepdims,initial=initial,where=where)
    else: return jnp.nanmax(a,axis=axis,out=out,keepdims=keepdims,initial=initial,where=where)

def minimum(x,y,/):
    if not rp.use_jax: return np.minimum(x,y)
    else: return jnp.minimum(x,y)

def min(a, axis=None, out=None, keepdims=False, initial=None, where=None):
    if not rp.use_jax: return np.min(a,axis=axis,out=out,keepdims=keepdims,initial=initial,where=where)
    else: return jnp.min(a,axis=axis,out=out,keepdims=keepdims,initial=initial,where=where)

def amin(a, axis=None, out=None, keepdims=False, initial=None, where=None):
    if not rp.use_jax: return np.amin(a,axis=axis,out=out,keepdims=keepdims,initial=initial,where=where)
    else: return jnp.amin(a,axis=axis,out=out,keepdims=keepdims,initial=initial,where=where)

def fmin(x1,x2):
    if not rp.use_jax: return np.fmin(x1,x2)
    else: return jnp.fmin(x1,x2)

def nanmin(a, axis=None, out=None, keepdims=False, initial=None, where=None):
    if not rp.use_jax: return np.nanmin(a,axis=axis,out=out,keepdims=keepdims,initial=initial,where=where)
    else: return jnp.nanmin(a,axis=axis,out=out,keepdims=keepdims,initial=initial,where=where)

def convolve(a, v, mode='full', *, precision=None, preferred_element_type=None):
    if not rp.use_jax: return np.convolve(a,v,mode=mode)
    else: return jnp.convolve(a,v,mode=mode,precision=precision,preferred_element_type=preferred_element_type)

def clip(arr=None, /, min=None, max=None,):
    if not rp.use_jax: return np.clip(arr=arr,a_min=min,a_max=max)
    else: return jnp.clip(arr=arr, min=min, max=max)

def sqrt(x,/):
    if  not rp.use_jax:  return np.sqrt(x)
    else: return jnp.sqrt(x)

def cbrt(x,/):
    if  not rp.use_jax:  return np.cbrt(x)
    else: return jnp.cbrt(x)

def square(x,/):
    if  not rp.use_jax:  return np.square(x)
    else: return jnp.square(x)
    
def absolute(x,/):
    if  not rp.use_jax:  return np.absolute(x)
    else: return jnp.absolute(x)

def fab(x,/):
    if  not rp.use_jax:  return np.fabs(x)
    else: return jnp.fabs(x)

def sign(x,/):
    if  not rp.use_jax:  return np.sign(x)
    else: return jnp.sign(x)

def heaviside(x1,x2,/):
    if not rp.use_jax: return np.heaviside(x1,x2)
    else: return jnp.heaviside(x1,x2)

def nan_to_num(x, copy=True, nan=0.0, posinf=None, neginf=None):
    if  not rp.use_jax:  return np.nan_to_num(x,copy=copy,nan=nan,posinf=posinf,neginf=neginf)
    else: return jnp.nan_to_num(x,copy=copy,nan=nan,posinf=posinf,neginf=neginf)

def real_if_close(): raise NotImplementedError # There is no JAX functionality

def interp(x, xp, fp, left=None, right=None, period=None):
    if not rp.use_jax: return np.interp(x,xp,fp,left=left,right=right,period=period)
    else: return jnp.interp(x,xp,fp,left=left,right=right,period=period)

def bitwise_count(x, /):
    if not rp.use_jax: return np.bitwise_count(x)
    else: return jnp.bitwise_count(x)

def vdot(a, b, *, precision=None, preferred_element_type=None):
    if not rp.use_jax: return np.vdot(a,b)
    else: return jnp.vdot(a,b,precision=precision,preferred_element_type=preferred_element_type)

def vecdot(a, b, *, precision=None, preferred_element_type=None):
    if not rp.use_jax: return np.vecdot(a,b)
    else: return jnp.vecdot(a,b,precision=precision,preferred_element_type=preferred_element_type)

def inner(a, b, *, precision=None, preferred_element_type=None):
    if not rp.use_jax: return np.inner(a,b)
    else: return jnp.inner(a,b,precision=precision,preferred_element_type=preferred_element_type)

def outer(a, b, out=None):
    if not rp.use_jax: return np.outer(a,b,out=out)
    else: return jnp.outer(a,b,out=out)

def matmul(a, b, *, precision=None, preferred_element_type=None):
    if not rp.use_jax: return np.inner(a,b)
    else: return jnp.inner(a,b,precision=precision,preferred_element_type=preferred_element_type)

def tensordot(a, b, axes=2, *, precision=None, preferred_element_type=None):
    if not rp.use_jax: return np.tensordot(a,b,axes=axes)
    else: return jnp.inner(a,b,axes=axes,precision=precision,preferred_element_type=preferred_element_type)

def einsum(subscript: str, /, *operands, out=None, optimize: str | bool | list[tuple[int, ...]] = 'optimal', precision =None, preferred_element_type =None):
    if not rp.use_jax: return np.einsum(subscript,*operands,out=out,order='K',casting='safe',optimize=optimize)
    else: return jnp.einsum(subscript,*operands,out=out,optimize=optimize,precision=precision,preferred_element_type=preferred_element_type)

def einsum_path(subscripts,/,*operands,optimize="greedy"):
    if not rp.use_jax: return np.einsum_path(subscripts,*operands,optimize=optimize)
    else: return jnp.einsum_path(subscripts,*operands,optimize=optimize)

def kron(a,b):
    if not rp.use_jax: return np.kron(a,b)
    else: return jnp.kron(a,b)

def trace(a, offset=0, axis1=0, axis2=1, dtype=None, out=None):
    if not rp.use_jax: return np.trace(a, offset=offset,axis1=axis1,axis2=axis2,dtype=dtype,out=out)
    else: return jnp.kron(a, offset=offset,axis1=axis1,axis2=axis2,dtype=dtype,out=out)

def diagonal(a, offset=0, axis1=0, axis2=1):
    if not rp.use_jax: return np.diagonal(a, offset=offset, axis1=axis1, axis2=axis2)
    else: return jnp.diagonal(a, offset=offset, axis1=axis1, axis2=axis2)

def load(file, mmap_mode=None, allow_pickle=False, fix_imports=True, encoding='ASCII', *, max_header_size=10000):
    if not rp.use_jax: return np.load(file, mmap_mode=mmap_mode, allow_pickle=allow_pickle, fix_imports=fix_imports, encoding=encoding, max_header_size=max_header_size)
    else: return jnp.load(file, mmap_mode=mmap_mode, allow_pickle=allow_pickle, fix_imports=fix_imports, encoding=encoding, max_header_size=max_header_size)

def save(file, arr, allow_pickle=True):
    if not rp.use_jax: return np.save(file, arr, allow_pickle=allow_pickle)
    else: return jnp.save(file, arr, allow_pickle=allow_pickle)

def savez(file, *args, **kwds):
    if not rp.use_jax: return np.savez(file,*args,**kwds)
    else: return jnp.savez(file,*args,**kwds)

def savez_compressed(): raise NotImplementedError

def loadtxt(): raise NotImplementedError

def savetxt(): raise NotImplementedError

def genfromtxt(): raise NotImplementedError

def fromregex(): raise NotImplementedError

def fromstring(string, dtype=float, count=-1, *, sep): 
    if not rp.use_jax: return np.fromstring(string, dtype=dtype, count=-1, sep=sep)
    else: return jnp.fromstring(string, dtype=dtype, count=-1, sep=sep)

def array2string(): raise NotImplementedError

def array_repr(arr, max_line_width=None, precision=None, suppress_small=None):
    if not rp.use_jax: return np.array_repr(arr, max_line_width=max_line_width, precision=precision, suppress_small=suppress_small)
    else: return jnp.array_repr(arr, max_line_width=max_line_width, precision=precision, suppress_small=suppress_small)

def array_str(a, max_line_width=None, precision=None, suppress_small=None):
    if not rp.use_jax: return np.array_str(a, max_line_width=max_line_width, precision=precision, suppress_small=suppress_small)
    else: return jnp.array_str(a, max_line_width=max_line_width, precision=precision, suppress_small=suppress_small)

def format_float_positional(): raise NotImplementedError

def format_float_scientific(): raise NotImplementedError

def memmap(): raise NotImplementedError

def set_printoptions(precision=None, threshold=None, edgeitems=None, linewidth=None, suppress=None, nanstr=None, infstr=None, \
                     formatter=None, sign=None, floatmode=None, *, legacy=None, override_repr=None):
    if not rp.use_jax: return np.set_printoptions(precision=precision, threshold=threshold, edgeitems=edgeitems, linewidth=linewidth, \
                                                  suppress=suppress, nanstr=nanstr, infstr=infstr, formatter=formatter, sign=sign, floatmode=floatmode, legacy=legacy)
    else: return jnp.set_printoptions(precision=precision, threshold=threshold, edgeitems=edgeitems, linewidth=linewidth, \
                                                  suppress=suppress, nanstr=nanstr, infstr=infstr, formatter=formatter, sign=sign, floatmode=floatmode, legacy=legacy, override_repr=override_repr)

def get_printoptions(): 
    if not rp.use_jax: return np.get_printoptions()
    else: return jnp.get_printoptions()

def printoptions(*args, **kwargs):
    if not rp.use_jax: np.printoptions(*args, **kwargs)
    else: jnp.printoptions(*args, **kwargs)

def binary_repr(): raise NotImplementedError

def base_repr(): raise NotImplementedError

def apply_along_axis(func1d, axis, arr, *args, **kwargs):
    if not rp.use_jax: return np.apply_along_axis(func1d, axis=axis, arr=arr, *args, **kwargs)
    else: return jnp.apply_along_axis(func1d, axis=axis, arr=arr, *args, **kwargs)

def apply_over_axes(func, a, axes):
    if not rp.use_jax: return np.apply_over_axes(func, a, axes)
    else: return jnp.apply_over_axes(func, a, axes)

def vectorize(pyfunc, *, excluded=frozenset({}), signature=None):
    if not rp.use_jax: return np.vectorize(pyfunc=pyfunc, otypes=None, doc=None, excluded=excluded, cache=False,signature=signature)
    else: return jnp.vectorize(pyfunc=pyfunc,excluded=excluded,signature=signature)

def frompyfunc(func, /, nin, nout, *, identity=None):
    if not rp.use_jax: return np.frompyfunc(func,nin,nout, identity=identity)
    else: return jnp.frompyfunc(func, nin, nout, identity=identity)

def piecewise(x, condlist, funclist, *args, **kw):
    if not rp.use_jax: return np.piecewise(x, condlist, funclist, *args, **kw)
    else: return jnp.piecewise(x, condlist, funclist, *args, **kw)

def empty(shape, dtype=float, *, device=None):
    if not rp.use_jax: return np.empty(shape,dtype=dtype,device=device)
    else: return jnp.empty(shape,dtype=dtype,device=device)

def empty_like(prototype, dtype=None, shape=None, *, device=None):
    if not rp.use_jax: return np.empty_like(prototype,dtype=dtype,shape=shape,device=device)
    else: return jnp.empty_like(prototype, dtype=dtype, shape=shape, device=device)

def eye(N, M=None, k=0, dtype=float, *, device=None):
    if not rp.use_jax: return np.eye(N,M=M,k=k,dtype=dtype,device=device)
    else: return jnp.eye(N,M=M,k=k,dtype=dtype,device=device)

def identity(n, dtype=None):
    if not rp.use_jax: return np.identity(n,dtype=dtype)
    else: return jnp.identity(n,dtype=dtype)

def ones(shape, dtype=None, *, device=None): 
    if not rp.use_jax: return np.ones(shape,dtype=dtype,device=device)
    else: return jnp.ones(shape, dtype=dtype, device=device)
 
def ones_like(a, dtype=None, shape=None, *, device=None):
    if not rp.use_jax: return np.ones_like(a, dtype=dtype, shape=shape, device=device)
    else: return jnp.ones_like(a, dtype=dtype, shape=shape, device=device)

def zeros(shape, dtype=None, *, device=None):
    if not rp.use_jax: return np.zeros(shape,dtype=dtype)
    else: return jnp.zeros(shape,dtype=dtype,device=device)

def zeros_like(a, dtype=None, shape=None, *, device=None):
    if not rp.use_jax: return np.zeros_like(a, dtype=dtype, shape=shape)
    else: return jnp.zeros_like(a, dtype=dtype, shape=shape, device=device)

def full(shape, fill_value, dtype=None, *, device=None):
    if not rp.use_jax: return np.full(shape, fill_value, dtype=dtype, device=device)
    else: return jnp.full(shape, fill_value, dtype=dtype, device=None)

def full_like(a, fill_value, dtype=None, shape=None, *, device=None):
    if not rp.use_jax: return np.full_like(a, fill_value, dtype=dtype, shape=shape, device=device)
    else: return jnp.full_like(a, fill_value, dtype=dtype, shape=shape, device=device)

def array(object, dtype=None, copy=True, order='K', ndmin=0, *, device=None):
    if not rp.use_jax: return NumpyArray(np.array(object, dtype=dtype, copy=copy, order=order, ndmin=ndmin))
    else: return cast(JaxArray,jnp.array(object, dtype=dtype, copy=copy, order=order, ndmin=ndmin, device=device))
        
def asarray(a, dtype=None, order=None, *, copy=None, device=None):
    if not rp.use_jax: return np.asanyarray(a, dtype=dtype, order=order, device=device, copy=copy)
    else: return jnp.asarray(a, dtype=dtype, order=order, copy=copy, device=device)
	
def asanyarray(): raise NotImplementedError

def ascontiguousarray(): raise NotImplementedError

def asmatrix(): raise NotImplementedError

def astype(x, dtype, /, *, copy=False, device=None):
	if not rp.use_jax: return np.astype(x, dtype, copy=copy, device=device)
	else: return jnp.astype(x, dtype, copy=copy, device=device)

def copy(a, order='K'):
	if not rp.use_jax: return np.copy(a, order=order)
	else: return jnp.copy(a, order=order)

def frombuffer(buffer, dtype=float, count=-1, offset=0):
	if not rp.use_jax: return np.frombuffer(buffer, dtype=dtype, count=count, offset=offset)
	else: return jnp.frombuffer(buffer, dtype=dtype, count=count, offset=offset)

def from_dlpack(x, /, *, device=None, copy=None):
	if not rp.use_jax: return np.from_dlpack(x, device=device, copy=copy)
	else: return jnp.from_dlpack(x, device=device, copy=copy)

def fromfile(): raise NotImplementedError

def fromfunction(function, shape, *, dtype=float, **kwargs):
	if not rp.use_jax: return np.fromfunction(function, shape, dtype=dtype, like=None, **kwargs)
	else: return jnp.fromfunction(function, shape, dtype=dtype, **kwargs)

def fromiter(): raise NotImplementedError

def arange(start, stop=None, step=None, dtype=None, *, device=None):
    if not rp.use_jax:
        if stop is not None:
            return np.arange(start, stop=stop, step=step, dtype=dtype, device=device)
        else:
            return np.arange(stop=start, step=step, dtype=dtype, device=device)
    else: return jnp.arange(start, stop=stop, step=step, dtype=dtype, device=device)

def linspace(start, stop, num=50, endpoint=True, retstep=False, dtype=None, axis=0, *, device=None):
	if not rp.use_jax: return np.linspace(start, stop, num=num, endpoint=endpoint, retstep=retstep, dtype=dtype, axis=axis, device=device)
	else: return jnp.linspace(start, stop, num=num, endpoint=endpoint, retstep=retstep, dtype=dtype, axis=axis, device=device)

def logspace(start, stop, num=50, endpoint=True, base=10.0, dtype=None, axis=0):
	if not rp.use_jax: return np.logspace(start, stop, num=num, endpoint=endpoint, base=base, dtype=dtype, axis=axis)
	else: return jnp.logspace(start, stop, num=num, endpoint=endpoint, base=base, dtype=dtype, axis=axis)

def geomspace(start, stop, num=50, endpoint=True, dtype=None, axis=0):
	if not rp.use_jax: return np.geomspace(start, stop, num=num, endpoint=endpoint, dtype=dtype, axis=axis)
	else: return jnp.geomspace(start, stop, num=num, endpoint=endpoint, dtype=dtype, axis=axis)

def meshgrid(*xi, copy=True, sparse=False, indexing='xy'):
	if not rp.use_jax: return np.meshgrid(*xi, copy=copy, sparse=sparse, indexing=indexing)
	else: return jnp.meshgrid(*xi, copy=copy, sparse=sparse, indexing=indexing)

def mgrid(): raise NotImplementedError

def ogrid(): raise NotImplementedError

def diag(v, k=0):
	if not rp.use_jax: return np.diag(v, k=k)
	else: return jnp.diag(v, k=k)

def diagflat(v, k=0):
	if not rp.use_jax: return np.diagflat(v, k=k)
	else: return jnp.diagflat(v, k=k)

def tri(N, M=None, k=0, dtype=float):
	if not rp.use_jax: return np.tri(N, M=M, k=k, dtype=dtype)
	else: return jnp.tri(N, M=M, k=k, dtype=dtype)

def tril(m, k=0):
	if not rp.use_jax: return np.tril(m, k=k)
	else: return jnp.tril(m, k=k)

def triu(m, k=0):
	if not rp.use_jax: return np.triu(m, k=k)
	else: return jnp.triu(m, k=k)

def vander(x, N=None, increasing=False):
	if not rp.use_jax: return np.vander(x, N=N, increasing=increasing)
	else: return jnp.vander(x, N=N, increasing=increasing)

def bmat(): raise NotImplementedError


def all	(a, axis=None, out=None, keepdims=False, *, where=None): raise NotImplementedError
def 	any	(): raise NotImplementedError
def 	isfinite	(): raise NotImplementedError
def 	isinf	(): raise NotImplementedError

def isnan(x,/):
    if not rp.use_jax: return np.isnan(x)
    else: return jnp.isnan(x)

def 	isnat	(): raise NotImplementedError
def 	isneginf	(): raise NotImplementedError
def 	isposinf	(): raise NotImplementedError
def 	iscomplex	(): raise NotImplementedError
def 	iscomplexobj	(): raise NotImplementedError
def 	isfortran	(): raise NotImplementedError
def 	isreal	(): raise NotImplementedError
def 	isrealobj	(): raise NotImplementedError
def 	isscalar	(): raise NotImplementedError
def 	logical_and	(): raise NotImplementedError
def 	logical_or	(): raise NotImplementedError
def 	logical_not	(): raise NotImplementedError
def 	logical_xor	(): raise NotImplementedError
def 	allclose	(): raise NotImplementedError
def 	isclose	(): raise NotImplementedError
def 	array_equal	(): raise NotImplementedError
def 	array_equiv	(): raise NotImplementedError
def 	greater	(): raise NotImplementedError
def 	greater_equal	(): raise NotImplementedError
def 	less	(): raise NotImplementedError
def 	less_equal	(): raise NotImplementedError
def 	equal	(): raise NotImplementedError
def 	not_equal	(): raise NotImplementedError

def bitwise_and(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.bitwise_and(*args,**kwargs)
# 	else: return jnp.bitwise_and(*args,**kwargs)

def bitwise_or(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.bitwise_or(*args,**kwargs)
# 	else: return jnp.bitwise_or(*args,**kwargs)

def bitwise_xor(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.bitwise_xor(*args,**kwargs)
# 	else: return jnp.bitwise_xor(*args,**kwargs)

def invert(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.invert(*args,**kwargs)
# 	else: return jnp.invert(*args,**kwargs)

def bitwise_invert(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.bitwise_invert(*args,**kwargs)
# 	else: return jnp.bitwise_invert(*args,**kwargs)

def left_shift(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.left_shift(*args,**kwargs)
# 	else: return jnp.left_shift(*args,**kwargs)

def bitwise_left_shift(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.bitwise_left_shift(*args,**kwargs)
# 	else: return jnp.bitwise_left_shift(*args,**kwargs)

def right_shift(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.right_shift(*args,**kwargs)
# 	else: return jnp.right_shift(*args,**kwargs)

def bitwise_right_shift(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.bitwise_right_shift(*args,**kwargs)
# 	else: return jnp.bitwise_right_shift(*args,**kwargs)

def packbits(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.packbits(*args,**kwargs)
# 	else: return jnp.packbits(*args,**kwargs)

def unpackbits(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.unpackbits(*args,**kwargs)
# 	else: return jnp.unpackbits(*args,**kwargs)

def binary_repr(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.binary_repr(*args,**kwargs)
# 	else: return jnp.binary_repr(*args,**kwargs)

def c_(*args,**kwargs): raise NotImplementedError # TODO: "c_", "index_exp", "mgrid", "ogrid", "r_", "s_" all require classes

def r_(*args,**kwargs): raise NotImplementedError

def s_(*args,**kwargs): raise NotImplementedError

def nonzero(a, *, size=None, fill_value=None):
    if not rp.use_jax: 
        warnings.warn("NP and JAX NP nonzero have different behavior, check JAX documentation")
        return np.nonzero(a)
    else: return jnp.nonzero(a, size=size, fill_value=fill_value)

def where(condition, x=None, y=None, /, *, size=None, fill_value=None):
    if not rp.use_jax:
        if x is None:
            warnings.warn("NP and JAX NP where have different behavior with a single input, check JAX documentation")
        return np.where(condition, x=x, y=y)
    else: return jnp.where(condition, x=x, y=y, size=size, fill_value=fill_value)

def indices(dimensions, dtype=None, sparse=False):
	if not rp.use_jax: return np.indices(dimensions, dtype=dtype, sparse=sparse)
	else: return jnp.indices(dimensions, dtype=dtype, sparse=sparse)

def ix_(*args):
	if not rp.use_jax: return np.ix_(*args)
	else: return jnp.ix_(*args)

def ravel_multi_index(multi_index, dims, mode='raise', order='C'):
	if not rp.use_jax: return np.ravel_multi_index(multi_index, dims, mode=mode, order=order)
	else: return jnp.ravel_multi_index(multi_index, dims, mode=mode, order=order)

def unravel_index(indices, shape):
	if not rp.use_jax: return np.unravel_index(indices, shape)
	else: return jnp.unravel_index(indices, shape)

def diag_indices(n, ndim=2):
	if not rp.use_jax: return np.diag_indices(n, ndim=ndim)
	else: return jnp.diag_indices(n, ndim=ndim)

def diag_indices_from(arr):
	if not rp.use_jax: return np.diag_indices_from(arr)
	else: return jnp.diag_indices_from(arr)

def mask_indices(n, mask_func, k=0, *, size=None):
    if not rp.use_jax: 
        if size is not None:
            warnings.warn("NP and JAX NP mask_indices have different behavior when size is specified, check JAX documentation")
        return np.mask_indices(n, mask_func, k=k)
    else: return jnp.mask_indices(n, mask_func, k=k, size=size)

def tril_indices(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.tril_indices(*args,**kwargs)
# 	else: return jnp.tril_indices(*args,**kwargs)

def tril_indices_from(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.tril_indices_from(*args,**kwargs)
# 	else: return jnp.tril_indices_from(*args,**kwargs)

def triu_indices(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.triu_indices(*args,**kwargs)
# 	else: return jnp.triu_indices(*args,**kwargs)

def triu_indices_from(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.triu_indices_from(*args,**kwargs)
# 	else: return jnp.triu_indices_from(*args,**kwargs)

def take(a, indices, axis=None, out=None, mode=None, unique_indices=False, indices_are_sorted=False, fill_value=None):  raise NotImplementedError
    # Note: this will take a little time to figure out how we want to handle the default mode
	# if not rp.use_jax: return np.take(a, indices, axis=axis, out=out, mode=mode)
	# else: return jnp.take(a,indices, axis=axis, out=out, mode=mode, unique_indices=unique_indices, indices_are_sorted=indices_are_sorted, fill_value=fill_value)

def take_along_axis(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.take_along_axis(*args,**kwargs)
# 	else: return jnp.take_along_axis(*args,**kwargs)

def choose(a, choices, out=None, mode='raise'):
	if not rp.use_jax: return np.choose(a, choices, out=out, mode=mode)
	else: return jnp.choose(a, choices, out=out, mode=mode)

def compress(*args,**kwargs):  raise NotImplementedError
# 	if not rp.use_jax: return np.compress(*args,**kwargs)
# 	else: return jnp.compress(*args,**kwargs)

def select(condlist, choicelist, default=0):
	if not rp.use_jax: return np.select(condlist, choicelist, default=default)
	else: return jnp.select(condlist, choicelist, default=default)

def place(arr, mask, vals, *, inplace=True): raise NotImplementedError
	# if not rp.use_jax: return np.place(*args,**kwargs)
	# else: return jnp.place(*args,**kwargs)

def put(*args,**kwargs): raise NotImplementedError
	# if not rp.use_jax: return np.put(*args,**kwargs)
	# else: return jnp.put(*args,**kwargs)

def put_along_axis(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.put_along_axis(*args,**kwargs)
# 	else: return jnp.put_along_axis(*args,**kwargs)

def putmask(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.putmask(*args,**kwargs)
# 	else: return jnp.putmask(*args,**kwargs)

def fill_diagonal(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.fill_diagonal(*args,**kwargs)
# 	else: return jnp.fill_diagonal(*args,**kwargs)

def nditer(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.nditer(*args,**kwargs)
# 	else: return jnp.nditer(*args,**kwargs)

def ndenumerate(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.ndenumerate(*args,**kwargs)
# 	else: return jnp.ndenumerate(*args,**kwargs)

def ndindex(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.ndindex(*args,**kwargs)
# 	else: return jnp.ndindex(*args,**kwargs)

def nested_iters(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.nested_iters(*args,**kwargs)
# 	else: return jnp.nested_iters(*args,**kwargs)

def flatiter(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.flatiter(*args,**kwargs)
# 	else: return jnp.flatiter(*args,**kwargs)

def iterable(y):
	if not rp.use_jax: return np.iterable(y)
	else: return jnp.iterable(y)

def unique(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.unique(*args,**kwargs)
# 	else: return jnp.unique(*args,**kwargs)

def unique_all(*args,**kwargs):  raise NotImplementedError
# 	if not rp.use_jax: return np.unique_all(*args,**kwargs)
# 	else: return jnp.unique_all(*args,**kwargs)

def unique_counts(*args,**kwargs):  raise NotImplementedError
# 	if not rp.use_jax: return np.unique_counts(*args,**kwargs)
# 	else: return jnp.unique_counts(*args,**kwargs)

def unique_inverse(*args,**kwargs):  raise NotImplementedError
# 	if not rp.use_jax: return np.unique_inverse(*args,**kwargs)
# 	else: return jnp.unique_inverse(*args,**kwargs)

def unique_values(*args,**kwargs):  raise NotImplementedError
# 	if not rp.use_jax: return np.unique_values(*args,**kwargs)
# 	else: return jnp.unique_values(*args,**kwargs)

def in1d(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.in1d(*args,**kwargs)
# 	else: return jnp.in1d(*args,**kwargs)

def intersect1d(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.intersect1d(*args,**kwargs)
# 	else: return jnp.intersect1d(*args,**kwargs)

def isin(*args,**kwargs):  raise NotImplementedError
# 	if not rp.use_jax: return np.isin(*args,**kwargs)
# 	else: return jnp.isin(*args,**kwargs)

def setdiff1d(*args,**kwargs):  raise NotImplementedError
# 	if not rp.use_jax: return np.setdiff1d(*args,**kwargs)
# 	else: return jnp.setdiff1d(*args,**kwargs)

def setxor1d(*args,**kwargs):  raise NotImplementedError
# 	if not rp.use_jax: return np.setxor1d(*args,**kwargs)
# 	else: return jnp.setxor1d(*args,**kwargs)

def union1d(*args,**kwargs):  raise NotImplementedError
# 	if not rp.use_jax: return np.union1d(*args,**kwargs)
# 	else: return jnp.union1d(*args,**kwargs)

def sort(a, axis=-1, *, kind=None, order=None, stable=True):
	if not rp.use_jax: return np.sort(a, axis=axis, kind=kind, order=order, stable=stable)
	else: return jnp.sort(a, axis=axis, kind=kind, order=order, stable=stable)

def lexsort(keys, axis=-1):
	if not rp.use_jax: return np.lexsort(keys, axis=axis)
	else: return jnp.lexsort(keys, axis=axis)

def argsort(a, axis=-1, *, kind=None, order=None, stable=True):
	if not rp.use_jax: return np.argsort(a, axis=axis, kind=kind, order=order, stable=stable)
	else: return jnp.argsort(a, axis=axis, kind=kind, order=order, stable=stable)

def sort_complex(a):
	if not rp.use_jax: return np.sort_complex(a)
	else: return jnp.sort_complex(a)

def partition(a, kth, axis=-1):
	if not rp.use_jax: return np.partition(a, kth, axis=axis)
	else: return jnp.partition(a, kth, axis=axis)

def argpartition(a, kth, axis=-1):
	if not rp.use_jax: return np.argpartition(a, kth, axis=axis)
	else: return jnp.argpartition(a, kth, axis=axis)

def argmax(a, axis=None, out=None, keepdims=None):
	if not rp.use_jax: return np.argmax(a, axis=axis, out=out, keepdims=keepdims)
	else: return jnp.argmax(a, axis=axis, out=out, keepdims=keepdims)

def nanargmax(a, axis=None, out=None, keepdims=None):
	if not rp.use_jax: return np.nanargmax(a, axis=axis, out=out, keepdims=keepdims)
	else: return jnp.nanargmax(a, axis=axis, out=out, keepdims=keepdims)

def argmin(a, axis=None, out=None, keepdims=None):
	if not rp.use_jax: return np.argmin(a, axis=axis, out=out, keepdims=keepdims)
	else: return jnp.argmin(a, axis=axis, out=out, keepdims=keepdims)

def nanargmin(a, axis=None, out=None, keepdims=None):
	if not rp.use_jax: return np.nanargmin(a, axis=axis, out=out, keepdims=keepdims)
	else: return jnp.nanargmin(a, axis=axis, out=out, keepdims=keepdims)

def argwhere(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.argwhere(*args,**kwargs)
# 	else: return jnp.argwhere(*args,**kwargs)

def flatnonzero(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.flatnonzero(*args,**kwargs)
# 	else: return jnp.flatnonzero(*args,**kwargs)

def searchsorted(a, v, side='left', sorter=None, *, method='scan'):
	if not rp.use_jax: return np.searchsorted(a, v, side=side, sorter=sorter)
	else: return jnp.searchsorted(a, v, side=side, sorter=sorter, method=method)

def extract(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.extract(*args,**kwargs)
# 	else: return jnp.extract(*args,**kwargs)

def count_nonzero(a, axis=None, keepdims=False):
	if not rp.use_jax: return np.count_nonzero(a, axis=axis, keepdims=keepdims)
	else: return jnp.count_nonzero(a, axis=axis, keepdims=keepdims)

def ptp(a, axis=None, out=None, keepdims=False):
	if not rp.use_jax: return np.ptp(a, axis=axis, out=out, keepdims=keepdims)
	else: return jnp.ptp(a, axis=axis, out=out, keepdims=keepdims)

def percentile(a, q, axis=None, out=None, overwrite_input=False, method='linear', keepdims=False):
	if not rp.use_jax: return np.percentile(a, q, axis=axis, out=out, overwrite_input=overwrite_input, method=method, keepdims=keepdims)
	else: return jnp.percentile(a, q, axis=axis, out=out, overwrite_input=overwrite_input, method=method, keepdims=keepdims)

def nanpercentile(a, q, axis=None, out=None, overwrite_input=False, method='linear', keepdims=False):
	if not rp.use_jax: return np.nanpercentile(a, q, axis=axis, out=out, overwrite_input=overwrite_input, method=method, keepdims=keepdims)
	else: return jnp.nanpercentile(a, q, axis=axis, out=out, overwrite_input=overwrite_input, method=method, keepdims=keepdims)

def quantile(a, q, axis=None, out=None, overwrite_input=False, method='linear', keepdims=False):
	if not rp.use_jax: return np.quantile(a, q, axis=axis, out=out, overwrite_input=overwrite_input, method=method, keepdims=keepdims)
	else: return jnp.quantile(a, q, axis=axis, out=out, overwrite_input=overwrite_input, method=method, keepdims=keepdims)

def nanquantile(a, q, axis=None, out=None, overwrite_input=False, method='linear', keepdims=False):
	if not rp.use_jax: return np.nanquantile(a, q, axis=axis, out=out, overwrite_input=overwrite_input, method=method, keepdims=keepdims)
	else: return jnp.nanquantile(a, q, axis=axis, out=out, overwrite_input=overwrite_input, method=method, keepdims=keepdims)

def median(a, axis=None, out=None, overwrite_input=False,keepdims=False):
	if not rp.use_jax: return np.median(a, axis=axis, out=out, overwrite_input=overwrite_input, keepdims=keepdims)
	else: return jnp.median(a, axis=axis, out=out, overwrite_input=overwrite_input, keepdims=keepdims)

def average(a, axis=None, weights=None, returned=False, *, keepdims=False):
	if not rp.use_jax: return np.average(a, axis=axis, weights=weights, returned=returned, keepdims=keepdims)
	else: return jnp.average(a, axis=axis, weights=weights, returned=returned, keepdims=keepdims)

def mean(a, axis=None, dtype=None, out=None, keepdims=False, *, where=None):
	if not rp.use_jax: return np.mean(a, axis=axis, dtype=dtype, out=out, keepdims=keepdims, where=where)
	else: return jnp.mean(a, axis=axis, dtype=dtype, out=out, keepdims=keepdims, where=where)

def std(a, axis=None, dtype=None, out=None, ddof=0, keepdims=False, *, where=None, correction=None):
    if not rp.use_jax: return np.std(a, axis=axis, dtype=dtype, out=out, ddof=ddof, keepdims=keepdims, where=where, correction=correction)
    else: return jnp.std(a, axis=axis, dtype=dtype, out=out, ddof=ddof, keepdims=keepdims, where=where, correction=correction)

def var(a, axis=None, dtype=None, out=None, ddof=0, keepdims=False, *, where=None, correction=None):
	if not rp.use_jax: return np.var(a, axis=axis, dtype=dtype, out=out, ddof=ddof, keepdims=keepdims, where=where, correction=correction)
	else: return jnp.var(a, axis=axis, dtype=dtype, out=out, ddof=ddof, keepdims=keepdims, where=where, correction=correction)

def nanmedian(a, axis=None, out=None, overwrite_input=False,keepdims=False):
	if not rp.use_jax: return np.nanmedian(a, axis=axis, out=out, overwrite_input=overwrite_input, keepdims=keepdims)
	else: return jnp.nanmedian(a, axis=axis, out=out, overwrite_input=overwrite_input, keepdims=keepdims)

def nanmean(a, axis=None, dtype=None, out=None, keepdims=False, *, where=None):
	if not rp.use_jax: return np.nanmean(a, axis=axis, dtype=dtype, out=out, keepdims=keepdims, where=where)
	else: return jnp.nanmean(a, axis=axis, dtype=dtype, out=out, keepdims=keepdims, where=where)

def nanstd(a, axis=None, dtype=None, out=None, ddof=0, keepdims=False, *, where=None, correction=None):
	if not rp.use_jax: return np.nanstd(a, axis=axis, dtype=dtype, out=out, ddof=ddof, keepdims=keepdims, where=where, correction=correction)
	else: return jnp.nanstd(a, axis=axis, dtype=dtype, out=out, ddof=ddof, keepdims=keepdims, where=where, correction=correction)

def nanvar(a, axis=None, dtype=None, out=None, ddof=0, keepdims=False, *, where=None, correction=None):
	if not rp.use_jax: return np.nanvar(a, axis=axis, dtype=dtype, out=out, ddof=ddof, keepdims=keepdims, where=where, correction=correction)
	else: return jnp.nanvar(a, axis=axis, dtype=dtype, out=out, ddof=ddof, keepdims=keepdims, where=where, correction=correction)

def corrcoef(x, y=None, rowvar=True):
	if not rp.use_jax: return np.corrcoef(x, y=y, rowvar=rowvar)
	else: return jnp.corrcoef(x, y=y, rowvar=rowvar)

def correlate(a, v, mode='valid', *, precision=None, preferred_element_type=None):
	if not rp.use_jax: return np.correlate(a, v, mode='valid')
	else: return jnp.correlate(a, v, mode='valid', precision=precision, preferred_element_type=preferred_element_type)

def cov(m, y=None, rowvar=True, bias=False, ddof=None, fweights=None, aweights=None):
	if not rp.use_jax: return np.cov(m, y=y, rowvar=rowvar, bias=bias, ddof=ddof, fweights=fweights, aweights=aweights)
	else: return jnp.cov(m, y=y, rowvar=rowvar, bias=bias, ddof=ddof, fweights=fweights, aweights=aweights)

def histogram(a, bins=10, range=None, weights=None, density=None):
	if not rp.use_jax: return np.histogram(a, bins=bins, range=range, weights=weights, density=density)
	else: return jnp.histogram(a, bins=bins, range=range, weights=weights, density=density)

def histogram2d(x, y, bins=10, range=None, weights=None, density=None):
	if not rp.use_jax: return np.histogram2d(x, y, bins=bins, range=range, weights=weights, density=density)
	else: return jnp.histogram2d(x, y, bins=bins, range=range, weights=weights, density=density)

def histogramdd(sample, bins=10, range=None, weights=None, density=None):
	if not rp.use_jax: return np.histogramdd(sample, bins=bins, range=range, weights=weights, density=density)
	else: return jnp.histogramdd(sample, bins=bins, range=range, weights=weights, density=density)

def bincount(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.bincount(*args,**kwargs)
# 	else: return jnp.bincount(*args,**kwargs)

def histogram_bin_edges(a, bins=10, range=None, weights=None):
	if not rp.use_jax: return np.histogram_bin_edges(a, bins=bins, range=range, weights=weights)
	else: return jnp.histogram_bin_edges(a, bins=bins, range=range, weights=weights)

def digitize(x, bins, right=False, *, method=None):
	if not rp.use_jax: return np.digitize(x, bins, right=right,)
	else: return jnp.digitize(x, bins, right=right, method=method)

def copyto(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.copyto(*args,**kwargs)
# 	else: return jnp.copyto(*args,**kwargs)

def ndim(a):
	if not rp.use_jax: return np.ndim(a)
	else: return jnp.ndim(a)

def shape(a):
	if not rp.use_jax: return np.shape(a)
	else: return jnp.shape(a)

def size(a, axis=None):
	if not rp.use_jax: return np.size(a, axis=axis)
	else: return jnp.size(a, axis=axis)

def reshape(a, shape=None, order='C', *, copy=None):
	if not rp.use_jax: return np.reshape(a, shape=shape, order=order, copy=copy)
	else: return jnp.reshape(a, shape=shape, order=order, copy=copy)

def ravel(a, order='C'):
	if not rp.use_jax: return np.ravel(a, order=order)
	else: return jnp.ravel(a, order=order)

def moveaxis(a, source, destination):
	if not rp.use_jax: return np.moveaxis(a, source, destination)
	else: return jnp.moveaxis(a, source, destination)

def rollaxis(a, axis, start=0):
	if not rp.use_jax: return np.rollaxis(a, axis, start=start)
	else: return jnp.rollaxis(a, axis, start=start)

def swapaxes(a, axis1, axis2):
	if not rp.use_jax: return np.swapaxes(a, axis1, axis2)
	else: return jnp.swapaxes(a, axis1, axis2)

def transpose(a, axes=None):
	if not rp.use_jax: return np.transpose(a, axes=axes)
	else: return jnp.transpose(a, axes=axes)

def permute_dims(a, /, axes):
	if not rp.use_jax: return np.permute_dims(a, axes=axes)
	else: return jnp.permute_dims(a, axes)

def matrix_transpose(x, /):
	if not rp.use_jax: return np.matrix_transpose(x)
	else: return jnp.matrix_transpose(x)

def atleast_1d(*arys):
	if not rp.use_jax: return np.atleast_1d(*arys)
	else: return jnp.atleast_1d(*arys)

def atleast_2d(*arys):
	if not rp.use_jax: return np.atleast_2d(*arys)
	else: return jnp.atleast_2d(*arys)

def atleast_3d(*arys):
	if not rp.use_jax: return np.atleast_3d(*arys)
	else: return jnp.atleast_3d(*arys)

def broadcast(*args,**kwargs): raise NotImplementedError

def broadcast_to(array, shape):
	if not rp.use_jax: return np.broadcast_to(array, shape)
	else: return jnp.broadcast_to(array, shape)

def broadcast_arrays(*args):
	if not rp.use_jax: return np.broadcast_arrays(*args)
	else: return jnp.broadcast_arrays(*args)

def expand_dims(a, axis):
	if not rp.use_jax: return np.expand_dims(a, axis)
	else: return jnp.expand_dims(a, axis)

def squeeze(a, axis=None):
	if not rp.use_jax: return np.squeeze(a, axis=axis)
	else: return jnp.squeeze(a, axis=axis)

def asanyarray(*args,**kwargs): raise NotImplementedError

def asmatrix(*args,**kwargs): raise NotImplementedError

def asfortranarray(*args,**kwargs): raise NotImplementedError

def ascontiguousarray(*args,**kwargs): raise NotImplementedError

def asarray_chkfinite(*args,**kwargs): raise NotImplementedError

def require(*args,**kwargs): raise NotImplementedError

def concatenate(arrays, axis=0, dtype=None):
	if not rp.use_jax: return np.concatenate(arrays, axis=axis, dtype=dtype)
	else: return jnp.concatenate(arrays, axis=axis, dtype=dtype)

def concat(arrays, /, *, axis=0):
	if not rp.use_jax: return np.concat(arrays, axis=axis)
	else: return jnp.concat(arrays, axis=axis)

def stack(arrays, axis=0, out=None, dtype=None):
	if not rp.use_jax: return np.stack(arrays, axis=axis, out=out, dtype=dtype)
	else: return jnp.stack(arrays, axis=axis, out=out, dtype=dtype)

def block(arrays):
	if not rp.use_jax: return np.block(arrays)
	else: return jnp.block(arrays)

def vstack(tup, dtype=None):
	if not rp.use_jax: return np.vstack(tup, dtype=dtype)
	else: return jnp.vstack(tup, dtype=dtype)

def hstack(tup, dtype=None):
	if not rp.use_jax: return np.hstack(tup, dtype=dtype)
	else: return jnp.hstack(tup, dtype=dtype)

def dstack(tup, dtype=None):
	if not rp.use_jax: return np.dstack(tup)
	else: return jnp.dstack(tup, dtype=dtype)

def column_stack(tup):
	if not rp.use_jax: return np.column_stack(tup)
	else: return jnp.column_stack(tup)

def split(ary, indices_or_sections, axis=0):
	if not rp.use_jax: return np.split(ary, indices_or_sections, axis=axis)
	else: return jnp.split(ary, indices_or_sections, axis=axis)

def array_split(ary, indices_or_sections, axis=0):
	if not rp.use_jax: return np.array_split(ary, indices_or_sections, axis=axis)
	else: return jnp.array_split(ary, indices_or_sections, axis=axis)

def dsplit(ary, indices_or_sections):
	if not rp.use_jax: return np.dsplit(ary, indices_or_sections)
	else: return jnp.dsplit(ary, indices_or_sections)

def hsplit(ary, indices_or_sections):
	if not rp.use_jax: return np.hsplit(ary, indices_or_sections)
	else: return jnp.hsplit(ary, indices_or_sections)

def vsplit(ary, indices_or_sections):
	if not rp.use_jax: return np.vsplit(ary, indices_or_sections)
	else: return jnp.vsplit(ary, indices_or_sections)

def unstack(x, /, *, axis=0):
	if not rp.use_jax: return np.unstack(x, axis=axis)
	else: return jnp.unstack(x, axis=axis)

def tile(A, reps):
	if not rp.use_jax: return np.tile(A, reps)
	else: return jnp.tile(A, reps)

def repeat(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.repeat(*args,**kwargs)
# 	else: return jnp.repeat(*args,**kwargs)

def delete(*args,**kwargs): raise NotImplementedError
# 	if not rp.use_jax: return np.delete(*args,**kwargs)
# 	else: return jnp.delete(*args,**kwargs)

def insert(arr, obj, values, axis=None):
	if not rp.use_jax: return np.insert(arr, obj, values, axis=axis)
	else: return jnp.insert(arr, obj, values, axis=axis)

def append(arr, values, axis=None):
	if not rp.use_jax: return np.append(arr, values, axis=axis)
	else: return jnp.append(arr, values, axis=axis)

def resize(a, new_shape):
	if not rp.use_jax: return np.resize(a, new_shape)
	else: return jnp.resize(a, new_shape)

def trim_zeros(filt, trim='fb'):
	if not rp.use_jax: return np.trim_zeros(filt, trim=trim)
	else: return jnp.trim_zeros(filt, trim=trim)

def pad(array, pad_width, mode='constant', **kwargs):
	if not rp.use_jax: return np.pad(array, pad_width, mode=mode, **kwargs)
	else: return jnp.pad(array, pad_width, mode=mode, **kwargs)

def flip(m, axis=None):
	if not rp.use_jax: return np.flip(m, axis=axis)
	else: return jnp.flip(m, axis=axis)

def fliplr(m):
	if not rp.use_jax: return np.fliplr(m)
	else: return jnp.fliplr(m)

def flipud(m):
	if not rp.use_jax: return np.flipud(m)
	else: return jnp.flipud(m)

def roll(a, shift, axis=None):
	if not rp.use_jax: return np.roll(a, shift, axis=axis)
	else: return jnp.roll(a, shift, axis=axis)

def rot90(m, k=1, axes=(0, 1)):
	if not rp.use_jax: return np.rot90(m, k=k, axes=axes)
	else: return jnp.rot90(m, k=k, axes=axes)


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
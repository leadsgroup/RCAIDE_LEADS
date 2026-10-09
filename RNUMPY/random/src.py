# src.py
# (c) Copyright 2024 Aerospace Research Community LLC

# Created:  Oct 2024 E. Botero
# Modified: Apr 2026, E. Botero

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------  

import RNUMPY as rp

j   = rp.jax_handle
np_h = rp.numpy_handle
tr  = rp.torch_handle

# For functional emulation on non-JAX backends
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Functions
# ----------------------------------------------------------------------------------------------------------------------  

def seed(seed_value):
    """Sets the global seed for the active backend (except JAX which is functional)."""
    if rp.use_torch and tr:
        tr.manual_seed(seed_value)
    
    # Always set numpy seed as default behavior for seed()
    np_h.random.seed(seed_value)
    return

def PRNGKey(seed):
    """Returns a backend-appropriate random key."""
    if rp.use_jax and j:
        return j.random.PRNGKey(seed)
    # For NumPy/PyTorch emulation, we use the integer seed as the key
    return seed

def split(key, num=2):
    """Splits a key into multiple keys."""
    if rp.use_jax and j:
        return j.random.split(key, num)
    
    # Emulation: use the current key as a seed to generate new seeds
    rng = np.random.default_rng(key)
    seeds = rng.integers(0, 2**31 - 1, size=num)
    return [int(s) for s in seeds]

def uniform(key, shape=(), dtype=float, minval=0.0, maxval=1.0):
    """Generates uniform random numbers."""
    if rp.use_jax and j:
        return j.random.uniform(key, shape, dtype, minval, maxval)
    elif rp.use_torch and tr:
        g = tr.Generator()
        g.manual_seed(key)
        # Note: we use rp.float32 or similar if dtype is a string or numpy type
        # But for now let's assume it handles basic types
        res = tr.rand(shape, generator=g) 
        res = (res * (maxval - minval)) + minval
        return rp.TorchArray(res)
    else:
        rng = np.random.default_rng(key)
        res = rng.uniform(minval, maxval, size=shape)
        return rp.NumpyArray(res)

def normal(key, shape=(), dtype=float):
    """Generates normally distributed random numbers."""
    if rp.use_jax and j:
        return j.random.normal(key, shape, dtype)
    elif rp.use_torch and tr:
        g = tr.Generator()
        g.manual_seed(key)
        res = tr.randn(shape, generator=g)
        return rp.TorchArray(res)
    else:
        rng = np.random.default_rng(key)
        res = rng.standard_normal(size=shape)
        return rp.NumpyArray(res)

def choice(key, a, shape=(), replace=True, p=None, axis=0):
    """Randomly chooses elements from an array."""
    if rp.use_jax and j:
        return j.random.choice(key, a, shape, replace, p, axis)
    elif rp.use_torch and tr:
        if isinstance(a, int):
            size = a
            arr = tr.arange(size)
            axis = 0 
        else:
            arr = tr.as_tensor(a)
            size = arr.shape[axis]
            
        num_samples = int(np.prod(shape))
        
        if p is None:
            probs = tr.ones(size)
        else:
            probs = tr.as_tensor(p)
            
        g = tr.Generator()
        g.manual_seed(key)
        
        indices = tr.multinomial(probs, num_samples, replacement=replace, generator=g)
        
        res = tr.index_select(arr, axis, indices)
        
        final_shape = arr.shape[:axis] + tuple(shape) + arr.shape[axis+1:]
        res = res.reshape(final_shape)
        
        return rp.TorchArray(res)
    else:
        rng = np.random.default_rng(key)
        # Handle int a for np.random.default_rng().choice if necessary 
        # (Actually rng.choice accepts int)
        res = rng.choice(a, size=shape, replace=replace, p=p, axis=axis)
        return rp.NumpyArray(res)

# src.py
# (c) Copyright 2024 Aerospace Research Community LLC

# Created:  Oct 2024 M. Clarke
# Modified: 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------  

import RNUMPY as rp

j   = rp.jax_handle
np  = rp.numpy_handle
sp  = rp.scipy_handle
jnp = j.numpy if j is not None else None
jspatial = j.scipy.spatial.transform if j is not None else None
    
def Slerp(times, rotations): 
    if rp.use_jax: 
        return jspatial.Slerp(times, rotations)
    elif rp.use_torch:
        raise NotImplementedError('Slerp not supported for Torch')
    else: 
        if sp is not None:
             return sp.spatial.transform.Slerp(times, rotations)
        raise ImportError("SciPy is not installed. Cannot use Slerp.")
   
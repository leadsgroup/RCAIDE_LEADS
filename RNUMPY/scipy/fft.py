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
        import jax.scipy.fft
    except ImportError:
        pass

jf  = j.scipy.fft if j else None
sf  = sp.fft if sp is not None else None
tf  = tr.fft if tr else None
    
def dct(x,type=2, n=None, axis=-1, norm=None): 
    if rp.use_jax: return jf.dct(x,type=type,n=n,axis=axis, norm=norm)
    elif rp.use_torch: raise NotImplementedError('dct not supported for Torch')
    else: return sf.dct(x,type=type, n=n, axis=axis, norm=norm)
    
def dctn(x, type=2, s=None, axes=None, norm=None): 
    if rp.use_jax: return jf.dctn(x, type=type, s=s, axes=axes, norm=norm)
    elif rp.use_torch: raise NotImplementedError('dctn not supported for Torch')
    else: return sf.dctn(x,type=type, s=s, axes=axes, norm=norm)
     
def idct(x, type=2, n=None, axis=-1, norm=None): 
    if rp.use_jax: return jf.idct(x, type=type, n=n, axis=axis, norm=norm)
    elif rp.use_torch: raise NotImplementedError('idct not supported for Torch')
    else: return sf.idct(x, type=type, n=n, axis=axis, norm=norm)
     
def idctn(x, type=2, s=None, axes=None, norm=None): 
    if rp.use_jax: return jf.idctn(x, type=type, s=s, axes=axes, norm=norm)
    elif rp.use_torch: raise NotImplementedError('idctn not supported for Torch')
    else: return sf.idctn(x,type=type, s=s, axes=axes, norm=norm)

def rfft(x, n=None, axis=-1, norm=None):
    if rp.use_jax: return j.numpy.fft.rfft(x, n=n, axis=axis, norm=norm)
    elif rp.use_torch: return tf.rfft(x, n=n, dim=axis, norm=norm)
    else: return sf.rfft(x, n=n, axis=axis, norm=norm)
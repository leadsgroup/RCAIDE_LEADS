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
        import jax.scipy.signal
    except ImportError:
        pass

js  = j.scipy.signal if j else None
ss  = sp.signal if sp is not None else None
ts  = tr.signal if tr else None

def fftconvolve(in1, in2, mode='full', axes=None): 
    if rp.use_jax: return js.fftconvolve(in1, in2, mode=mode, axes=axes)
    elif rp.use_torch: raise NotImplementedError('fftconvolve not supported for Torch easily')
    else: return ss.fftconvolve(in1, in2, mode=mode, axes=axes)
    
def convolve(in1, in2, mode='full', method='auto'): 
    if rp.use_jax: return js.convolve(in1, in2, mode=mode, method=method)
    elif rp.use_torch: raise NotImplementedError('convolve not supported for Torch in scipy.signal')
    else: return ss.convolve(in1, in2, mode=mode, method=method) 

def convolve2d(in1, in2, mode='full', boundary='fill', fillvalue=0, precision=None): 
    if rp.use_jax: return js.convolve2d(in1, in2, mode=mode, boundary=boundary, fillvalue=fillvalue, precision=precision)
    elif rp.use_torch:
        # basic implementation using conv2d if possible
        if boundary != 'fill' or fillvalue != 0: raise NotImplementedError('boundary/fillvalue not supported for Torch convolve2d')
        h, w = in1.shape
        kh, kw = in2.shape
        # pad in1
        pad_h = kh - 1
        pad_w = kw - 1
        x = tr.nn.functional.pad(in1, (pad_w, pad_w, pad_h, pad_h))
        res = tr.nn.functional.conv2d(x.unsqueeze(0).unsqueeze(0), in2.flip(0, 1).unsqueeze(0).unsqueeze(0))
        res = res.squeeze(0).squeeze(0)
        if mode == 'full': return rp.TorchArray(res)
        elif mode == 'same':
             # crop to same size
             start_h = (kh - 1) // 2
             start_w = (kw - 1) // 2
             return rp.TorchArray(res[start_h:start_h+h, start_w:start_w+w])
        elif mode == 'valid':
             return rp.TorchArray(res[kh-1:h, kw-1:w]) # check indices
    else: return ss.convolve2d(in1, in2, mode=mode, boundary=boundary, fillvalue=fillvalue)
 
def correlate():   raise NotImplementedError
def correlate2d(): raise NotImplementedError
def csd():         raise NotImplementedError
def detrend():     raise NotImplementedError
def istft():       raise NotImplementedError
def stft():        raise NotImplementedError
def welch():       raise NotImplementedError

def lfilter(b, a, x, axis=-1, zi=None):
    if rp.use_jax: 
        raise NotImplementedError('lfilter not supported for JAX easily')
    elif rp.use_torch: 
        return _torch_lfilter(b, a, x, axis=axis, zi=zi)
    else: 
        return ss.lfilter(b, a, x, axis=axis, zi=zi)

def _torch_lfilter(b, a, x, axis=-1, zi=None):
    """
    Differentiable 1D IIR/FIR filter for PyTorch.
    Computes: a[0]*y[n] = b[0]*x[n] + ... + b[M]*x[n-M] - a[1]*y[n-1] - ... - a[N]*y[n-N]
    """
    import torch
    
    b = torch.atleast_1d(torch.as_tensor(b, dtype=torch.float64))
    a = torch.atleast_1d(torch.as_tensor(a, dtype=torch.float64))
    x_t = torch.as_tensor(x, dtype=torch.float64)
    
    # Normalize by a[0]
    a0 = a[0]
    b = b / a0
    a = a / a0
    
    # Ensure working on the last dimension for simplicity, then transpose back
    if axis != -1 and axis != x_t.ndim - 1:
        x_t = x_t.transpose(axis, -1)
        
    orig_shape = x_t.shape
    x_flat = x_t.reshape(-1, orig_shape[-1])
    batch_size, seq_len = x_flat.shape
    
    # FIR part (Feedforward) using conv1d
    # conv1d expects (batch, in_channels, length)
    x_conv = x_flat.unsqueeze(1)
    # kernel expects (out_channels, in_channels, length)
    weight = b.unsqueeze(0).unsqueeze(0).flip(2)
    
    # Pad input to simulate causal filtering: add len(b)-1 zeros to the left
    pad_len = len(b) - 1
    if pad_len > 0:
        x_padded = torch.nn.functional.pad(x_conv, (pad_len, 0))
    else:
        x_padded = x_conv
        
    y_fir = torch.nn.functional.conv1d(x_padded, weight).squeeze(1)
    
    # Initial conditions
    if zi is not None:
        zi = torch.as_tensor(zi, dtype=torch.float64)
        if axis != -1 and axis != zi.ndim - 1:
            zi = zi.transpose(axis, -1)
        zi_flat = zi.reshape(-1, zi.shape[-1])
        # Add zi to the first few elements of y_fir
        add_len = min(seq_len, zi_flat.shape[1])
        y_fir[:, :add_len] += zi_flat[:, :add_len]
        
    # IIR part (Feedback)
    if len(a) > 1:
        a_rest = a[1:]
        n_a = len(a_rest)
        y = torch.zeros_like(x_flat)
        # We must iterate sequentially because y[n] depends on y[n-1]
        for n in range(seq_len):
            current_y = y_fir[:, n].clone()
            # feedback: sum(a[i] * y[n-i])
            for i in range(1, min(n + 1, n_a + 1)):
                current_y -= a_rest[i - 1] * y[:, n - i]
            y[:, n] = current_y
    else:
        y = y_fir
        
    # Final state calculation (zi_out)
    if zi is not None:
        raise NotImplementedError("Returning zi is not yet fully implemented for PyTorch lfilter")
        
    y_out = y.reshape(orig_shape)
    if axis != -1 and axis != x_t.ndim - 1:
        y_out = y_out.transpose(axis, -1)
        
    return rp.TorchArray(y_out)


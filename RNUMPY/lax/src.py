# src.py
# (c) Copyright 2024 Aerospace Research Community LLC

# Created:  Aug 2024 E. Botero
# Modified: 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------  

import RNUMPY as rp

j   = rp.jax_handle
np  = rp.numpy_handle
tr  = rp.torch_handle
jnp = j.numpy if j else None

def cond(pred, true_fun, false_fun, *operands, operand=object()):

    if rp.use_jax:return j.lax.cond(pred, true_fun, false_fun, *operands, operand=operand)
    else:
        if pred:
            return true_fun(*operands)
        else:
            return false_fun(*operands)
        
def scan(f,init,xs=None,length=None,reverse=False, unroll=1, _split_transpose=False):
    if rp.use_jax: return j.lax.scan(f,init,xs,length,reverse, unroll, _split_transpose)
    else: 
        if xs is None:
            xs = [None] * length
        carry = init
        ys = []
        for x in xs:
            carry, y = f(carry, x)
            ys.append(y)
        return carry, rp.stack(ys)
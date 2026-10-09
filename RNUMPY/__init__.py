# __init__.py
# (c) Copyright 2024 Aerospace Research Community LLC

# Created:  Aug 2024 E. Botero
# Modified: 

# ----------------------------------------------------------------------------------------------------------------------
#  Package Imports
# ----------------------------------------------------------------------------------------------------------------------  

import numpy as np
import builtins
import warnings
import inspect
import ast

# Set the handles to None initially to allow submodules to import RNUMPY
jax_handle   = None
numpy_handle = np
scipy_handle = None
torch_handle = None

try:
    import scipy as sp
    scipy_handle = sp
except ImportError:
    warnings.warn("Scipy is not installed.", ImportWarning)

try:
    import jax
    from jax import Array as jarray
    jax_handle = jax
except ImportError:
    warnings.warn("The optional package, JAX is not installed. Autograd and JIT are unavailable", ImportWarning)
    jax = None
    jarray = None

try:
    import torch
    from torch import Tensor as ttensor
    torch_handle = torch
except ImportError:
    warnings.warn("The optional package, PyTorch is not installed. Torch backend is unavailable", ImportWarning)
    torch = None
    ttensor = None

# Set the default environment
use_jax      = False
use_torch    = False
ensure_differentiable = True

# Set pi
pi = 3.141592653589793

# x64 toggle
x64_enabled = False

def enable_x64(enabled=True):
    global x64_enabled
    x64_enabled = enabled
    if torch_handle is not None:
        torch_handle.set_default_dtype(torch_handle.float64 if enabled else torch_handle.float32)
    if jax_handle is not None:
        jax_handle.config.update("jax_enable_x64", enabled)

# For JAX this is straight numpy
from numpy import inf, newaxis, nan

# Data Types
# These are handled dynamically via __getattr__ for backend compatibility
_DTYPE_MAP = {
    'float16': 'float16', 'float32': 'float32', 'float64': 'float64', 'double': 'float64',
    'int8': 'int8', 'int16': 'int16', 'int32': 'int32', 'int64': 'int64',
    'uint8': 'uint8', 'uint16': 'uint16', 'uint32': 'uint32', 'uint64': 'uint64',
    'bool_': 'bool', 'complex64': 'complex64', 'complex128': 'complex128'
}

_NUMPY_DTYPE_FALLBACKS = {
    'float16': np.float16, 'float32': np.float32, 'float64': np.float64, 'double': np.double,
    'int8': np.int8, 'int16': np.int16, 'int32': np.int32, 'int64': np.int64,
    'uint8': np.uint8, 'uint16': np.uint16, 'uint32': np.uint32, 'uint64': np.uint64,
    'bool_': np.bool_, 'complex64': np.complex64, 'complex128': np.complex128
}

def __getattr__(name):
    if name in _DTYPE_MAP:
        if use_torch and torch_handle:
            tr_name = _DTYPE_MAP[name]
            if builtins.hasattr(torch_handle, tr_name):
                 return builtins.getattr(torch_handle, tr_name)
        elif use_jax and jax_handle:
             # Jax reuses numpy dtypes effectively
             return _NUMPY_DTYPE_FALLBACKS[name]
        return _NUMPY_DTYPE_FALLBACKS[name]
    
    if name == 'float': return builtins.float
    if name == 'int': return builtins.int
    if name == 'bool': return builtins.bool
    if name == 'complex': return builtins.complex

    raise AttributeError(f"module '{__name__}' has no attribute '{name}'")

def __dir__():
     return sorted(set(globals().keys()) | set(_DTYPE_MAP.keys()) | {'int', 'float', 'bool', 'complex'})

# ----------------------------------------------------------------------------------------------------------------------
#  Basic Array Stuff
# ----------------------------------------------------------------------------------------------------------------------  

from ._basearrays import _set_array_base_attributes

def _get_obj_name(obj):
    if isinstance(obj, (int, float, str, bool, type(None))):
        return "x"
    try:
        frame = inspect.currentframe()
        while frame:
            if frame.f_code.co_name in ('__setitem__', '_inplace_error', '_get_obj_name', '_format_val', '<lambda>') or \
               frame.f_code.co_filename.endswith('RNUMPY/__init__.py'):
                frame = frame.f_back
                continue
            
            for name, val in frame.f_locals.items():
                if val is obj and name != 'self':
                    return name
            for name, val in frame.f_globals.items():
                if val is obj:
                    return name
            frame = frame.f_back
    except:
        pass
    return "x"

def _format_val(val):
    if isinstance(val, tuple):
        return "(" + ", ".join(_format_val(v) for v in val) + ")"
    
    name = _get_obj_name(val)
    if name != "x":
        return name
    
    # Special handling for slice
    if isinstance(val, slice):
        start = "" if val.start is None else str(val.start)
        stop = "" if val.stop is None else str(val.stop)
        step = "" if val.step is None else str(val.step)
        if val.step is None:
             return f"{start}:{stop}"
        return f"{start}:{stop}:{step}"

    s = str(val)
    if len(s) > 40:
        return s[:37] + "..."
    return s

class _RemoveMatchingSubscript(ast.NodeTransformer):
    def __init__(self, match_slice_str):
        self.match_slice_str = match_slice_str
    
    def visit_Subscript(self, node):
        self.generic_visit(node)
        try:
            if ast.unparse(node.slice) == self.match_slice_str:
                return node.value
        except:
            pass
        return node

def _get_source_exprs():
    try:
        frame = inspect.currentframe().f_back
        # Go back until we hit the user code
        while frame:
            if frame.f_code.co_name not in ('__setitem__', '__iadd__', '__isub__', '__imul__', '__itruediv__', '__ipow__', '_inplace_error', '_get_obj_name', '_format_val', '_get_source_exprs', '<lambda>') and \
               not frame.f_code.co_filename.endswith('RNUMPY/__init__.py'):
                break
            frame = frame.f_back
            
        if not frame:
            return None, None, None, None
            
        filename = frame.f_code.co_filename
        lineno = frame.f_lineno
        
        with open(filename, "r") as f:
            source = f.read()
        
        tree = ast.parse(source)
        for node in ast.walk(tree):
            if isinstance(node, (ast.Assign, ast.AugAssign)):
                if hasattr(node, "lineno") and hasattr(node, "end_lineno"):
                    if node.lineno <= lineno <= node.end_lineno:
                        if isinstance(node, ast.Assign):
                            target = node.targets[0]
                            if isinstance(target, ast.Subscript):
                                slice_node = target.slice
                                key_expr_str_with_parens = ast.unparse(slice_node)
                                if isinstance(slice_node, ast.Tuple):
                                    key_expr_str_no_parens = ", ".join(ast.unparse(e) for e in slice_node.elts)
                                else:
                                    key_expr_str_no_parens = key_expr_str_with_parens
                                
                                val_node = node.value
                                original_val_str = ast.unparse(val_node)
                                
                                transformer = _RemoveMatchingSubscript(key_expr_str_with_parens)
                                transformed_val_node = transformer.visit(val_node)
                                
                                return key_expr_str_no_parens, key_expr_str_with_parens, original_val_str, ast.unparse(transformed_val_node)
                        elif isinstance(node, ast.AugAssign):
                            target = node.target
                            if isinstance(target, ast.Subscript):
                                slice_node = target.slice
                                key_expr_str_with_parens = ast.unparse(slice_node)
                                if isinstance(slice_node, ast.Tuple):
                                    key_expr_str_no_parens = ", ".join(ast.unparse(e) for e in slice_node.elts)
                                else:
                                    key_expr_str_no_parens = key_expr_str_with_parens
                                
                                val_node = node.value
                                original_val_str = ast.unparse(val_node)
                                
                                transformer = _RemoveMatchingSubscript(key_expr_str_with_parens)
                                transformed_val_node = transformer.visit(val_node)
                                return key_expr_str_no_parens, key_expr_str_with_parens, original_val_str, ast.unparse(transformed_val_node)
                            return None, None, None, ast.unparse(node.value)
    except:
        pass
    return None, None, None, None

def _is_boolean_mask(obj):
    if type(obj) is bool:
        return True
    if hasattr(obj, 'dtype'):
        try:
            if obj.dtype in (bool, np.bool_):
                return True
            if torch_handle is not None and obj.dtype == torch_handle.bool:
                return True
        except:
            pass
    if isinstance(obj, list) and len(obj) > 0 and type(obj[0]) is bool:
        return True
    return False

def _contains_boolean_mask(obj):
    if _is_boolean_mask(obj):
        return True
    if isinstance(obj, tuple):
        return any(_contains_boolean_mask(i) for i in obj)
    return False

def _is_library_call():
    try:
        frame = inspect.currentframe().f_back
        while frame:
            filename = frame.f_code.co_filename
            if frame.f_code.co_name in ('__setitem__', '__iadd__', '__isub__', '__imul__', '__itruediv__', '__ipow__') or \
               filename.endswith('RNUMPY/__init__.py') or filename.endswith('RNUMPY/_basearrays.py'):
                frame = frame.f_back
                continue
            
            if 'site-packages' in filename or \
               'dist-packages' in filename or \
               'lib/python' in filename or \
               '/numpy/' in filename or \
               '/scipy/' in filename or \
               '/torch/' in filename or \
               '/jax/' in filename:
                return True
            return False
    except:
        pass
    return False

class Array():
    pass

if jax is not None:
    class JaxArray(Array, jarray):
        pass
else:
    class JaxArray(Array):
        pass

class NumpyArray(Array,np.ndarray):
    def __new__(cls, input_array, *args, **kwargs):
        # Convert input_array to an instance of MyArray
        obj = np.asarray(input_array).view(cls)
        return obj

    def __setitem__(self, key, value):
        if not ensure_differentiable or _is_library_call():
            return super().__setitem__(key, value)
        
        source_key_np, source_key_wp, orig_val_str, trans_val_str = _get_source_exprs()
        
        name = _get_obj_name(self)
        val_name_desc = orig_val_str if orig_val_str else _format_val(value)
        val_name_where = trans_val_str if trans_val_str else val_name_desc
        val_name_at = orig_val_str if orig_val_str else val_name_desc
        
        key_name_desc = source_key_np if source_key_np else _format_val(key)
        if not source_key_wp:
            key_name_wp = f"({_format_val(key)})" if isinstance(key, tuple) else _format_val(key)
        else:
            key_name_wp = source_key_wp
        
        is_simple_bool = _is_boolean_mask(key)
        is_any_bool = is_simple_bool or _contains_boolean_mask(key)

        if is_simple_bool:
             raise TypeError(
                f"RNUMPY: Inplace assignment {name}[{key_name_desc}] = {val_name_desc} with a boolean mask is not allowed in differentiable code. "
                f"Please use \n{name} = rp.where({key_name_wp}, {val_name_where}, {name})\ninstead."
            )
        else:
             msg = f"RNUMPY: Inplace assignment {name}[{key_name_desc}] = {val_name_desc} is not allowed in differentiable code. "
             if is_any_bool:
                  msg += f"Please use \n{name} = {name}.at[{key_name_desc}].set({val_name_at})\nor\n{name} = rp.where({key_name_wp}, {val_name_where}, {name})\ninstead."
             else:
                  msg += f"Please use \n{name} = {name}.at[{key_name_desc}].set({val_name_at})\ninstead."
             raise TypeError(msg)

    def __iadd__(self, other):
        if not ensure_differentiable or _is_library_call(): return super().__iadd__(other)
        _, _, _, source_val = _get_source_exprs()
        name = _get_obj_name(self)
        val_name = source_val if source_val else _format_val(other)
        raise TypeError(f"RNUMPY: Inplace operator {name} += {val_name} is not allowed in differentiable code. "
                        f"Please use {name} = {name} + {val_name} or {name}.at[...].add({val_name}) instead.")

    def __isub__(self, other):
        if not ensure_differentiable or _is_library_call(): return super().__isub__(other)
        _, _, _, source_val = _get_source_exprs()
        name = _get_obj_name(self)
        val_name = source_val if source_val else _format_val(other)
        raise TypeError(f"RNUMPY: Inplace operator {name} -= {val_name} is not allowed in differentiable code. "
                        f"Please use {name} = {name} - {val_name} or {name}.at[...].subtract({val_name}) instead.")

    def __imul__(self, other):
        if not ensure_differentiable or _is_library_call(): return super().__imul__(other)
        _, _, _, source_val = _get_source_exprs()
        name = _get_obj_name(self)
        val_name = source_val if source_val else _format_val(other)
        raise TypeError(f"RNUMPY: Inplace operator {name} *= {val_name} is not allowed in differentiable code. "
                        f"Please use {name} = {name} * {val_name} or {name}.at[...].multiply({val_name}) instead.")

    def __itruediv__(self, other):
        if not ensure_differentiable or _is_library_call(): return super().__itruediv__(other)
        _, _, _, source_val = _get_source_exprs()
        name = _get_obj_name(self)
        val_name = source_val if source_val else _format_val(other)
        raise TypeError(f"RNUMPY: Inplace operator {name} /= {val_name} is not allowed in differentiable code. "
                        f"Please use {name} = {name} / {val_name} or {name}.at[...].divide({val_name}) instead.")

    def __ipow__(self, other):
        if not ensure_differentiable or _is_library_call(): return super().__ipow__(other)
        _, _, _, source_val = _get_source_exprs()
        name = _get_obj_name(self)
        val_name = source_val if source_val else _format_val(other)
        raise TypeError(f"RNUMPY: Inplace operator {name} **= {val_name} is not allowed in differentiable code. "
                        f"Please use {name} = {name} ** {val_name} or {name}.at[...].power({val_name}) instead.")

if torch is not None:
    class TorchArray(Array, ttensor):
        def __new__(cls, x, *args, **kwargs):
            return torch.as_tensor(x, *args, **kwargs).as_subclass(cls)

        def __getitem__(self, index):
            # NumPy allows slicing 0-d arrays: scalar[:] -> scalar, scalar[:, None] -> (1, 1) scalar
            # PyTorch raises IndexError: too many indices for tensor of dimension 0
            if self.dim() == 0:
                if isinstance(index, slice) and index == builtins.slice(None):
                    return self
                elif isinstance(index, tuple):
                    # Check if all elements are either None (newaxis) or a full slice [:]
                    if builtins.all(i is None or (isinstance(i, builtins.slice) and i == builtins.slice(None)) for i in index):
                        res = self
                        for i in index:
                            if i is None:
                                res = res.unsqueeze(0)
                        return res.as_subclass(TorchArray)
            
            # Handle negative steps if torch version is old or for subclass compatibility
            # We check if any slice in the index has a negative step.
            has_neg_step = False
            if isinstance(index, slice):
                if index.step is not None and index.step < 0:
                    has_neg_step = True
            elif isinstance(index, tuple):
                if builtins.any(isinstance(i, slice) and getattr(i, 'step', None) is not None and i.step < 0 for i in index):
                    has_neg_step = True
            if has_neg_step:
                # Attempt native PyTorch negative slicing first
                try:
                    res = super().__getitem__(index)
                    if isinstance(res, ttensor) and not isinstance(res, TorchArray):
                        return res.as_subclass(TorchArray)
                    return res
                except (ValueError, RuntimeError, IndexError):
                    # Autograd-safe fallback using torch.flip for full reversals
                    index_tuple = index if isinstance(index, tuple) else (index,)
                    flip_dims = []
                    pos_index = []
                    
                    out_dim = 0
                    for i in index_tuple:
                        if i is None:
                            pos_index.append(i)
                            out_dim += 1
                        elif isinstance(i, int):
                            pos_index.append(i)
                            # Integer indices reduce the dimension count
                        else:
                            if isinstance(i, slice) and getattr(i, 'step', None) is not None and i.step < 0:
                                if i.step != -1 or i.start is not None or i.stop is not None:
                                    raise NotImplementedError(
                                        "TorchArray autograd fallback only supports full [::-1] reversals."
                                    )
                                flip_dims.append(out_dim)
                                pos_index.append(builtins.slice(None))
                            else:
                                pos_index.append(i)
                            out_dim += 1
                    
                    # Apply the positive slices
                    res = super().__getitem__(tuple(pos_index) if isinstance(index, tuple) else pos_index[0])
                    
                    # Apply the flips to the target output dimensions
                    if flip_dims:
                        res = torch.flip(res, dims=flip_dims)
                    
                    if isinstance(res, ttensor) and not isinstance(res, TorchArray):
                        return res.as_subclass(TorchArray)
                    return res

            res = super().__getitem__(index)
            if isinstance(res, ttensor) and not isinstance(res, TorchArray):
                return res.as_subclass(TorchArray)
            return res

        def __setitem__(self, key, value):
            if not ensure_differentiable or _is_library_call():
                return super().__setitem__(key, value)
            
            source_key_np, source_key_wp, orig_val_str, trans_val_str = _get_source_exprs()
            
            name = _get_obj_name(self)
            val_name_desc = orig_val_str if orig_val_str else _format_val(value)
            val_name_where = trans_val_str if trans_val_str else val_name_desc
            val_name_at = orig_val_str if orig_val_str else val_name_desc
            
            key_name_desc = source_key_np if source_key_np else _format_val(key)
            if not source_key_wp:
                key_name_wp = f"({_format_val(key)})" if isinstance(key, tuple) else _format_val(key)
            else:
                key_name_wp = source_key_wp
            
            is_simple_bool = _is_boolean_mask(key)
            is_any_bool = is_simple_bool or _contains_boolean_mask(key)

            if is_simple_bool:
                 raise TypeError(
                    f"RNUMPY: Inplace assignment {name}[{key_name_desc}] = {val_name_desc} with a boolean mask is not allowed in differentiable code. "
                    f"Please use \n{name} = rp.where({key_name_wp}, {val_name_where}, {name})\ninstead."
                )
            else:
                 msg = f"RNUMPY: Inplace assignment {name}[{key_name_desc}] = {val_name_desc} is not allowed in differentiable code. "
                 if is_any_bool:
                      msg += f"Please use \n{name} = {name}.at[{key_name_desc}].set({val_name_at})\nor\n{name} = rp.where({key_name_wp}, {val_name_where}, {name})\ninstead."
                 else:
                      msg += f"Please use \n{name} = {name}.at[{key_name_desc}].set({val_name_at})\ninstead."
                 raise TypeError(msg)

        def __iadd__(self, other):
            if not ensure_differentiable or _is_library_call(): return super().__iadd__(other)
            _, _, _, source_val = _get_source_exprs()
            name = _get_obj_name(self)
            val_name = source_val if source_val else _format_val(other)
            raise TypeError(f"RNUMPY: Inplace operator {name} += {val_name} is not allowed in differentiable code. "
                            f"Please use {name} = {name} + {val_name} or {name}.at[...].add({val_name}) instead.")

        def __isub__(self, other):
            if not ensure_differentiable or _is_library_call(): return super().__isub__(other)
            _, _, _, source_val = _get_source_exprs()
            name = _get_obj_name(self)
            val_name = source_val if source_val else _format_val(other)
            raise TypeError(f"RNUMPY: Inplace operator {name} -= {val_name} is not allowed in differentiable code. "
                            f"Please use {name} = {name} - {val_name} or {name}.at[...].subtract({val_name}) instead.")

        def __imul__(self, other):
            if not ensure_differentiable or _is_library_call(): return super().__imul__(other)
            _, _, _, source_val = _get_source_exprs()
            name = _get_obj_name(self)
            val_name = source_val if source_val else _format_val(other)
            raise TypeError(f"RNUMPY: Inplace operator {name} *= {val_name} is not allowed in differentiable code. "
                            f"Please use {name} = {name} * {val_name} or {name}.at[...].multiply({val_name}) instead.")

        def __itruediv__(self, other):
            if not ensure_differentiable or _is_library_call(): return super().__itruediv__(other)
            _, _, _, source_val = _get_source_exprs()
            name = _get_obj_name(self)
            val_name = source_val if source_val else _format_val(other)
            raise TypeError(f"RNUMPY: Inplace operator {name} /= {val_name} is not allowed in differentiable code. "
                            f"Please use {name} = {name} / {val_name} or {name}.at[...].divide({val_name}) instead.")

        def __ipow__(self, other):
            if not ensure_differentiable or _is_library_call(): return super().__ipow__(other)
            _, _, _, source_val = _get_source_exprs()
            name = _get_obj_name(self)
            val_name = source_val if source_val else _format_val(other)
            raise TypeError(f"RNUMPY: Inplace operator {name} **= {val_name} is not allowed in differentiable code. "
                            f"Please use {name} = {name} ** {val_name} or {name}.at[...].power({val_name}) instead.")

        def __deepcopy__(self, memo):
            return TorchArray(self.clone())

        @property
        def size(self):
            # NumPy uses .size as an attribute (int), but PyTorch uses .size() as a method (returns torch.Size).
            # To support both, we return a subclass of int that also implements __call__ to return the shape.
            class SizeInt(int):
                def __call__(self, *args, **kwargs):
                    return ttensor.size(self._obj, *args, **kwargs)
            res = SizeInt(self.numel())
            res._obj = self
            return res

        def new_empty(self, size, dtype=None, device=None, requires_grad=False):
            return ttensor.new_empty(self, size, dtype=dtype, device=device, requires_grad=requires_grad).as_subclass(TorchArray)

        # Reductions to match NumPy signature
        def max(self, axis=None, out=None, keepdims=False, **kwargs):
            dim = axis if axis is not None else kwargs.get('dim', None)
            if dim is None:
                res = super().max()
                if out is not None: out.copy_(res)
                if isinstance(res, ttensor) and not isinstance(res, TorchArray):
                    return res.as_subclass(TorchArray)
                return res
            res_tuple = torch.max(self, dim=dim, keepdim=bool(keepdims), out=out)
            return TorchArray(res_tuple[0] if out is not None else res_tuple[0])

        def min(self, axis=None, out=None, keepdims=False, **kwargs):
            dim = axis if axis is not None else kwargs.get('dim', None)
            if dim is None:
                res = super().min()
                if out is not None: out.copy_(res)
                if isinstance(res, ttensor) and not isinstance(res, TorchArray):
                    return res.as_subclass(TorchArray)
                return res
            res_tuple = torch.min(self, dim=dim, keepdim=bool(keepdims), out=out)
            return TorchArray(res_tuple[0] if out is not None else res_tuple[0])

        def sum(self, axis=None, out=None, keepdims=False, dtype=None, **kwargs):
            dim = axis if axis is not None else kwargs.get('dim', None)
            if dim is not None:
                return TorchArray(torch.sum(self, dim=dim, keepdim=bool(keepdims), dtype=dtype, out=out))
            
            # Global sum does not support keepdim in all torch versions
            res = torch.sum(self, dtype=dtype)
            if keepdims:
                for _ in range(self.dim()):
                    res = res.unsqueeze(0)
            
            if out is not None:
                out.copy_(res)
                
            return TorchArray(res)

        def mean(self, axis=None, out=None, keepdims=False, dtype=None, **kwargs):
            dim = axis if axis is not None else kwargs.get('dim', None)
            if dim is not None:
                 return TorchArray(torch.mean(self, dim=dim, keepdim=bool(keepdims), dtype=dtype, out=out))
            return TorchArray(torch.mean(self, keepdim=bool(keepdims), dtype=dtype, out=out))

        def std(self, axis=None, out=None, keepdims=False, unbiased=True, **kwargs):
            dim = axis if axis is not None else kwargs.get('dim', None)
            if dim is not None:
                 return TorchArray(torch.std(self, dim=dim, keepdim=bool(keepdims), unbiased=unbiased, out=out))
            return TorchArray(torch.std(self, keepdim=bool(keepdims), unbiased=unbiased, out=out))

        def var(self, axis=None, out=None, keepdims=False, unbiased=True, **kwargs):
            dim = axis if axis is not None else kwargs.get('dim', None)
            if dim is not None:
                 return TorchArray(torch.var(self, dim=dim, keepdim=bool(keepdims), unbiased=unbiased, out=out))
            return TorchArray(torch.var(self, keepdim=bool(keepdims), unbiased=unbiased, out=out))

        def all(self, axis=None, out=None, keepdims=False, **kwargs):
            dim = axis if axis is not None else kwargs.get('dim', None)
            if dim is not None:
                 return TorchArray(torch.all(self, dim=dim, keepdim=bool(keepdims), out=out))
            return TorchArray(torch.all(self))

        def any(self, axis=None, out=None, keepdims=False, **kwargs):
            dim = axis if axis is not None else kwargs.get('dim', None)
            if dim is not None:
                 return TorchArray(torch.any(self, dim=dim, keepdim=bool(keepdims), out=out))
            return TorchArray(torch.any(self))

        def ravel(self, order='C'):
            if order == 'C':
                return TorchArray(self.reshape(-1))
            elif order == 'F':
                return TorchArray(self.permute(tuple(reversed(range(self.dim())))).reshape(-1))
            elif order == 'A' or order == 'K':
                return TorchArray(self.reshape(-1))
            else:
                raise ValueError(f"order '{order}' not understood")

        def flatten(self, order='C'):
            return self.ravel(order=order)

        def __len__(self):
            if self.dim() == 0:
                return 1
            return super().__len__()

        def __array_wrap__(self, array, context=None):
            if isinstance(array, np.ndarray) and array.dtype == np.dtype('O'):
                return NumpyArray(array)
            res = super().__array_wrap__(array, context)
            if isinstance(res, ttensor) and not isinstance(res, TorchArray):
                return res.as_subclass(TorchArray)
            return res
else:
    class TorchArray(Array):
        pass

# Dynamically register JAX-style methods on the arrays
_set_array_base_attributes(NumpyArray, exclude={'__getitem__'})
_set_array_base_attributes(TorchArray, exclude={'__getitem__'})


# ----------------------------------------------------------------------------------------------------------------------
# Project Imports
# ----------------------------------------------------------------------------------------------------------------------  

# Finally import scripts
from .src import *
from .linalg import *
from .lax import *
from .autograd import *
from . import scipy
from . import random



# ----------------------------------------------------------------------------------------------------------------------
# Index updates on plain arrays (added for RCAIDE; not yet upstream)
# ----------------------------------------------------------------------------------------------------------------------  
from ._basearrays import _IndexUpdateHelper

def at(array):
    """Returns the index-update helper of any array: at(x)[idx].set(v) is x.at[idx].set(v) for a JAX array,
    and a modified copy of the same type for a numpy array or torch tensor."""
    if jarray is not None and isinstance(array, jarray):
        return array.at
    return _IndexUpdateHelper(array)

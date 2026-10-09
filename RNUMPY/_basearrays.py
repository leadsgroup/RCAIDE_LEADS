# _basearrays.py
# (c) Copyright 2024 Aerospace Research Community LLC

# Created:  Nov 2024 E. Botero
## Modified: Mar 2026, E. Botero

import numpy as np
try:
    import torch
except ImportError:
    torch = None

# ----------------------------------------------------------------------------------------------------------------------
#  Basic Array Setup, a la JAX
# ----------------------------------------------------------------------------------------------------------------------  

class _IndexUpdateHelper:
  def __init__(self, array):
    self.array = array

  def __getitem__(self, index):
    return _IndexUpdateRef(self.array, index)

  def __repr__(self):
    return f"_IndexUpdateHelper({self.array!r})"


class _IndexUpdateRef:

    __slots__ = ("array", "index")

    def __init__(self, array, index):
        self.array = array
        self.index = index

    def __repr__(self) -> str:
        return f"_IndexUpdateRef({self.array!r}, {self.index!r})"

    def get(self, *, indices_are_sorted=False, unique_indices=False,
            mode=None, fill_value=None):
        return self.array[self.index]

    def set(self, values, *, indices_are_sorted=False, unique_indices=False,
            mode=None):
        if torch is not None and isinstance(self.array, torch.Tensor):
            res = self.array.clone()
            torch.Tensor.__setitem__(res, self.index, values)
        else:
            res = self.array.copy()
            np.ndarray.__setitem__(res, self.index, values)
        return res

    def apply(self, func, *, indices_are_sorted=False, unique_indices=False,
                mode=None):
        raise NotImplementedError('Not Implemented')

    def add(self, values, *, indices_are_sorted=False, unique_indices=False,
            mode=None):
        if torch is not None and isinstance(self.array, torch.Tensor):
            res = self.array.clone()
            torch.Tensor.__setitem__(res, self.index, res[self.index] + values)
        else:
            res = self.array.copy()
            np.ndarray.__setitem__(res, self.index, res[self.index] + values)
        return res

    def subtract(self, values, *, indices_are_sorted=False, unique_indices=False,
                mode=None):
        if torch is not None and isinstance(self.array, torch.Tensor):
            res = self.array.clone()
            torch.Tensor.__setitem__(res, self.index, res[self.index] - values)
        else:
            res = self.array.copy()
            np.ndarray.__setitem__(res, self.index, res[self.index] - values)
        return res

    def multiply(self, values, *, indices_are_sorted=False, unique_indices=False,
                mode=None):
        if torch is not None and isinstance(self.array, torch.Tensor):
            res = self.array.clone()
            torch.Tensor.__setitem__(res, self.index, res[self.index] * values)
        else:
            res = self.array.copy()
            np.ndarray.__setitem__(res, self.index, res[self.index] * values)
        return res

    mul = multiply

    def divide(self, values, *, indices_are_sorted=False, unique_indices=False,
                mode=None):
        if torch is not None and isinstance(self.array, torch.Tensor):
            res = self.array.clone()
            torch.Tensor.__setitem__(res, self.index, res[self.index] / values)
        else:
            res = self.array.copy()
            np.ndarray.__setitem__(res, self.index, res[self.index] / values)
        return res

    def power(self, values, *, indices_are_sorted=False, unique_indices=False,
            mode=None):
        if torch is not None and isinstance(self.array, torch.Tensor):
            res = self.array.clone()
            torch.Tensor.__setitem__(res, self.index, res[self.index] ** values)
        else:
            res = self.array.copy()
            np.ndarray.__setitem__(res, self.index, res[self.index] ** values)
        return res

    def min(self, values, *, indices_are_sorted=False, unique_indices=False,
            mode=None):
        if torch is not None and isinstance(self.array, torch.Tensor):
            res = self.array.clone()
            torch.Tensor.__setitem__(res, self.index, torch.minimum(res[self.index], torch.as_tensor(values)))
        else:
            res = self.array.copy()
            np.ndarray.__setitem__(res, self.index, np.minimum(res[self.index], values))
        return res

    def max(self, values, *, indices_are_sorted=False, unique_indices=False,
            mode=None):
        if torch is not None and isinstance(self.array, torch.Tensor):
            res = self.array.clone()
            torch.Tensor.__setitem__(res, self.index, torch.maximum(res[self.index], torch.as_tensor(values)))
        else:
            res = self.array.copy()
            np.ndarray.__setitem__(res, self.index, np.maximum(res[self.index], values))
        return res
  

_array_operators = {}

_array_methods = {}

_impl_only_array_methods = {}

_array_properties = {
    "at": _IndexUpdateHelper,
}


def _set_array_base_attributes(array_impl, include=None, exclude=None):
  # Forward operators, methods, and properties on Array to lax_numpy
  # functions (with no Tracers involved; this forwarding is direct)
  def maybe_setattr(attr_name, target):
    if exclude is not None and attr_name in exclude:
      return
    if not include or attr_name in include:
      setattr(array_impl, attr_name, target)

  for operator_name, function in _array_operators.items():
    maybe_setattr(f"__{operator_name}__", function)
  for method_name, method in _array_methods.items():
    maybe_setattr(method_name, method)
  for prop_name, prop in _array_properties.items():
    maybe_setattr(prop_name, property(prop))

  for name, func in _impl_only_array_methods.items():
    setattr(array_impl, name, func)

# autograd.py
# (c) Copyright 2024 Aerospace Research Community LLC

import functools
import RNUMPY as rp

def _to_array(x):
    if rp.use_jax:
        import jax.numpy as jnp
        if not isinstance(x, (rp.JaxArray, jnp.ndarray)):
            return rp.array(x)
    elif rp.use_torch:
        import torch
        if not isinstance(x, (rp.TorchArray, torch.Tensor)):
            return rp.array(x)
    else:
        if not isinstance(x, (rp.NumpyArray, rp.numpy_handle.ndarray)):
            return rp.array(x)
    return x

def _ensure_backend_args(args):
    return tuple(_to_array(arg) for arg in args)

def _torch_collect_argnums(argnums):
    if isinstance(argnums, int):
        return [argnums]
    return list(argnums)

def _torch_prepare_args(args, argnums):
    import torch

    args = list(args)
    diff_indices = _torch_collect_argnums(argnums)
    tracked = []
    for i in diff_indices:
        a = args[i]
        if isinstance(a, torch.Tensor):
            if not a.requires_grad:
                a = a.detach().requires_grad_(True)
                args[i] = a
            tracked.append(a)
    return args, tracked, diff_indices

def _torch_scalarize(val):
    import torch
    if isinstance(val, torch.Tensor) and val.ndim > 0:
        return val.sum()
    return val

def _torch_grad_impl(f, argnums, args, kwargs):
    import torch

    args, tracked, diff_indices = _torch_prepare_args(args, argnums)
    if not tracked:
        raise ValueError("No torch.Tensor arguments selected for differentiation.")
    val = _torch_scalarize(f(*args, **kwargs))
    grads = torch.autograd.grad(val, tracked, allow_unused=True)
    if isinstance(argnums, int):
        return grads[0]
    return tuple(grads)

def _torch_value_and_grad_impl(f, argnums, args, kwargs):
    import torch

    args, tracked, diff_indices = _torch_prepare_args(args, argnums)
    if not tracked:
        raise ValueError("No torch.Tensor arguments selected for differentiation.")
    val = f(*args, **kwargs)
    loss = _torch_scalarize(val)
    grads = torch.autograd.grad(loss, tracked, allow_unused=True)
    if isinstance(argnums, int):
        return val, grads[0]
    return val, tuple(grads)

def _finite_diff_grad(f, argnums, args, kwargs, eps=1e-6):
    import numpy as np

    if isinstance(argnums, int):
        target_indices = [argnums]
        single_arg = True
    else:
        target_indices = argnums
        single_arg = False

    grads = []
    for arg_idx in target_indices:
        x = np.asarray(_to_array(args[arg_idx]), dtype=float).copy()
        g = np.zeros_like(x)

        x_flat = x.ravel()
        g_flat = g.ravel()

        for i in range(x_flat.size):
            orig_val = x_flat[i]

            xp = x_flat.copy()
            xp[i] = orig_val + eps
            args_plus = list(args)
            args_plus[arg_idx] = rp.array(xp.reshape(x.shape))
            y_plus = f(*args_plus, **kwargs)

            xm = x_flat.copy()
            xm[i] = orig_val - eps
            args_minus = list(args)
            args_minus[arg_idx] = rp.array(xm.reshape(x.shape))
            y_minus = f(*args_minus, **kwargs)

            g_flat[i] = (np.asarray(y_plus) - np.asarray(y_minus)) / (2 * eps)

        grads.append(rp.array(g.reshape(x.shape)))

    if single_arg:
        return grads[0]
    return tuple(grads)

def _finite_diff_jac(f, argnums, args, kwargs, eps=1e-6):
    import numpy as np

    if isinstance(argnums, int):
        target_indices = [argnums]
        single_arg = True
    else:
        target_indices = argnums
        single_arg = False

    jacs = []
    for arg_idx in target_indices:
        x = np.asarray(_to_array(args[arg_idx]), dtype=float).copy()
        y0 = f(*args, **kwargs)
        y0_flat = np.asarray(y0).ravel()

        jac = np.zeros((y0_flat.size, x.size))
        x_flat = x.ravel()

        for i in range(x_flat.size):
            orig_val = x_flat[i]

            xp = x_flat.copy()
            xp[i] = orig_val + eps
            args_plus = list(args)
            args_plus[arg_idx] = rp.array(xp.reshape(x.shape))
            y_plus = np.asarray(f(*args_plus, **kwargs)).ravel()

            xm = x_flat.copy()
            xm[i] = orig_val - eps
            args_minus = list(args)
            args_minus[arg_idx] = rp.array(xm.reshape(x.shape))
            y_minus = np.asarray(f(*args_minus, **kwargs)).ravel()

            jac[:, i] = (y_plus - y_minus) / (2 * eps)

        y_shape = np.asarray(y0).shape
        x_shape = x.shape
        full_jac_shape = y_shape + x_shape
        jacs.append(rp.array(jac.reshape(full_jac_shape)))

    if single_arg:
        return jacs[0]
    return tuple(jacs)


def grad(f, argnums=0, has_aux=False):
    def grad_wrapper(*args, **kwargs):
        args = _ensure_backend_args(args)
        if rp.use_jax:
            import jax
            return jax.grad(f, argnums=argnums, has_aux=has_aux)(*args, **kwargs)
        elif rp.use_torch:
            if has_aux:
                raise NotImplementedError("has_aux=True is not supported in Torch mode for RNUMPY.grad yet.")
            return _torch_grad_impl(f, argnums, args, kwargs)
        else:
            if has_aux:
                raise NotImplementedError("has_aux=True is not supported in NumPy finite difference mode.")
            return _finite_diff_grad(f, argnums, args, kwargs)
    return grad_wrapper

def value_and_grad(f, argnums=0, has_aux=False):
    def v_and_g_wrapper(*args, **kwargs):
        args = _ensure_backend_args(args)
        if rp.use_jax:
            import jax
            return jax.value_and_grad(f, argnums=argnums, has_aux=has_aux)(*args, **kwargs)
        elif rp.use_torch:
            if has_aux:
                raise NotImplementedError("has_aux=True is not supported in Torch mode for RNUMPY.value_and_grad yet.")
            return _torch_value_and_grad_impl(f, argnums, args, kwargs)
        else:
            if has_aux:
                raise NotImplementedError("has_aux=True is not supported in NumPy finite difference mode.")
            val = f(*args, **kwargs)
            g = _finite_diff_grad(f, argnums, args, kwargs)
            return val, g
    return v_and_g_wrapper

def jacfwd(f, argnums=0, has_aux=False):
    def jacfwd_wrapper(*args, **kwargs):
        args = _ensure_backend_args(args)
        if rp.use_jax:
            import jax
            return jax.jacfwd(f, argnums=argnums, has_aux=has_aux)(*args, **kwargs)
        elif rp.use_torch:
            try:
                from torch.func import jacfwd as tjacfwd
                return tjacfwd(f, argnums=argnums, has_aux=has_aux)(*args, **kwargs)
            except ImportError:
                raise ImportError("torch.func is required for RNUMPY.jacfwd in Torch mode.")
        else:
            if has_aux:
                raise NotImplementedError("has_aux=True is not supported in NumPy finite difference mode.")
            return _finite_diff_jac(f, argnums, args, kwargs)
    return jacfwd_wrapper

def jacrev(f, argnums=0, has_aux=False):
    def jacrev_wrapper(*args, **kwargs):
        args = _ensure_backend_args(args)
        if rp.use_jax:
            import jax
            return jax.jacrev(f, argnums=argnums, has_aux=has_aux)(*args, **kwargs)
        elif rp.use_torch:
            try:
                from torch.func import jacrev as tjacrev
                return tjacrev(f, argnums=argnums, has_aux=has_aux)(*args, **kwargs)
            except ImportError:
                raise ImportError("torch.func is required for RNUMPY.jacrev in Torch mode.")
        else:
            if has_aux:
                raise NotImplementedError("has_aux=True is not supported in NumPy finite difference mode.")
            return _finite_diff_jac(f, argnums, args, kwargs)
    return jacrev_wrapper

def jacobian(f, argnums=0, has_aux=False):
    # Default to jacrev as it's often more efficient for many outputs
    return jacrev(f, argnums=argnums, has_aux=has_aux)

def hessian(f, argnums=0, has_aux=False):
    def hessian_wrapper(*args, **kwargs):
        args = _ensure_backend_args(args)
        if rp.use_jax:
            import jax
            return jax.hessian(f, argnums=argnums, has_aux=has_aux)(*args, **kwargs)
        elif rp.use_torch:
            try:
                from torch.func import hessian as thessian
                # torch.func.hessian does not accept has_aux
                return thessian(f, argnums=argnums)(*args, **kwargs)
            except ImportError:
                raise ImportError("torch.func is required for RNUMPY.hessian in Torch mode.")
        else:
            raise NotImplementedError("Hessian is not implemented for NumPy mode yet.")
    return hessian_wrapper

def jvp(f, primals, tangents):
    primals = _ensure_backend_args(primals)
    tangents = _ensure_backend_args(tangents)
    if rp.use_jax:
        import jax
        return jax.jvp(f, primals, tangents)
    elif rp.use_torch:
        try:
            from torch.func import jvp as tjvp
            return tjvp(f, primals, tangents)
        except ImportError:
            raise ImportError("torch.func is required for RNUMPY.jvp in Torch mode.")
    else:
        # jvp(f, (x,), (v,)) = grad(f)(x) @ v
        # For simplicity, implement via finite difference: (f(x + eps*v) - f(x - eps*v)) / (2*eps)
        eps = 1e-6
        
        args_plus = []
        args_minus = []
        for p, t in zip(primals, tangents):
            args_plus.append(p + eps * t)
            args_minus.append(p - eps * t)
            
        y_plus = f(*args_plus)
        y_minus = f(*args_minus)
        
        y0 = f(*primals)
        return y0, (y_plus - y_minus) / (2 * eps)

def vjp(f, *primals, has_aux=False):
    primals = _ensure_backend_args(primals)
    if rp.use_jax:
        import jax
        return jax.vjp(f, *primals, has_aux=has_aux)
    elif rp.use_torch:
        try:
            from torch.func import vjp as tvjp
            return tvjp(f, *primals, has_aux=has_aux)
        except ImportError:
            raise ImportError("torch.func is required for RNUMPY.vjp in Torch mode.")
    else:
        # vjp is harder to implement generically via finite differences without full jacobian
        # but JAX's vjp returns (y, vjp_fun)
        y0 = f(*primals)
        def vjp_fun(v):
            # grad(v^T f(x))
            def scalar_fun(*args):
                return rp.dot(v, f(*args))
            return grad(scalar_fun)(*primals),
            
        if has_aux:
             # JAX vjp behavior with has_aux returning (y, aux, vjp_fun)
             if isinstance(y0, tuple) and len(y0) == 2:
                  return y0[0], y0[1], vjp_fun
             return y0, None, vjp_fun # Fallback
        return y0, vjp_fun


def jit(f=None, *, static_argnums=None, **kwargs):
    """JIT-compile a function for the active backend.

    Uses lazy initialization: the function is compiled exactly once per backend
    configuration and that handle is cached for all subsequent calls. If the
    active backend changes at runtime (e.g. NumPy -> JAX), a single
    recompilation is triggered automatically.

    Supports both bare and parameterized decorator syntax::

        @rp.jit
        def func(x): ...

        @rp.jit(static_argnums=(1,))
        def func(x, static_param): ...

    Parameters
    ----------
    static_argnums : int or tuple of int, optional
        Indices of arguments treated as compile-time constants (JAX only).
        Silently ignored for Torch and NumPy backends.
    **kwargs
        Additional keyword arguments forwarded to the backend compiler,
        e.g. ``mode`` or ``backend`` for ``torch.compile``.
    """
    def decorator(func):
        _compiled   = None
        _last_state = None

        @functools.wraps(func)
        def wrapper(*args, **kw):
            nonlocal _compiled, _last_state

            current_state = (rp.use_jax, rp.use_torch)

            # Compile exactly once per backend state; recompile only if toggled.
            if _compiled is None or _last_state != current_state:
                _last_state = current_state

                if rp.use_jax:
                    import jax
                    _compiled = jax.jit(func, static_argnums=static_argnums, **kwargs)

                elif rp.use_torch:
                    # Dynamo compiling is filled with issues here
                    _compiled = func

                else:
                    # NumPy has no JIT; act as a transparent passthrough
                    _compiled = func

            return _compiled(*args, **kw)

        return wrapper

    # Allow both @rp.jit and @rp.jit(static_argnums=...)
    if f is not None:
        return decorator(f)
    return decorator

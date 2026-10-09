# optimize.py
# (c) Copyright 2024 Aerospace Research Community LLC

# Created:  Oct 2024 M. Clarke
# Modified: March 2026 E. Botero (Add fmin_slsqp with gradients)

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------  

import RNUMPY as rp

j   = rp.jax_handle
np  = rp.numpy_handle
sp  = rp.scipy_handle
tr  = rp.torch_handle

jo = None
if j is not None:
    try:
        import jax.scipy.optimize as jso
        jo = jso
    except ImportError:
        pass

so  = sp.optimize if sp is not None else None

from ._numpy_bridge import finite_diff_jacobian, to_numpy

_torch_aux_store = {}
_MISSING = object()

def _jax_concrete_args(args):
    import jax
    return jax.tree_util.tree_map(jax.lax.stop_gradient, args)

def _detach_torch_args(args_tuple):
    found = _find_tensors(args_tuple)
    if not found:
        return args_tuple
    return _replace_tensors(args_tuple, [p.detach() for p in found], {'idx': 0})

def _as_1d_numpy(x):
    """Flatten a real-valued array-like to a 1-D NumPy vector for SciPy.

    Complex residuals are not supported: SciPy ``hybrj`` and this wrapper
    operate in real floating point.
    """
    return np.asarray(to_numpy(x), dtype=float).reshape(-1)


def _require_fsolve_info(meta, backend):
    info = meta.get("info")
    if info is None:
        raise RuntimeError(
            f"RNUMPY fsolve: {backend} full_output requested but solver metadata "
            "was not recorded. This happens if the host solve did not run in this "
            "Python call (for example under JAX jit/vmap)."
        )
    extras = meta.get("n_solves", 1)
    if extras > 1:
        raise RuntimeError(
            "RNUMPY fsolve: full_output is not supported when the host solve ran "
            "more than once in this call (batched/vmap). Request the solution "
            "array only, or call fsolve on one problem at a time."
        )
    return info

def _copy_fsolve_info(infodict, ier, mesg):
    info = {}
    for key, value in dict(infodict).items():
        if isinstance(value, np.ndarray):
            info[key] = np.array(value, copy=True)
        else:
            info[key] = value
    return info, int(ier), str(mesg)

def _torch_ad_jacobian(func, x_np, args_tuple, dtype, device):
    """Jacobian of a Torch residual on a fresh local graph, then discarded."""
    import torch as tr

    detached_args = _detach_torch_args(args_tuple)
    x = tr.tensor(_as_1d_numpy(x_np), dtype=dtype, device=device)

    def objective(x_v):
        residual = func(rp.array(x_v, dtype=dtype, device=device), *detached_args)
        return tr.as_tensor(residual, dtype=dtype, device=device).reshape(-1)

    with tr.enable_grad():
        jac = tr.autograd.functional.jacobian(objective, x.detach(), create_graph=False, strict=False)
    n_x = int(x.numel())
    n_res = int(objective(x.detach()).numel())
    return np.asarray(jac.detach().cpu().numpy(), dtype=float).reshape(n_res, n_x)

def _torch_ift_param_grads(func, args_template, params_in, grad_x, dt, x_sol_np):
    """Implicit-function-theorem parameter gradients on a local Torch graph.

    The solve itself is not on this graph. Inputs are detached, so an outer
    optimizer that calls this backward does not keep the residual tape.
    """
    import torch as tr

    if grad_x is None:
        return [None] * len(params_in)
    if x_sol_np is None:
        raise RuntimeError("RNUMPY fsolve: missing concrete solution in backward context")

    device = next((p.device for p in params_in if hasattr(p, "device")), None)
    gx = tr.as_tensor(grad_x.detach(), dtype=dt, device=device).reshape(-1)
    detached_saved = [p.detach() for p in params_in]
    args_detached = _replace_tensors(args_template, detached_saved, {'idx': 0})
    jac_np = _torch_ad_jacobian(func, x_sol_np, args_detached, dt, device)
    jac = tr.tensor(jac_np, dtype=dt, device=device)
    try:
        lam = tr.linalg.solve(jac.mT, gx)
    except RuntimeError:
        lam = tr.linalg.lstsq(jac.mT, gx.unsqueeze(-1)).solution.reshape(-1)

    live = []
    rebuilt = []
    indices = []
    for i, p in enumerate(params_in):
        if getattr(p, "requires_grad", False):
            leaf = p.detach().requires_grad_(True)
            live.append(leaf)
            rebuilt.append(leaf)
            indices.append(i)
        else:
            rebuilt.append(p.detach())

    grad_params = [None] * len(params_in)
    if not live:
        return grad_params

    with tr.enable_grad():
        x = tr.tensor(_as_1d_numpy(x_sol_np), dtype=dt, device=device)
        current_args = _replace_tensors(args_template, rebuilt, {'idx': 0})
        residual = func(rp.array(x, dtype=dt, device=device), *current_args)
        residual = tr.as_tensor(residual, dtype=dt, device=device).reshape(-1)
        sensitivity = tr.dot(-lam.detach(), residual)
        if sensitivity.requires_grad:
            grads = tr.autograd.grad(sensitivity, live, allow_unused=True)
        else:
            grads = (None,) * len(live)

    for i, g in zip(indices, grads):
        grad_params[i] = tr.zeros_like(params_in[i]) if g is None else g.detach()
    return grad_params

def _build_fsolve_scipy_callbacks(func, args_tuple, fprime=None, dtype=None, device=None):
    """NumPy callbacks for SciPy's fsolve.

    JAX and Torch Jacobians are automatic derivatives of the residual.
    Those graphs are local to the callback: values are detached before they
    are handed back to SciPy, so an outer optimizer cannot tape the iterations.
    NumPy mode has no AD and keeps a central difference.
    """
    if rp.use_jax:
        import jax
        import jax.numpy as jnp
        concrete_args = _jax_concrete_args(args_tuple)

        def fun_np(x_v):
            residual = func(rp.array(_as_1d_numpy(x_v)), *concrete_args)
            return np.asarray(jax.device_get(residual), dtype=float).reshape(-1)

        if fprime is None:
            def objective(x_v):
                return jnp.ravel(func(rp.array(jnp.ravel(x_v)), *concrete_args))

            jac_fn = jax.jit(jax.jacobian(objective))

            def fprime_np(x_v):
                jac = jac_fn(jnp.asarray(_as_1d_numpy(x_v)))
                return np.asarray(jax.device_get(jac), dtype=float)
        else:
            def fprime_np(x_v):
                residual = fprime(rp.array(_as_1d_numpy(x_v)), *concrete_args)
                return np.asarray(jax.device_get(residual), dtype=float)
        return fun_np, fprime_np

    if rp.use_torch:
        import torch as tr
        detached_args = _detach_torch_args(args_tuple)

        def fun_np(x_v):
            with tr.no_grad():
                residual = func(
                    rp.array(_as_1d_numpy(x_v), dtype=dtype, device=device),
                    *detached_args,
                )
            return _as_1d_numpy(residual)

        if fprime is None:
            def fprime_np(x_v):
                return _torch_ad_jacobian(func, x_v, detached_args, dtype, device)
        else:
            def fprime_np(x_v):
                with tr.no_grad():
                    residual = fprime(
                        rp.array(_as_1d_numpy(x_v), dtype=dtype, device=device),
                        *detached_args,
                    )
                return np.asarray(to_numpy(residual), dtype=float)
        return fun_np, fprime_np

    def fun_np(x_v):
        residual = func(rp.array(_as_1d_numpy(x_v)), *args_tuple)
        return _as_1d_numpy(residual)

    if fprime is None:
        def fprime_np(x_v):
            return finite_diff_jacobian(fun_np, _as_1d_numpy(x_v))
    else:
        def fprime_np(x_v):
            return np.asarray(to_numpy(fprime(rp.array(_as_1d_numpy(x_v)), *args_tuple)), dtype=float)
    return fun_np, fprime_np

# ----------------------------------------------------------------------------------------------------------------------
#  Functions
# ----------------------------------------------------------------------------------------------------------------------  

def minimize(fun, x0, args=(), *, method='BFGS', bounds=None, constraints=(), tol=None, options=None): 
    if not isinstance(args, tuple):
        args = (args,)
    if rp.use_jax: 
        if bounds is not None:
             raise NotImplementedError('bounds are not supported for minimize in JAX')
        if constraints and len(constraints) > 0:
             raise NotImplementedError('constraints are not supported for minimize in JAX')
        return jo.minimize(fun, x0, args=args, method=method, tol=tol, options=options)
    elif rp.use_torch: 
        import torch as tr
        
        # Normalize constraints
        cons_list = []
        if isinstance(constraints, dict):
            cons_list = [constraints]
        elif isinstance(constraints, (list, tuple)):
            cons_list = list(constraints)
        
        for c in cons_list:
            if 'args' in c and not isinstance(c['args'], tuple):
                c['args'] = (c['args'],)

        # Collect all tensors from args, bounds, and constraints
        all_inputs = [args, bounds, cons_list]
        params = _find_tensors(all_inputs)
        dt = x0.dtype if hasattr(x0, 'dtype') else (params[0].dtype if params else tr.get_default_dtype())

        class Minimize(tr.autograd.Function):
            @staticmethod
            def forward(x0_in, *params_in):

                # Sever the trailing graph for the inner solver loop
                detached_params_in = [p.detach() if hasattr(p, 'detach') else p for p in params_in]
                
                def get_current_inputs(p_in):
                    return _replace_tensors(all_inputs, p_in, {'idx': 0})
                
                def fun_np(x_val):
                    curr_args, curr_bounds, curr_cons = get_current_inputs(detached_params_in)
                    with tr.no_grad(): # Kills the trailing graph during line search
                        res_val = fun(rp.array(x_val, dtype=dt), *curr_args)
                    return tr.as_tensor(res_val).detach().cpu().numpy()

                def jac_np(x_val):
                    curr_args, _, _ = get_current_inputs(detached_params_in)
                    with tr.enable_grad():
                        x_tr = tr.tensor(x_val, dtype=dt, requires_grad=True)
                        obj_val = tr.as_tensor(fun(rp.array(x_tr, dtype=dt), *curr_args))
                        grad = tr.autograd.grad(obj_val, x_tr)[0]
                    return grad.detach().cpu().numpy()

                so_cons = []
                curr_all = get_current_inputs(detached_params_in)
                curr_cons_list = curr_all[2]
                
                for c_idx, c in enumerate(curr_cons_list):
                    def c_np(x_v, idx=c_idx):
                        c_curr_all = get_current_inputs(detached_params_in)
                        c_curr = c_curr_all[2][idx]
                        c_args = c_curr.get('args', ())
                        with tr.no_grad(): # Kills the trailing graph during line search
                            val = c_curr['fun'](rp.array(x_v, dtype=dt), *c_args)
                        return tr.as_tensor(val).detach().cpu().numpy()
                    def c_jac_np(x_v, idx=c_idx):
                        c_curr_all = get_current_inputs(detached_params_in)
                        c_curr = c_curr_all[2][idx]
                        c_args = c_curr.get('args', ())
                        def c_fun_pure(x_t):
                            return tr.as_tensor(c_curr['fun'](rp.array(x_t, dtype=dt), *c_args))
                        with tr.enable_grad():
                            J = tr.autograd.functional.jacobian(c_fun_pure, tr.tensor(x_v, dtype=dt))
                        return J.detach().cpu().numpy()

                    so_cons.append({'type': c['type'], 'fun': c_np, 'jac': c_jac_np})

                curr_bounds = curr_all[1]
                res = so.minimize(
                    fun_np, 
                    tr.as_tensor(x0_in).detach().cpu().numpy(),
                    method=method, 
                    jac=jac_np,          
                    bounds=curr_bounds, 
                    constraints=so_cons, 
                    tol=tol, 
                    options=options
                )
                x_sol = tr.as_tensor(res.x, dtype=dt)
                
                # Identify Active Constraints and Multipliers
                multipliers = getattr(res, 'multipliers', None)
                active_cons_indices = []
                active_multipliers = []
                
                if multipliers is not None:
                    meq = sum(1 for c in cons_list if c['type'] == 'eq')
                    eq_mults = multipliers[:meq]
                    ineq_mults = multipliers[meq:]
                    ei, ii = 0, 0
                    for c_idx, c in enumerate(cons_list):
                        if c['type'] == 'eq':
                            active_cons_indices.append(c_idx)
                            active_multipliers.append(float(eq_mults[ei]))
                            ei += 1
                        else:
                            if ineq_mults[ii] > 1e-8:
                                active_cons_indices.append(c_idx)
                                active_multipliers.append(float(ineq_mults[ii]))
                            ii += 1
                else:
                    # Fallback identification by value
                    curr_all_fb = get_current_inputs(params_in)
                    curr_cons_fb = curr_all_fb[2]
                    for c_idx, c in enumerate(curr_cons_fb):
                        c_args = c.get('args', ())
                        val = c['fun'](rp.array(res.x, dtype=dt), *c_args)
                        if c['type'] == 'eq' or abs(val) < 1e-6:
                            active_cons_indices.append(c_idx)
                            active_multipliers.append(1.0) # Dummy

                active_bounds = []
                if bounds is not None:
                    res_x_np = res.x
                    for i, (l, u) in enumerate(bounds):
                        if l is not None and abs(res_x_np[i] - l) < 1e-7:
                            active_bounds.append((i, float(l), -1.0))
                        elif u is not None and abs(res_x_np[i] - u) < 1e-7:
                            active_bounds.append((i, float(u), 1.0))

                ctx_active_info = (active_cons_indices, tr.tensor(active_multipliers, dtype=dt), active_bounds)
                _torch_aux_store[id(x_sol)] = ctx_active_info
                return x_sol

            @staticmethod
            def setup_context(ctx, inputs, output):
                x0_in, *params_in = inputs
                ctx.save_for_backward(output, *params_in)
                ctx.active_info = _torch_aux_store.pop(id(output), ([], tr.tensor([]), []))

            @staticmethod
            def backward(ctx, grad_x):
                x_sol = ctx.saved_tensors[0]
                params_in = ctx.saved_tensors[1:]
                active_cons_indices, active_multipliers, active_bounds = ctx.active_info
                
                def get_inputs(p_in):
                    return _replace_tensors(all_inputs, p_in, {'idx': 0})

                with tr.enable_grad():
                    diff_params = [p.detach().requires_grad_(True) for p in params_in]
                    x = x_sol.detach().requires_grad_(True)
                    
                    def eval_lagrangian(x_v, p_v):
                         curr_args, curr_bounds, curr_cons_list = get_inputs(p_v)
                         f_v = fun(rp.array(x_v, dtype=dt), *curr_args)
                         L_val = tr.as_tensor(f_v).sum()
                         
                         for m_idx, c_idx in enumerate(active_cons_indices):
                              c_curr = curr_cons_list[c_idx]
                              c_a = c_curr.get('args', ())
                              cv = c_curr['fun'](rp.array(x_v, dtype=dt), *c_a)
                              L_val = L_val - active_multipliers[m_idx] * tr.as_tensor(cv).sum()
                         
                         for b_i, b_v, b_d in active_bounds:
                              m_b = active_multipliers[len(active_cons_indices) + active_bounds.index((b_i, b_v, b_d))] if len(active_multipliers) > len(active_cons_indices) else 1.0
                              if b_d == -1.0: L_val = L_val - m_b * (x_v[b_i] - b_v)
                              else: L_val = L_val - m_b * (b_v - x_v[b_i])
                         return L_val

                    c_active_vals = []
                    curr_args, curr_bounds, curr_cons_list = get_inputs(diff_params)
                    for c_idx in active_cons_indices:
                        c_curr = curr_cons_list[c_idx]
                        c_a = c_curr.get('args', ())
                        cv = c_curr['fun'](rp.array(x, dtype=dt), *c_a)
                        c_active_vals.append(tr.as_tensor(cv).sum())
                    for b_i, b_v, b_d in active_bounds:
                        if b_d == -1.0: c_active_vals.append(x[b_i] - b_v)
                        else: c_active_vals.append(b_v - x[b_i])

                    def grad_L_x_pure_x(x_v):
                         return tr.autograd.grad(eval_lagrangian(x_v, diff_params), x_v, create_graph=True)[0]
                    H = tr.autograd.functional.jacobian(grad_L_x_pure_x, x)
                    
                    if len(c_active_vals) > 0:
                        def c_stack_pure_x(x_v):
                             ga, gb, gc_list = get_inputs(diff_params)
                             cvs = []
                             for ci in active_cons_indices:
                                  cc = gc_list[ci]
                                  ca = cc.get('args', ())
                                  cvs.append(tr.as_tensor(cc['fun'](rp.array(x_v, dtype=dt), *ca)).sum())
                             for bi, bv, bd in active_bounds:
                                  if bd == -1.0: cvs.append(x_v[bi] - bv)
                                  else: cvs.append(bv - x_v[bi])
                             return tr.stack(cvs)
                        A = tr.autograd.functional.jacobian(c_stack_pure_x, x)
                    else:
                        A = tr.zeros((0, x.shape[0]), dtype=dt)

                n_x, n_c = x.shape[0], A.shape[0]
                KKT = tr.zeros((n_x + n_c, n_x + n_c), dtype=dt)
                KKT[:n_x, :n_x] = H.detach()
                KKT[:n_x, n_x:] = A.detach().T
                KKT[n_x:, :n_x] = A.detach()
                rhs = tr.zeros(n_x + n_c, dtype=dt)
                rhs[:n_x] = grad_x.detach()
                try:    sol = tr.linalg.solve(KKT, rhs.unsqueeze(-1)).squeeze(-1)
                except: sol = tr.linalg.lstsq(KKT, rhs.unsqueeze(-1)).solution.squeeze(-1)
                v_x  = sol[:n_x].detach()
                v_lam = sol[n_x:].detach()

                # IFT parameter gradient:
                #   dL_loss/dp = -v_x . d(grad_L_x)/dp  -  v_lam . d(c_active)/dp
                # where x is fixed at x* and we differentiate w.r.t. parameters p only.
                # We build a scalar sensitivity = v_x . grad_L_x(x*, p) + v_lam . c(x*, p)
                # and take -d(sensitivity)/dp.
                grad_params = [None] * len(params_in)
                params_req_grad = [p for p in params_in if p.requires_grad]
                params_indices  = [i for i, p in enumerate(params_in) if p.requires_grad]

                if params_req_grad:
                    with tr.enable_grad():
                        dp = [p.detach().requires_grad_(p.requires_grad) for p in params_in]
                        x_star = x_sol.detach().requires_grad_(True)
                        L_xp = eval_lagrangian(x_star, dp)
                        gL_x = tr.autograd.grad(L_xp, x_star, create_graph=True)[0]
                        sensitivity = (v_x * gL_x).sum()

                        if v_lam.numel() > 0:
                            _, _, c_list_dp = get_inputs(dp)
                            for m_i, c_idx in enumerate(active_cons_indices):
                                cc = c_list_dp[c_idx]
                                ca = cc.get('args', ())
                                cv = tr.as_tensor(cc['fun'](rp.array(x_sol.detach(), dtype=dt), *ca), dtype=dt).reshape([])
                                sensitivity = sensitivity + v_lam[m_i] * cv

                        dp_req = [p for p in dp if p.requires_grad]
                        if dp_req and sensitivity.requires_grad:
                            grads = tr.autograd.grad(sensitivity, dp_req, allow_unused=True)
                            j = 0
                            for i in params_indices:
                                g = grads[j] if j < len(grads) else None
                                grad_params[i] = -g if g is not None else tr.zeros_like(params_in[i])
                                j += 1
                        elif dp_req:
                            for i in params_indices:
                                grad_params[i] = tr.zeros_like(params_in[i])

                return (None, *grad_params)

        res_x = Minimize.apply(x0, *params)

        # Sever trailing graph for the metadata run
        params = [p.detach() if hasattr(p, 'detach') else p for p in params]
        
        def fun_np_meta(x_v):
             with tr.no_grad(): # Kills the trailing graph during metadata retrieval
                 res_val = fun(rp.array(x_v, dtype=dt), *_replace_tensors(args, params, {'idx': 0}))
             return tr.as_tensor(res_val).detach().cpu().numpy()
             
        def jac_np_meta(x_v):
             with tr.enable_grad():
                 x_tr = tr.tensor(x_v, dtype=dt, requires_grad=True)
                 obj_val = tr.as_tensor(fun(rp.array(x_tr, dtype=dt), *_replace_tensors(args, params, {'idx': 0})))
                 grad = tr.autograd.grad(obj_val, x_tr)[0]
             return grad.detach().cpu().numpy().astype('float64')
        
        # Metadata retrieval
        meta_cons = []
        curr_all_meta = _replace_tensors(all_inputs, params, {'idx': 0})
        curr_cons_meta = curr_all_meta[2]
        for c_idx, c in enumerate(curr_cons_meta):
             def c_meta_np(x_v, idx=c_idx):
                  m_curr_all = _replace_tensors(all_inputs, params, {'idx': 0})
                  m_c = m_curr_all[2][idx]
                  m_a = m_c.get('args', ())
                  with tr.no_grad(): # Kills the trailing graph during metadata retrieval
                      val = m_c['fun'](rp.array(x_v, dtype=dt), *m_a)
                  return tr.as_tensor(val).detach().cpu().numpy()
                  
             def c_meta_jac_np(x_v, idx=c_idx):
                  m_curr_all = _replace_tensors(all_inputs, params, {'idx': 0})
                  m_c = m_curr_all[2][idx]
                  m_a = m_c.get('args', ())
                  def c_fun_pure(x_t):
                      return tr.as_tensor(m_c['fun'](rp.array(x_t, dtype=dt), *m_a))
                  with tr.enable_grad():
                      J = tr.autograd.functional.jacobian(c_fun_pure, tr.tensor(x_v, dtype=dt))
                  return J.detach().cpu().numpy().astype('float64')
                  
             meta_cons.append({'type': c['type'], 'fun': c_meta_np, 'jac': c_meta_jac_np})
             
        res = so.minimize(
            fun_np_meta, 
            tr.as_tensor(x0).detach().cpu().numpy().astype('float64'), # Cast initial guess
            method=method, 
            jac=jac_np_meta,      
            bounds=curr_all_meta[1], 
            constraints=meta_cons, 
            tol=tol, 
            options=options
        )
        res = _convert_optimize_result(res)
        res.x = rp.array(res_x, dtype=dt)
        return res

    else: return so.minimize(fun, x0, args=args, method=method, bounds=bounds, constraints=constraints, tol=tol, options=options)

def fmin_slsqp(func, x0, fprime=None, f_eqcons=None, fprime_eqcons=None, 
               f_ieqcons=None, fprime_ieqcons=None, bounds=(), iter=100, 
               acc=1e-06, iprint=1, disp=None, full_output=0, 
               epsilon=1.4901161193847656e-08, callback=None, args=()): 
    if not isinstance(args, tuple):
        args = (args,)
    
    

    if rp.use_jax:
        import jax
        jnp = jax.numpy

        def wrap_func(x_np, *args_passthrough):
            x_jax = jnp.array(x_np)
            res = func(x_jax, *args_passthrough)
            return np.array(res)

        def wrap_fprime(x_np, *args_passthrough):
            x_jax = jnp.array(x_np)
            grad_func = jax.grad(lambda x: func(x, *args_passthrough).sum())
            return np.array(grad_func(x_jax))

        fprime_to_use = wrap_fprime if fprime is None else fprime

        # Handle constraints
        wrapped_f_eqcons = None
        wrapped_fprime_eqcons = None
        if f_eqcons is not None:
             def wrapped_f_eqcons(x_np, *args_passthrough):
                  return np.array(f_eqcons(jnp.array(x_np), *args_passthrough))
             if fprime_eqcons is None:
                  def wrapped_fprime_eqcons(x_np, *args_passthrough):
                       jac_func = jax.jacobian(f_eqcons)
                       return np.array(jac_func(jnp.array(x_np), *args_passthrough))
             else:
                  wrapped_fprime_eqcons = fprime_eqcons

        wrapped_f_ieqcons = None
        wrapped_fprime_ieqcons = None
        if f_ieqcons is not None:
             def wrapped_f_ieqcons(x_np, *args_passthrough):
                  return np.array(f_ieqcons(jnp.array(x_np), *args_passthrough))
             if fprime_ieqcons is None:
                  def wrapped_fprime_ieqcons(x_np, *args_passthrough):
                       jac_func = jax.jacobian(f_ieqcons)
                       return np.array(jac_func(jnp.array(x_np), *args_passthrough))
             else:
                  wrapped_fprime_ieqcons = fprime_ieqcons

        x0_np = np.array(x0)
        res = so.fmin_slsqp(wrap_func, x0_np, fprime=fprime_to_use, 
                            f_eqcons=wrapped_f_eqcons, fprime_eqcons=wrapped_fprime_eqcons, 
                            f_ieqcons=wrapped_f_ieqcons, fprime_ieqcons=wrapped_fprime_ieqcons, 
                            bounds=bounds, iter=iter, acc=acc, iprint=iprint, disp=disp, 
                            full_output=full_output, epsilon=epsilon, callback=callback, args=args)
        
        if full_output:
             x, obj, niter, imode, smessage = res
             return rp.array(x), obj, niter, imode, smessage
        else:
             return rp.array(res)

    elif rp.use_torch:

        dt = x0.dtype
        
        def wrap_func(x_np, *args_passthrough):
            x_tr = rp.array(x_np, dtype=dt)
            res = func(x_tr, *args_passthrough)
            return tr.as_tensor(res).detach().cpu().numpy()

        def wrap_fprime(x_np, *args_passthrough):
            x_tr = tr.tensor(x_np, dtype=dt, requires_grad=True)
            res = func(rp.TorchArray(x_tr), *args_passthrough)
            res_sum = tr.as_tensor(res).sum()
            grad = tr.autograd.grad(res_sum, x_tr)[0]
            return grad.detach().cpu().numpy()

        fprime_to_use = wrap_fprime if fprime is None else fprime

        # Handle constraints
        wrapped_f_eqcons = None
        wrapped_fprime_eqcons = None
        if f_eqcons is not None:
             def wrapped_f_eqcons(x_np, *args_passthrough):
                  x_tr = tr.as_tensor(x_np, dtype=dt)
                  return tr.as_tensor(f_eqcons(rp.TorchArray(x_tr), *args_passthrough)).detach().cpu().numpy()
             if fprime_eqcons is None:
                  def wrapped_fprime_eqcons(x_np, *args_passthrough):
                       x_tr = tr.as_tensor(x_np, dtype=dt)
                       # Check for empty constraints
                       test_out = tr.as_tensor(f_eqcons(rp.TorchArray(x_tr), *args_passthrough))
                       if test_out.nelement() == 0:
                            return np.zeros((0, len(x_np)))
                       
                       def func_for_jac(x_t):
                            return tr.as_tensor(f_eqcons(rp.TorchArray(x_t), *args_passthrough))
                    
                       jac = tr.autograd.functional.jacobian(func_for_jac, x_tr)
                       return jac.detach().cpu().numpy()
             else:
                  wrapped_fprime_eqcons = fprime_eqcons

        wrapped_f_ieqcons = None
        wrapped_fprime_ieqcons = None
        if f_ieqcons is not None:
             def wrapped_f_ieqcons(x_np, *args_passthrough):
                  x_tr = tr.as_tensor(x_np, dtype=dt)
                  return tr.as_tensor(f_ieqcons(rp.TorchArray(x_tr), *args_passthrough)).detach().cpu().numpy()
             if fprime_ieqcons is None:
                  def wrapped_fprime_ieqcons(x_np, *args_passthrough):
                       x_tr = tr.as_tensor(x_np, dtype=dt)
                       # Check for empty constraints
                       test_out = tr.as_tensor(f_ieqcons(rp.TorchArray(x_tr), *args_passthrough))
                       if test_out.nelement() == 0:
                            return np.zeros((0, len(x_np)))

                       def func_for_jac(x_t):
                            return tr.as_tensor(f_ieqcons(rp.TorchArray(x_t), *args_passthrough))
                       jac = tr.autograd.functional.jacobian(func_for_jac, x_tr)
                       return jac.detach().cpu().numpy()
             else:
                  wrapped_fprime_ieqcons = fprime_ieqcons

        x0_np = np.asarray(tr.as_tensor(x0).detach().cpu())
        res = so.fmin_slsqp(wrap_func, x0_np, fprime=fprime_to_use, 
                            f_eqcons=wrapped_f_eqcons, fprime_eqcons=wrapped_fprime_eqcons, 
                            f_ieqcons=wrapped_f_ieqcons, fprime_ieqcons=wrapped_fprime_ieqcons, 
                            bounds=bounds, iter=iter, acc=acc, iprint=iprint, disp=disp, 
                            full_output=full_output, epsilon=epsilon, callback=callback, args=args)
            
        if full_output:
             x, obj, niter, imode, smessage = res
             return rp.array(x), obj, niter, imode, smessage
        else:
             return rp.array(res)

    else:
        return so.fmin_slsqp(func, x0, fprime=fprime, f_eqcons=f_eqcons, fprime_eqcons=fprime_eqcons, 
                             f_ieqcons=f_ieqcons, fprime_ieqcons=fprime_ieqcons, bounds=bounds, iter=iter, 
                             acc=acc, iprint=iprint, disp=disp, full_output=full_output, 
                             epsilon=epsilon, callback=callback, args=args)

def _find_tensors(obj):
    import torch as tr
    tensors = []
    
    if isinstance(obj, type):
        return tensors
    elif isinstance(obj, tr.Tensor):
        tensors.append(obj)
    elif isinstance(obj, (list, tuple)):
        for item in obj:
            tensors.extend(_find_tensors(item))
    elif isinstance(obj, dict):
        for k, v in obj.items():
            tensors.extend(_find_tensors(v))
    elif hasattr(obj, 'items'): 
        for k, v in obj.items():
            tensors.extend(_find_tensors(v))
            
    return tensors

def _replace_tensors(obj, tensors, state):
    import torch as tr
    
    if isinstance(obj, type):
        return obj
    elif isinstance(obj, tr.Tensor):
        val = tensors[state['idx']]
        state['idx'] += 1
        return val
    elif type(obj) is list:
        return [_replace_tensors(item, tensors, state) for item in obj]
    elif type(obj) is tuple:
        return tuple(_replace_tensors(item, tensors, state) for item in obj)
    elif type(obj) is dict:
        return {k: _replace_tensors(v, tensors, state) for k, v in obj.items()}
    elif hasattr(obj, 'items') and hasattr(obj, 'copy'):
        import copy
        new_obj = copy.copy(obj)
        for k, v in obj.items():
            new_obj[k] = _replace_tensors(v, tensors, state)
        return new_obj
        
    return obj

def _convert_optimize_result(res):
    """Recursively converts NumPy arrays in an OptimizeResult or dict to RNUMPY arrays."""
    if isinstance(res, dict) or hasattr(res, 'items'):
        for key, value in res.items():
             if isinstance(value, (np.ndarray, np.generic)):
                  res[key] = rp.array(value)
             elif isinstance(value, (dict)) or hasattr(value, 'items'):
                  _convert_optimize_result(value)
    return res

def minimize_scalar(fun, bracket=None, bounds=None, args=(), method=None, tol=None, options=None): 
    if not isinstance(args, tuple):
        args = (args,)
    if rp.use_jax:
        import jax
        import jax.numpy as jnp
        
        args_flat, args_tree = jax.tree_util.tree_flatten(args)
        
        @jax.custom_jvp
        def _jax_solve(bracket_in, bounds_in, *a_flat):
            current_args = jax.tree_util.tree_unflatten(args_tree, a_flat)
            concrete_args = _jax_concrete_args(current_args)
            def fun_np(x_val):
                 res = fun(rp.array(x_val), *concrete_args)
                 return np.asarray(jax.device_get(res))
            
            res = so.minimize_scalar(fun_np, bracket=bracket_in, bounds=bounds_in, method=method, tol=tol, options=options)
            return jnp.array(res.x), jnp.array(res.fun)

        @_jax_solve.defjvp
        def _jax_solve_jvp(primals, tangents):
            bracket_p, bounds_p = primals[:2]
            params_p = primals[2:]
            params_t = tangents[2:]
            
            x_sol, f_sol = _jax_solve(*primals)
            
            # IFT: f'(x, p) = 0  => dx/dp = - (d^2 f / dx dp) / (d^2 f / dx^2)
            def objective(x_v, *p_v):
                current_args = jax.tree_util.tree_unflatten(args_tree, p_v)
                return fun(rp.array(x_v), *current_args).sum()

            f_prime_v_x = jax.grad(objective, argnums=0)
            f_pp_v_x = jax.grad(f_prime_v_x, argnums=0)(x_sol, *params_p)
            
            # JVP for f_prime at (x_sol, params)
            _, f_prime_tangent = jax.jvp(lambda *p: f_prime_v_x(x_sol, *p), params_p, params_t)
            dx = -f_prime_tangent / f_pp_v_x
            
            # Envelope theorem: df/dp = partial_f/partial_p at fixed x_sol
            _, df_dp_partial = jax.jvp(lambda *p: objective(x_sol, *p), params_p, params_t)
            return (x_sol, f_sol), (dx, df_dp_partial)

        x_val, f_val = _jax_solve(bracket, bounds, *args_flat)
        
        # To get the full OptimizeResult object (metadata like success, nit), 
        # we run it once with stop_gradient to avoid tracer errors.
        def get_metadata():
            def fun_np_meta(x_v):
                # Use stop_gradient to get concrete values for the metadata solver
                res_v = fun(rp.array(x_v), *jax.tree_util.tree_map(jax.lax.stop_gradient, args))
                return np.asarray(jax.device_get(res_v))
            return so.minimize_scalar(fun_np_meta, bracket=bracket, bounds=bounds, method=method, tol=tol, options=options)
            
        try:
            res = get_metadata()
            res = _convert_optimize_result(res)
        except Exception:
            # Fallback if metadata retrieval fails
            from scipy.optimize import OptimizeResult
            res = OptimizeResult(x=x_val, fun=f_val, success=True, status=0, message='Success', nit=0, nfev=0)

        res.x = x_val
        res.fun = f_val
        return res

    elif rp.use_torch:
        import torch as tr
        
        params = _find_tensors(args)
        dt = params[0].dtype if params else tr.get_default_dtype()
        
        class MinimizeScalar(tr.autograd.Function):
            @staticmethod
            def forward(bracket, bounds, *args_tensors):
                def fun_np(x_val):
                    current_args = _replace_tensors(args, args_tensors, {'idx': 0})
                    res = fun(rp.array(x_val, dtype=dt), *current_args)
                    return tr.as_tensor(res).detach().cpu().numpy()

                res = so.minimize_scalar(fun_np, bracket=bracket, bounds=bounds, method=method, tol=tol, options=options)
                x_sol = tr.as_tensor(res.x, dtype=dt)
                return x_sol, tr.as_tensor(res.fun, dtype=dt)

            @staticmethod
            def setup_context(ctx, inputs, output):
                x_sol, f_val = output
                bracket, bounds, *args_tensors = inputs
                ctx.save_for_backward(x_sol, *args_tensors)

            @staticmethod
            def backward(ctx, grad_x, grad_f):
                x_sol = ctx.saved_tensors[0]
                params_in = ctx.saved_tensors[1:]
                
                with tr.enable_grad():
                    x = x_sol.detach().requires_grad_(True)
                    current_args = _replace_tensors(args, params_in, {'idx': 0})
                    f = fun(rp.array(x, dtype=dt), *current_args)
                    f_tensor = tr.as_tensor(f).sum()
                    
                    # f_prime = df/dx
                    f_prime = tr.autograd.grad(f_tensor, x, create_graph=True)[0]
                    # f_double_prime = d^2 f / dx^2
                    f_double_prime = tr.autograd.grad(f_prime, x, retain_graph=True)[0]
                
                # IFT: dx/dp = - (1/f'') * (df'/dp)
                multiplier = - grad_x / f_double_prime
                
                grad_params = [None] * len(params_in)
                params_to_diff = []
                params_indices = []
                for i, p in enumerate(params_in):
                    if p.requires_grad:
                        params_to_diff.append(p)
                        params_indices.append(i)
                
                if params_to_diff:
                    # G_x = grad_x * dx/dp
                    vjp_x = tr.autograd.grad(f_prime, params_to_diff, grad_outputs=multiplier, retain_graph=True, allow_unused=True)
                    # G_f = grad_f * df/dp (direct)
                    vjp_f = tr.autograd.grad(f_tensor, params_to_diff, grad_outputs=grad_f.expand_as(f_tensor), allow_unused=True)
                    
                    for i, (gx, gf) in zip(params_indices, zip(vjp_x, vjp_f)):
                        g = 0
                        if gx is not None: g = g + gx
                        if gf is not None: g = g + gf
                        grad_params[i] = g
                
                return (None, None, *grad_params)

        params = _find_tensors(args)
        x_val, f_val = MinimizeScalar.apply(bracket, bounds, *params)
        
        # OptimizeResult
        def fun_np_final(x_val): 
             res = fun(rp.array(x_val, dtype=dt), *_replace_tensors(args, params, {'idx': 0}))
             return tr.as_tensor(res).detach().cpu().numpy()
        res = so.minimize_scalar(fun_np_final, bracket=bracket, bounds=bounds, method=method, tol=tol, options=options)
        res = _convert_optimize_result(res)
        res.x = rp.array(x_val)
        res.fun = rp.array(f_val)
        return res
    
    else:
        return so.minimize_scalar(fun, bracket=bracket, bounds=bounds, args=args, method=method, tol=tol, options=options)

def brentq(f, a, b, args=(), xtol=2e-12, rtol=8.881784197001252e-16, maxiter=100, full_output=False, disp=True):
    if not isinstance(args, tuple):
        args = (args,)
    if rp.use_jax:
        import jax
        import jax.numpy as jnp
        
        args_flat, args_tree = jax.tree_util.tree_flatten(args)
        
        @jax.custom_jvp
        def _jax_brentq(a_in, b_in, *a_flat):
            current_args = jax.tree_util.tree_unflatten(args_tree, a_flat)
            concrete_args = _jax_concrete_args(current_args)
            def fun_np(x_v): 
                res = f(rp.array(x_v), *concrete_args)
                return np.asarray(jax.device_get(res))
            
            x_sol = so.brentq(fun_np, a_in, b_in, args=(), xtol=xtol, rtol=rtol, maxiter=maxiter, disp=disp)
            return jnp.array(x_sol)

        @_jax_brentq.defjvp
        def _jax_brentq_jvp(primals, tangents):
            params_p = primals[2:]
            params_t = tangents[2:]
            x_sol = _jax_brentq(*primals)
            
            def objective(x_v, *p_v):
                current_args = jax.tree_util.tree_unflatten(args_tree, p_v)
                return f(rp.array(x_v), *current_args).sum()

            df_dx = jax.grad(objective, argnums=0)(x_sol, *params_p)
            _, df_dp_tangent = jax.jvp(lambda *p: objective(x_sol, *p), params_p, params_t)
            
            dx = -df_dp_tangent / df_dx
            return x_sol, dx

        x_sol = _jax_brentq(a, b, *args_flat)
        
        if full_output:
            def get_metadata():
                def fun_np_meta(x_v):
                    res_v = f(rp.array(x_v), *jax.tree_util.tree_map(jax.lax.stop_gradient, args))
                    return np.asarray(jax.device_get(res_v))
                return so.brentq(fun_np_meta, a, b, args=(), xtol=xtol, rtol=rtol, maxiter=maxiter, full_output=True, disp=disp)
            xr, r = get_metadata()
            return x_sol, r
        else:
            return x_sol

    elif rp.use_torch:
        import torch as tr
        
        params = _find_tensors(args)
        dt = params[0].dtype if params else tr.get_default_dtype()

        class BrentQ(tr.autograd.Function):
            @staticmethod
            def forward(a_in, b_in, *params_in):
                def fun_np(x_val):
                    current_args = _replace_tensors(args, params_in, {'idx': 0})
                    res = f(rp.array(x_val, dtype=dt), *current_args)
                    return tr.as_tensor(res).detach().cpu().numpy()

                x_sol = so.brentq(fun_np, a_in, b_in, args=(), xtol=xtol, rtol=rtol, maxiter=maxiter, disp=disp)
                return tr.as_tensor(x_sol, dtype=dt)

            @staticmethod
            def setup_context(ctx, inputs, output):
                a_in, b_in, *params_in = inputs
                ctx.save_for_backward(output, *params_in)

            @staticmethod
            def backward(ctx, grad_x):
                x_sol = ctx.saved_tensors[0]
                params_in = ctx.saved_tensors[1:]
                
                with tr.enable_grad():
                    x = x_sol.detach().requires_grad_(True)
                    current_args = _replace_tensors(args, params_in, {'idx': 0})
                    val = f(rp.array(x, dtype=dt), *current_args)
                    f_tensor = tr.as_tensor(val).sum()
                    df_dx = tr.autograd.grad(f_tensor, x, retain_graph=True)[0]
                
                multiplier = - grad_x / df_dx
                grad_params = [None] * len(params_in)
                params_to_diff = []
                params_indices = []
                for i, p in enumerate(params_in):
                    if p.requires_grad:
                        params_to_diff.append(p)
                        params_indices.append(i)
                
                if params_to_diff:
                    grads_p = tr.autograd.grad(f_tensor, params_to_diff, grad_outputs=multiplier.expand_as(f_tensor), allow_unused=True)
                    for i, g in zip(params_indices, grads_p):
                        grad_params[i] = g
                
                return (None, None, *grad_params)

        params = _find_tensors(args)
        x_sol = BrentQ.apply(a, b, *params)
        
        if full_output:
            def fun_np_meta(x_val):
                 res = f(rp.array(x_val, dtype=dt), *_replace_tensors(args, params, {'idx': 0}))
                 return tr.as_tensor(res).detach().cpu().numpy()
            xr, r = so.brentq(fun_np_meta, a, b, args=(), xtol=xtol, rtol=rtol, maxiter=maxiter, full_output=True, disp=disp)
            return rp.array(x_sol, dtype=dt), r
        else:
            return rp.array(x_sol, dtype=dt)
    
    else:
        return so.brentq(f, a, b, args=args, xtol=xtol, rtol=rtol, maxiter=maxiter, full_output=full_output, disp=disp)

def fminbound(func, x1, x2, args=(), xtol=1e-05, maxfun=500, full_output=0, disp=1):
    if not isinstance(args, tuple):
        args = (args,)
    if rp.use_jax:
        import jax
        import jax.numpy as jnp
        
        args_flat, args_tree = jax.tree_util.tree_flatten(args)
        
        @jax.custom_jvp
        def _jax_fminbound(x1_in, x2_in, *a_flat):
            current_args = jax.tree_util.tree_unflatten(args_tree, a_flat)
            concrete_args = _jax_concrete_args(current_args)
            def fun_np(x_v): 
                res = func(rp.array(x_v), *concrete_args)
                return np.asarray(jax.device_get(res))
            
            x_sol = so.fminbound(fun_np, x1_in, x2_in, args=(), xtol=xtol, maxfun=maxfun, disp=disp)
            return jnp.array(x_sol)

        @_jax_fminbound.defjvp
        def _jax_fminbound_jvp(primals, tangents):
            params_p = primals[2:]
            params_t = tangents[2:]
            x_sol = _jax_fminbound(*primals)
            
            def objective(x_v, *p_v):
                current_args = jax.tree_util.tree_unflatten(args_tree, p_v)
                return func(rp.array(x_v), *current_args).sum()

            f_prime = jax.grad(objective, argnums=0)
            f_pp = jax.grad(f_prime, argnums=0)(x_sol, *params_p)
            _, f_prime_tangent = jax.jvp(lambda *p: f_prime(x_sol, *p), params_p, params_t)
            
            dx = -f_prime_tangent / f_pp
            return x_sol, dx

        x_sol = _jax_fminbound(x1, x2, *args_flat)
        
        if full_output:
            def get_metadata():
                def fun_np_meta(x_v):
                    res_v = func(rp.array(x_v), *jax.tree_util.tree_map(jax.lax.stop_gradient, args))
                    return np.asarray(jax.device_get(res_v))
                return so.fminbound(fun_np_meta, x1, x2, args=(), xtol=xtol, maxfun=maxfun, full_output=True, disp=disp)
            xr, fval, ierr, numfunc = get_metadata()
            return x_sol, fval, ierr, numfunc
        else:
            return x_sol

    elif rp.use_torch:
        import torch as tr
        
        params = _find_tensors(args)
        dt = params[0].dtype if params else tr.get_default_dtype()

        class FMinBound(tr.autograd.Function):
            @staticmethod
            def forward(x1_in, x2_in, *params_in):
                def fun_np(x_val):
                    current_args = _replace_tensors(args, params_in, {'idx': 0})
                    res = func(rp.array(x_val, dtype=dt), *current_args)
                    return tr.as_tensor(res).detach().cpu().numpy()

                x_sol = so.fminbound(fun_np, x1_in, x2_in, args=(), xtol=xtol, maxfun=maxfun, disp=disp)
                return tr.as_tensor(x_sol, dtype=dt)

            @staticmethod
            def setup_context(ctx, inputs, output):
                x1_in, x2_in, *params_in = inputs
                ctx.save_for_backward(output, *params_in)

            @staticmethod
            def backward(ctx, grad_x):
                x_sol = ctx.saved_tensors[0]
                params_in = ctx.saved_tensors[1:]
                
                with tr.enable_grad():
                    x = x_sol.detach().requires_grad_(True)
                    current_args = _replace_tensors(args, params_in, {'idx': 0})
                    val = func(rp.array(x, dtype=dt), *current_args)
                    f_tensor = tr.as_tensor(val).sum()
                    f_prime = tr.autograd.grad(f_tensor, x, create_graph=True)[0]
                    f_double_prime = tr.autograd.grad(f_prime, x, retain_graph=True)[0]
                
                multiplier = - grad_x / f_double_prime
                grad_params = [None] * len(params_in)
                params_to_diff = []
                params_indices = []
                for i, p in enumerate(params_in):
                    if p.requires_grad:
                        params_to_diff.append(p)
                        params_indices.append(i)
                
                if params_to_diff:
                    grads_p = tr.autograd.grad(f_prime, params_to_diff, grad_outputs=multiplier.expand_as(f_prime), allow_unused=True)
                    for i, g in zip(params_indices, grads_p):
                        grad_params[i] = g
                
                return (None, None, *grad_params)

        params = _find_tensors(args)
        x_sol = FMinBound.apply(x1, x2, *params)
        
        if full_output:
            def fun_np_meta(x_val):
                 res = func(rp.array(x_val, dtype=dt), *_replace_tensors(args, params, {'idx': 0}))
                 return tr.as_tensor(res).detach().cpu().numpy()
            xr, fval, ierr, numfunc = so.fminbound(fun_np_meta, x1, x2, args=(), xtol=xtol, maxfun=maxfun, full_output=True, disp=disp)
            return rp.array(x_sol, dtype=dt), fval, ierr, numfunc
        else:
            return rp.array(x_sol, dtype=dt)
    
    else:
        return so.fminbound(func, x1, x2, args=args, xtol=xtol, maxfun=maxfun, full_output=full_output, disp=disp)

def fsolve(func, x0, args=(), fprime=None, full_output=0, col_deriv=0, xtol=1.49012e-08, maxfev=0, band=None, epsfcn=None, factor=100, diag=None):
    """Find a root of ``func(x, *args) = 0``.

    JAX and Torch use automatic derivatives of the residual for the solver
    Jacobian and for implicit-function gradients. Residuals must be real-valued.
    ``full_output=True`` is recorded from the host SciPy solve and is not
    available under JAX ``vmap`` or when that host solve does not run in this
    Python call.
    """
    if not isinstance(args, tuple):
        args = (args,)
    # A Jacobian we build is d f_i / d x_j. SciPy only expects the transpose
    # when the caller supplied fprime and set col_deriv.
    active_col_deriv = col_deriv if fprime is not None else 0
    if rp.use_jax:
        import jax
        import jax.numpy as jnp

        meta = {}
        args_flat, args_tree = jax.tree_util.tree_flatten(args)
        static_leaves = []
        array_leaves = []
        for leaf in args_flat:
            if isinstance(leaf, jax.Array):
                static_leaves.append(_MISSING)
                array_leaves.append(leaf)
            else:
                static_leaves.append(leaf)

        def _merge_leaves(dyn_leaves):
            dyn = iter(dyn_leaves)
            merged = [next(dyn) if leaf is _MISSING else leaf for leaf in static_leaves]
            return jax.tree_util.tree_unflatten(args_tree, merged)

        def _host_solve(x0_host, *dyn_leaves):
            # Host-side solve. JAX stages this callback, so the iterations are
            # not part of an outer jit or grad trace.
            current_args = _merge_leaves(dyn_leaves)
            fun_np, fprime_np = _build_fsolve_scipy_callbacks(func, current_args, fprime=fprime)
            x_sol_np, infodict, ier, mesg = so.fsolve(
                fun_np, _as_1d_numpy(x0_host), args=(), fprime=fprime_np, full_output=True,
                col_deriv=active_col_deriv, xtol=xtol, maxfev=maxfev, band=band,
                epsfcn=epsfcn, factor=factor, diag=diag,
            )
            meta["n_solves"] = meta.get("n_solves", 0) + 1
            meta["info"] = _copy_fsolve_info(infodict, ier, mesg)
            return np.asarray(x_sol_np, dtype=np.dtype(x0_host.dtype))

        @jax.custom_jvp
        def _jax_fsolve(x0_in, *dyn_leaves):
            result_shape = jax.ShapeDtypeStruct(tuple(x0_in.reshape(-1).shape), x0_in.dtype)
            return jax.pure_callback(
                _host_solve, result_shape, x0_in, *dyn_leaves, vmap_method="sequential",
            )

        @_jax_fsolve.defjvp
        def _jax_fsolve_jvp(primals, tangents):
            params_p = primals[1:]
            params_t = tangents[1:]
            x_sol = _jax_fsolve(*primals)

            def objective(x_v, *dyn):
                current_args = _merge_leaves(dyn)
                return func(rp.array(jnp.ravel(x_v)), *current_args).ravel()

            jac_x = jax.jacobian(objective, argnums=0)(x_sol, *params_p)
            _, f_p_tangent = jax.jvp(lambda *dyn: objective(x_sol, *dyn), params_p, params_t)
            dx = -jnp.linalg.solve(jac_x, f_p_tangent)
            return x_sol, dx

        x0_flat = jnp.ravel(jnp.asarray(x0))
        x_sol = _jax_fsolve(x0_flat, *array_leaves)

        if full_output:
            infodict, ier, mesg = _require_fsolve_info(meta, "JAX")
            return rp.array(x_sol), _convert_optimize_result(infodict), ier, mesg
        return rp.array(x_sol)

    elif rp.use_torch:
        import torch as tr

        meta = {}
        params = _find_tensors(args)
        if hasattr(x0, "dtype"):
            dt = x0.dtype
        else:
            dt = params[0].dtype if params else tr.get_default_dtype()
        if hasattr(x0, "device"):
            device = x0.device
        elif params:
            device = params[0].device
        else:
            device = None

        class FSolve(tr.autograd.Function):
            @staticmethod
            def forward(x0_in, *params_in):
                # no_grad is already on inside Function.forward. Keep the
                # residual callbacks on detached inputs so a local Jacobian
                # graph cannot attach to an outer optimizer tape.
                detached_params = tuple(p.detach() for p in params_in)
                current_args = _replace_tensors(args, detached_params, {'idx': 0})
                fun_np, fprime_np = _build_fsolve_scipy_callbacks(
                    func, current_args, fprime=fprime, dtype=dt, device=device,
                )
                x_sol_np, infodict, ier, mesg = so.fsolve(
                    fun_np, _as_1d_numpy(x0_in), args=(), fprime=fprime_np, full_output=True,
                    col_deriv=active_col_deriv, xtol=xtol, maxfev=maxfev, band=band,
                    epsfcn=epsfcn, factor=factor, diag=diag,
                )
                meta["n_solves"] = meta.get("n_solves", 0) + 1
                meta["info"] = _copy_fsolve_info(infodict, ier, mesg)
                return tr.tensor(np.array(x_sol_np, dtype=float, copy=True), dtype=dt, device=device)

            @staticmethod
            def setup_context(ctx, inputs, output):
                x0_in, *params_in = inputs
                ctx.save_for_backward(*params_in)
                ctx.x_sol_np = np.array(output.detach().cpu().numpy(), copy=True)

            @staticmethod
            def backward(ctx, grad_x):
                params_in = ctx.saved_tensors
                grad_params = _torch_ift_param_grads(func, args, params_in, grad_x, dt, ctx.x_sol_np)
                return (None, *grad_params)

        x_sol = FSolve.apply(x0, *params)
        if full_output:
            infodict, ier, mesg = _require_fsolve_info(meta, "Torch")
            return rp.array(x_sol, dtype=dt), _convert_optimize_result(infodict), ier, mesg
        return rp.array(x_sol, dtype=dt)

    else:
        fun_np, fprime_np = _build_fsolve_scipy_callbacks(func, args, fprime=fprime)
        res = so.fsolve(fun_np, _as_1d_numpy(x0), args=(), fprime=fprime_np, full_output=full_output,
                         col_deriv=active_col_deriv, xtol=xtol, maxfev=maxfev, band=band,
                         epsfcn=epsfcn, factor=factor, diag=diag)
        if full_output:
            x, infodict, ier, mesg = res
            return rp.array(x), _convert_optimize_result(infodict), ier, mesg
        return rp.array(res)

def root(fun, x0, args=(), method='hybr', jac=None, tol=None, callback=None, options=None):
    if not isinstance(args, tuple):
        args = (args,)
    if rp.use_jax:
        import jax
        import jax.numpy as jnp
        
        args_flat, args_tree = jax.tree_util.tree_flatten(args)
        
        @jax.custom_jvp
        def _jax_root(x0_in, *a_flat):
            current_args = jax.tree_util.tree_unflatten(args_tree, a_flat)
            concrete_args = _jax_concrete_args(current_args)
            def fun_np(x_v): 
                 res = fun(rp.array(x_v), *concrete_args)
                 return np.asarray(jax.device_get(res))
            
            res = so.root(fun_np, np.asarray(x0_in), method=method, tol=tol, callback=callback, options=options)
            return jnp.array(res.x)

        @_jax_root.defjvp
        def _jax_root_jvp(primals, tangents):
            params_p = primals[1:]
            params_t = tangents[1:]
            x_sol = _jax_root(*primals)
            
            def objective(x_v, *p_v):
                current_args = jax.tree_util.tree_unflatten(args_tree, p_v)
                return fun(rp.array(x_v), *current_args).ravel()

            Jx = jax.jacobian(objective, argnums=0)(x_sol, *params_p)
            _ , f_p_tangent = jax.jvp(lambda *p: objective(x_sol, *p), params_p, params_t)
            
            # Solve Jx * dx = -f_p_tangent
            dx = -jnp.linalg.solve(Jx, f_p_tangent)
            return x_sol, dx

        x_sol = _jax_root(x0, *args_flat)
        
        # Metadata retrieval
        def get_metadata():
            def fun_np_meta(x_v):
                res_v = fun(rp.array(x_v), *jax.tree_util.tree_map(jax.lax.stop_gradient, args))
                return np.asarray(jax.device_get(res_v))
            return so.root(fun_np_meta, np.asarray(x0), method=method, tol=tol, callback=callback, options=options)
            
        try:
            res = get_metadata()
            res = _convert_optimize_result(res)
        except Exception:
            from scipy.optimize import OptimizeResult
            res = OptimizeResult(x=x_sol, success=True, status=0, fun=x_sol*0, message='Success')

        res.x = x_sol
        return res

    elif rp.use_torch:
        import torch as tr
        
        params = _find_tensors(args)
        dt = x0.dtype if hasattr(x0, 'dtype') else (params[0].dtype if params else tr.get_default_dtype())

        class Root(tr.autograd.Function):
            @staticmethod
            def forward(x0_in, *params_in):
                def fun_np(x_val):
                    current_args = _replace_tensors(args, params_in, {'idx': 0})
                    res = fun(rp.array(x_val, dtype=dt), *current_args)
                    return tr.as_tensor(res).detach().cpu().numpy()

                res = so.root(fun_np, to_numpy(tr.as_tensor(x0_in).detach().cpu().numpy()), method=method, tol=tol, callback=callback, options=options)
                x_sol = tr.as_tensor(res.x, dtype=dt)
                _torch_aux_store[id(x_sol)] = np.array(res.x, copy=True)
                return x_sol

            @staticmethod
            def setup_context(ctx, inputs, output):
                x0_in, *params_in = inputs
                ctx.save_for_backward(*params_in)
                ctx.x_sol_np = _torch_aux_store.pop(id(output), None)

            @staticmethod
            def backward(ctx, grad_x):
                params_in = ctx.saved_tensors
                grad_params = _torch_ift_param_grads(fun, args, params_in, grad_x, dt, ctx.x_sol_np)
                return (None, *grad_params)

        params = _find_tensors(args)
        x_sol = Root.apply(x0, *params)
        
        # Full result
        def fun_np_meta(x_val):
             res = fun(rp.array(x_val, dtype=dt), *_replace_tensors(args, params, {'idx': 0}))
             return tr.as_tensor(res).detach().cpu().numpy()
        res = so.root(fun_np_meta, tr.as_tensor(x0).detach().cpu().numpy(), method=method, tol=tol, callback=callback, options=options)
        res = _convert_optimize_result(res)
        res.x = rp.array(x_sol, dtype=dt)
        return res
    
    else:
        return so.root(fun, x0, args=args, method=method, jac=jac, tol=tol, callback=callback, options=options)

def OptimizeResults(x, success, status, fun, jac, hess_inv, nfev, njev, nit): 
    if rp.use_jax: return jo.OptimizeResults(x, success, status, fun, jac, hess_inv, nfev, njev, nit)
    elif rp.use_torch: raise NotImplementedError('OptimizeResults not supported for Torch')
    else: return so.OptimizeResults(x, success, status, fun, jac, hess_inv, nfev, njev, nit)
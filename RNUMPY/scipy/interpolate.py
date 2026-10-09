# src.py
# (c) Copyright 2024 Aerospace Research Community LLC

# Created:  Oct 2024 M. Clarke
# Modified: May 2026, E. Botero

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
        import jax.scipy.interpolate
    except ImportError:
        pass

ji  = j.scipy.interpolate if j else None
si  = sp.interpolate if sp is not None else None
TorchArray = rp.TorchArray


def RegularGridInterpolator(points, values, method='linear', bounds_error=False, fill_value=np.nan): 
    if rp.use_jax: 
        f = ji.RegularGridInterpolator(points=points, values=values, method=method, bounds_error=bounds_error, fill_value=fill_value)
        def wrapped_ji_interp(xi):
             return rp.array(f(xi))
        return wrapped_ji_interp
    elif rp.use_torch:
        return _TorchRegularGridInterpolator(points=points, values=values, method=method, bounds_error=bounds_error, fill_value=fill_value)
    else:
        f = si.RegularGridInterpolator(points=points, values=values, method=method, bounds_error=bounds_error, fill_value=fill_value)
        def wrapped_si_interp(xi):
             return rp.array(f(xi))
        return wrapped_si_interp

def griddata(points, values, xi, method='linear', fill_value=np.nan, rescale=False):
    if rp.use_jax:
        jnp = rp.jax_handle.numpy
        if isinstance(points, tuple):
            points = jnp.stack([jnp.asarray(p) for p in points], axis=-1)
        else:
            points = jnp.asarray(points)
            
        values = jnp.asarray(values)
        
        if isinstance(xi, tuple):
            xi = jnp.stack([jnp.asarray(x) for x in xi], axis=-1)
        else:
            xi = jnp.asarray(xi)
            
        original_shape = xi.shape[:-1]
        D = points.shape[-1]
        xi_flat = xi.reshape(-1, D)
        
        diff = xi_flat[:, jnp.newaxis, :] - points[jnp.newaxis, :, :]
        dist = jnp.sqrt(jnp.sum(diff**2, axis=-1))
        
        if method == 'nearest':
            idx = jnp.argmin(dist, axis=-1)
            res = values[idx]
        else:
            power = 1.0 if method == 'linear' else 3.0
            eps = 1e-12
            weights = 1.0 / (dist ** power + eps)
            
            exact_match = dist < eps
            has_exact = jnp.any(exact_match, axis=-1)
            
            weights = weights / jnp.sum(weights, axis=-1, keepdims=True)
            
            weights_expanded = weights
            for _ in range(values.ndim - 1):
                weights_expanded = jnp.expand_dims(weights_expanded, -1)
                
            res = jnp.sum(weights_expanded * jnp.expand_dims(values, 0), axis=1)
            
            match_idx = jnp.argmax(exact_match, axis=-1)
            
            has_exact_expanded = has_exact
            for _ in range(res.ndim - 1):
                has_exact_expanded = jnp.expand_dims(has_exact_expanded, -1)
                
            res = jnp.where(has_exact_expanded, values[match_idx], res)
            
        res = res.reshape(*original_shape, *values.shape[1:])
        return rp.array(res)

    elif rp.use_torch:
        if isinstance(points, tuple):
            points = tr.stack([tr.as_tensor(p) for p in points], dim=-1)
        else:
            points = tr.as_tensor(points)
            
        values = tr.as_tensor(values)
        
        if isinstance(xi, tuple):
            xi = tr.stack([tr.as_tensor(x) for x in xi], dim=-1)
        else:
            xi = tr.as_tensor(xi)
            
        original_shape = xi.shape[:-1]
        D = points.shape[-1]
        xi_flat = xi.reshape(-1, D)
        
        dist = tr.cdist(xi_flat.to(tr.float32), points.to(tr.float32)).to(xi.dtype)
        
        if method == 'nearest':
            idx = tr.argmin(dist, dim=-1)
            res = values[idx]
        else:
            power = 1.0 if method == 'linear' else 3.0
            eps = 1e-12
            weights = 1.0 / (dist ** power + eps)
            
            exact_match = dist < eps
            has_exact = exact_match.any(dim=-1)
            
            weights = weights / weights.sum(dim=-1, keepdim=True)
            
            weights_expanded = weights
            for _ in range(values.ndim - 1):
                weights_expanded = weights_expanded.unsqueeze(-1)
                
            res = (weights_expanded * values.unsqueeze(0)).sum(dim=1)
            
            if has_exact.any():
                match_idx = exact_match.float().argmax(dim=-1)
                res[has_exact] = values[match_idx[has_exact]]
                
        res = res.reshape(*original_shape, *values.shape[1:])
        return TorchArray(res)

    else:
        points_sp = tuple(rp.array(p) for p in points) if isinstance(points, tuple) else rp.array(points)
        xi_sp = tuple(rp.array(x) for x in xi) if isinstance(xi, tuple) else rp.array(xi)
        res = si.griddata(points_sp, rp.array(values), xi_sp, method=method, fill_value=fill_value, rescale=rescale)
        return rp.array(res)

def interp1d(x, y, kind='linear', axis=-1, copy=True, bounds_error=None, fill_value=np.nan, assume_sorted=False):
    if rp.use_jax:
        jnp = rp.jax_handle.numpy
        if kind == 'linear':
             def wrapped_jnp_interp_linear(x_new):
                  return jnp.interp(x_new, x, y)
             return wrapped_jnp_interp_linear
        elif kind == 'cubic':
             x_j = jnp.asarray(x)
             y_j = jnp.asarray(y)
             tck = splrep(x_j, y_j, k=3, s=0)
             def jax_interp_cubic(x_new):
                 x_new_j = jnp.asarray(x_new)
                 x_new_j = jnp.clip(x_new_j, x_j[0], x_j[-1])
                 return rp.array(splev(x_new_j, tck))
             return jax_interp_cubic
        raise NotImplementedError(f'JAX interp1d kind {kind} not implemented')
        
    elif rp.use_torch:
        if kind == 'linear':
            x_t = tr.as_tensor(x)
            y_t = tr.as_tensor(y)
            def torch_interp_linear(x_new):
                x_new_t = tr.as_tensor(x_new)
                indices = tr.searchsorted(x_t, x_new_t)
                indices = tr.clamp(indices, 1, len(x_t) - 1)
                x_left = x_t[indices - 1]
                x_right = x_t[indices]
                y_left = y_t[indices - 1]
                y_right = y_t[indices]
                weight = (x_new_t - x_left) / (x_right - x_left)
                res = y_left + weight * (y_right - y_left)
                return TorchArray(res)
            return torch_interp_linear
        elif kind == 'cubic':
            x_t = tr.as_tensor(x)
            y_t = tr.as_tensor(y)
            tck = splrep(x_t, y_t, k=3, s=0)
            def torch_interp_cubic(x_new):
                x_new_t = tr.as_tensor(x_new)
                x_new_t = tr.clamp(x_new_t, x_t[0], x_t[-1])
                return TorchArray(splev(x_new_t, tck))
            return torch_interp_cubic
        raise NotImplementedError(f'PyTorch interp1d kind {kind} not implemented')
    
    else:
        f = si.interp1d(rp.array(x), rp.array(y), kind=kind, axis=axis, copy=copy, bounds_error=bounds_error, fill_value=fill_value, assume_sorted=assume_sorted)
        def wrapped_si_interp(x_new):
             return rp.array(f(rp.array(x_new)))
        return wrapped_si_interp

def _torch_bspline_basis(x_eval, t, k):
    x_eval = x_eval.unsqueeze(0)
    t_view = t.unsqueeze(1)
    B = ((t_view[:-1] <= x_eval) & (x_eval < t_view[1:])).to(x_eval.dtype)

    for d in range(1, k + 1):
        # Basis of degree d depends on knots t[i], t[i+d], t[i+1], t[i+d+1]
        # and basis of degree d-1
        t_i = t[:-d - 1]
        t_id = t[d:-1]
        t_i1 = t[1:-d]
        t_id1 = t[d + 1:]

        denom1 = t_id - t_i
        mask1 = denom1 > 0
        denom1_safe = tr.where(mask1, denom1, tr.ones_like(denom1))
        T1 = ((x_eval - t_i.unsqueeze(1)) / denom1_safe.unsqueeze(1)) * B[:-1]
        term1 = tr.where(mask1.unsqueeze(1), T1, tr.zeros_like(T1))

        denom2 = t_id1 - t_i1
        mask2 = denom2 > 0
        denom2_safe = tr.where(mask2, denom2, tr.ones_like(denom2))
        T2 = ((t_id1.unsqueeze(1) - x_eval) / denom2_safe.unsqueeze(1)) * B[1:]
        term2 = tr.where(mask2.unsqueeze(1), T2, tr.zeros_like(T2))

        B = term1 + term2

    boundary_mask = (x_eval >= t[-1])  # shape (1, n_eval)
    n_basis = B.shape[0]
    is_last_row = (
        tr.arange(n_basis, device=B.device) == n_basis - 1
    ).unsqueeze(1)  # shape (n_basis, 1)
    correction = boundary_mask & is_last_row  # broadcasts to (n_basis, n_eval)
    B = tr.where(correction, tr.ones_like(B), B)

    return B.T

def _jax_bspline_basis(x_eval, t, k):
    jnp = rp.jax_handle.numpy
    x_eval = x_eval[jnp.newaxis, :]
    t_view = t[:, jnp.newaxis]
    
    B = jnp.logical_and(t_view[:-1] <= x_eval, x_eval < t_view[1:]).astype(x_eval.dtype)
    
    for d in range(1, k + 1):
        t_i = t[:-d-1]
        t_id = t[d:-1]
        t_i1 = t[1:-d]
        t_id1 = t[d+1:]
        
        denom1 = t_id - t_i
        denom1_safe = jnp.where(denom1 > 0, denom1, 1.0)
        mask1 = denom1 > 0
        T1 = ((x_eval - t_i[:, jnp.newaxis]) / denom1_safe[:, jnp.newaxis]) * B[:-1]
        term1 = jnp.where(mask1[:, jnp.newaxis], T1, jnp.zeros_like(T1))
        
        denom2 = t_id1 - t_i1
        denom2_safe = jnp.where(denom2 > 0, denom2, 1.0)
        mask2 = denom2 > 0
        T2 = ((t_id1[:, jnp.newaxis] - x_eval) / denom2_safe[:, jnp.newaxis]) * B[1:]
        term2 = jnp.where(mask2[:, jnp.newaxis], T2, jnp.zeros_like(T2))
        
        B = term1 + term2

    is_right_boundary = (x_eval == t[-1])[0]
    B = B.at[-1].set(jnp.where(is_right_boundary, 1.0, B[-1]))
    
    return B.T

def splprep(x, w=None, u=None, ub=None, ue=None, k=3, s=None, per=0, quiet=1):
    if (rp.use_jax or rp.use_torch) and (s == 0 or s is None):
        if rp.use_torch:
            arrays = [tr.as_tensor(arr) for arr in x]
            n = len(arrays[0])
            if u is None:
                diffs = tr.stack([arr[1:] - arr[:-1] for arr in arrays])
                dist = (diffs**2).sum(dim=0).sqrt()
                u_knots = tr.cat([tr.zeros(1, device=dist.device, dtype=dist.dtype), tr.cumsum(dist, dim=0)])
                if u_knots[-1] > 0:
                    u_knots = u_knots / u_knots[-1]
            else:
                u_knots = tr.as_tensor(u)
                
            t_knots = tr.zeros(n + k + 1, dtype=u_knots.dtype, device=u_knots.device)
            t_knots[:k+1] = u_knots[0]
            t_knots[-k-1:] = u_knots[-1]
            if n > 2 * k - 2:
                trim = (k + 1) // 2
                t_knots[k+1:n] = u_knots[trim:-trim]
                
            A = _torch_bspline_basis(u_knots, t_knots, k)
            coeffs = []
            for arr in arrays:
                c_inner = tr.linalg.solve(A, arr)
                c = tr.zeros(len(t_knots), dtype=c_inner.dtype, device=c_inner.device)
                c[:n] = c_inner
                coeffs.append(c)
                
                
            tck = (rp.array(t_knots), [rp.array(c) for c in coeffs], k)
            return tck, rp.array(u_knots)
            
        elif rp.use_jax:
            jnp = rp.jax_handle.numpy
            arrays = [jnp.asarray(arr) for arr in x]
            n = len(arrays[0])
            if u is None:
                diffs = jnp.stack([arr[1:] - arr[:-1] for arr in arrays], axis=0)
                dist = jnp.sqrt((diffs**2).sum(axis=0))
                u_knots = jnp.concatenate([jnp.zeros(1), jnp.cumsum(dist)])
                if u_knots[-1] > 0:
                    u_knots = u_knots / u_knots[-1]
            else:
                u_knots = jnp.asarray(u)
                
            t_knots = jnp.zeros(n + k + 1, dtype=u_knots.dtype)
            t_knots = t_knots.at[:k+1].set(u_knots[0])
            t_knots = t_knots.at[-k-1:].set(u_knots[-1])
            if n > 2 * k - 2:
                trim = (k + 1) // 2
                t_knots = t_knots.at[k+1:n].set(u_knots[trim:-trim])
                
            A = _jax_bspline_basis(u_knots, t_knots, k)
            coeffs = []
            for arr in arrays:
                c_inner = jnp.linalg.solve(A, arr)
                c = jnp.zeros(len(t_knots), dtype=c_inner.dtype)
                c = c.at[:n].set(c_inner)
                coeffs.append(c)
                
                
            tck = (rp.array(t_knots), [rp.array(c) for c in coeffs], k)
            return tck, rp.array(u_knots)

    if rp.use_jax or rp.use_torch:
         raise NotImplementedError("Differentiable splprep only implemented for interpolation (s=0) for now.")
         
    x_np = [np.array(arr) for arr in x]
    tck, u_out = si.splprep(x_np, w=w, u=u, ub=ub, ue=ue, k=k, s=s, per=per, quiet=quiet)
    return (rp.array(tck[0]), [rp.array(c) for c in tck[1]], tck[2]), rp.array(u_out)

def splev(x, tck, der=0, ext=0):
    if (rp.use_torch or rp.use_jax) and isinstance(tck, tuple) and len(tck) == 3 and der == 0:
        t, c, k = tck
        c_list = c if isinstance(c, list) else [c]
        
        if rp.use_torch:
            x_in = tr.as_tensor(x)
            A = _torch_bspline_basis(x_in, t, k)
            n_basis = len(t) - k - 1
            results = []
            for c_arr in c_list:
                c_n = c_arr[:n_basis]
                res = tr.matmul(A, c_n.to(A.dtype))
                results.append(TorchArray(res))
        elif rp.use_jax:
            jnp = rp.jax_handle.numpy
            x_in = jnp.asarray(x)
            A = _jax_bspline_basis(x_in, t, k)
            n_basis = len(t) - k - 1
            results = []
            for c_arr in c_list:
                c_n = c_arr[:n_basis]
                res = jnp.dot(A, c_n.astype(A.dtype))
                results.append(rp.array(res))
                
        if not isinstance(c, list):
            return results[0]
        return results

    if rp.use_jax or rp.use_torch:
         raise NotImplementedError("Differentiable splev only implemented for evaluation (der=0) for now.")

    res = si.splev(x, tck, der=der, ext=ext)
    if isinstance(res, list):
         return [rp.array(r) for r in res]
    return rp.array(res)

def splrep(x, y, w=None, xb=None, xe=None, k=3, s=None, t=None, task=0, full_output=0, per=0, quiet=1):
    if (rp.use_torch or rp.use_jax) and (s == 0 or s is None):
        if rp.use_torch:
            x_t = tr.as_tensor(x)
            y_t = tr.as_tensor(y)
            n = len(x_t)
            
            t_knots = tr.zeros(n + k + 1, dtype=x_t.dtype, device=x_t.device)
            t_knots[:k+1] = x_t[0]
            t_knots[-k-1:] = x_t[-1]
            if n > 2 * k - 2:
                trim = (k + 1) // 2
                t_knots[k+1:n] = x_t[trim:-trim]
                
            A = _torch_bspline_basis(x_t, t_knots, k)
            c_inner = tr.linalg.solve(A, y_t)
            
            c = tr.zeros(len(t_knots), dtype=c_inner.dtype, device=c_inner.device)
            c[:n] = c_inner
            
            tck = (rp.array(t_knots), rp.array(c), k)
            return tck
            
        elif rp.use_jax:
            jnp = rp.jax_handle.numpy
            x_j = jnp.asarray(x)
            y_j = jnp.asarray(y)
            n = len(x_j)
            
            t_knots = jnp.zeros(n + k + 1, dtype=x_j.dtype)
            t_knots = t_knots.at[:k+1].set(x_j[0])
            t_knots = t_knots.at[-k-1:].set(x_j[-1])
            if n > 2 * k - 2:
                trim = (k + 1) // 2
                t_knots = t_knots.at[k+1:n].set(x_j[trim:-trim])
                
            A = _jax_bspline_basis(x_j, t_knots, k)
            c_inner = jnp.linalg.solve(A, y_j)
            
            c = jnp.zeros(len(t_knots), dtype=c_inner.dtype)
            c = c.at[:n].set(c_inner)
            
            tck = (rp.array(t_knots), rp.array(c), k)
            return tck
    
    if rp.use_jax or rp.use_torch:
         raise NotImplementedError("Differentiable splrep only implemented for interpolation (s=0) for now.")

    res = si.splrep(x, y, w=w, xb=xb, xe=xe, k=k, s=s, t=t, task=task, full_output=full_output, per=per, quiet=quiet)
    if full_output:
        tck, fp, ier, msg = res
        return (rp.array(tck[0]), rp.array(tck[1]), tck[2]), fp, ier, msg
    else:
        tck = res
        return (rp.array(tck[0]), rp.array(tck[1]), tck[2])
    
class _TorchRegularGridInterpolator:
    def __init__(self, points, values, method='linear', bounds_error=False, fill_value=np.nan):
        self.points = [tr.as_tensor(p) for p in points]
        self.values = tr.as_tensor(values)
        self.method = method
        self.bounds_error = bounds_error
        self.fill_value = fill_value
        
        if self.method != 'linear':
            raise NotImplementedError("Only 'linear' method is supported for PyTorch RegularGridInterpolator")
            
    def __call__(self, xi):
        xi = tr.as_tensor(xi)
        ndim = len(self.points)
        
        if xi.shape[-1] != ndim:
            raise ValueError(f"The requested sample points xi have dimension {xi.shape[-1]}, but this RegularGridInterpolator has dimension {ndim}")

        original_shape = xi.shape[:-1]
        xi_flat = xi.reshape(-1, ndim)
        n_points = xi_flat.shape[0]
        
        out_of_bounds = tr.zeros(n_points, dtype=tr.bool, device=xi.device)
        indices = []
        norm_weights = []
        
        for i, p in enumerate(self.points):
            out_of_bounds |= (xi_flat[:, i] < p[0]) | (xi_flat[:, i] > p[-1])
            
            idx = tr.searchsorted(p, xi_flat[:, i], right=True) - 1
            idx = tr.clamp(idx, 0, len(p) - 2)
            indices.append(idx)
            
            p_lo = p[idx]
            p_hi = p[idx + 1]
            
            dp = p_hi - p_lo
            w = tr.where(dp > 0, (xi_flat[:, i] - p_lo) / dp, tr.zeros_like(dp))
            if self.fill_value is not None:
                w = tr.clamp(w, 0.0, 1.0)
            norm_weights.append(w)
            
        if self.bounds_error and out_of_bounds.any():
            raise ValueError("One of the requested xi is out of bounds")
            
        import itertools
        corners = list(itertools.product([0, 1], repeat=ndim))
        
        res_shape = (n_points,) + self.values.shape[ndim:]
        res = tr.zeros(res_shape, dtype=self.values.dtype, device=self.values.device)
        
        for corner in corners:
            corner_indices = [indices[i] + corner[i] for i in range(ndim)]
            corner_vals = self.values[tuple(corner_indices)]
            
            corner_weight = tr.ones(n_points, dtype=xi_flat.dtype, device=xi.device)
            for i in range(ndim):
                corner_weight *= norm_weights[i] if corner[i] == 1 else (1.0 - norm_weights[i])
                
            while corner_weight.ndim < corner_vals.ndim:
                corner_weight = corner_weight.unsqueeze(-1)
                
            corner_weight = corner_weight.to(corner_vals.dtype)
            res += corner_vals * corner_weight
            
        if not self.bounds_error and self.fill_value is not None:
            oob_mask = out_of_bounds
            while oob_mask.ndim < res.ndim:
                oob_mask = oob_mask.unsqueeze(-1)
            fill_val_tensor = tr.tensor(self.fill_value, dtype=res.dtype, device=res.device)
            res = tr.where(oob_mask, fill_val_tensor, res)
            
        return TorchArray(res.reshape(*original_shape, *res.shape[1:]))


class PPoly:
    def __init__(self, c, x, extrapolate=None, axis=0):
        if rp.use_jax:
            jnp = rp.jax_handle.numpy
            self.c = jnp.asarray(c)
            self.x = jnp.asarray(x)
            self._nan_val = j.numpy.nan
            self._device = None
        elif rp.use_torch:
            self.c = tr.as_tensor(c)
            self.x = tr.as_tensor(x)
            self._nan_val = tr.tensor(float('nan'), dtype=self.c.dtype, device=self.c.device)
            # Access the device directly to honor the strict fail-loudly mandate
            self._device = self.c.device
        else:
            self.c = np.asarray(c)
            self.x = np.asarray(x)
            self._nan_val = np.nan
            self._device = None
            
        self.axis = axis
        self.extrapolate = extrapolate
        self._nu_factors = {}
        self._perm_cache = {}
        
        # Cache unchanging structural dimensions at build time to clear the hot path
        self._k = self.c.shape[0] - 1
        self._len_T = self.c.ndim - 2
        self._n_minus_2 = len(self.x) - 2
        self._trailing_dims = (...,) + (None,) * self._len_T

    def __call__(self, x_new, nu=0, extrapolate=None):
        if extrapolate is None:
            extrapolate = self.extrapolate
        if extrapolate is None:
            extrapolate = True

        x_new = rp.asarray(x_new)
        x_eval = x_new
        
        if extrapolate == 'periodic':
            period = self.x[-1] - self.x[0]
            x_eval = self.x[0] + rp.remainder(x_new - self.x[0], period)
            
        idx = rp.searchsorted(self.x, x_eval, side='right') - 1
        idx = rp.clip(idx, 0, self._n_minus_2)
        
        dt = x_eval - self.x[idx]
        
        if nu > self._k:
            res_shape = x_new.shape + self.c.shape[2:]
            res = rp.zeros(res_shape, dtype=self.c.dtype, device=self._device)
        else:
            if nu not in self._nu_factors:
                import math
                factors = []
                for m in range(1, self._k - nu + 1):
                    factors.append(math.factorial(self._k - m) // math.factorial(self._k - m - nu))
                first_factor = math.factorial(self._k) // math.factorial(self._k - nu)
                self._nu_factors[nu] = (first_factor, factors)
                
            first_factor, factors = self._nu_factors[nu]
            res = self.c[0, idx] * first_factor
            
            dt_expanded = dt[self._trailing_dims]
            
            for m in range(1, self._k - nu + 1):
                res = res * dt_expanded + self.c[m, idx] * factors[m - 1]
                
            if not extrapolate:
                out_of_bounds = (x_eval < self.x[0]) | (x_eval > self.x[-1])
                out_of_bounds_expanded = out_of_bounds[self._trailing_dims]
                res = rp.where(out_of_bounds_expanded, self._nan_val, res)
                
        N_new = x_new.ndim
        
        # Simplify the lookup key since internal matrix layout properties are frozen constants
        if N_new not in self._perm_cache:
            self._perm_cache[N_new] = list(range(N_new, N_new + self.axis)) + list(range(N_new)) + list(range(N_new + self.axis, N_new + self._len_T))
            
        perm = self._perm_cache[N_new]
        if perm:
            res = rp.transpose(res, perm)
        return res

    def derivative(self, nu=1):
        k = self.c.shape[0] - 1
        if nu < 0:
            return self.antiderivative(-nu)
        if nu == 0:
            return self
            
        if nu > k:
            if rp.use_torch:
                c_new = tr.zeros((1,) + self.c.shape[1:], dtype=self.c.dtype, device=self.c.device)
            elif rp.use_jax:
                jnp = rp.jax_handle.numpy
                c_new = jnp.zeros((1,) + self.c.shape[1:], dtype=self.c.dtype)
            else:
                c_new = np.zeros((1,) + self.c.shape[1:], dtype=self.c.dtype)
        else:
            import math
            new_slices = []
            for m in range(k - nu + 1):
                factor = math.factorial(k - m) // math.factorial(k - m - nu)
                new_slices.append(self.c[m] * factor)
                
            if rp.use_torch:
                c_new = tr.stack(new_slices, dim=0)
            elif rp.use_jax:
                jnp = rp.jax_handle.numpy
                c_new = jnp.stack(new_slices, axis=0)
            else:
                c_new = np.stack(new_slices, axis=0)
                
        return PPoly(c_new, self.x, extrapolate=self.extrapolate, axis=self.axis)

    def antiderivative(self, nu=1):
        if nu < 0:
            return self.derivative(-nu)
        if nu == 0:
            return self
            
        extrap = self.extrapolate
        if extrap == 'periodic':
            extrap = False
            
        current = self
        for _ in range(nu):
            k = current.c.shape[0] - 1
            if rp.use_torch:
                h = current.x[1:] - current.x[:-1]
                h_exp = h
                for _ in range(current.c.dim() - 2):
                    h_exp = h_exp.unsqueeze(-1)
                
                Delta = 0.0
                for m in range(k + 1):
                    Delta = Delta + (current.c[m] / (k + 1 - m)) * (h_exp ** (k + 1 - m))
                    
                zero = tr.zeros((1,) + Delta.shape[1:], dtype=Delta.dtype, device=Delta.device)
                C = tr.cumsum(tr.cat([zero, Delta], dim=0), dim=0)[:-1]
                
                new_slices = []
                for m in range(k + 1):
                    new_slices.append(current.c[m] / (k + 1 - m))
                new_slices.append(C)
                c_new = tr.stack(new_slices, dim=0)
                
            elif rp.use_jax:
                jnp = rp.jax_handle.numpy
                h = current.x[1:] - current.x[:-1]
                h_exp = h
                for _ in range(current.c.ndim - 2):
                    h_exp = jnp.expand_dims(h_exp, -1)
                
                Delta = 0.0
                for m in range(k + 1):
                    Delta = Delta + (current.c[m] / (k + 1 - m)) * (h_exp ** (k + 1 - m))
                    
                zero = jnp.zeros((1,) + Delta.shape[1:], dtype=Delta.dtype)
                C = jnp.cumsum(jnp.concatenate([zero, Delta], axis=0), axis=0)[:-1]
                
                new_slices = []
                for m in range(k + 1):
                    new_slices.append(current.c[m] / (k + 1 - m))
                new_slices.append(C)
                c_new = jnp.stack(new_slices, axis=0)
                
            else:
                h = current.x[1:] - current.x[:-1]
                h_exp = h
                for _ in range(current.c.ndim - 2):
                    h_exp = h_exp[..., None]
                
                Delta = 0.0
                for m in range(k + 1):
                    Delta = Delta + (current.c[m] / (k + 1 - m)) * (h_exp ** (k + 1 - m))
                    
                zero = np.zeros((1,) + Delta.shape[1:], dtype=Delta.dtype)
                C = np.cumsum(np.concatenate([zero, Delta], axis=0), axis=0)[:-1]
                
                new_slices = []
                for m in range(k + 1):
                    new_slices.append(current.c[m] / (k + 1 - m))
                new_slices.append(C)
                c_new = np.stack(new_slices, axis=0)
                
            current = PPoly(c_new, current.x, extrapolate=extrap, axis=current.axis)
            
        return current

    def integrate(self, a, b, extrapolate=None):
        if extrapolate is None:
            extrapolate = self.extrapolate
        if extrapolate is None:
            extrapolate = True

        # Swap integration bounds if needed
        sign = 1.0
        if b < a:
            a, b = b, a
            sign = -1.0

        I = self.antiderivative()

        if extrapolate == 'periodic':
            xs, xe = self.x[0], self.x[-1]
            period = xe - xs
            interval = b - a
            
            n_periods = int(interval // period)
            left = interval % period
            
            val_xs = I(xs, extrapolate=False)
            val_xe = I(xe, extrapolate=False)
            period_integral = val_xe - val_xs
            
            res = period_integral * n_periods
            
            a_mapped = xs + (a - xs) % period
            b_mapped = a_mapped + left
            
            if b_mapped <= xe:
                res = res + (I(b_mapped, extrapolate=False) - I(a_mapped, extrapolate=False))
            else:
                res = res + (val_xe - I(a_mapped, extrapolate=False))
                res = res + (I(xs + (b_mapped - xe), extrapolate=False) - val_xs)
        else:
            res = I(b, extrapolate=extrapolate) - I(a, extrapolate=extrapolate)
            
        return res * sign

    def roots(self, discontinuity=True, extrapolate=None):
        if extrapolate is None:
            extrapolate = self.extrapolate
        if extrapolate is None:
            extrapolate = True
            
        if self.c.ndim > 2:
            raise NotImplementedError("For multidimensional PPoly, this method is not implemented.")
            
        k = self.c.shape[0] - 1
        if k == 0:
            if rp.use_torch:
                return TorchArray(tr.zeros(0, dtype=self.x.dtype, device=self.x.device))
            else:
                return rp.array(np.zeros(0, dtype=self.x.dtype))
                
        n = len(self.x)
        h = self.x[1:] - self.x[:-1]
        
        c0 = self.c[0]
        if rp.use_torch:
            c0_safe = tr.where(tr.abs(c0) < 1e-12, tr.tensor(1e-12, dtype=c0.dtype, device=c0.device), c0)
            # c_norm shape: (k, n-1); transpose to (n-1, k) for row assignment
            c_norm = self.c[1:] / c0_safe.unsqueeze(0)
            c_norm_T = c_norm.permute(1, 0)  # (n-1, k)

            C = tr.zeros((n - 1, k, k), dtype=self.c.dtype, device=self.c.device)
            C[:, 0, :] = -c_norm_T
            if k > 1:
                idx_row = tr.arange(1, k, device=self.c.device)
                idx_col = tr.arange(k - 1, device=self.c.device)
                C[:, idx_row, idx_col] = 1.0

            roots_raw = tr.linalg.eigvals(C).detach().cpu().numpy()
        elif rp.use_jax:
            jnp = rp.jax_handle.numpy
            c0_safe = jnp.where(jnp.abs(c0) < 1e-12, 1e-12, c0)
            # c_norm shape: (k, n-1); transpose to (n-1, k)
            c_norm = self.c[1:] / c0_safe[None, :]
            c_norm_T = jnp.swapaxes(c_norm, 0, 1)  # (n-1, k)

            C = jnp.zeros((n - 1, k, k), dtype=self.c.dtype)
            C = C.at[:, 0, :].set(-c_norm_T)
            if k > 1:
                C = C.at[:, jnp.arange(1, k), jnp.arange(k - 1)].set(1.0)

            roots_raw = np.array(jnp.linalg.eigvals(C))
        else:
            c0_safe = np.where(np.abs(c0) < 1e-12, 1e-12, c0)
            # c_norm shape: (k, n-1); transpose to (n-1, k)
            c_norm = np.asarray(self.c[1:]) / c0_safe[None, :]
            c_norm_T = c_norm.T.reshape(n - 1, k)  # (n-1, k)

            C = np.zeros((n - 1, k, k), dtype=float)
            C[:, 0, :] = -c_norm_T
            if k > 1:
                C[:, np.arange(1, k), np.arange(k - 1)] = 1.0

            roots_raw = np.linalg.eigvals(C)
            
        x_np = np.array(self.x)
        h_np = np.array(h)
        real_roots = []
        
        for i in range(n - 1):
            roots_seg = roots_raw[i]
            h_i = h_np[i]
            x_i = x_np[i]

            is_real = np.abs(roots_seg.imag) < 1e-12
            r_seg = roots_seg.real[is_real]

            if i == n - 2:
                in_interval = (r_seg >= 0) & (r_seg <= h_i)
            else:
                in_interval = (r_seg >= 0) & (r_seg < h_i)

            if extrapolate:
                if i == 0:
                    in_interval = in_interval | (r_seg < 0)
                if i == n - 2:
                    in_interval = in_interval | (r_seg > h_i)

            valid_roots = r_seg[in_interval]
            for r in valid_roots:
                real_roots.append(x_i + r)

        if not real_roots:
            result = np.zeros(0)
        else:
            arr = np.sort(np.array(real_roots))
            # Deduplicate roots that are within numerical tolerance of each other
            mask = np.concatenate([[True], np.diff(arr) > 1e-10])
            result = arr[mask]

        if rp.use_torch:
            return TorchArray(tr.as_tensor(result, dtype=self.x.dtype, device=self.x.device))
        elif rp.use_jax:
            jnp = rp.jax_handle.numpy
            return rp.array(jnp.asarray(result, dtype=self.x.dtype))
        else:
            return rp.array(np.asarray(result, dtype=self.x.dtype))

    def solve(self, y=0.0, discontinuity=True, extrapolate=None):
        if rp.use_torch:
            c_last_shifted = self.c[-1] - y
            c_shifted = tr.cat([self.c[:-1], c_last_shifted.unsqueeze(0)], dim=0)
        elif rp.use_jax:
            c_shifted = self.c.at[-1].set(self.c[-1] - y)
        else:
            c_last_shifted = np.asarray(self.c[-1]) - y
            c_shifted = np.concatenate([np.asarray(self.c[:-1]), c_last_shifted[None]], axis=0)

        temp = PPoly(c_shifted, self.x, extrapolate=self.extrapolate, axis=self.axis)
        return temp.roots(discontinuity=discontinuity, extrapolate=extrapolate)


class CubicSpline(PPoly):
    def __init__(self, x, y, axis=0, bc_type='not-a-knot', extrapolate=None):
        if extrapolate is None:
            if bc_type == 'periodic':
                extrapolate = 'periodic'
            else:
                extrapolate = True
                
        if rp.use_jax:
            jnp = rp.jax_handle.numpy
            x = jnp.asarray(x)
            y = jnp.asarray(y)
        elif rp.use_torch:
            x = tr.as_tensor(x)
            y = tr.as_tensor(y)
        else:
            x = np.asarray(x)
            y = np.asarray(y)
            
        n = len(x)
        if n < 2:
            raise ValueError("x must contain at least 2 elements")
            
        # Normalize axis to be positive
        if axis < 0:
            axis = axis + y.ndim
            
        if y.shape[axis] != n:
            raise ValueError(f"y shape along axis {axis} must be {n}, got {y.shape[axis]}")
            
        if rp.use_torch:
            y_rolled = tr.movedim(y, axis, 0)
        elif rp.use_jax:
            jnp = rp.jax_handle.numpy
            y_rolled = jnp.moveaxis(y, axis, 0)
        else:
            y_rolled = np.moveaxis(y, axis, 0)
            
        h = x[1:] - x[:-1]
        
        h_exp = h
        for _ in range(y_rolled.ndim - 1):
            h_exp = h_exp[..., None]
            
        d = (y_rolled[1:] - y_rolled[:-1]) / h_exp
        
        h_left = h[1:]
        h_right = h[:-1]
        for _ in range(y_rolled.ndim - 1):
            h_left = h_left[..., None]
            h_right = h_right[..., None]
            
        if bc_type == 'periodic':
            left_type = right_type = 'periodic'
        else:
            if isinstance(bc_type, tuple):
                left_bc, right_bc = bc_type
            else:
                left_bc = right_bc = bc_type
                
            left_type, left_val = self._parse_bc(left_bc)
            right_type, right_val = self._parse_bc(right_bc)
            
        trailing_shape = y_rolled.shape[1:]
        
        if rp.use_jax:
            jnp = rp.jax_handle.numpy
            A = jnp.zeros((n, n), dtype=x.dtype)
            B = jnp.zeros((n,) + trailing_shape, dtype=y_rolled.dtype)
            
            rows = jnp.arange(1, n - 1)
            A = A.at[rows, rows - 1].set(h[1:])
            A = A.at[rows, rows].set(2.0 * (h[:-1] + h[1:]))
            A = A.at[rows, rows + 1].set(h[:-1])
            B = B.at[1:-1].set(3.0 * (h_left * d[:-1] + h_right * d[1:]))
            
            if bc_type == 'periodic':
                A = A.at[0, 0].set(2.0 * (h[0] + h[n-2]))
                A = A.at[0, 1].set(h[n-2])
                A = A.at[0, n-2].set(h[0])
                B = B.at[0].set(3.0 * (h[n-2] * d[0] + h[0] * d[n-2]))
                A = A.at[n-1, 0].set(-1.0)
                A = A.at[n-1, n-1].set(1.0)
            else:
                deriv_l = jnp.asarray(left_val, dtype=y_rolled.dtype) if left_val is not None else 0.0
                deriv_r = jnp.asarray(right_val, dtype=y_rolled.dtype) if right_val is not None else 0.0
                
                if left_type == 1:
                    A = A.at[0, 0].set(1.0)
                    B = B.at[0].set(deriv_l)
                elif left_type == 2:
                    A = A.at[0, 0].set(2.0)
                    A = A.at[0, 1].set(1.0)
                    B = B.at[0].set(3.0 * d[0] - 0.5 * h[0] * deriv_l)
                elif left_type == 'not-a-knot':
                    if n == 2:
                        A = A.at[0, 0].set(1.0)
                        B = B.at[0].set(d[0])
                    elif n == 3:
                        A = A.at[0, 0].set(1.0)
                        A = A.at[0, 1].set(1.0)
                        B = B.at[0].set(2.0 * d[0])
                    else:
                        A = A.at[0, 0].set(h[1] ** 2)
                        A = A.at[0, 1].set(h[1] ** 2 - h[0] ** 2)
                        A = A.at[0, 2].set(-h[0] ** 2)
                        B = B.at[0].set(2.0 * h[1] ** 2 * d[0] - 2.0 * h[0] ** 2 * d[1])
                
                if right_type == 1:
                    A = A.at[n-1, n-1].set(1.0)
                    B = B.at[n-1].set(deriv_r)
                elif right_type == 2:
                    A = A.at[n-1, n-2].set(1.0)
                    A = A.at[n-1, n-1].set(2.0)
                    B = B.at[n-1].set(3.0 * d[n-2] + 0.5 * h[n-2] * deriv_r)
                elif right_type == 'not-a-knot':
                    if n == 2:
                        A = A.at[n-1, n-1].set(1.0)
                        B = B.at[n-1].set(d[0])
                    elif n == 3:
                        A = A.at[n-1, n-2].set(1.0)
                        A = A.at[n-1, n-1].set(1.0)
                        B = B.at[n-1].set(2.0 * d[1])
                    else:
                        A = A.at[n-1, n-3].set(-h[n-2] ** 2)
                        A = A.at[n-1, n-2].set(h[n-3] ** 2 - h[n-2] ** 2)
                        A = A.at[n-1, n-1].set(h[n-3] ** 2)
                        B = B.at[n-1].set(2.0 * h[n-3] ** 2 * d[n-2] - 2.0 * h[n-2] ** 2 * d[n-3])
                        
            B_flat = B.reshape(n, -1)
            s_flat = jnp.linalg.solve(A, B_flat)
            s = s_flat.reshape(B.shape)
            
        elif rp.use_torch:
            A = tr.zeros((n, n), dtype=x.dtype, device=x.device)
            B = tr.zeros((n,) + trailing_shape, dtype=y_rolled.dtype, device=y_rolled.device)
            
            rows = tr.arange(1, n - 1, device=x.device)
            A[rows, rows - 1] = h[1:]
            A[rows, rows] = 2.0 * (h[:-1] + h[1:])
            A[rows, rows + 1] = h[:-1]
            B[1:-1] = 3.0 * (h_left * d[:-1] + h_right * d[1:])
            
            if bc_type == 'periodic':
                A[0, 0] = 2.0 * (h[0] + h[n-2])
                A[0, 1] = h[n-2]
                A[0, n-2] = h[0]
                B[0] = 3.0 * (h[n-2] * d[0] + h[0] * d[n-2])
                A[n-1, 0] = -1.0
                A[n-1, n-1] = 1.0
            else:
                deriv_l = tr.as_tensor(left_val, dtype=y_rolled.dtype, device=y_rolled.device) if left_val is not None else tr.tensor(0.0, dtype=y_rolled.dtype, device=y_rolled.device)
                deriv_r = tr.as_tensor(right_val, dtype=y_rolled.dtype, device=y_rolled.device) if right_val is not None else tr.tensor(0.0, dtype=y_rolled.dtype, device=y_rolled.device)
                
                if left_type == 1:
                    A[0, 0] = 1.0
                    B[0] = deriv_l
                elif left_type == 2:
                    A[0, 0] = 2.0
                    A[0, 1] = 1.0
                    B[0] = 3.0 * d[0] - 0.5 * h[0] * deriv_l
                elif left_type == 'not-a-knot':
                    if n == 2:
                        A[0, 0] = 1.0
                        B[0] = d[0]
                    elif n == 3:
                        A[0, 0] = 1.0
                        A[0, 1] = 1.0
                        B[0] = 2.0 * d[0]
                    else:
                        A[0, 0] = h[1] ** 2
                        A[0, 1] = h[1] ** 2 - h[0] ** 2
                        A[0, 2] = -h[0] ** 2
                        B[0] = 2.0 * h[1] ** 2 * d[0] - 2.0 * h[0] ** 2 * d[1]
                        
                if right_type == 1:
                    A[n-1, n-1] = 1.0
                    B[n-1] = deriv_r
                elif right_type == 2:
                    A[n-1, n-2] = 1.0
                    A[n-1, n-1] = 2.0
                    B[n-1] = 3.0 * d[n-2] + 0.5 * h[n-2] * deriv_r
                elif right_type == 'not-a-knot':
                    if n == 2:
                        A[n-1, n-1] = 1.0
                        B[n-1] = d[0]
                    elif n == 3:
                        A[n-1, n-2] = 1.0
                        A[n-1, n-1] = 1.0
                        B[n-1] = 2.0 * d[1]
                    else:
                        A[n-1, n-3] = -h[n-2] ** 2
                        A[n-1, n-2] = h[n-3] ** 2 - h[n-2] ** 2
                        A[n-1, n-1] = h[n-3] ** 2
                        B[n-1] = 2.0 * h[n-3] ** 2 * d[n-2] - 2.0 * h[n-2] ** 2 * d[n-3]
                        
            B_flat = B.reshape(n, -1)
            s_flat = tr.linalg.solve(A, B_flat)
            s = s_flat.reshape(B.shape)
            
        else:
            A = np.zeros((n, n), dtype=x.dtype)
            B = np.zeros((n,) + trailing_shape, dtype=y_rolled.dtype)
            
            rows = np.arange(1, n - 1)
            A[rows, rows - 1] = h[1:]
            A[rows, rows] = 2.0 * (h[:-1] + h[1:])
            A[rows, rows + 1] = h[:-1]
            B[1:-1] = 3.0 * (h_left * d[:-1] + h_right * d[1:])
            
            if bc_type == 'periodic':
                A[0, 0] = 2.0 * (h[0] + h[n-2])
                A[0, 1] = h[n-2]
                A[0, n-2] = h[0]
                B[0] = 3.0 * (h[n-2] * d[0] + h[0] * d[n-2])
                A[n-1, 0] = -1.0
                A[n-1, n-1] = 1.0
            else:
                deriv_l = np.asarray(left_val, dtype=y_rolled.dtype) if left_val is not None else 0.0
                deriv_r = np.asarray(right_val, dtype=y_rolled.dtype) if right_val is not None else 0.0
                
                if left_type == 1:
                    A[0, 0] = 1.0
                    B[0] = deriv_l
                elif left_type == 2:
                    A[0, 0] = 2.0
                    A[0, 1] = 1.0
                    B[0] = 3.0 * d[0] - 0.5 * h[0] * deriv_l
                elif left_type == 'not-a-knot':
                    if n == 2:
                        A[0, 0] = 1.0
                        B[0] = d[0]
                    elif n == 3:
                        A[0, 0] = 1.0
                        A[0, 1] = 1.0
                        B[0] = 2.0 * d[0]
                    else:
                        A[0, 0] = h[1] ** 2
                        A[0, 1] = h[1] ** 2 - h[0] ** 2
                        A[0, 2] = -h[0] ** 2
                        B[0] = 2.0 * h[1] ** 2 * d[0] - 2.0 * h[0] ** 2 * d[1]
                        
                if right_type == 1:
                    A[n-1, n-1] = 1.0
                    B[n-1] = deriv_r
                elif right_type == 2:
                    A[n-1, n-2] = 1.0
                    A[n-1, n-1] = 2.0
                    B[n-1] = 3.0 * d[n-2] + 0.5 * h[n-2] * deriv_r
                elif right_type == 'not-a-knot':
                    if n == 2:
                        A[n-1, n-1] = 1.0
                        B[n-1] = d[0]
                    elif n == 3:
                        A[n-1, n-2] = 1.0
                        A[n-1, n-1] = 1.0
                        B[n-1] = 2.0 * d[1]
                    else:
                        A[n-1, n-3] = -h[n-2] ** 2
                        A[n-1, n-2] = h[n-3] ** 2 - h[n-2] ** 2
                        A[n-1, n-1] = h[n-3] ** 2
                        B[n-1] = 2.0 * h[n-3] ** 2 * d[n-2] - 2.0 * h[n-2] ** 2 * d[n-3]
                        
            B_flat = B.reshape(n, -1)
            s_flat = np.linalg.solve(A, B_flat)
            s = s_flat.reshape(B.shape)
            
        c3 = y_rolled[:-1]
        c2 = s[:-1]
        
        if rp.use_torch:
            c1 = 3.0 * d / h_exp - (2.0 * s[:-1] + s[1:]) / h_exp
            c0 = (s[:-1] + s[1:]) / (h_exp ** 2) - 2.0 * d / (h_exp ** 2)
            c = tr.stack([c0, c1, c2, c3], dim=0)
        elif rp.use_jax:
            c1 = 3.0 * d / h_exp - (2.0 * s[:-1] + s[1:]) / h_exp
            c0 = (s[:-1] + s[1:]) / (h_exp ** 2) - 2.0 * d / (h_exp ** 2)
            c = jnp.stack([c0, c1, c2, c3], axis=0)
        else:
            c1 = 3.0 * d / h_exp - (2.0 * s[:-1] + s[1:]) / h_exp
            c0 = (s[:-1] + s[1:]) / (h_exp ** 2) - 2.0 * d / (h_exp ** 2)
            c = np.stack([c0, c1, c2, c3], axis=0)
            
        super().__init__(c, x, extrapolate=extrapolate, axis=axis)
        self.bc_type = bc_type

        # ----------------------------------------------------------------------
        #  Rank-Invariant Cache Warm-Up
        # ----------------------------------------------------------------------
        try:
            if rp.use_jax:
                jnp = rp.jax_handle.numpy
                # Pass a 1D array matching the operational rank (-1,) used 
                # by the surrogate wrappers. This forces _perm_cache[1] to 
                # initialize completely before the first JAX trace.
                self(jnp.array([x[0]], dtype=x.dtype))
            elif rp.use_torch:
                self(tr.as_tensor([x[0]], dtype=x.dtype))
            else:
                self(np.array([x[0]], dtype=x.dtype))
        except Exception:
            pass

    def _parse_bc(self, bc):
        if isinstance(bc, str):
            if bc == 'clamped':
                return 1, 0.0
            elif bc == 'natural':
                return 2, 0.0
            elif bc == 'not-a-knot':
                return 'not-a-knot', None
            else:
                raise ValueError(f"Unknown boundary condition: {bc}")
        elif isinstance(bc, tuple):
            if len(bc) != 2:
                raise ValueError("Boundary condition tuple must have length 2")
            order, val = bc
            if order not in (1, 2):
                raise ValueError("Boundary condition order must be 1 or 2")
            return order, val
        else:
            raise ValueError(f"Invalid boundary condition type: {bc}")


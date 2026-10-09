# src.py
# (c) Copyright 2024 Aerospace Research Community LLC

# Created:  Oct 2024 M. Clarke
# Modified: Jan 2026, E. Botero

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------  

import RNUMPY as rp
try:
    import scipy.spatial.transform
except ImportError:
    pass

j   = rp.jax_handle
np  = rp.numpy_handle
sp  = rp.scipy_handle
tr  = rp.torch_handle

if j is not None:
    try:
        import jax.scipy.spatial.transform
    except ImportError:
        pass

jnp = j.numpy if j is not None else None
jspatial = j.scipy.spatial.transform if j is not None else None

if tr is not None:
    class _TorchRotation:
        def __init__(self, quat, normalize=True):
            self._quat = quat.clone() if isinstance(quat, tr.Tensor) else tr.as_tensor(quat)
            if normalize:
                norm = tr.linalg.norm(self._quat, dim=-1, keepdim=True)
                self._quat = tr.where(norm > 0, self._quat / norm, self._quat)
                
        @classmethod
        def from_quat(cls, quat):
            return cls(quat, normalize=True)
            
        def as_quat(self, canonical=False, scalar_first=False):
            q = self._quat.clone()
            if canonical:
                w = q[..., 3:4]
                q = tr.where(w < 0, -q, q)
            if scalar_first:
                q = tr.cat([q[..., 3:4], q[..., :3]], dim=-1)
            return q
            
        def as_matrix(self):
            x, y, z, w = self._quat[..., 0], self._quat[..., 1], self._quat[..., 2], self._quat[..., 3]
            x2, y2, z2, w2 = x*x, y*y, z*z, w*w
            xy, zw, xz, yw, yz, xw = x*y, z*w, x*z, y*w, y*z, x*w
            
            m00 = x2 - y2 - z2 + w2
            m11 = -x2 + y2 - z2 + w2
            m22 = -x2 - y2 + z2 + w2
            m01 = 2 * (xy - zw)
            m10 = 2 * (xy + zw)
            m02 = 2 * (xz + yw)
            m20 = 2 * (xz - yw)
            m12 = 2 * (yz - xw)
            m21 = 2 * (yz + xw)
            
            row0 = tr.stack([m00, m01, m02], dim=-1)
            row1 = tr.stack([m10, m11, m12], dim=-1)
            row2 = tr.stack([m20, m21, m22], dim=-1)
            return tr.stack([row0, row1, row2], dim=-2)

        def __mul__(self, other):
            q1 = self._quat
            q2 = other._quat
            
            x1, y1, z1, w1 = q1[..., 0], q1[..., 1], q1[..., 2], q1[..., 3]
            x2, y2, z2, w2 = q2[..., 0], q2[..., 1], q2[..., 2], q2[..., 3]
            
            x = w1*x2 + x1*w2 + y1*z2 - z1*y2
            y = w1*y2 - x1*z2 + y1*w2 + z1*x2
            z = w1*z2 + x1*y2 - y1*x2 + z1*w2
            w = w1*w2 - x1*x2 - y1*y2 - z1*z2
            
            q_new = tr.stack([x, y, z, w], dim=-1)
            return _TorchRotation(q_new, normalize=True)

        @classmethod
        def from_rotvec(cls, rotvec, degrees=False):
            rv = tr.as_tensor(rotvec)
            if degrees:
                rv = rv * (np.pi / 180.0)
            angle = tr.linalg.norm(rv, dim=-1, keepdim=True)
            half_angle = 0.5 * angle
            
            small_angle = angle < 1e-4
            sin_half_over_angle = tr.where(small_angle,
                                           0.5 - (angle**2) / 48.0,
                                           tr.sin(half_angle) / tr.clamp(angle, min=1e-8))
                                           
            q_vec = rv * sin_half_over_angle
            q_w = tr.cos(half_angle)
            
            q = tr.cat([q_vec, q_w], dim=-1)
            return cls(q, normalize=True)
            
        def as_rotvec(self, degrees=False):
            q = self._quat
            q_vec = q[..., :3]
            q_w = q[..., 3:4]
            
            sin_half = tr.linalg.norm(q_vec, dim=-1, keepdim=True)
            cos_half = q_w
            
            angle = 2.0 * tr.atan2(sin_half, cos_half)
            
            small_angle = sin_half < 1e-4
            angle_over_sin_half = tr.where(small_angle,
                                           2.0 + (sin_half**2) * (2.0/3.0),
                                           angle / tr.clamp(sin_half, min=1e-8))
            
            rv = q_vec * angle_over_sin_half
            if degrees:
                rv = rv * (180.0 / np.pi)
            return rv

        @classmethod
        def from_euler(cls, seq, angles, degrees=False):
            angles = tr.as_tensor(angles)
            if degrees:
                angles = angles * (np.pi / 180.0)
                
            intrinsic = seq.islower()
            seq = seq.lower()
            
            axes_dict = {'x': [1.,0.,0.], 'y': [0.,1.,0.], 'z': [0.,0.,1.]}
            
            if angles.ndim == 0 or (angles.ndim >= 1 and angles.shape[-1] != len(seq)):
                angles = angles.unsqueeze(-1)
                
            rotations = []
            for i, ax in enumerate(seq):
                axis = tr.tensor(axes_dict[ax], dtype=angles.dtype, device=angles.device)
                a = angles[..., i:i+1]
                rv = a * axis
                rotations.append(cls.from_rotvec(rv, degrees=False))
                
            if not intrinsic:
                rotations = rotations[::-1]
                
            res = rotations[0]
            for r in rotations[1:]:
                res = res * r
            return res

        def as_euler(self, seq, degrees=False):
            # NOTE: as_euler is computed via SciPy; gradients DO NOT flow through this method.
            # For differentiable applications, use as_matrix() or as_rotvec() instead.
            import scipy.spatial.transform
            import warnings
            if self._quat.requires_grad:
                warnings.warn(
                    "as_euler() in Torch mode does not support gradients. "
                    "Use as_matrix() or as_rotvec() for differentiable outputs.",
                    UserWarning, stacklevel=2
                )
            q_np = self._quat.detach().cpu().numpy()
            r_sp = scipy.spatial.transform.Rotation.from_quat(q_np)
            euler_np = r_sp.as_euler(seq, degrees=degrees)
            return tr.tensor(euler_np, dtype=self._quat.dtype, device=self._quat.device)
            
        def magnitude(self):
            return tr.linalg.norm(self.as_rotvec(), dim=-1)
            
        @classmethod
        def concatenate(cls, rotations):
            quats = [r._quat for r in rotations]
            q = tr.cat(quats, dim=0)
            return cls(q, normalize=False)
            
        @classmethod
        def identity(cls, num=None):
            if num is None:
                q = tr.tensor([0., 0., 0., 1.])
            else:
                q = tr.zeros((num, 4))
                q[..., 3] = 1.0
            return cls(q, normalize=False)

        def mean(self, weights=None):
            q = self._quat
            if weights is not None:
                w = tr.as_tensor(weights, dtype=q.dtype, device=q.device)
                q0 = q[0]
                sign = tr.sign((q * q0).sum(dim=-1, keepdim=True))
                q = q * sign
                q_mean = (q * w.unsqueeze(-1)).sum(dim=0)
            else:
                q0 = q[0]
                sign = tr.sign((q * q0).sum(dim=-1, keepdim=True))
                q_mean = (q * sign).mean(dim=0)
            return _TorchRotation(q_mean, normalize=True)

        def __len__(self):
            if self._quat.ndim == 1:
                return 1
            return self._quat.shape[0]

        def __getitem__(self, key):
            return _TorchRotation(self._quat[key], normalize=False)
            
        @classmethod
        def from_matrix(cls, matrix):
            m = tr.as_tensor(matrix)
            m00, m01, m02 = m[..., 0, 0], m[..., 0, 1], m[..., 0, 2]
            m10, m11, m12 = m[..., 1, 0], m[..., 1, 1], m[..., 1, 2]
            m20, m21, m22 = m[..., 2, 0], m[..., 2, 1], m[..., 2, 2]
            
            trace = m00 + m11 + m22
            
            # Four candidate quaternions, one per branch of the Shuster method.
            # We compute ALL four and select via tr.where so autograd flows fully.
            
            # Branch 0: trace > 0
            S0 = 2.0 * tr.sqrt(tr.clamp(trace + 1.0, min=1e-6))
            q0 = tr.stack([
                (m21 - m12) / S0, (m02 - m20) / S0, (m10 - m01) / S0, 0.25 * S0
            ], dim=-1)
            
            # Branch 1: m00 dominant
            S1 = 2.0 * tr.sqrt(tr.clamp(1.0 + m00 - m11 - m22, min=1e-6))
            q1 = tr.stack([
                0.25 * S1, (m01 + m10) / S1, (m02 + m20) / S1, (m21 - m12) / S1
            ], dim=-1)
            
            # Branch 2: m11 dominant
            S2 = 2.0 * tr.sqrt(tr.clamp(1.0 + m11 - m00 - m22, min=1e-6))
            q2 = tr.stack([
                (m01 + m10) / S2, 0.25 * S2, (m12 + m21) / S2, (m02 - m20) / S2
            ], dim=-1)
            
            # Branch 3: m22 dominant
            S3 = 2.0 * tr.sqrt(tr.clamp(1.0 + m22 - m00 - m11, min=1e-6))
            q3 = tr.stack([
                (m02 + m20) / S3, (m12 + m21) / S3, 0.25 * S3, (m10 - m01) / S3
            ], dim=-1)
            
            # Select the numerically stable branch via tr.where (differentiable)
            cond0 = (trace > 0)[..., None]
            cond1 = ((m00 > m11) & (m00 > m22))[..., None]
            cond2 = (m11 > m22)[..., None]
            
            q = tr.where(cond0, q0,
                    tr.where(cond1, q1,
                        tr.where(cond2, q2, q3)))
            
            return cls(q, normalize=True)

        def apply(self, vectors, inverse=False):
            v = tr.as_tensor(vectors).to(self._quat.dtype)
            q = self._quat
            if inverse:
                q = tr.cat([-q[..., :3], q[..., 3:4]], dim=-1)
                
            q_vec = q[..., :3]
            q_w = q[..., 3:4]
            
            # Fix for PyTorch linalg.cross dimension mismatch (requires same ndim)
            if v.ndim > q.ndim:
                for _ in range(v.ndim - q.ndim):
                    q_vec = q_vec.unsqueeze(-2)
                    q_w = q_w.unsqueeze(-2)
            elif q.ndim > v.ndim:
                for _ in range(q.ndim - v.ndim):
                    v = v.unsqueeze(-2)
                
            a = tr.cross(q_vec, v, dim=-1)
            b = tr.cross(q_vec, a + q_w * v, dim=-1)
            return v + 2.0 * b

        def inv(self):
            q = tr.cat([-self._quat[..., :3], self._quat[..., 3:4]], dim=-1)
            return _TorchRotation(q, normalize=False)

# End of _TorchRotation class guard
else:
    # Stub when PyTorch is not installed
    class _TorchRotation:  # noqa: F811
        def __init__(self, *args, **kwargs):
            raise ImportError("PyTorch is not installed. Cannot use _TorchRotation.")

class Rotation(): 
    def __init__(self, native):
        self._native = native

    # --- Methods supporting both Instance and Functional API ---

    def apply(self, vectors, inverse=False): 
        native_R = getattr(self, '_native', self)
        res = native_R.apply(vectors, inverse=inverse)
        if rp.use_jax: return res
        elif rp.use_torch: return rp.TorchArray(res)
        else: return rp.NumpyArray(res)
        
    def as_euler(R, seq, degrees=False): 
        native_R = getattr(R, '_native', R)
        res = native_R.as_euler(seq=seq, degrees=degrees)
        if rp.use_jax: return res
        elif rp.use_torch: return rp.TorchArray(res)
        else: return rp.NumpyArray(res)
        
    def as_matrix(R): 
        native_R = getattr(R, '_native', R)
        res = native_R.as_matrix()
        if rp.use_jax: return res
        elif rp.use_torch: return rp.TorchArray(res)
        else: return rp.NumpyArray(res)
        
    def as_mrp(R): 
        native_R = getattr(R, '_native', R)
        res = native_R.as_mrp()
        if rp.use_jax: return res
        elif rp.use_torch: return rp.TorchArray(res)
        else: return rp.NumpyArray(res)
        
    def as_quat(R, canonical=False, scalar_first=False): 
        native_R = getattr(R, '_native', R)
        res = native_R.as_quat(canonical=canonical, scalar_first=scalar_first)
        if rp.use_jax: return res
        elif rp.use_torch: return rp.TorchArray(res)
        else: return rp.NumpyArray(res)
        
    def as_rotvec(R, degrees=False): 
        native_R = getattr(R, '_native', R)
        res = native_R.as_rotvec(degrees=degrees)
        if rp.use_jax: return res
        elif rp.use_torch: return rp.TorchArray(res)
        else: return rp.NumpyArray(res)

    def inv(R): 
        native_R = getattr(R, '_native', R)
        res = native_R.inv()
        if rp.use_jax: return res
        else: return Rotation(res)
        
    def magnitude(R): 
        native_R = getattr(R, '_native', R)
        res = native_R.magnitude()
        if rp.use_jax: return res
        elif rp.use_torch: return rp.TorchArray(res)
        else: return rp.NumpyArray(res)
        
    def mean(R, weights=None): 
        native_R = getattr(R, '_native', R)
        res = native_R.mean(weights=weights)
        if rp.use_jax: return res
        else: return Rotation(res)

    def __mul__(self, other):
        native_self = getattr(self, '_native', self)
        other_native = getattr(other, '_native', other)
        res = native_self * other_native
        if rp.use_jax: return res
        else: return Rotation(res)

    def __getitem__(self, key):
        res = self._native[key]
        if rp.use_jax: return res
        else: return Rotation(res)

    def __len__(self):
        return len(self._native)

    def __repr__(self):
        return f"RNUMPY.Rotation({repr(self._native)})"

    # --- Static Factory Methods ---

    @staticmethod
    def concatenate(rotations): 
        native_rotations = [getattr(r, '_native', r) for r in rotations]
        if rp.use_jax: 
            return jspatial.Rotation.concatenate(rotations=native_rotations)
        elif rp.use_torch:
            res = _TorchRotation.concatenate(rotations=native_rotations)
            return Rotation(res)
        else: 
            res = sp.spatial.transform.Rotation.concatenate(rotations=native_rotations)
            return Rotation(res)
         
    @staticmethod
    def from_euler(seq, angles, degrees=False): 
        if rp.use_jax: 
            return jspatial.Rotation.from_euler(seq=seq, angles=angles, degrees=degrees)
        elif rp.use_torch:
            res = _TorchRotation.from_euler(seq=seq, angles=angles, degrees=degrees)
            return Rotation(res)
        else: 
            res = sp.spatial.transform.Rotation.from_euler(seq=seq, angles=angles, degrees=degrees)
            return Rotation(res)
        
    @staticmethod
    def from_matrix(matrix): 
        if rp.use_jax: 
            return jspatial.Rotation.from_matrix(matrix=matrix)
        elif rp.use_torch:
            res = _TorchRotation.from_matrix(matrix=matrix)
            return Rotation(res)
        else: 
            res = sp.spatial.transform.Rotation.from_matrix(matrix=matrix)
            return Rotation(res)
        
    @staticmethod
    def from_mrp(mrp): 
        if rp.use_jax: 
            return jspatial.Rotation.from_mrp(mrp=mrp)
        elif rp.use_torch:
            raise NotImplementedError("from_mrp native for PyTorch is not yet fully implemented.")
        else: 
            res = sp.spatial.transform.Rotation.from_mrp(mrp=mrp)
            return Rotation(res)
        
    @staticmethod
    def from_quat(quat): 
        if rp.use_jax: 
            return jspatial.Rotation.from_quat(quat=quat)
        elif rp.use_torch:
            res = _TorchRotation.from_quat(quat=quat)
            return Rotation(res)
        else: 
            res = sp.spatial.transform.Rotation.from_quat(quat=quat)
            return Rotation(res)
        
    @staticmethod
    def from_rotvec(rotvec, degrees=False): 
        if rp.use_jax: 
            return jspatial.Rotation.from_rotvec(rotvec=rotvec, degrees=degrees)
        elif rp.use_torch:
            res = _TorchRotation.from_rotvec(rotvec=rotvec, degrees=degrees)
            return Rotation(res)
        else: 
            res = sp.spatial.transform.Rotation.from_rotvec(rotvec=rotvec, degrees=degrees)
            return Rotation(res)
        
    @staticmethod
    def identity(num=None): 
        if rp.use_jax: 
            return jspatial.Rotation.identity(num=num)
        elif rp.use_torch:
            res = _TorchRotation.identity(num=num)
            return Rotation(res)
        else: 
            res = sp.spatial.transform.Rotation.identity(num=num)
            return Rotation(res)
        
    @staticmethod
    def random(num=None, random_state=None): 
        if rp.use_jax: 
            return jspatial.Rotation.random(num=num, random_state=random_state)
        elif rp.use_torch:
            # We don't implement full random generation in PyTorch natively yet, fallback or raise
            res = sp.spatial.transform.Rotation.random(num=num, random_state=random_state)
            return Rotation(_TorchRotation.from_quat(res.as_quat()))
        else: 
            res = sp.spatial.transform.Rotation.random(num=num, random_state=random_state)
            return Rotation(res)
     
        
    def count():    raise NotImplementedError 
    def index():    raise NotImplementedError
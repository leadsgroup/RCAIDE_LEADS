# ConvexHull.py
# (c) Copyright 2024 Aerospace Research Community LLC

# Created:  March 2026 E. Botero
# Modified: 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------  

import RNUMPY as rp
import scipy.spatial
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  ConvexHull
# ----------------------------------------------------------------------------------------------------------------------  

class ConvexHull:
    """
    Convex hulls in N dimensions.
    """
    def __init__(self, points, incremental=False, qhull_options=None):
        
        # Handle backend-specific point conversion
        if rp.use_jax:
            # Convert JAX array to NumPy for Qhull
            jnp = rp.jax_handle.numpy
            points_np = np.array(jnp.asarray(points))
        elif rp.use_torch:
            # Convert Torch tensor to NumPy for Qhull
            points_rp = rp.array(points)
            points_np = points_rp.detach().cpu().numpy()
        else:
            # Default NumPy/SciPy case
            points_np = np.array(points)

        # Compute the hull using SciPy's Qhull wrapper
        self._hull = scipy.spatial.ConvexHull(points_np, incremental=incremental, qhull_options=qhull_options)
        
        # Wrap results back into RNUMPY arrays
        dtype = rp.float64 if rp.x64_enabled else rp.float32
        self.points      = rp.array(self._hull.points,dtype=dtype)
        self.vertices    = rp.array(self._hull.vertices)
        self.simplices   = rp.array(self._hull.simplices)
        self.neighbors   = rp.array(self._hull.neighbors)
        self.equations   = rp.array(self._hull.equations)
        self.min_bound   = rp.array(self._hull.min_bound,dtype=dtype)
        self.max_bound   = rp.array(self._hull.max_bound,dtype=dtype)
        
        # Compute area and volume differentiably if possible
        if rp.use_jax or rp.use_torch:
            self._compute_differentiable_attributes(rp.array(points))
        else:
            # NumPy/SciPy case: use directly
            self.area        = rp.array(self._hull.area)
            self.volume      = rp.array(self._hull.volume)

        # Additional attributes
        self.nsimplex    = len(self.simplices)

    def _compute_differentiable_attributes(self, points):
        """
        Computes area and volume using backend-native operations to maintain gradient flow.
        """
        ndim = points.shape[-1]
        
        if ndim == 2:
            # Volume is Area in 2D
            v_indices = self.vertices
            p = points[v_indices]
            p_next = rp.roll(p, -1, axis=0) 
            
            # Shoelace formula for area (volume in SciPy convention)
            self.volume = 0.5 * rp.abs(rp.sum(p[:, 0] * p_next[:, 1] - p_next[:, 0] * p[:, 1]))
            
            # Perimeter (area in SciPy convention)
            self.area = rp.sum(rp.sqrt(rp.sum((p_next - p)**2, axis=-1)))
            
        elif ndim == 3:
            # Simplices are triangles
            s_indices = self.simplices
            p1 = points[s_indices[:, 0]]
            p2 = points[s_indices[:, 1]]
            p3 = points[s_indices[:, 2]]
            
            # Surface area: Sum of 0.5 * |(p2-p1) x (p3-p1)|
            cross_prod = rp.cross(p2 - p1, p3 - p1)
            self.area = 0.5 * rp.sum(rp.sqrt(rp.sum(cross_prod**2, axis=-1)))
            
            # Volume: 1/6 * |sum(dot(p1, cross(p2, p3)))|
            self.volume = (1.0/6.0) * rp.abs(rp.sum(rp.sum(p1 * rp.cross(p2, p3), axis=-1)))
            
        else:
            # For higher dimensions, fallback to non-differentiable SciPy values
            self.area = rp.array(self._hull.area)
            self.volume = rp.array(self._hull.volume)

    def add_points(self, points, restart=False):
        """
        Process a set of additional new points.
        """
        if rp.use_jax:
            jnp = rp.jax_handle.numpy
            points_np = np.array(jnp.asarray(points))
        elif rp.use_torch:
            points_rp = rp.array(points)
            points_np = points_rp.detach().cpu().numpy()
        else:
            points_np = np.array(points)
            
        self._hull.add_points(points_np, restart=restart)
        
        # Update attributes
        self.points      = rp.array(self._hull.points)
        self.vertices    = rp.array(self._hull.vertices)
        self.simplices   = rp.array(self._hull.simplices)
        self.neighbors   = rp.array(self._hull.neighbors)
        self.equations   = rp.array(self._hull.equations)
        self.min_bound   = rp.array(self._hull.min_bound)
        self.max_bound   = rp.array(self._hull.max_bound)
        
        if rp.use_jax or rp.use_torch:
            self._compute_differentiable_attributes(rp.array(self.points)) # Use all points
        else:
            self.area        = rp.array(self._hull.area)
            self.volume      = rp.array(self._hull.volume)
            
        self.nsimplex    = len(self.simplices)

    def close(self):
        """
        Finish incremental processing.
        """
        self._hull.close()

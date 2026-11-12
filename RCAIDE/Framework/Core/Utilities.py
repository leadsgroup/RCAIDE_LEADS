 
# RCAIDE/Core/Utilities.py
# 
# 
# Created:  Jul 2023, M. Clarke
# Modified: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# Package imports 
import numpy as np

from RCAIDE.Framework.Core.Arrays import array_namespace

# ----------------------------------------------------------------------------------------------------------------------
#  interp2d
# ----------------------------------------------------------------------------------------------------------------------
 
def interp2d(x,y,xp,yp,zp,fill_value= None):
    """
    Bilinear interpolation on a grid. ``CartesianGrid`` is much faster if the data
    lies on a regular grid.
    Args:
        x, y: 1D arrays of point at which to interpolate. Any out-of-bounds
            coordinates will be clamped to lie in-bounds.
        xp, yp: 1D arrays of points specifying grid points where function values
            are provided.
        zp: 2D array of function values. For a function `f(x, y)` this must
            satisfy `zp[i, j] = f(xp[i], yp[j])`
    Returns:
        1D array `z` satisfying `z[i] = f(x[i], y[i])`.
    """ 
    # xp is the grid argument here, so the array module is named xnp
    xnp = array_namespace(x, y, xp, yp, zp)
    xp, yp, zp = xnp.asarray(xp), xnp.asarray(yp), xnp.asarray(zp)   # NumPy tables indexed by JAX indices
    ix = xnp.clip(xnp.searchsorted(xp, x, side="right"), 1, len(xp) - 1)
    iy = xnp.clip(xnp.searchsorted(yp, y, side="right"), 1, len(yp) - 1)

    # Using Wikipedia's notation (https://en.wikipedia.org/wiki/Bilinear_interpolation)
    z_11 = zp[ix - 1, iy - 1]
    z_21 = zp[ix, iy - 1]
    z_12 = zp[ix - 1, iy]
    z_22 = zp[ix, iy]

    z_xy1 = (xp[ix] - x) / (xp[ix] - xp[ix - 1]) * z_11 + (x - xp[ix - 1]) / (
        xp[ix] - xp[ix - 1]
    ) * z_21
    z_xy2 = (xp[ix] - x) / (xp[ix] - xp[ix - 1]) * z_12 + (x - xp[ix - 1]) / (
        xp[ix] - xp[ix - 1]
    ) * z_22
    z = (yp[iy] - y) / (yp[iy] - yp[iy - 1]) * z_xy1 + (y - yp[iy - 1]) / (
        yp[iy] - yp[iy - 1]
    ) * z_xy2

    if fill_value is not None:
        oob = xnp.logical_or(
            x < xp[0], xnp.logical_or(x > xp[-1], xnp.logical_or(y < yp[0], y > yp[-1]))
        )
        z = xnp.where(oob, fill_value, z)
    return z

# ----------------------------------------------------------------------------------------------------------------------
#  regular_grid_interpolation
# ----------------------------------------------------------------------------------------------------------------------
def regular_grid_interpolation(grid, values, points):
    """
    Multilinear interpolation on a regular grid, for NumPy or JAX; the same as scipy's
    RegularGridInterpolator(grid, values, bounds_error=False, fill_value=None), which extrapolates linearly.

    Inputs:
       grid      tuple of d increasing 1D arrays, the grid points along each axis
       values    array of shape (len(grid[0]), ..., len(grid[d-1]), ...) with the values at the grid points
       points    array of shape (n, d), the points at which to interpolate

    Outputs:
       result    array of shape (n, ...), the interpolated values
    """
    xp     = array_namespace(values, points)
    values = xp.asarray(values)   # NumPy tables indexed by JAX indices

    # for each axis, the grid interval that holds each point and the point's position in it
    lower   = []
    weights = []
    for axis, g in enumerate(grid):
        g = xp.asarray(g)
        x = points[:, axis]
        i = xp.clip(xp.searchsorted(g, x, side="right") - 1, 0, len(g) - 2)
        lower.append(i)
        weights.append((x - g[i]) / (g[i + 1] - g[i]))

    # sum over the 2^d corners of each grid cell
    result = 0.
    for corner in range(2**len(grid)):
        index  = []
        weight = 1.
        for axis in range(len(grid)):
            upper   = (corner >> axis) & 1
            index.append(lower[axis] + upper)
            weight  = weight * (weights[axis] if upper else 1. - weights[axis])
        weight = weight.reshape(weight.shape + (1,)*(values.ndim - len(grid)))
        result = result + weight * values[tuple(index)]
    return result

# ----------------------------------------------------------------------------------------------------------------------
# orientation_product
# ----------------------------------------------------------------------------------------------------------------------
 
def orientation_product(T,Bb):
    """Computes the product of a tensor and a vector.

    Assumptions:
    None

    Source:
    N/A

    Inputs:
    T         [-] 3-dimensional array with rotation matrix
                  patterned along dimension zero
    Bb        [-] 3-dimensional vector

    Outputs:
    C         [-] transformed vector

    Properties Used:
    N/A
    """            
    
    assert T.ndim == 3
    xp = array_namespace(T, Bb)
    
    if Bb.ndim == 3:
        C = xp.einsum('aij,ajk->aik', T, Bb )
    elif Bb.ndim == 2:
        C = xp.einsum('aij,aj->ai', T, Bb )
    else:
        raise Exception('bad B rank')
        
    return C

# ----------------------------------------------------------------------------------------------------------------------
# orientation_transpose
# ----------------------------------------------------------------------------------------------------------------------

def orientation_transpose(T):
    """Computes the transpose of a tensor.

    Assumptions:
    None

    Source:
    N/A

    Inputs:
    T         [-] 3-dimensional array with rotation matrix
                  patterned along dimension zero

    Outputs:
    Tt        [-] transformed tensor

    Properties Used:
    N/A
    """   
    
    assert T.ndim == 3
    
    Tt = array_namespace(T).swapaxes(T,1,2)
        
    return Tt

# ----------------------------------------------------------------------------------------------------------------------
# angles_to_dcms
# ----------------------------------------------------------------------------------------------------------------------

def angles_to_dcms(rotations,sequence=(2,1,0)):
    """Builds an euler angle rotation matrix
    
    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
    rotations     [radians]  [r1s r2s r3s], column array of rotations
    sequence      [-]        (2,1,0) (default), (2,1,2), etc.. a combination of three column indices

    Outputs:
    transform     [-]        3-dimensional array with direction cosine matrices
                             patterned along dimension zero

    Properties Used:
    N/A
    """         
    # transform map
    Ts = { 0:T0, 1:T1, 2:T2 }
    
    # a bunch of eyes
    transform = new_tensor(rotations[:,0])
    
    # build the tranform
    for dim in sequence[::-1]:
        angs = rotations[:,dim]
        transform = orientation_product( transform, Ts[dim](angs) )
    
    # done!
    return transform

# ----------------------------------------------------------------------------------------------------------------------
# T0
# ----------------------------------------------------------------------------------------------------------------------  

def T0(a):
    """Rotation matrix about first axis
    
    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
    a        [radians] angle of rotation

    Outputs:
    T        [-]       rotation matrix

    Properties Used:
    N/A
    """      
    # T = np.array([[1,   0,  0],
    #               [0, cos,sin],
    #               [0,-sin,cos]])
    
    xp  = array_namespace(a)
    cos = xp.cos(a)
    sin = xp.sin(a)
                  
    zero = xp.zeros_like(cos)
    one  = xp.ones_like(cos)
    T    = xp.stack([ one, zero, zero, zero,  cos,  sin, zero, -sin,  cos], axis=-1).reshape(-1,3,3)
    return T

# ----------------------------------------------------------------------------------------------------------------------
# T1
# ----------------------------------------------------------------------------------------------------------------------          

def T1(a):
    """Rotation matrix about second axis
    
    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
    a        [radians] angle of rotation

    Outputs:
    T        [-]       rotation matrix

    Properties Used:
    N/A
    """      
    # T = np.array([[cos,0,-sin],
    #               [0  ,1,   0],
    #               [sin,0, cos]])
    
    xp  = array_namespace(a)
    cos = xp.cos(a)
    sin = xp.sin(a)     
    
    zero = xp.zeros_like(cos)
    one  = xp.ones_like(cos)
    T    = xp.stack([ cos, zero, -sin, zero,  one, zero,  sin, zero,  cos], axis=-1).reshape(-1,3,3)
    return T

# ----------------------------------------------------------------------------------------------------------------------
# T2
# ----------------------------------------------------------------------------------------------------------------------  

def T2(a):
    """Rotation matrix about third axis
    
    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
    a        [radians] angle of rotation

    Outputs:
    T        [-]       rotation matrix

    Properties Used:
    N/A
    """      
    # T = np.array([[cos ,sin,0],
    #               [-sin,cos,0],
    #               [0   ,0  ,1]])
        
    xp  = array_namespace(a)
    cos = xp.cos(a)
    sin = xp.sin(a)     
    
    zero = xp.zeros_like(cos)
    one  = xp.ones_like(cos)
    T    = xp.stack([ cos,  sin, zero, -sin,  cos, zero, zero, zero,  one], axis=-1).reshape(-1,3,3)
    return T

# ----------------------------------------------------------------------------------------------------------------------
# new_tensor
# ----------------------------------------------------------------------------------------------------------------------  

def new_tensor(a):
    """Initializes the required tensor. Able to handle imaginary values.
    
    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
    a        [radians] angle of rotation

    Outputs:
    T        [-]       3-dimensional array with identity matrix
                       patterned along dimension zero

    Properties Used:
    N/A
    """      
    assert a.ndim == 1
    n_a = len(a)
    
    xp = array_namespace(a)
    T = xp.eye(3)
    
    if a.dtype is np.dtype('complex'):
        T = T + 0j
    
    T = xp.broadcast_to(T,(n_a,3,3))
    
    return T
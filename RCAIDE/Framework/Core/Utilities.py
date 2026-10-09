 
# RCAIDE/Core/Utilities.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# Package imports 
import numpy as np

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
    ix = np.clip(np.searchsorted(xp, x, side="right"), 1, len(xp) - 1)
    iy = np.clip(np.searchsorted(yp, y, side="right"), 1, len(yp) - 1)

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
        oob = np.logical_or(
            x < xp[0], np.logical_or(x > xp[-1], np.logical_or(y < yp[0], y > yp[-1]))
        )
        z = np.where(oob, fill_value, z)

    return z

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
    
    if Bb.ndim == 3:
        C = np.einsum('aij,ajk->aik', T, Bb )
    elif Bb.ndim == 2:
        C = np.einsum('aij,aj->ai', T, Bb )
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
    
    Tt = np.swapaxes(T,1,2)
        
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
    
    cos = np.cos(a)
    sin = np.sin(a)
                  
    zero = np.zeros_like(cos)
    one  = np.ones_like(cos)
    T    = np.stack([ one, zero, zero, zero,  cos,  sin, zero, -sin,  cos], axis=-1).reshape(-1,3,3)
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
    
    cos = np.cos(a)
    sin = np.sin(a)     
    
    zero = np.zeros_like(cos)
    one  = np.ones_like(cos)
    T    = np.stack([ cos, zero, -sin, zero,  one, zero,  sin, zero,  cos], axis=-1).reshape(-1,3,3)
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
        
    cos = np.cos(a)
    sin = np.sin(a)     
    
    zero = np.zeros_like(cos)
    one  = np.ones_like(cos)
    T    = np.stack([ cos,  sin, zero, -sin,  cos, zero, zero, zero,  one], axis=-1).reshape(-1,3,3)
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
    
    T = np.eye(3)
    
    if a.dtype is np.dtype('complex'):
        T = T + 0j
    
    T = np.resize(T,[n_a,3,3])
    
    return T
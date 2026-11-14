# RCAIDE/Framework/Core/Arrays.py
#
# Modified: Nov 2026, M. Clarke

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import numpy as np

# ----------------------------------------------------------------------
#   Array
# ----------------------------------------------------------------------       

array_type  = np.ndarray
matrix_type = np.matrixlib.defmatrix.matrix

def array_namespace(*arrays):
    """NumPy, or jax.numpy when any argument is a JAX array, including a traced value inside jax.jit."""
    for A in arrays:
        if type(A).__module__.startswith(('jax', 'jaxlib')):
            import jax.numpy as jnp
            return jnp
    return np

def set_at(A, index, value):
    """A with A[index] = value: written in place for NumPy, a new array for JAX, whose arrays are immutable."""
    if isinstance(A, np.ndarray) and array_namespace(value) is np:
        A[index] = value
        return A
    import jax.numpy as jnp
    return jnp.asarray(A).at[index].set(value)
 
def safe_sqrt(A):
    """Square root with a zero derivative at zero, where the exact one is infinite and makes JAX gradients NaN."""
    xp       = array_namespace(A)
    positive = A > 0
    return xp.where(positive, xp.sqrt(xp.where(positive, A, 1.)), 0.)

def gradient(f, x):
    """np.gradient(f, x) for samples f at unevenly spaced points x, both one-dimensional, on NumPy or JAX."""
    xp = array_namespace(f, x)
    if xp is np:
        return np.gradient(f, x)
    # second order inside, first order at the two ends, as np.gradient
    before   = x[1:-1] - x[:-2]
    after    = x[2:] - x[1:-1]
    inside   = (before**2 * f[2:] + (after**2 - before**2) * f[1:-1] - after**2 * f[:-2]) / (before * after * (before + after))
    first    = (f[1] - f[0]) / (x[1] - x[0])
    last     = (f[-1] - f[-2]) / (x[-1] - x[-2])
    return xp.concatenate([first[None], inside, last[None]])

def atleast_2d_col(A):
    """Makes a 2D array in column format

    Assumptions:
    None

    Source:
    N/A

    Inputs:
    A      [1-D Array]

    Outputs:
    A      [2-D Array]

    Properties Used:
    N/A
    """       
    return atleast_2d(A,'col')


def atleast_2d_row(A):
    """Makes a 2D array in row format

    Assumptions:
    None

    Source:
    N/A

    Inputs:
    A      [1-D Array]

    Outputs:
    A      [2-D Array]

    Properties Used:
    N/A
    """       
    return atleast_2d(A,'row')


def atleast_2d(A,oned_as='row'):
    """ensures A is an array and at least of rank 2

    Assumptions:
    Defaults as row

    Source:
    N/A

    Inputs:
    A      [1-D Array]

    Outputs:
    A      [2-D Array]

    Properties Used:
    N/A
    """       
    
    # not an array yet
    if not isinstance(A,(array_type,matrix_type)) and array_namespace(A) is np:
        if not isinstance(A,(list,tuple)):
            A = [A]
        A = np.array(A)
    elif A.ndim == 0:
        A = A.reshape(1)   # a rank-0 array is a scalar, as above
        
    # check rank
    if A.ndim < 2:
        # expand row or col
        if oned_as == 'row':
            A = A[None,:]
        elif oned_as == 'col':
            A = A[:,None]
        else:
            raise Exception("oned_as must be 'row' or 'col' ")
            
    return A
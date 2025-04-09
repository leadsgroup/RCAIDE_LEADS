# RCAIDE/Library/Missions/Common/Initialize/differentials_dimensionless.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE Imports 
from RCAIDE.Framework.Core.Arrays  import atleast_2d_col 

# ----------------------------------------------------------------------------------------------------------------------
#  Initialize Differentials
# ----------------------------------------------------------------------------------------------------------------------
def differentials_dimensionless(segment):
    """
    Initializes dimensionless differential operators for mission segment discretization

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed

    Notes
    -----
    This function sets up the numerical discretization operators used for
    solving differential equations in mission segments. It creates the
    control points and differential/integral operators based on the
    specified discretization method.

    The function performs the following steps:
        1. Gets number of control points and discretization method
        2. Generates control points and operators
        3. Ensures proper dimensioning of arrays
        4. Stores results in segment numerics

    **Required Segment State Variables**

    state.numerics:
        - number_of_control_points : int
            Number of points for discretization
        - discretization_method : function
            Method to generate discretization operators

    **Generated Operators**

    numerics.dimensionless:
        - control_points : array
            Normalized points for evaluation
        - differentiate : array
            Differentiation operator matrix
        - integrate : array
            Integration operator matrix

    **Major Assumptions**
        * Valid discretization method provided
        * Number of control points > 1
        * Method generates consistent operators

    Returns
    -------
    None
        Updates segment state directly

    See Also
    --------
    RCAIDE.Framework.Core.Arrays
    RCAIDE.Framework.Mission.Segments
    """     
    
    # unpack
    numerics              = segment.state.numerics
    N                     = numerics.number_of_control_points
    discretization_method = numerics.discretization_method
    
    # get operators
    x,D,I = discretization_method(N,**numerics)
    x = atleast_2d_col(x)
    
    # pack
    numerics.dimensionless.control_points = x
    numerics.dimensionless.differentiate  = D
    numerics.dimensionless.integrate      = I    
    
    return
 
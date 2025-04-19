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
    """ Discretizes the differential operators
    
        Assumptions:
        N/A
        
        Inputs:
            state.numerics:
                number_of_control_points [int]
                discretization_method    [function]
            
        Outputs:
            numerics.dimensionless:            
                control_points           [array]
                differentiate            [array]
                integrate                [array]

        Properties Used:
        N/A
                                
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
 
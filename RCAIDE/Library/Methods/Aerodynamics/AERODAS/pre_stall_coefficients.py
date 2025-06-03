# RCAIDE/Library/Methods/Aerdoynamics/AERODAS/pre_stall_coefficients.py
# 
# 
# Created:  Jul 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
# Imports 
# ----------------------------------------------------------------------------------------------------------------------  
from RCAIDE.Framework.Core import Data

# python imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
# Pre Stall Coefficients
# ----------------------------------------------------------------------------------------------------------------------    
def pre_stall_coefficients(state,settings,geometry):
    """
    Computes pre-stall lift and drag coefficients using the AERODAS method for wing sections.

    Parameters
    ----------
    state : Data
        Aircraft state data containing flight conditions
            - conditions.aerodynamics.angle_of_attack : array_like
                Angle of attack in radians
    settings : Data
        AERODAS method configuration settings
            - section_zero_lift_angle_of_attack : float
                Section zero-lift angle of attack in radians
    geometry : Data
        Wing geometry and aerodynamic properties
            - tag : str
                Wing identifier
            - vertical : bool
                Flag indicating if wing is vertical (rudder/fin)
            - section.angle_attack_max_prestall_lift : array_like
                Angle of attack at maximum pre-stall lift in radians
            - section.minimum_drag_coefficient : array_like
                Minimum section drag coefficient
            - section.minimum_drag_coefficient_angle_of_attack : array_like
                Angle of attack at minimum drag coefficient in radians
            - pre_stall_maximum_drag_coefficient_angle : array_like
                Angle of attack at maximum pre-stall drag coefficient in radians
            - pre_stall_maximum_lift_coefficient : array_like
                Maximum pre-stall lift coefficient
            - pre_stall_lift_curve_slope : array_like
                Pre-stall lift curve slope per radian
            - pre_stall_maximum_lift_drag_coefficient : array_like
                Maximum pre-stall drag coefficient

    Returns
    -------
    CL1 : array_like
        Pre-stall lift coefficient
    CD1 : array_like
        Pre-stall drag coefficient

    Notes
    -----
    The AERODAS model provides empirical relationships for computing pre-stall aerodynamic 
    coefficients for a wing section/airfoil based on wing geometry and operating conditions.

    For vertical surfaces (fins, rudders), the angle of attack is set to zero
    as these surfaces primarily operate in the crossflow direction.

    The method uses piecewise functions with different formulations depending
    on the angle of attack relative to the zero-lift angle. Results are stored
    in the state conditions for subsequent analysis steps.

    **Major Assumptions**
        * Pre-stall flow conditions (attached flow)
        * Two-dimensional airfoil section behavior
        * Steady-state aerodynamics

    **Theory**

    Pre-stall lift coefficient uses a modified linear relationship:

    .. math::
        CL_1 = S_1(\\alpha - A_0) - R_{CL1}\\left(\\frac{\\alpha - A_0}{A_{CL1} - A_0}\\right)^{N_1}

    where :math:`R_{CL1} = S_1(A_{CL1} - A_0) - CL_{1max}` and :math:`N_1 = 1 + \\frac{CL_{1max}}{R_{CL1}}`.

    Pre-stall drag coefficient follows:

    .. math::
        CD_1 = CD_{min} + (CD_{1max} - CD_{min})\\left(\\frac{\\alpha - A_{CDmin}}{A_{CD1} - A_{CDmin}}\\right)^M

    **Definitions**

    'Pre-stall Region'
        Operating range where flow remains attached to the airfoil surface

    'Zero-lift Angle'
        Angle of attack at which the airfoil produces zero lift

    References
    ----------
    [1] Spera, D. A., "Models of Lift and Drag Coefficients of Stalled and Unstalled Airfoils in Wind Turbines and Wind Tunnels", NASA, CR-2008-215434, October 2008

    See Also
    --------
    RCAIDE.Library.Methods.Aerodynamics.AERODAS.post_stall_coefficients : Post-stall lift and drag coefficients
    """  
    
    # unpack inputs
    wing   = geometry
    alpha  = state.conditions.aerodynamics.angle_of_attack * 1.0
    A0     = settings.section_zero_lift_angle_of_attack
    ACL1   = wing.section.angle_attack_max_prestall_lift 
    ACD1   = wing.pre_stall_maximum_drag_coefficient_angle
    CL1max = wing.pre_stall_maximum_lift_coefficient
    S1     = wing.pre_stall_lift_curve_slope  
    CD1max = wing.pre_stall_maximum_lift_drag_coefficient
    CDmin  = wing.section.minimum_drag_coefficient 
    ACDmin = wing.section.minimum_drag_coefficient_angle_of_attack 
    
    if wing.vertical == True:
        alpha = 0. * np.ones_like(alpha)
        
    # Equation 6c
    RCL1          = S1*(ACL1-A0)-CL1max
    RCL1[RCL1<=0] = 1.e-16
    
    # Equation 6d
    N1            = 1 + CL1max/RCL1
    
    # Equation 6a or 6b depending on the alpha
    CL1            = 0.0 * np.ones_like(alpha)
    CL1[alpha>A0]  = S1*(alpha[alpha>A0]-A0)-RCL1[alpha>A0]*((alpha[alpha>A0]-A0)/(ACL1[alpha>A0]-A0))**N1[alpha>A0]
    CL1[alpha==A0] = 0.0
    CL1[alpha<A0]  = S1*(alpha[alpha<A0]-A0)+RCL1[alpha<A0]*((A0-alpha[alpha<A0])/(ACL1[alpha<A0]-A0))**N1[alpha<A0]
    
    # M what is m?
    M              = 2. # This parameter is airfoil dependent. Does this need changing?

    # Equation 7a
    con      = np.logical_and((2*A0-ACD1)<=alpha,alpha<=ACD1)
    CD1      = np.ones_like(alpha)
    CD1[con] = CDmin[con] + (CD1max[con]-CDmin[con])*((alpha[con] - ACDmin)/(ACD1[con]-ACDmin))**M    
    
    # Equation 7b
    CD1[np.logical_not(con)] = 0.
    
    # Pack outputs
    wing_result = Data(
        lift_coefficient = CL1,
        drag_coefficient = CD1
        )

    state.conditions.aerodynamics.pre_stall_coefficients[wing.tag] =  wing_result
    

    return CL1, CD1
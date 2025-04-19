# RCAIDE/Library/Methods/Weights/Correlation_Buildups/FLOPS/compute_fuselage_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE
import RCAIDE 
from RCAIDE.Framework.Core    import Units

# python imports 
import  numpy as  np
 
# ----------------------------------------------------------------------------------------------------------------------
# Fuselage Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_fuselage_weight(vehicle):
    """
    Calculate the weight of a general aviation aircraft fuselage using NASA FLOPS methodology.

    Parameters
    ----------
    vehicle : RCAIDE.Vehicle()
        Vehicle data structure containing:
            - fuselages.lengths.total : float
                Total length of the fuselage [m]
            - fuselages.width : float
                Maximum width of the fuselage [m]
            - fuselages.heights.maximum : float
                Maximum height of the fuselage [m]
            - flight_envelope.ultimate_load : float
                Ultimate load factor
            - mass_properties.max_takeoff : float
                Maximum takeoff weight [kg]
            - design_dynamic_pressure : float
                Design dynamic pressure [Pa]

    Returns
    -------
    fuselage_weight : float
        Weight of the fuselage structure [kg]

    Notes
    -----
    The function implements the FLOPS (Flight Optimization System) weight estimation
    method for general aviation aircraft fuselages. The calculation accounts for geometric 
    parameters, design loads, and cruise conditions.
    
    **Major Assumptions**
        * Single fuselage configuration (NFUSE = 1)
        * Structural weight scales with wetted area and design loads
    
    **Theory**

    The FLOPS fuselage weight estimation follows:

    .. math::
        W_{fus} = 0.052 * S_{wf}^{1.086} * (N_{ult} * W_{TO})^{0.177} * q^{0.241}

    Where:
        - W_{fus} is fuselage weight [lb]
        - S_{wf} is fuselage wetted area [ft^2], calculated as π(L/D - 1.7)D^2
        - N_{ult} is ultimate load factor
        - W_{TO} is maximum takeoff weight [lb]
        - q is design dynamic pressure [psf]
        - L is fuselage length [ft]
        - D is average fuselage diameter [ft]

    References
    ----------
    [1] NASA. The Flight Optimization System Weights Estimation Method. 
        NASA Technical Report.
    """
    
    L =  0
    for fuselage in vehicle.fuselages:
        if L < fuselage.lengths.total: 
            total_length = fuselage.lengths.total
            width        = fuselage.width
            max_height   = fuselage.heights.maximum
    
    XL  = total_length / Units.ft  # Fuselage length, ft
    DAV = (width + max_height) / 2. * 1 / Units.ft
    
    DG       = vehicle.mass_properties.max_takeoff / Units.lbs  # Design gross weight in lb
    QCRUS    = vehicle.design_dynamic_pressure / Units.psf
    ULF      = vehicle.flight_envelope.ultimate_load  
    SWFUS    = 3.14159*(XL/DAV -1.7)*(DAV**2)

    WFUSE = 0.052 * SWFUS**1.086 * (ULF*DG)**0.177 * QCRUS**0.241 
    
    return WFUSE * Units.lbs
# RCAIDE/Library/Methods/Weights/Correlation_Buildups/FLOPS/compute_horizontal_tail_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE 
from RCAIDE.Framework.Core    import Units
 
# ----------------------------------------------------------------------------------------------------------------------
#  Horizontal Tail Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_horizontal_tail_weight(vehicle):
    """
    Calculates the weight of an aircraft's horizontal tail using FLOPS methodology.

    Parameters
    ----------
    vehicle : RCAIDE.Vehicle()
        Vehicle data structure containing:
            - wings.horizontal_tail.areas.reference
            - mass_properties.max_takeoff
            - flight_envelope.ultimate_load
            - design_dynamic_pressure
    

    Returns
    -------
    horizontal_tail_weight : float
        Estimated weight of the horizontal tail assembly in kilograms

    Notes
    -----
    The function implements the FLOPS (Flight Optimization System) weight estimation
    method for horizontal tail surfaces for general aviation aircraft. The calculation accounts 
    for geometric parameters and design loads based on maximum takeoff weight. 
    
    **Major Assumptions**
        * The weight is a function of the horizontal tail area, ultimate load factor, takeoff weight, and design dynamic pressure
    
    **Theory**

    The FLOPS horizontal tail weight estimation follows:

    .. math::
        W_{ht} = 0.016 * S_{ht}^{0.873} * (N_{ult} * W_{TO})^{0.414} * q^{0.122}

    Where:
        - W_{ht} is horizontal tail weight [lb]
        - S_{ht} is horizontal tail area [ft^2]
        - N_{ult} is ultimate load factor
        - W_{TO} is maximum takeoff weight [lb]
        - q is design dynamic pressure [psf]
    """
    SHT     = vehicle.wings.horizontal_stabilizer.areas.reference / Units.ft **2
    DG      = vehicle.mass_properties.max_takeoff / Units.lbs
    QCRUS    = vehicle.design_dynamic_pressure / Units.psf
    ULF      = vehicle.flight_envelope.ultimate_load   
    WHT     = 0.016*SHT**0.873 * ( ULF * DG)**0.414 * QCRUS**0.122 
    return WHT * Units.lbs

# RCAIDE/Library/Methods/Weights/Correlation_Buildups/FLOPS/compute_vertical_tail_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE 
from RCAIDE.Framework.Core    import Units 
 
# ----------------------------------------------------------------------------------------------------------------------
#  Vertical Tail Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_vertical_tail_weight(vehicle, wing):
    """
    Calculate the vertical tail weight for general aviation aircraft using FLOPS methodology.

    Parameters
    ----------
    vehicle : RCAIDE.Vehicle()
        Vehicle data structure containing:
            - mass_properties.max_takeoff : float
                Maximum takeoff weight [kg]
            - design_dynamic_pressure : float
                Design dynamic pressure [Pa]
            - flight_envelope.ultimate_load : float
                Ultimate load factor
    wing : RCAIDE.Component()
        Vertical tail data structure containing:
            - t_tail : bool
                Flag indicating if vertical tail is a T-tail configuration

    Returns
    -------
    vertical_tail_weight : float
        Weight of the vertical tail structure [kg]

    Notes
    -----
    The function implements the FLOPS weight estimation method for vertical tail surfaces for 
    general aviation aircraft. The calculation accounts for T-tail configuration, design loads, 
    and dynamic pressure.
    
    **Major Assumptions**
        * Conventional type construction
        * T-tail factor increases weight by 20%
        * Structural weight scales with design loads and dynamic pressure
    
    **Theory**

    The FLOPS vertical tail weight estimation follows:



    .. math::
        W_{vt} = 0.073 * (1 + 0.2H_{t}) * (N_{ult} * W_{TO})^{0.376} * q^{0.122}

    Where:
        - W_{vt} is vertical tail weight [lb]
        - H_{t} is T-tail flag (1 for T-tail, 0 for conventional)
        - N_{ult} is ultimate load factor
        - W_{TO} is maximum takeoff weight [lb]
        - q is design dynamic pressure [psf]

    References
    ----------
    [1] NASA. (1979). The Flight Optimization System Weights Estimation Method. 
        NASA Technical Report.
        """
    DG       = vehicle.mass_properties.max_takeoff / Units.lbs  # Design gross weight in lb
    if wing.t_tail:  
        HHT = 1. 
    else: 
        HHT = 0.

    QCRUS    = vehicle.design_dynamic_pressure / Units.psf
    ULF      = vehicle.flight_envelope.ultimate_load   
    WVT      = 0.073* (1+0.2*HHT) * (ULF * DG)**0.376 * QCRUS**0.122 
    return WVT * Units.lbs



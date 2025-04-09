# RCAIDE/Library/Methods/Weights/Correlation_Buildups/Raymer/compute_landing_gear_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE
import RCAIDE
from RCAIDE.Framework.Core    import Units ,  Data 
 
# ----------------------------------------------------------------------------------------------------------------------
#  Horizontal Tail Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_landing_gear_weight(vehicle):
    """
    Calculates the weight of main and nose landing gear for transport aircraft using Raymer's method.

    Parameters
    ----------
    vehicle : RCAIDE.Vehicle()
        Vehicle data structure containing:
            - design_range : float
                Design range of aircraft [m]
            - mass_properties.max_takeoff : float
                Maximum takeoff weight [kg]
            - systems.accessories : str
                Aircraft type ('short-range', 'commuter', 'medium-range',
                'long-range', 'sst', 'cargo')
            - landing_gears : list
                List of landing gear components containing:
                    - strut_length : float
                        Length of strut [m]
                    - wheels : int
                        Number of wheels per strut

    Returns
    -------
    output : Data()
        Landing gear weights:
            - main : float
                Weight of main landing gear [kg]
            - nose : float
                Weight of nose landing gear [kg]

    Notes
    -----
    This method implements Raymer's empirical correlations for transport aircraft
    landing gear weight estimation. Separate correlations are used for main and
    nose gear.

    **Major Assumptions**
        * Gear load factor = 3
        * Number of main gear shock struts = 2
        * Stall speed assumes max Cl of 2.5, density of 1.225 kg/m^3
        * All main landing gear is assumed to be of the same type
        * All nose landing gear is assumed to be of the same type

    **Theory**
    The landing gear weights are calculated using:
    .. math::
        W_{main} = 0.0106K_{mp}W_{l}^{0.888}N_l^{0.25}L_m^{0.4}N_{mw}^{0.321}N_{mss}^{-0.5}V_{stall}^{0.1}

    .. math::
        W_{nose} = 0.032K_{np}W_{l}^{0.646}N_l^{0.2}L_n^{0.5}N_{nw}^{0.45}

    where:
        * :math:`W_l` is landing weight
        * :math:`N_l` is ultimate landing load factor
        * :math:`L_m, L_n` are strut lengths
        * :math:`N_{mw}, N_{nw}` are number of wheels
        * :math:`N_{mss}` is number of main gear shock struts

    References
    ----------
    [1] Raymer, D., "Aircraft Design: A Conceptual Approach", AIAA 
        Education Series, 2018. 

    See Also
    --------
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport.Raymer.compute_operating_empty_weight
    """
    Kmp         = 1  # assuming not a kneeling gear
    if vehicle.systems.accessories == "sst":
        RFACT   = 0.00009
    else:
        RFACT   = 0.00004
    DESRNG      = vehicle.flight_envelope.design_range / Units.nmi  # Design range in nautical miles
    WLDG        = vehicle.mass_properties.max_takeoff / Units.lbs * (1 - RFACT * DESRNG)
    Ngear       = 3  # gear load factor, usually around 3
    Nl          = Ngear * 1.5  # ultimate landing load factor
      
    Nmw  = 0
    Lm   = 0 
    Ln   = 0
    Nnw  = 0
    Nmss = 2  # number of main gear shock struts assumed to be 2
    for landing_gear in  vehicle.landing_gears:
        if isinstance(landing_gear,RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear):
            Nmw  = landing_gear.wheels * Nmss    
            Lm   = landing_gear.strut_length / Units.inch 
        elif isinstance(landing_gear,RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear): 
            Ln          = landing_gear.strut_length / Units.inch
            Nnw         = landing_gear.wheels  
        
    Vstall      = (2 * vehicle.mass_properties.max_takeoff * 9.81 / (vehicle.wings.main_wing.areas.reference * 1.225 * 2.5)) ** 0.5 # Assumes max Cl of 2.5, density of 1.225 kg/m^3
    Knp         = 1  # assuming not a kneeling gear
    W_main_landing_gear = 0.0106 * Kmp * WLDG ** 0.888 * Nl ** 0.25 * Lm ** 0.4 * Nmw ** 0.321 * Nmss ** (-0.5) * Vstall ** 0.1
    W_nose_landing_gear = 0.032 * Knp * WLDG ** 0.646 * Nl ** 0.2 * Ln ** 0.5 * Nnw ** 0.45

    output          = Data()
    output.main     = W_main_landing_gear * Units.lbs
    output.nose     = W_nose_landing_gear * Units.lbs
    return output
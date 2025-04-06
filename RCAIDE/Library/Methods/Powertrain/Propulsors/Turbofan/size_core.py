# RCAIDE/Methods/Energy/Propulsors/Turbofan/size_core.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan                      import compute_thrust

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  size_core
# ---------------------------------------------------------------------------------------------------------------------- 
def size_core(turbofan, conditions):
    """
    Sizes the core flow for a turbofan engine at the design condition by computing the
    non-dimensional thrust.
    
    Parameters
    ----------
    turbofan : RCAIDE.Library.Components.Propulsors.Turbofan
        Turbofan engine component with the following attributes:
            - tag : str
                Identifier for the turbofan
            - reference_temperature : float
                Reference temperature for mass flow scaling [K]
            - reference_pressure : float
                Reference pressure for mass flow scaling [Pa]
            - design_thrust : float
                Design thrust at the design point [N]
            - bypass_ratio : float
                Bypass ratio of the turbofan
    conditions : RCAIDE.Framework.Mission.Common.Conditions
        Flight conditions with:
            - freestream : Data
                Freestream properties
                - speed_of_sound : numpy.ndarray
                    Speed of sound [m/s]
            - energy.propulsors[turbofan.tag] : Data
                Turbofan-specific conditions
                - bypass_ratio : numpy.ndarray
                    Bypass ratio
                - total_temperature_reference : numpy.ndarray
                    Reference total temperature [K]
                - total_pressure_reference : numpy.ndarray
                    Reference total pressure [Pa]
                - throttle : float
                    Throttle setting [0-1]
                - thrust_specific_fuel_consumption : numpy.ndarray
                    Thrust specific fuel consumption [kg/(N·s)]
                - non_dimensional_thrust : numpy.ndarray
                    Non-dimensional thrust
    
    Returns
    -------
    None
        Results are stored in the turbofan object:
            - TSFC : float
                Thrust specific fuel consumption [kg/(N·s)]
            - design_mass_flow_rate : float
                Core mass flow rate at design point [kg/s]
            - compressor_nondimensional_massflow : float
                Non-dimensional mass flow parameter [kg·√K/(s·Pa)]
        
    Notes
    -----
    This function determines the core mass flow rate required to produce the design thrust
    at the specified design conditions. It uses the non-dimensional thrust parameter to
    scale the mass flow appropriately, accounting for the bypass ratio.
    
    **Major Assumptions**
        * Perfect gas behavior
        * Design point is at maximum throttle (throttle = 1.0)
    
    **Theory**
    The core mass flow rate is calculated from the design thrust, non-dimensional thrust,
    and bypass ratio:
    
    .. math::
        \\dot{m}_{core} = \\frac{F_{design}}{F_{sp} \\cdot a_0 \\cdot (1 + BPR) \\cdot \\text{throttle}}
    
    The non-dimensional mass flow parameter is then calculated:
    
    .. math::
        \\dot{m}_{hc} = \\frac{\\dot{m}_{core}}{\\sqrt{\\frac{T_{ref}}{T_{t,ref}}} \\cdot \\frac{P_{t,ref}}{P_{ref}}}
    
    References
    ----------
    [1] Cantwell, B., "AA283 Course Notes", Stanford University. https://web.stanford.edu/~cantwell/AA283_Course_Material/
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.compute_thrust
    """
    # Unpack flight conditions 
    a0                  = conditions.freestream.speed_of_sound

    # Unpack turbofan flight conditions 
    Tref                = turbofan.reference_temperature
    Pref                = turbofan.reference_pressure 
    turbofan_conditions = conditions.energy.propulsors[turbofan.tag]
    bypass_ratio        = turbofan_conditions.bypass_ratio
    Tt_ref              = turbofan_conditions.total_temperature_reference  
    Pt_ref              = turbofan_conditions.total_pressure_reference
    
    # Compute nondimensional thrust
    turbofan_conditions.throttle = 1.0
    compute_thrust(turbofan,conditions) 

    # Compute dimensional mass flow rates
    TSFC       = turbofan_conditions.thrust_specific_fuel_consumption
    Fsp        = turbofan_conditions.non_dimensional_thrust
    mdot_core  = turbofan.design_thrust/(Fsp*a0*(1+bypass_ratio)*turbofan_conditions.throttle)  
    mdhc       = mdot_core/ (np.sqrt(Tref/Tt_ref)*(Pt_ref/Pref))

    # Store results on turbofan data structure 
    turbofan.TSFC                                = TSFC
    turbofan.design_mass_flow_rate               = mdot_core
    turbofan.compressor_nondimensional_massflow  = mdhc

    return  

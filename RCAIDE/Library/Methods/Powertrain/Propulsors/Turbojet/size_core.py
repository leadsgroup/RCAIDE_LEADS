# RCAIDE/Methods/Energy/Propulsors/Turbojet/size_core.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet           import compute_thrust

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  size_core
# ---------------------------------------------------------------------------------------------------------------------- 
def size_core(turbojet, conditions):
    """
    Sizes the core flow for a turbojet engine at the design condition.
    
    Parameters
    ----------
    turbojet : RCAIDE.Library.Components.Propulsors.Turbojet
        Turbojet engine component with the following attributes:
            - tag : str
                Identifier for the turbojet
            - reference_temperature : float
                Reference temperature for mass flow scaling [K]
            - reference_pressure : float
                Reference pressure for mass flow scaling [Pa]
            - design_thrust : float
                Design thrust at the design point [N]
    conditions : RCAIDE.Framework.Mission.Common.Conditions
        Flight conditions with:
            - freestream : Data
                Freestream properties
                - speed_of_sound : numpy.ndarray
                    Speed of sound [m/s]
            - energy.propulsors[turbojet.tag] : Data
                Turbojet-specific conditions
                - total_temperature_reference : numpy.ndarray
                    Reference total temperature [K]
                - total_pressure_reference : numpy.ndarray
                    Reference total pressure [Pa]
                - non_dimensional_thrust : numpy.ndarray
                    Non-dimensional thrust
    
    Returns
    -------
    None
        Results are stored in the turbojet object:
          - design_mass_flow_rate : float
              Core mass flow rate at design point [kg/s]
          - compressor_nondimensional_massflow : float
              Non-dimensional mass flow parameter [kg·√K/(s·Pa)]
    
    Notes
    -----
    This function determines the core mass flow rate required to produce the design thrust
    at the specified design conditions. It uses the non-dimensional thrust parameter to
    scale the mass flow appropriately.
    
    **Major Assumptions**
        * Perfect gas behavior
        * Design point is at maximum throttle (throttle = 1.0)
    
    **Theory**
    The core mass flow rate is calculated from the design thrust and non-dimensional thrust:
    
    .. math::
        \\dot{m}_{core} = \\frac{F_{design}}{F_{sp} \\cdot a_0 \\cdot \\text{throttle}}
    
    The non-dimensional mass flow parameter is then calculated:
    
    .. math::
        \\dot{m}_{hc} = \\frac{\\dot{m}_{core}}{\\sqrt{\\frac{T_{ref}}{T_{t,ref}}} \\cdot \\frac{P_{t,ref}}{P_{ref}}}
    
    References
    ----------
    [1] Cantwell, B., "AA283 Course Notes", Stanford University. https://web.stanford.edu/~cantwell/AA283_Course_Material/
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.compute_thrust
    """
    #unpack inputs
    a0                   = conditions.freestream.speed_of_sound
    throttle             = 1.0

    #unpack from turbojet 
    Tref                        = turbojet.reference_temperature
    Pref                        = turbojet.reference_pressure  
    turbojet_conditions         = conditions.energy.propulsors[turbojet.tag]
    total_temperature_reference = turbojet_conditions.total_temperature_reference  
    total_pressure_reference    = turbojet_conditions.total_pressure_reference 

    #compute nondimensional thrust
    turbojet_conditions.throttle = 1.0
    compute_thrust(turbojet,conditions)

    #unpack results 
    Fsp                         = turbojet_conditions.non_dimensional_thrust

    #compute dimensional mass flow rates
    mdot_core                   = turbojet.design_thrust/(Fsp*a0*throttle)  
    mdhc                        = mdot_core/ (np.sqrt(Tref/total_temperature_reference)*(total_pressure_reference/Pref))

    #pack outputs
    turbojet.design_mass_flow_rate               = mdot_core
    turbojet.compressor_nondimensional_massflow  = mdhc

    return    

# RCAIDE/Methods/Energy/Propulsors/Turbofan_Propulsor/size_core.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan_Propulsor            import compute_thrust

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  size_core
# ---------------------------------------------------------------------------------------------------------------------- 
def size_core(turbofan,turbofan_conditions,conditions):
    """
    Sizes the core flow for the design condition by computing non-dimensional thrust 
    and required mass flow rates.
 
    Parameters
    ----------
    turbofan : Turbofan
        Turbofan engine object containing design parameters
            * bypass_ratio : float
                Engine bypass ratio [-]
            * reference_temperature : float
                Reference temperature [K]
            * reference_pressure : float
                Reference pressure [Pa]
            * design_thrust : float
                Design thrust requirement [N]
    turbofan_conditions : Data
        Turbofan operating conditions
            * total_temperature_reference : float
                Total temperature at reference station [K]
            * total_pressure_reference : float
                Total pressure at reference station [Pa]
            * throttle : float
                Throttle setting [-]
    conditions : Data
        Flight conditions
            * freestream.speed_of_sound : float
                Freestream speed of sound [m/s]
 
    Returns
    -------
    None
        Updates turbofan object attributes in-place:
            * TSFC : float
                Thrust specific fuel consumption [kg/(N*s)]
            * mass_flow_rate_design : float
                Design core mass flow rate [kg/s]
            * compressor_nondimensional_massflow : float
                Non-dimensional mass flow parameter [-]
 
    Notes
    -----
    This function determines the required core mass flow rate to achieve the design
    thrust using non-dimensional analysis.
 
    **Major Assumptions**
        * Working fluid is a perfect gas
        * Steady state operation
        * One-dimensional flow
    
    **Theory**
    The core sizing is based on non-dimensional thrust parameters:
    .. math::
         \\dot{m}_{core} = \\frac{F_{design}}{F_{sp} a_0 (1+BPR) \\theta}
    where:
         - :math:`F_{design}` is the design thrust
         - :math:`F_{sp}` is the specific thrust
         - :math:`a_0` is the freestream speed of sound
         - :math:`BPR` is the bypass ratio
         - :math:`\\theta` is the throttle setting
        
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan_Propulsor.compute_thrust
    """            
    # Unpack flight conditions 
    a0             = conditions.freestream.speed_of_sound

    # Unpack turbofan flight conditions 
    bypass_ratio   = turbofan_conditions.bypass_ratio
    Tref           = turbofan.reference_temperature
    Pref           = turbofan.reference_pressure 
    Tt_ref         = turbofan_conditions.total_temperature_reference  
    Pt_ref         = turbofan_conditions.total_pressure_reference
    
    # Compute nondimensional thrust
    turbofan_conditions.throttle = 1.0
    compute_thrust(turbofan,turbofan_conditions,conditions) 

    # Compute dimensional mass flow rates
    TSFC       = turbofan_conditions.thrust_specific_fuel_consumption
    Fsp        = turbofan_conditions.non_dimensional_thrust
    mdot_core  = turbofan.design_thrust/(Fsp*a0*(1+bypass_ratio)*turbofan_conditions.throttle)  
    mdhc       = mdot_core/ (np.sqrt(Tref/Tt_ref)*(Pt_ref/Pref))

    # Store results on turbofan data structure 
    turbofan.TSFC                                = TSFC
    turbofan.mass_flow_rate_design               = mdot_core
    turbofan.compressor_nondimensional_massflow  = mdhc

    return  

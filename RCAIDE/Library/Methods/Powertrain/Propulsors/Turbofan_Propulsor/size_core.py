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
    """Sizes the core flow for the design condition by computing the
    non-dimensional thrust 

    Assumptions:
        Working fluid is a perfect gas

    Source:
        https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_Notes/

    Args:
        conditions.freestream.speed_of_sound  (numpy.ndarray): [m/s]  
        turbofan
          .bypass_ratio                (float): bypass_ratio                [-]
          .total_temperature_reference (float): total temperature reference [K]
          .total_pressure_reference    (float): total pressure reference    [Pa]  
          .reference_temperature              (float): reference temperature       [K]
          .reference_pressure                 (float): reference pressure          [Pa]
          .design_thrust                      (float): design thrust               [N]  

    Returns:
        None 
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

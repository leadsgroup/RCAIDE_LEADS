# RCAIDE/Methods/Energy/Propulsors/Turboprop_Propulsor/size_core.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop          .compute_thrust import compute_thrust 

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  size_core
# ---------------------------------------------------------------------------------------------------------------------- 
def size_core(turboprop,conditions):
    """Sizes the core flow for the design condition.

    Assumptions:
    Perfect gas
    Turboprop Engine

    Sources:
    [1] 

    Inputs:
    conditions.freestream.speed_of_sound [m/s] (conditions is also passed to turboprop.compute(..))
    turboprop.inputs.
      bypass_ratio                            [-]
      total_temperature_reference             [K]
      total_pressure_reference                [Pa]
      number_of_engines                       [-]

    Outputs:
    turboprop.outputs.non_dimensional_power  [-]

    Properties Used:
    turboprop.
      reference_temperature                   [K]
      reference_pressure                      [Pa]
      total_design                            [W] - Design power
    """             
    turboprop_conditions = conditions.energy.propulsors[turboprop.tag]
    Tt_ref               = turboprop_conditions.total_temperature_reference  
    Pt_ref               = turboprop_conditions.total_pressure_reference  
    Tref                 = turboprop.reference_temperature
    Pref                 = turboprop.reference_pressure
    
    # Compute nondimensional thrust
    turboprop_conditions.throttle = 1.0
    compute_thrust(turboprop,conditions) 

    # Store results on turboprop data structure 
    TSFC        = turboprop_conditions.thrust_specific_fuel_consumption
    Fsp         = turboprop_conditions.non_dimensional_thrust  
    mdot_core   = turboprop.design_thrust*turboprop_conditions.throttle/(Fsp) 
    mdhc        = mdot_core/ (np.sqrt(Tref/Tt_ref)*(Pt_ref/Pref))   
    
    turboprop.TSFC                                = TSFC
    turboprop.design_mass_flow_rate               = mdot_core 
    turboprop.compressor_nondimensional_massflow  = mdhc
       
    return    

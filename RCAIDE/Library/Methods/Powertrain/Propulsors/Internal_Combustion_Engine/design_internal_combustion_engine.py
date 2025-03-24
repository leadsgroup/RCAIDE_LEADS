# RCAIDE/Library/Methods/Powertrain/Propulsors/Internal_Combustion_Engine/design_internal_combustion_engine.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jul 2024, RCAIDE Team

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE Imports
import RCAIDE 
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor   import design_propeller  

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Design Turbofan
# ---------------------------------------------------------------------------------------------------------------------- 
def design_internal_combustion_engine(ICE):
    """Compute perfomance properties of a propeller-driven internal combustion engine model
    Turbofan is created by manually linking the different components
    
    
    Assumtions:
       None 
    
    Source:
    
    Args:
        ICE (dict):  propeller-driven internal combustion engine [-]
    
    Returns:
        None 
    
    """
    
    # Step 1 Design the Propeller  
    design_propeller(ICE.propeller)
     
    # Step 2 Static Sea Level Thrust  
    #atmo_data_sea_level   = atmosphere.compute_values(0.0,0.0)   
    #V                     = atmo_data_sea_level.speed_of_sound[0][0]*0.01 
    #operating_state       = setup_operating_conditions(turbofan, altitude = 0,velocity_range=np.array([V]))  
    #operating_state.conditions.energy.propulsors[turbofan.tag].throttle[:,0] = 1.0  
    #sls_T,_,sls_P,_,_,_                          = turbofan.compute_performance(operating_state) 
    #turbofan.sealevel_static_thrust              = sls_T[0][0]
    #turbofan.sealevel_static_power               = sls_P[0][0]
     
    return 
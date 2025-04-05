# RCAIDE/Library/Methods/Powertrain/Propulsors/Internal_Combustion_Engine/design_internal_combustion_engine.py
# 
# Created:  Mar 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE Imports
import RCAIDE 
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor  import design_propeller  
from RCAIDE.Library.Methods.Powertrain                   import setup_operating_conditions 

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Design Electric Rotor 
# ----------------------------------------------------------------------------------------------------------------------
def design_internal_combustion_engine(ICE,number_of_stations = 20,solver_name= 'SLSQP',iterations = 200,
                      solver_sense_step = 1E-5,solver_tolerance = 1E-4,print_iterations = False):
    """Sizes the propeller of an propeller-driven internal combustion engine 
    
    Assumtions:
       None 
    
    Source:
    
    Args:
        ICE (dict):  propeller-driven internal combustion engine [-]
    
    Returns:
        None 
    
    """
    
    # Step 1 Design the Propeller  
    design_propeller(ICE.propeller,number_of_stations = 20) 
     
    # Static Sea Level Thrust   
    atmosphere            = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976() 
    atmo_data_sea_level   = atmosphere.compute_values(0.0,0.0)   
    V                     = atmo_data_sea_level.speed_of_sound[0][0]*0.01 
    operating_state       = setup_operating_conditions(ICE, altitude = 0,velocity_range=np.array([V]))  
    operating_state.conditions.energy.propulsors[ICE.tag].throttle[:,0] = 1.0  
    sls_T,_,sls_P,_,_,_               = ICE.compute_performance(operating_state) 
    ICE.sealevel_static_thrust        = sls_T[0][0]
    ICE.sealevel_static_power         = sls_P[0][0]
    return 
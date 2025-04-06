# RCAIDE/Library/Methods/Powertrain/Propulsors/Constant_Speed_Internal_Combustion_Engine/design_constant_speed_internal_combustion_engine.py
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
def design_constant_speed_internal_combustion_engine(ICE_CS, number_of_stations=20):
    """
    Sizes the propeller of a propeller-driven constant speed internal combustion engine and computes
    sea level static performance.
    
    Parameters
    ----------
    ICE_CS : RCAIDE.Library.Components.Propulsors.Constant_Speed_Internal_Combustion_Engine
        Constant speed internal combustion engine propulsor component with the following attributes:
            - tag : str
                Identifier for the propulsor
            - propeller : Data
                Propeller component to be designed
    number_of_stations : int, optional
        Number of radial stations for propeller blade discretization
        Default: 20
    
    Returns
    -------
    None
        Results are stored in the ICE_CS object:
            - sealevel_static_thrust : float
                Sea level static thrust [N]
            - sealevel_static_power : float
                Sea level static power [W]
    
    Notes
    -----
    This function performs two main tasks:
        1. Designs the propeller using the design_propeller function with the specified
           number of stations
        2. Computes the sea level static performance (thrust and power) of the engine-propeller
           combination at full throttle
    
    The sea level static performance is calculated by:
        - Setting up atmospheric conditions at sea level
        - Creating a low-speed operating state (1% of sea level speed of sound)
        - Setting the throttle to maximum (1.0)
        - Computing the performance at these conditions
    
    Unlike variable speed internal combustion engines, the constant speed variant maintains
    a fixed RPM and varies the propeller blade pitch to control thrust. This is common in
    generator applications and some aircraft engines where constant speed operation is
    beneficial for efficiency or noise reasons.
    
    **Major Assumptions**
        * US Standard Atmosphere 1976 is used for atmospheric properties
        * Sea level static conditions are approximated with a very low velocity (1% of speed of sound)
        * Full throttle (throttle = 1.0) is used for sea level static performance
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.design_propeller
    RCAIDE.Library.Methods.Powertrain.setup_operating_conditions
    RCAIDE.Library.Methods.Powertrain.Propulsors.Constant_Speed_Internal_Combustion_Engine.compute_performance
    """
    
    # Step 1 Design the Propeller  
    design_propeller(ICE_CS.propeller,number_of_stations = 20) 
     
    # Static Sea Level Thrust   
    atmosphere            = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976() 
    atmo_data_sea_level   = atmosphere.compute_values(0.0,0.0)   
    V                     = atmo_data_sea_level.speed_of_sound[0][0]*0.01 
    operating_state       = setup_operating_conditions(ICE_CS, altitude = 0,velocity_range=np.array([V]))  
    operating_state.conditions.energy.propulsors[ICE_CS.tag].throttle[:,0] = 1.0  
    sls_T,_,sls_P,_,_,_               = ICE_CS.compute_performance(operating_state) 
    ICE_CS.sealevel_static_thrust        = sls_T[0][0]
    ICE_CS.sealevel_static_power         = sls_P[0][0]
    return 
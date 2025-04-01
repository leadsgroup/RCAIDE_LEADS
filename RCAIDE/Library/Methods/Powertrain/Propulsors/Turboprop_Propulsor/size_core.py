# RCAIDE/Methods/Energy/Propulsors/Turboprop_Propulsor/size_core.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop_Propulsor.compute_thrust import compute_thrust 

# ----------------------------------------------------------------------------------------------------------------------
#  size_core
# ---------------------------------------------------------------------------------------------------------------------- 
def size_core(turboprop,turboprop_conditions,conditions):
    """
    Sizes the core flow for the turboprop engine at design conditions.

    Parameters
    ----------
    turboprop : Turboprop
        Turboprop engine object containing design parameters
            - inputs.bypass_ratio : float
                Engine bypass ratio
            - inputs.total_temperature_reference : float
                Reference total temperature [K]
            - inputs.total_pressure_reference : float
                Reference total pressure [Pa]
            - inputs.number_of_engines : int
                Number of engines
            - reference_temperature : float
                Reference temperature [K]
            - reference_pressure : float
                Reference pressure [Pa]
            - total_design : float
                Design power output [W]
    turboprop_conditions : Conditions
        Turboprop operating conditions data structure
    conditions : Conditions
        Freestream conditions data structure
            - freestream.speed_of_sound : float
                Freestream speed of sound [m/s]

    Returns
    -------
    None
        Results are stored in turboprop_conditions:
            - non_dimensional_power : float
                Non-dimensional power output [-]

    Notes
    -----
    This function sizes the core flow path of the turboprop engine to achieve
    the required design power output while maintaining appropriate component
    matching.

    **Major Assumptions**
        * Perfect gas behavior
        * Core flow is sized at design point conditions
        * Component efficiencies are constant

    **Theory**

    The core sizing is based on achieving the required power output while
    maintaining appropriate flow conditions through each component. The
    non-dimensional power parameter is used to scale the core flow appropriately.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop_Propulsor.compute_thrust 
    """             

    compute_thrust(turboprop,turboprop_conditions,conditions)  
       
    return    

# RCAIDE/Library/Methods/Powertrain/Propulsors/Turboshaft_Propulsor/append_turboshaft_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_turboshaft_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_turboshaft_conditions(turboshaft,segment):  
    """
    Initializes the state conditions data structure for a turboshaft engine in a mission segment.

    Parameters
    ----------
    turboshaft : Turboshaft
        Turboshaft engine object for which conditions are being initialized
    segment : Segment
        Mission segment object containing the state conditions
            - state.ones_row : function
                Returns array of ones with specified length

    Returns
    -------
    None
        Initializes the following conditions in segment.state.conditions:
            - energy[turboshaft.tag] : Conditions
                - throttle : array
                    Engine throttle setting
                - commanded_thrust_vector_angle : array
                    Commanded thrust vector angle [rad]
                - power : array
                    Engine power output [W]
                - fuel_flow_rate : array
                    Fuel mass flow rate [kg/s]
                - inputs : Conditions
                    Input parameters for engine analysis
                - outputs : Conditions
                    Output parameters from engine analysis
            - noise[turboshaft.tag] : Conditions
                - turboshaft : Conditions
                    Engine noise parameters
                - turboshaft.core_nozzle : Conditions
                    Core nozzle noise parameters

    Notes
    -----
    This function creates and initializes the necessary data structures to store
    turboshaft engine operating conditions throughout a mission segment. All
    numeric arrays are initialized to zero with the appropriate dimensions based
    on the segment's discretization.

    **Major Assumptions**
        * All initial conditions are set to zero
        * Single row arrays are used for all parameters
        * Consistent units are maintained throughout

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turboshaft_Propulsor.compute_turboshaft_performance 
    """

    ones_row    = segment.state.ones_row                  
    segment.state.conditions.energy[turboshaft.tag]                               = Conditions()  
    segment.state.conditions.energy[turboshaft.tag].throttle                      = 0. * ones_row(1)     
    segment.state.conditions.energy[turboshaft.tag].commanded_thrust_vector_angle = 0. * ones_row(1)   
    segment.state.conditions.energy[turboshaft.tag].power                         = 0. * ones_row(1)
    segment.state.conditions.energy[turboshaft.tag].fuel_flow_rate                = 0. * ones_row(1)
    segment.state.conditions.energy[turboshaft.tag].inputs                        = Conditions()
    segment.state.conditions.energy[turboshaft.tag].outputs                       = Conditions() 
    segment.state.conditions.noise[turboshaft.tag]                                = Conditions() 
    segment.state.conditions.noise[turboshaft.tag].turboshaft                     = Conditions() 
    segment.state.conditions.noise[turboshaft.tag].turboshaft.core_nozzle         = Conditions()   
    return 
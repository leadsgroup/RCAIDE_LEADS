# RCAIDE/Library/Methods/Powertrain/Propulsors/Internal_Combustion_Engine/append_internal_combustion_engine_residual_and_unknown.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
from RCAIDE.Framework.Core import  Units

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_internal_combustion_engine_residual_and_unknown
# ----------------------------------------------------------------------------------------------------------------------  
def append_internal_combustion_engine_residual_and_unknown(propulsor, segment):
    """
    Appends the torque matching residual and unknown for an internal combustion engine.
    
    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Propulsors.Internal_Combustion_Engine
        Internal combustion engine propulsor component with the following attributes:
            - tag : str
                Identifier for the engine
            - propeller : Data
                Propeller component
                    - cruise : Data
                        Cruise conditions
                            - design_angular_velocity : float
                                Design angular velocity of the propeller [rad/s]
    segment : RCAIDE.Framework.Mission.Segments.Segment
        Mission segment with the following attributes:
            - state : Data
                Segment state
                    - ones_row : function
                        Function to create array of ones with specified length
                    - unknowns : dict
                        Dictionary of unknown variables for the solver
                    - residuals : Data
                        Residuals for the solver
                            - network : dict
                                Network residuals
        
    Returns
    -------
    None
        Results are stored in segment.state.unknowns and segment.state.residuals.network
    
    Notes
    -----
    This function sets up the necessary variables for solving the torque balance between
    the internal combustion engine and the propeller. It adds:
        * An unknown variable for the propeller angular velocity, initialized with the 
          design angular velocity
        * A residual for the torque balance between the engine and propeller
    
    The torque balance residual will be driven to zero during the numerical solution
    process, ensuring that the engine provides exactly the torque required by the
    propeller at the current flight condition.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Internal_Combustion_Engine.compute_internal_combustion_engine_performance
    """
    
    ones_row    = segment.state.ones_row                   
    propeller   = propulsor.propeller 
    segment.state.unknowns[propulsor.tag  + '_propeller_omega'] = ones_row(1) * float(propeller.cruise.design_angular_velocity)  
    segment.state.residuals.network[ propulsor.tag + '_rotor_engine_torque'] = 0. * ones_row(1)
    
    return 
# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/append_electric_rotor_residual_and_unknown.py
# 
# Created:  Jun 2024, M. Clarke  
 
# ---------------------------------------------------------------------------------------------------------------------- 
#  append_electric_rotor_residual_and_unknown
# ----------------------------------------------------------------------------------------------------------------------  
def append_electric_rotor_residual_and_unknown(propulsor, segment):
    """
    Appends the torque matching residual and unknown for an electric rotor propulsor.
    
    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Propulsors.Electric_Rotor
        Electric rotor propulsor component with the following attributes:
            - tag : str
                Identifier for the propulsor
            - motor : Data
                Electric motor component
                - design_current : float
                    Design current of the motor [A]
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
    the electric motor and the rotor. It adds:
        - An unknown variable for the motor current, initialized with the design current
        - A residual for the torque balance between the motor and rotor
    
    The torque balance residual will be driven to zero during the numerical solution
    process, ensuring that the motor provides exactly the torque required by the
    rotor at the current flight condition.
    
    **Major Assumptions**
        * The motor current is initialized with the design value
        * The torque balance residual is initialized as zero
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Rotor.compute_electric_rotor_performance
    """
    ones_row    = segment.state.ones_row  
    motor       = propulsor.motor  
    segment.state.unknowns[propulsor.tag + '_motor_current']              = motor.design_current * ones_row(1) 
    segment.state.residuals.network[propulsor.tag +'_rotor_motor_torque'] = 0. * ones_row(1) 
    return 
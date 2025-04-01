# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/unpack_electric_rotor_unknowns.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke   

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports

import RCAIDE

# ---------------------------------------------------------------------------------------------------------------------- 
#  unpack electric rotor network unknowns 
# ----------------------------------------------------------------------------------------------------------------------  

def unpack_electric_rotor_unknowns(propulsor,segment): 
    """
    Unpacks the power coefficient unknown from the solver state into the motor conditions.
    This function transfers the solver's estimate of the rotor power coefficient to the energy
    conditions where it will be used to compute motor performance.

    Parameters
    ----------
    propulsor : RCAIDE.Core.Systems.Propulsors
        The electric rotor propulsion system
            - tag : str
                Identifier for the propulsor
            - motor : Component
                The motor component that receives the power coefficient
    segment : RCAIDE.Core.Analyses.Mission.Segments
        The mission segment being analyzed
            - state : State
                Contains solver unknowns and energy conditions
                    - unknowns : dict
                        Contains the power coefficient estimate
                    - conditions : Conditions
                        Energy conditions to be updated

    Returns
    -------
    None
        Function modifies the segment state conditions in-place
    """
    
    results = segment.state.conditions.energy[propulsor.tag]
    motor   = propulsor.motor  
    rotor   = propulsor.rotor
    if rotor.fidelity == 'Blade_Element_Momentum_Theory_Helmholtz_Wake': 
        if (type(motor) == RCAIDE.Library.Components.Powertrain.Converters.PMSM_Motor):
            results[motor.tag].current = segment.state.unknowns[propulsor.tag + '_current'] 
        elif (type(motor) == RCAIDE.Library.Components.Powertrain.Converters.DC_Motor):
            results[motor.tag].rotor_power_coefficient = segment.state.unknowns[propulsor.tag + '_rotor_cp'] 
    return 
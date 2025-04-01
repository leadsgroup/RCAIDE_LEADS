# RCAIDE/Library/Missions/Common/Unpack_Unknowns/energy.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  Unpack Unknowns
# ---------------------------------------------------------------------------------------------------------------------- 
def unknowns(segment):  
    """
    Updates energy system controls from solver unknowns

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - assigned_control_variables : Data
                Control configurations
                - throttle : Data
                    Throttle control settings
                - thrust_vector_angle : Data
                    Thrust vectoring settings
            - analyses.energy.vehicle.networks : list
                Propulsion network definitions
            - state.conditions.energy : Data
                Energy system conditions
            - state.unknowns : Data
                Solver unknown values

    Returns
    -------
    None
        Updates segment conditions directly

    Notes
    -----
    This function applies energy system control values from the solver's
    unknowns to the vehicle's propulsion systems. It handles throttle
    settings and thrust vector angles for all propulsors.

    The function processes:
        1. Throttle settings
            - Direct segment throttle values
            - Solver-determined throttle values
        2. Thrust vector angles
            - Commanded thrust vectoring angles

    **Major Assumptions**
        * Valid propulsion system definitions
        * Proper propulsor assignments
        * Compatible control values
        * Well-defined energy networks

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """
    
    ACV_T  =  segment.assigned_control_variables.throttle
    ACV_TA =  segment.assigned_control_variables.thrust_vector_angle
    
    for network in segment.analyses.energy.vehicle.networks: 
        if 'throttle' in segment: 
            for propulsor in network.propulsors: 
                segment.state.conditions.energy[propulsor.tag].throttle[:,0] = segment.throttle
            
        if ACV_T.active: 
            for i in range(len(ACV_T.assigned_propulsors)): 
                propulsor_group = ACV_T.assigned_propulsors[i]
                for propulsor_name in propulsor_group:  
                    segment.state.conditions.energy[propulsor_name].throttle = segment.state.unknowns["throttle_" + str(i)]  
    
       # Thrust Vector Control 
        if ACV_TA.active:                
            for i in range(len(ACV_TA.assigned_propulsors)): 
                propulsor_group = ACV_TA.assigned_propulsors[i]
                for propulsor_name in propulsor_group:  
                    segment.state.conditions.energy[propulsor_name].commanded_thrust_vector_angle = segment.state.unknowns["thrust_vector_" + str(i)]
    return 
     
 
    

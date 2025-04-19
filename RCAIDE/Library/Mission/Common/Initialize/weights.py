# RCAIDE/Library/Missions/Common/Initialize/weights.py
# 
# 
# Created:  Jul 2023, M. Clarke
 

# ----------------------------------------------------------------------------------------------------------------------
#  Initialize Weights
# ---------------------------------------------------------------------------------------------------------------------- 
def weights(segment):
    """ Initializes weight of vehicle
    
        Assumptions:   
            Uses max takeoff weight if no weight analysis is performed 
            
        Inputs: 
            state.
                initials.conditions.weights.total_mass  [kg]
            state.conditions:           
                conditions.weights.total_mass           [kg]
            
        Outputs: 
            state.conditions.weights.total_mass         [kg] 
           
        Properties Used:
        N/A 
    """      
 
    if segment.state.initials:
        m_initial = segment.state.initials.conditions.weights.total_mass[-1,0] 
    else: 
        if segment.analyses.weights != None: 
            m_initial = segment.analyses.weights.vehicle.mass_properties.takeoff
        else: 
            m_initial = segment.analyses.energy.vehicle.networks[list(segment.analyses.energy.vehicle.networks.keys())[0]].mass_properties.mass

    m_current = segment.state.conditions.weights.total_mass
    
    segment.state.conditions.weights.total_mass[:,:] = m_current + (m_initial - m_current[0,0])
        
    return 
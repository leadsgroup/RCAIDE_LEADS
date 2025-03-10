# RCAIDE/Library/Missions/Common/Update/thrust.py
# 
# 
# Created:  Jul 2023, M. Clarke
import  RCAIDE
import  numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Update Thrust
# ---------------------------------------------------------------------------------------------------------------------- 
def thrust(segment):
    """ Updates the thrust vector of the vehicle from the propulsors 
        
        Assumptions:
        N/A
        
        Inputs:
            None 
                 
        Outputs: 
            None
      
        Properties Used:
        N/A
                    
    """ 

    # unpack
    energy_model = segment.analyses.energy

    # evaluate
    energy_model.evaluate(segment.state)    

    # pack conditions
    conditions = segment.state.conditions
    conditions.frames.body.thrust_force_vector       = conditions.energy.thrust_force_vector
    conditions.frames.body.thrust_moment_vector      = conditions.energy.thrust_moment_vector 
    
    if type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude or\
        type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude_AVL_Trimmed or \
         type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude_No_Propulsion or\
          type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Throttle:
        pass
    else: 
        I = segment.state.numerics.time.integrate         
        conditions.energy.fuel_consumption        = np.dot(I,conditions.weights.vehicle_mass_rate)
        conditions.energy.cumulative_fuel_consumption =  conditions.energy.fuel_consumption
        if segment.state.initials:  
            conditions.energy.cumulative_fuel_consumption += segment.state.initials.conditions.energy.cumulative_fuel_consumption[-1]
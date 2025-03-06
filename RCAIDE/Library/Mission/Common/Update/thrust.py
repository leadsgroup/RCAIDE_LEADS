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

    
    if segment.process.initialize.expand_state ==  RCAIDE.Library.Methods.skip:
        pass
    else: 
        I = segment.state.numerics.time.integrate         
        conditions.energy.fuel_consumption        = np.dot(I,conditions.weights.vehicle_mass_rate)
        conditions.energy.cumulative_fuel_consumption =  conditions.energy.fuel_consumption
        if segment.state.initials:  
            conditions.energy.cumulative_fuel_consumption += segment.state.initials.conditions.energy.cumulative_fuel_consumption[-1]
            
    if "vehicle_additional_fuel_rate" in conditions.energy: 
        conditions.weights.has_additional_fuel             = True
        conditions.weights.vehicle_fuel_rate               = conditions.energy.vehicle_fuel_rate
        conditions.weights.vehicle_additional_fuel_rate    = conditions.energy.vehicle_additional_fuel_rate  
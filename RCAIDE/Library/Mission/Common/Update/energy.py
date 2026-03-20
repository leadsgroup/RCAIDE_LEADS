# RCAIDE/Library/Missions/Common/Update/energy.py
# 
# 
# Created:  Jul 2023, M. Clarke 

import RCAIDE

# ----------------------------------------------------------------------------------------------------------------------
#  Update Battery Age
# ---------------------------------------------------------------------------------------------------------------------- 
def energy(segment):  
    """Updates battery age based on operating conditions, cell temperature and time of operation.
       Source: 
       Cell specific. See individual battery cell for more details

       Assumptions:
       Cell specific. See individual battery cell for more details

       Inputs: 
       segment.
           conditions                         - conditions of battery at each segment  [unitless]
           increment_battery_age_by_one_day   - flag to increment battery cycle day    [boolean]

       Outputs:
       N/A  

       Properties Used:
       N/A 
    """  
    # loop throuh networks in vehicle 
    for network in segment.analyses.vehicle.networks:  
        for source in network.sources:
            if issubclass(type(source), RCAIDE.Library.Components.Powertrain.Sources.Batteries.Battery_Pack):
                increment_day       = segment.increment_battery_age_by_one_day
                battery_conditions  = segment.conditions.energy.sources[source.tag]
                network.sources[source.tag].update_battery_age(segment,battery_conditions,increment_battery_age_by_one_day = increment_day) 
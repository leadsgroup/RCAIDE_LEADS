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
    for network in segment.analyses.energy.vehicle.networks:  
        for distributor in network.distributors:
            if issubclass(type(distributor), RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus):
                for source in distributor.assigned_sources: 
                    increment_day = segment.increment_battery_age_by_one_day
                    battery_conditions  = segment.conditions.energy.sources[source.tag]
                    source.update_battery_age(segment,battery_conditions,increment_battery_age_by_one_day = increment_day) 
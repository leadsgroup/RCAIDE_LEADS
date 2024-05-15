## @ingroup Methods-Energy-Sources-Battery 
# RCAIDE/Methods/Energy/Sources/Battery/Common/compute_thermal_performance.py
# 
# 
# Created:  Mar 20234 M. Clarke, S S. Shekar

 
# ----------------------------------------------------------------------------------------------------------------------
# compute_thermal_performance
# ---------------------------------------------------------------------------------------------------------------------- 
## @ingroup Energy-Sources-Batteries-Lithium_Ion_NMC 
def compute_thermal_performance(HAS,HEX,RES,battery_conditions,battery,Q_heat_cell,state,delta_t,i): 
    '''
    Computes the thermal properties of the battery for each control point.
    Inputs:  
            HAS
            HEX
            RES
            battery_conditions
            state
            
           
        Outputs:
                battery_conditions.thermal_management_system.power[
           
    
    '''   
    # compute heat transfer between coolant and battery pack 
    HAS.compute_heat_removed(battery_conditions,battery,Q_heat_cell[i],state,delta_t[i],i)   
    heat_removal = battery_conditions.thermal_management_system.heat_removal_flag
    
    if heat_removal == True: 
        HEX.compute_heat_removed(battery_conditions,state,delta_t[i],i) 
        # compute heat transfer between coolant and reservoir
        RES.compute_reservior_coolant_temperature(battery_conditions,state,delta_t[i],i,heat_removal)          
    
    # Compute Heat loss/gain 
    RES.compute_reservior_heat_transfer(battery_conditions,state,delta_t[i],i)  
    
    # compute total power 
    battery_conditions.thermal_management_system.power[i+1] = battery_conditions.thermal_management_system.HAS.power[i+1] +\
                                                              battery_conditions.thermal_management_system.HEX.power[i+1] 
                                                              
    
    return 
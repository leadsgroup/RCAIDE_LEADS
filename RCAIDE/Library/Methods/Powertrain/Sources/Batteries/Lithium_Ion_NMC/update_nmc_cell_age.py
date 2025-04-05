# RCAIDE/Methods/Powertrain/Sources/Batteries/Lithium_Ion_NMC/update_nmc_cell_age.py
# 
# 
# Created:  Feb 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np  
 
# ----------------------------------------------------------------------------------------------------------------------
# update_nmc_cell_age
# ----------------------------------------------------------------------------------------------------------------------  
def update_nmc_cell_age(battery,segment, battery_conditions,increment_battery_age_by_one_day):  
    """ This is an aging model for 18650 lithium-nickel-manganese-cobalt-oxide batteries. 
   
    Source: 
    Schmalstieg, Johannes, et al. "A holistic aging model for Li (NiMnCo) O2
    based 18650 lithium-ion batteries." Journal of Power Sources 257 (2014): 325-334.
      
    Assumptions:
    None

    Inputs:
      segment.conditions.energy. 
         battery.cycle_in_day                                                   [unitless]
         battery.cell.temperature                                               [Kelvin] 
         battery.voltage_open_circuit                                           [Volts] 
         battery.charge_throughput                                              [Amp-hrs] 
         battery.cell.state_of_charge                                           [unitless] 
    
    Outputs:
       segment.conditions.energy.
         battery.capacity_fade_factor     (internal resistance growth factor)   [unitless]
         battery.resistance_growth_factor (capactance (energy) growth factor)   [unitless]  
         
    Properties Used:
    N/A 
    """    
    n_series   = battery.electrical_configuration.series
    SOC        = battery_conditions.cell.state_of_charge
    V_ul       = battery_conditions.voltage_under_load/n_series
    t          = battery_conditions.cell.cycle_in_day         
    Q_prior    = battery_conditions.cell.charge_throughput[-1,0] 
    Temp       = np.mean(battery_conditions.cell.temperature) 
    
    # aging model  
    delta_DOD = abs(SOC[0][0] - SOC[-1][0])
    rms_V_ul  = np.sqrt(np.mean(V_ul**2)) 
    alpha_cap = (7.542*np.mean(V_ul) - 23.75) * 1E6 * np.exp(-6976/(Temp))  
    alpha_res = (5.270*np.mean(V_ul) - 16.32) * 1E5 * np.exp(-5986/(Temp))  
    beta_cap  = 7.348E-3 * (rms_V_ul - 3.667)**2 +  7.60E-4 + 4.081E-3*delta_DOD
    beta_res  = 2.153E-4 * (rms_V_ul - 3.725)**2 - 1.521E-5 + 2.798E-4*delta_DOD
    
    E_fade_factor   = 1 - alpha_cap*(t**0.75) - beta_cap*np.sqrt(Q_prior)   
    R_growth_factor = 1 + alpha_res*(t**0.75) + beta_res*Q_prior  
    
    battery_conditions.cell.capacity_fade_factor     = np.minimum(E_fade_factor,battery_conditions.cell.capacity_fade_factor)
    battery_conditions.cell.resistance_growth_factor = np.maximum(R_growth_factor,battery_conditions.cell.resistance_growth_factor)
    
    if increment_battery_age_by_one_day:
        battery_conditions.cell.cycle_in_day += 1 # update battery age by one day 
  
    return  

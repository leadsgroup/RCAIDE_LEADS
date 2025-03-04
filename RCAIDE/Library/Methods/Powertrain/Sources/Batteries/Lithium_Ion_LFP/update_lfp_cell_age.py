# RCAIDE/Methods/Powertrain/Sources/Batteries/Lithium_Ion_LFP/update_lfp_cell_age.py
# 
# 
# Created: Nov 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np  
 
# ----------------------------------------------------------------------------------------------------------------------
# update_lfp_cell_age
# ----------------------------------------------------------------------------------------------------------------------  
def update_lfp_cell_age(battery_module,segment, battery_conditions,increment_battery_age_by_one_day):  
    """ This is an aging model for 26650 A123 LFP cell. 
   
    Source: 
    Nájera, J., J.R. Arribas, R.M. De Castro, and C.S. Núñez. 
    “Semi-Empirical Ageing Model for LFP and NMC Li-Ion Battery Chemistries.”
    Journal of Energy Storage 72 (November 2023): 108016.
    https://doi.org/10.1016/j.est.2023.108016.

      
    Assumptions:
    None
    """
    SOC                = battery_conditions.cell.state_of_charge
    I                  = battery_conditions.cell.current
    t                  = battery_conditions.cell.cycle_in_day         
    charge_thougput    = battery_conditions.cell.charge_throughput
    Temp               = (battery_conditions.cell.temperature) 
    C_rate             = np.sqrt(np.mean(I**2)) /battery_module.cell.nominal_capacity
    
    # Semi Emperical aging model  
    E_fade_factor = 1-(((Temp**2*2.0916e-8)+(-1.2179e-5*Temp)+0.0018)*np.exp(((-1.7082e-6*Temp)+0.0556)*C_rate) \
                        * charge_thougput + (5.9808e6) * np.exp(0.68989*SOC) * np.exp(-6.4647e3/Temp) * t**(0.5))


    battery_conditions.cell.capacity_fade_factor     = np.minimum(E_fade_factor[-1],battery_conditions.cell.capacity_fade_factor)
    
    if increment_battery_age_by_one_day:
        battery_conditions.cell.cycle_in_day += 1 # update battery age by one day 
    
    return 

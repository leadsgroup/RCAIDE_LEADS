# RCAIDE/Methods/Powertrain/Sources/Fuel_Cells/Larminie_Model/compute_power_difference.py
#  
# Created: Jan 2025, M. Clarke
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from .compute_power import compute_power

# ----------------------------------------------------------------------------------------------------------------------
#  Find Power Difference Larminie
# ----------------------------------------------------------------------------------------------------------------------
def compute_power_difference(current_density, fuel_cell, power_desired):
    '''
    function that determines the power difference between the actual power
    and a desired input power, based on an input current density

    Assumptions:
    None
    
    Inputs:
    current_density                [Amps/m**2]
    power_desired                  [Watts]
    fuel_cell
      
    
    Outputs
    (power_desired-power_out)**2   [Watts**2]
    '''
    power_out     = compute_power(current_density, fuel_cell)              
    
    return (power_desired-power_out)**2.
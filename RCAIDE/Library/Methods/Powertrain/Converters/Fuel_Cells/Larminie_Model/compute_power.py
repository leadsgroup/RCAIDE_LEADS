# RCAIDE/Methods/Powertrain/Sources/Fuel_Cells/Larminie_Model/compute_power.py
#  
# Created: Jan 2025, M. Clarke
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from .compute_voltage import compute_voltage 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Find Power Larminie
# ----------------------------------------------------------------------------------------------------------------------
def compute_power(current_density, fuel_cell, sign=1.0):
    '''
    Function that determines the power output per cell, based on in 
    input current density
    
    Assumptions:
    None(calls other functions)
    
    Inputs:
    current_density      [Amps/m**2]
    fuel cell.
        interface area   [m**2]
        
    Outputs:
    power_out            [W]
    
    '''
    
    # sign variable is used so that you can maximize the power, by minimizing the -power
    i1            = current_density
    A             = fuel_cell.interface_area
    v             = compute_voltage(fuel_cell,current_density)   # useful voltage vector
    power_out     = sign* np.multiply(v,i1)*A                    # obtain power output in W/cell 
    return power_out
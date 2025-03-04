# RCAIDE/Methods/Powertrain/Sources/Fuel_Cells/Larminie_Model/compute_voltage.py
#  
# Created: Jan 2025, M. Clarke
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Find Voltage Larminie
# ----------------------------------------------------------------------------------------------------------------------
def compute_voltage(fuel_cell,current_density):
    '''
    function that determines the fuel cell voltage based on an input
    current density and some semi-empirical values to describe the voltage
    drop off with current
    
    Assumptions:
    voltage curve is a function of current density of the form
    v = Eoc-r*i1-A1*np.log(i1)-m*np.exp(n*i1)
    
    Inputs:
    current_density           [A/m**2]
    fuel_cell.
        r                     [Ohms*m**2]
        A1                    [V]
        m                     [V]
        n                     [m**2/A]
        Eoc                   [V]
   
    Outputs:
        V                     [V] 
    '''
    r   = fuel_cell.r/(1000*(Units.cm**2))
    Eoc = fuel_cell.Eoc 
    A1  = fuel_cell.A1  
    m   = fuel_cell.m   
    n   = fuel_cell.n   
    
    i1 = current_density/(0.001/(Units.cm**2.)) # current density(mA cm^-2)
    v  = Eoc-r*i1-A1*np.log(i1)-m*np.exp(n*i1)     #useful voltage vector

    return v
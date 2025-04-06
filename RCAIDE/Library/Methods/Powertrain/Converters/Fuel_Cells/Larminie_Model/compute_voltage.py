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
def compute_voltage(fuel_cell, current_density):
    """
    Calculates fuel cell voltage based on current density using the Larminie-Dicks model.
    
    Parameters
    ----------
    fuel_cell : RCAIDE.Components.Energy.Converters.Fuel_Cell
        The fuel cell component containing electrochemical parameters
        - r : float
            Area-specific resistance [Ohms*cm²]
        - A1 : float
            Tafel slope [V]
        - m : float
            Mass transport loss coefficient [V]
        - n : float
            Mass transport loss exponential coefficient [cm²/A]
        - Eoc : float
            Open circuit voltage [V]
    current_density : float or array
        Current density [A/m²]
        
    Returns
    -------
    v : float or array
        Cell voltage [V]
    
    Notes
    -----
    This function implements the Larminie-Dicks semi-empirical model to calculate
    fuel cell voltage as a function of current density. The model accounts for
    activation losses, ohmic losses, and concentration losses.
    
    **Major Assumptions**
        * Voltage curve follows the Larminie-Dicks model form
        * Steady-state operation (no transient effects)
        * Uniform current distribution across the cell
        * Constant temperature operation
    
    **Theory**
    
    The Larminie-Dicks model calculates cell voltage as:
    
    .. math::
        V = E_{oc} - r \cdot i - A_1 \ln(i) - m \exp(n \cdot i)
    
    where:
        - E_{oc} is the open circuit voltage
        - r is the area-specific resistance
        - A_1 is the Tafel slope for activation losses
        - m and n are parameters for mass transport losses
        - i is the current density
    
    References
    ----------
    [1] Larminie, J., & Dicks, A. (2003). Fuel Cell Systems Explained (2nd ed.). John Wiley & Sons Ltd.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Fuel_Cells.Larminie_Model.compute_power
    """
    r   = fuel_cell.r/(1000*(Units.cm**2))
    Eoc = fuel_cell.Eoc 
    A1  = fuel_cell.A1  
    m   = fuel_cell.m   
    n   = fuel_cell.n   
    
    i1 = current_density/(0.001/(Units.cm**2.)) # current density(mA cm^-2)
    v  = Eoc-r*i1-A1*np.log(i1)-m*np.exp(n*i1)     #useful voltage vector

    return v
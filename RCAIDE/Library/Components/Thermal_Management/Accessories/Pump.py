# RCAIDE/Library/Compoments/Thermal_Management/Accessories/Pump.py
# 
# Created: March 2024  S. Shekar 

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
from RCAIDE.Framework.Core import Data

# ----------------------------------------------------------------------
#  Pump
# ----------------------------------------------------------------------
class Pump(Data):
    """
    A class representing a coolant pump for thermal management systems.

    Attributes
    ----------
    tag : str
        Unique identifier for the pump component, defaults to 'Pump'
        
    efficiency : float
        Overall pump efficiency including mechanical and electrical losses, 
        defaults to 1.0

    Notes
    -----
    The pump class models liquid circulation devices used in thermal management 
    systems. It provides functionality for:
    
    * Power consumption calculation
    * Performance modeling based on operating conditions
    * Integration with liquid cooling systems

    **Definitions**

    'Pressure Differential'
        The difference in pressure across the pump
        
    'Mass Flow Rate'
        Rate of coolant mass flow through the pump

    See Also
    --------
    RCAIDE.Library.Components.Thermal_Management.Accessories.Fan
        Similar component for gas (air) cooling systems
    """

    def __defaults__(self):
        """
        Sets default values for the pump attributes.
        """
        self.tag        = 'Pump'
        self.efficiency = 1.0
        return
   
    def compute_power_consumed(pressure_differential, density, mass_flow_rate, efficiency):
        """
        Calculates the power consumed by the pump.

        Parameters
        ----------
        pressure_differential : float
            Pressure rise across the pump
            
        density : float
            Coolant density
            
        mass_flow_rate : float
            Mass flow rate through the pump
            
        efficiency : float
            Overall pump efficiency

        Returns
        -------
        float
            Power consumed by the pump

        Notes
        -----
        Uses the standard pump power equation:
        Power = (mass_flow_rate * pressure_differential) / (density * efficiency)
        """
        return mass_flow_rate * pressure_differential / (density * efficiency)
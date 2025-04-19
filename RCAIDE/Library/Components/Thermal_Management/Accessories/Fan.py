# RCAIDE/Library/Compoments/Thermal_Management/Accessories/Fan.py
# 
#
# Created: March 2024  S. Shekar

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
from RCAIDE.Framework.Core import Data

# ----------------------------------------------------------------------
#  Fan
# ----------------------------------------------------------------------
class Fan(Data):
    """
    A class representing a cooling fan for thermal management systems.

    Attributes
    ----------
    tag : str
        Unique identifier for the fan component, defaults to 'Fan'
        
    efficiency : float
        Overall fan efficiency including mechanical and electrical losses, 
        defaults to 1.0

    Notes
    -----
    The fan class models forced air movement devices used in thermal management 
    systems. It provides functionality for:
    
    * Power consumption calculation
    * Performance modeling based on operating conditions
    * Integration with cooling system analysis

    **Definitions**

    'Pressure Differential'
        The difference in pressure across the fan
        
    'Mass Flow Rate'
        Rate of air mass flow through the fan

    See Also
    --------
    RCAIDE.Library.Components.Thermal_Management.Accessories.Pump
        Similar component for liquid cooling systems
    """

    def __defaults__(self):
        """
        Sets default values for the fan attributes.
        """
        self.tag        = 'Fan'
        self.efficiency = 1.0
        return
   
    def compute_power_consumed(pressure_differential, density, mass_flow_rate, efficiency):
        """
        Calculates the power consumed by the fan.

        Parameters
        ----------
        pressure_differential : float
            Pressure rise across the fan
            
        density : float
            Fluid density
            
        mass_flow_rate : float
            Mass flow rate through the fan
            
        efficiency : float
            Overall fan efficiency

        Returns
        -------
        float
            Power consumed by the fan

        Notes
        -----
        Uses the standard fan power equation:
        Power = (mass_flow_rate * pressure_differential) / (density * efficiency)
        """
        return mass_flow_rate * pressure_differential / (density * efficiency)
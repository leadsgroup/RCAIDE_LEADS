# RCAIDE/Library/Components/Powertrain/Energy/Sources/Battery_Modules/Lithium_Air.py
# 
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from RCAIDE.Framework.Core import Units
from .Generic_Battery_Module import Generic_Battery_Module  

# ----------------------------------------------------------------------------------------------------------------------
#  Lithium_Air
# ----------------------------------------------------------------------------------------------------------------------   
class Lithium_Air(Generic_Battery_Module):
    """
    Class for modeling lithium-air battery characteristics and performance
    
    Attributes
    ----------
    cell.specific_energy : float
        Energy capacity per unit mass [J/kg] (default: 2000 Wh/kg)
        
    cell.specific_power : float
        Maximum power output per unit mass [W/kg] (default: 0.66 kW/kg)
        
    cell.mass_gain_factor : float
        Rate of mass increase from oxygen absorption [kg/Wh] (default: 1.92E-4)

    Notes
    -----
    Lithium-air batteries use ambient oxygen as the cathode material, which is
    absorbed during discharge, leading to mass gain. They offer very high specific
    energy but have power density limitations.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Generic_Battery_Module
        Base battery module class
    """
    
    def __defaults__(self):
        """
        Sets default values for lithium-air battery attributes
        """      
        self.cell.specific_energy  = 2000.     *Units.Wh/Units.kg    # convert to Joules/kg
        self.cell.specific_power   = 0.66      *Units.kW/Units.kg    # convert to W/kg
        self.cell.mass_gain_factor = (1.92E-4) /Units.Wh
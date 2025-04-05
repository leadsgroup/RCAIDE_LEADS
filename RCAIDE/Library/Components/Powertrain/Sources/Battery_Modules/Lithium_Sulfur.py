# RCAIDE/Library/Components/Powertrain/Energy/Sources/Battery_Modules/Lithium_Sulfur.py
# 
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# package imports
from RCAIDE.Framework.Core import Units
from .Generic_Battery_Module import Generic_Battery_Module

# ----------------------------------------------------------------------------------------------------------------------
#  Lithium_Sulfur
# ----------------------------------------------------------------------------------------------------------------------  
class Lithium_Sulfur(Generic_Battery_Module):
    """
    Class for modeling lithium-sulfur battery characteristics and performance
    
    Attributes
    ----------
    cell.specific_energy : float
        Energy capacity per unit mass [J/kg] (default: 500 Wh/kg)
        
    cell.specific_power : float
        Maximum power output per unit mass [W/kg] (default: 1 kW/kg)
        
    cell.ragone.const_1 : float
        First constant in Ragone curve fit [kW/kg] (default: 245.848 kW/kg)
        
    cell.ragone.const_2 : float
        Second constant in Ragone curve fit [1/(Wh/kg)] (default: -0.00478 kg/Wh)
        
    cell.ragone.lower_bound : float
        Minimum specific energy on Ragone plot [Wh/kg] (default: 300 Wh/kg)
        
    cell.ragone.upper_bound : float
        Maximum specific energy on Ragone plot [Wh/kg] (default: 700 Wh/kg)

    Notes
    -----
    Lithium-sulfur batteries offer higher specific energy than conventional
    lithium-ion batteries but typically have lower power density. The Ragone
    plot parameters define the relationship between specific power and energy.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Generic_Battery_Module
        Base battery module class
    """
    
    def __defaults__(self):
        """
        Sets default values for lithium-sulfur battery attributes
        """   
        self.cell.specific_energy    = 500     *Units.Wh/Units.kg
        self.cell.specific_power     = 1       *Units.kW/Units.kg
        self.cell.ragone.const_1     = 245.848 *Units.kW/Units.kg
        self.cell.ragone.const_2     = -.00478 /(Units.Wh/Units.kg)
        self.cell.ragone.lower_bound = 300     *Units.Wh/Units.kg
        self.cell.ragone.upper_bound = 700     *Units.Wh/Units.kg
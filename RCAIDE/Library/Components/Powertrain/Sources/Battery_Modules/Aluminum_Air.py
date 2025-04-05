# RCAIDE/Library/Components/Powertrain/Energy/Sources/Battery_Modules/Aluminum_Air.py
# 
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

 # RCAIDE imports
from RCAIDE.Framework.Core import Units
from .Generic_Battery_Module    import Generic_Battery_Module

# ----------------------------------------------------------------------------------------------------------------------
#  Aluminum_Air
# ----------------------------------------------------------------------------------------------------------------------   
class Aluminum_Air(Generic_Battery_Module):
    """
    Class for modeling aluminum-air battery characteristics and performance
    
    Attributes
    ----------
    tag : str
        Identifier for the battery module (default: 'Aluminum Air')
        
    cell.specific_energy : float
        Energy capacity per unit mass [J/kg] (default: 1300 Wh/kg)
        
    cell.specific_power : float
        Maximum power output per unit mass [W/kg] (default: 0.2 kW/kg)
        
    mass_gain_factor : float
        Rate of mass increase during discharge [kg/Wh] (default: 0.000110145 kg/Wh)
        
    cell.water_mass_gain_factor : float
        Rate of water consumption [kg/Wh] (default: 0.000123913 kg/Wh)
        
    cell.aluminum_mass_factor : float
        Rate of aluminum consumption [kg/Wh] (default: 0.000123828 kg/Wh)
        
    cell.ragone.const_1 : float
        First constant in Ragone curve fit [kW/kg] (default: 0.8439 kW/kg)
        
    cell.ragone.const_2 : float
        Second constant in Ragone curve fit [1/(Wh/kg)] (default: -4.8647e-4)
        
    cell.ragone.lower_bound : float
        Minimum specific energy on Ragone plot [Wh/kg] (default: 1100 Wh/kg)
        
    cell.ragone.upper_bound : float
        Maximum specific energy on Ragone plot [Wh/kg] (default: 1600 Wh/kg)

    Notes
    -----
    Aluminum-air batteries are a type of metal-air battery that uses aluminum
    as the anode and ambient oxygen as the cathode. During discharge, the battery
    consumes aluminum and water while gaining mass from the reaction products.

    **Definitions**

    'Ragone Plot'
        Graph showing the relationship between specific power and specific energy,
        characterized by constants that define the performance envelope
        
    'Mass Gain Factor'
        Rate at which the battery gains mass during discharge due to the
        conversion of aluminum to aluminum hydroxide

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Generic_Battery_Module
        Base battery module class
    """
    
    def __defaults__(self):
        """
        Sets default values for aluminum-air battery attributes
        """
        self.tag                         = 'Aluminum Air'
        self.cell.specific_energy        = 1300.*Units.Wh/Units.kg    # convert to Joules/kg
        self.cell.specific_power         = 0.2*Units.kW/Units.kg      # convert to W/kg
        self.mass_gain_factor            = 0.000110145*Units.kg/Units.Wh
        self.cell.water_mass_gain_factor = 0.000123913*Units.kg/Units.Wh
        self.cell.aluminum_mass_factor   = 0.000123828*Units.kg/Units.Wh # aluminum consumed per energy
        self.cell.ragone.const_1         = 0.8439*Units.kW/Units.kg
        self.cell.ragone.const_2         = -4.8647e-004/(Units.Wh/Units.kg)
        self.cell.ragone.lower_bound     = 1100.*Units.Wh/Units.kg
        self.cell.ragone.upper_bound     = 1600.*Units.Wh/Units.kg
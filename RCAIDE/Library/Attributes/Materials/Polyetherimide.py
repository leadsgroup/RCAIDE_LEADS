# RCAIDE/Library/Attributes/Solids/Polyetherimide.py
# 
# 
#
# Created: Sep 2024 S. Shekar

#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------

from .Solid import Solid
from RCAIDE.Framework.Core import Units

#-------------------------------------------------------------------------------
# Polyetherimide for Reservoir Casing
#-------------------------------------------------------------------------------
class Polyetherimide(Solid):

    """
    A class representing polyetherimide (PEI) material properties, commonly used in reservoir casings 
    and other high-temperature thermoplastic applications.

    Attributes
    ----------
    conductivity : float
        Thermal conductivity in W/(m·K) (0.2)
    
    emissivity : float
        Surface emissivity, unitless (0.96)
    
    specific_heat : float
        Specific heat capacity in J/(kg·K) (1100)

    Notes
    -----
    This class implements thermal properties for polyetherimide, a high-performance 
    thermoplastic known for its heat resistance and dimensional stability. The material 
    is commonly used in aerospace and high-temperature applications.

    **Definitions**
    
    'Thermal Conductivity'
        The property of a material to conduct heat, measured in watts per meter-kelvin
    
    'Emissivity'
        The effectiveness of a material's surface in emitting energy as thermal radiation
    
    'Specific Heat'
        The amount of heat required to raise the temperature of 1 kg of the material by 1 Kelvin

    References
    ----------
    [1] MatWeb. (n.d.-b).  Overview of materials for Polyetherimide (PEI). MatWeb Material Property Data. https://www.matweb.com/search/DataSheet.aspx?MatGUID=65baf7a4f90c4a54a6ace03e16b1125b&amp%3Bckck=1&ckck=1 
    """
    def __defaults__(self):
        """Sets material properties at instantiation.

        Assumptions:
        None

        Source:
        https://www.matweb.com/search/DataSheet.aspx?MatGUID=65baf7a4f90c4a54a6ace03e16b1125b&amp%3bckck=1&ckck=1

        Inputs:
        N/A

        Outputs:
        N/A

        Properties Used:
        None
        """ 
        self.conductivity              = 0.2    # [W/m-K]
        self.emissivity                = 0.96   # [uniteless]
        self.specific_heat             = 1100  

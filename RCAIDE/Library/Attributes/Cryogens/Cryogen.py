# RCAIDE/Library/Attributes/Cryogens/Cryogens.py
# 
#
# Created:  Mar 2024, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 

from RCAIDE.Framework.Core import Data

# ---------------------------------------------------------------------------------------------------------------------- 
# Cryogen
# ----------------------------------------------------------------------------------------------------------------------  
class Cryogen(Data):
    """
    Base class for defining cryogenic fluid properties used in aerospace applications.

    This class serves as a template for specific cryogenic fluid implementations and provides
    structure for essential thermophysical properties required for cryogenic system analysis.

    Attributes
    ----------
    tag : str
        Identifier for the cryogenic fluid type, defaults to 'Cryogen'
    density : float
        Density of the cryogenic fluid [kg/m^3]
    specific_energy : float
        Specific energy of the cryogenic fluid [MJ/kg]
    energy_density : float
        Energy density of the cryogenic fluid [MJ/m^3]
    temperatures : Data
        Temperatures of the cryogenic fluid [K]

    Notes
    -----
    This base class provides a minimal framework for implementing specific cryogenic fluids.
    Derived classes should implement additional properties and methods specific to each
    cryogenic fluid type.

    **Major Assumptions**
        * Properties are initially undefined
        * Additional properties to be defined in derived classes

    See Also
    --------
    RCAIDE.Library.Components.Energy: Energy storage systems components
    RCAIDE.Library.Components.Thermal_Management: Thermal management systems components
    """

    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None
        
        Source:
            None
        """    
        self.tag                       = 'Cryogen'
        self.density                   = 0.0                       # kg/m^3
        self.specific_energy           = 0.0                       # MJ/kg
        self.energy_density            = 0.0                       # MJ/m^3
        self.temperatures              = Data()
        self.temperatures.freeze       = 0.0                       # K
        self.temperatures.boiling      = 0.0                       # K
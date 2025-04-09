# RCAIDE/Library/Attributes/Coolants/Coolant.py
# 
#
# Created:  Mar 2024, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 

from RCAIDE.Framework.Core import Data

# ---------------------------------------------------------------------------------------------------------------------- 
#  Class
# ----------------------------------------------------------------------------------------------------------------------  
class Coolant(Data):
    """
    Base class for defining liquid coolant properties used in thermal management systems.

    This class serves as a template for specific coolant implementations and provides
    structure for essential thermophysical properties required for heat transfer calculations.

    Attributes
    ----------
    tag : str
        Identifier for the coolant type 
    density : float
        Mass per unit volume of the coolant [kg/m³] 
    specific_heat_capacity : float
        Amount of heat required to raise temperature by one degree [J/kg·K]
    thermal_conductivity : float
        Ability to conduct heat [W/m·K]
    dynamic_viscosity : float
        Resistance to flow [Pa·s]
    temperatures : Data
        Container for temperature-dependent properties

    Notes
    -----
    This base class provides a framework for implementing specific coolants.
    All properties are initialized to 0.0 and should be properly defined in
    derived classes for specific coolants.

    Temperature dependence of properties can be implemented through the
    temperatures Data container for more sophisticated models.

    **Major Assumptions**
        * Properties are initially set as constants
        * All properties are for liquid phase
        * Standard atmospheric pressure conditions

    See Also
    --------
    RCAIDE.Library.Attributes.Coolants.Glycol_Water : Glycol-water mixture implementation
    """

    def __defaults__(self):
        """This sets the default values.

        Assumptions:
            None
    
        Source:
            None
        """
        self.tag                       = 'Coolant'
        self.density                   = 0.0                       # kg/m^3
        self.specific_heat_capacity    = 0.0                       # J/kg.K
        self.thermal_conductivity      = 0.0                       # W/m.K
        self.dynamic_viscosity         = 0.0                       # Pa.s
        self.temperatures              = Data()

# RCAIDE/Library/Components/Powertrain/Energy/Sources/Fuel_Tanks/Integral_Tank.py
# 
# Created:  Sep 2024, A. Molloy and M. Clarke 
# Modified: Aug 2025, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
import RCAIDE
from .Fuel_Tank  import Fuel_Tank 
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.append_fuel_tank_conditions import append_fuel_tank_conditions 
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Integral_Tank.compute_integral_tank_volume import *

# ----------------------------------------------------------------------------------------------------------------------
#  Fuel Tank
# ---------------------------------------------------------------------------------------------------------------------    
class Integral_Tank(Fuel_Tank):
    """Fuel tank compoment.
    
    Class for modeling integral fuel tank characteristics and behavior
    
    Attributes
    ----------
    tag : str
        Identifier for the fuel tank (default: 'wing_fuel_tank')
        
    fuel_selector_ratio : float
        Ratio of fuel flow allocation (default: 1.0)
        
    mass_properties.empty_mass : float
        Mass of empty tank structure [kg] (default: 0.0)
        
    secondary_fuel_flow : float
        Secondary fuel flow rate [kg/s] (default: 0.0)
        
    length : float
        Tank length [m] (default: 0.0)
        
    width : float
        Tank width [m] (default: 0.0)
        
    height : float
        Tank height [m] (default: 0.0)
        
    fuel : Component, optional
        Fuel type stored in tank (default: None)

    Notes
    -----
    The intergral tank is integrated into the aircraft wing structure,
    utilizing the space between wing spars and ribs for fuel storage.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank
        Base fuel tank class
    RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank
        Non-integral fuel tank
    """
    
    def __defaults__(self):
        """
        Sets default values for wing fuel tank attributes
        """          
        self.tag                         = 'integral_tank' 
        self.assigned_distributors       = []
        self.electrical_efficiency       = 1.0
        self.mechanical_efficiency       = 1.0
        self.hydraulic_efficiency        = 1.0
        self.thermal_efficiency          = 1.0

    def __init__ (self, compoment=None):
        """
        Initialize  
        """ 
        if compoment is not None:
            if isinstance(compoment, RCAIDE.Library.Components.Wings.Wing):  
                self.wing_tag  = compoment.tag
            if isinstance(compoment, RCAIDE.Library.Components.Fuselages.Fuselage):  
                self.fuselage_tag = compoment.tag
                
    def append_operating_conditions(self,segment):  
        """
        Append fuel tank operating conditions for a flight segment
        
        Parameters
        ----------
        segment : Segment
            Flight segment containing state conditions
        fuel_line : Component
            Connected fuel line component
        """
        append_fuel_tank_conditions(self,segment)  
        return       

    def compute_volume(self, wings, fuselages):
        """
        Compute the internal volume of an integral fuel tank based on its location.

        Calculates the fuel tank volume by determining whether the tank is integrated
        into a wing or fuselage structure and computing the appropriate volume using
        geometric properties of the host component.

        Parameters
        ----------
        wings : dict
            Dictionary containing wing components indexed by wing tag
        fuselages : dict
            Dictionary containing fuselage components indexed by fuselage tag

        Returns
        -------
        volume : float
            Internal volume of the fuel tank [m³]

        Notes
        -----
        The function determines the tank location using the wing_tag or fuselage_tag
        attributes set during initialization. If wing_tag is set, it computes wing
        integral tank volume. If fuselage_tag is set, it computes fuselage integral
        tank volume. See compute_wing_integral_tank_volume and compute_fuselage_integral_tank_fuel_volume
        for more details on the volume calculations.

        **Major Assumptions**
            * Tank is fully integrated into the host component structure
            * Wing tanks use the space between front and rear spars
            * Fuselage tanks use the space between specified fuselage segments
            * Volume calculations assume truncated prism geometry for wing tanks
            * Volume calculations assume truncated cone geometry for fuselage tanks

        **Definitions**

        'Integral Tank'
            Fuel tank that is structurally integrated into the aircraft's primary
            structure (wing or fuselage) rather than being a separate component

        'Wing Integral Tank'
            Fuel tank that utilizes the space between wing spars and ribs for
            fuel storage, typically located in the wing box

        'Fuselage Integral Tank'
            Fuel tank that utilizes the space within fuselage segments for fuel
            storage, typically located between structural frames

        See Also
        --------
        RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Integral_Tank.compute_integral_tank_volume
            Equations for computing integral tank volume for both wing and fuselage tanks
        """
        if self.wing_tag is not None:
            wing = wings[self.wing_tag]  
            compute_wing_integral_tank_volume(self, wing)
        elif self.fuselage_tag is not None: 
            fuselage = fuselages[self.fuselage_tag]  
            compute_fuselage_integral_tank_fuel_volume(self, fuselage)
        return
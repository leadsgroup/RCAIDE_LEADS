# RCAIDE/Library/Components/Powertrain/Energy/Sources/Fuel_Tanks/Liquid_Hydrogen_Tank.py
# 
# Created: Aug 2025, S. Shekar
#
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from .Non_Integral_Tank  import Non_Integral_Tank 
from RCAIDE.Framework.Core import Units

from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank.compute_non_integral_tank_volume       import *
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank.compute_liquid_hydrogen_tank_volume import compute_liquid_hydrogen_tank_volume

# ----------------------------------------------------------------------------------------------------------------------
#  Liquid Hydrogen Tank
# ---------------------------------------------------------------------------------------------------------------------    
class Liquid_Hydrogen_Tank(Non_Integral_Tank):
    """
    A class representing a non-integral liquid hydrogen (LH₂) fuel tank.  

    Attributes
    ----------
    tag : str
        Identifier for the fuel tank (default: 'Liquid_Hydrogen_Tank').  
    material : Solid
        Primary tank material (default: None).  
    insulation_material : Solid
        Insulation material used to reduce heat leak (default: None).  
    design_inlet_temperature : float
        Nominal inlet temperature of liquid hydrogen [K] (default: 15 K).  
    design_altitude : float
        Design altitude for thermal/structural performance calculations [m].  
    design_isa_deviation : float
        ISA deviation at design altitude [°C] (default: 0).  
    acceptable_heat_leak : float
        Maximum allowable heat leak into the tank [W] (default: 20).  
    ullage_volume_fraction : float
        Fraction of total tank volume reserved for ullage (default: 0.07).  
    design_external_pressure : float
        External design pressure [Pa] (default: 0).  

    Notes
    -----
    The liquid hydrogen tank is modeled as a non-integral tank, meaning it is 
    not structurally part of the wing or fuselage but rather a dedicated 
    cryogenic vessel. The class supports thermal and structural analyses to 
    capture boil-off, heat leak, and load response.  

    **Major Assumptions**
        * Tank shape is cylindrical with optional hemispherical end caps.  
        * Ullage fraction accounts for vapor volume above liquid hydrogen.  
        * Structural analysis assumes isotropic tank materials.  
        * Thermal analysis assumes steady-state conductive and radiative heat leak.  

    **Definitions**

    'Non-Integral Tank'
        A fuel tank that is not part of the primary aircraft structure and is 
        instead an independent container.  

    'Ullage'
        The empty space in a tank above the liquid fuel, typically filled with 
        vapor, to allow for thermal expansion and slosh control.  

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank.Integral_Tank  
        Class for integral fuel tanks within wing or fuselage structure.  
    RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank.compute_non_integral_tank_volume  
        Equations for computing volume of non-integral tanks.  
    RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank.compute_structural_performance  
        Structural solver for cryogenic hydrogen tanks.  
    RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank.compute_thermal_performance  
        Thermal solver for cryogenic hydrogen tanks.  
    """

    def __defaults__(self):
        """
        Set default values for liquid hydrogen tank attributes.  

        Parameters
        ----------
        None  

        Returns
        -------
        None  
        """
        self.tag                      = 'Liquid_Hydrogen_Tank'
        self.fuel                     = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
        self.material                 = None
        self.insulation_material      = None
        self.design_inlet_temperature = 20
        self.design_altitiude         = 0
        self.acceptable_heat_leak     = 20
        self.design_altitude          = 30000 * Units.ft
        self.design_isa_deviation     = 0
        self.ullage_volume_fraction   = 0.07
        self.design_external_pressure = 0 

    def compute_volume(self, wings, fuselages,fuel_tanks):
        """
        Compute the internal volume of the liquid hydrogen tank.  

        Determines the non-integral tank volume based on whether it is 
        attached to a wing or configured as an aft BWB tank, and runs 
        corresponding structural and thermal solvers.  

        Parameters
        ----------
        wings : dict
            Dictionary containing wing components indexed by wing tag.  
        fuselages : dict
            Dictionary containing fuselage components indexed by fuselage tag.  

        Returns
        -------
        None  

        Notes
        -----
        * If `wing_tag` is set, computes wing-mounted non-integral tank volume.  
        * If `bwb_aft_tank` is True, computes blended wing body aft tank volume.  
        * After volume computation, structural and thermal solvers are executed.  

        See Also
        --------
        RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank.compute_non_integral_tank_volume  
            Equations for computing non-integral tank volumes.  
        RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank.compute_structural_performance  
            Structural solver for cryogenic hydrogen tanks.  
        RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank.compute_thermal_performance  
            Thermal solver for cryogenic hydrogen tanks.  
        """
        if self.geometry_type == 'cylindrical':
            if self.wing_tag != None and self.bwb_aft_tank is False:
                wing = wings[self.wing_tag]  
                compute_wing_non_integral_tank_volume(self, wing,fuel_tanks)
                if hasattr(fuel_tanks,self.tag):
                    compute_liquid_hydrogen_tank_volume(self)
                  
            else:
                if self.bwb_aft_tank == True:
                    if self.wing_tag != None:
                        wing = wings[self.wing_tag]  
                        compute_bwb_aft_tank_volume(self, wing)
                        compute_liquid_hydrogen_tank_volume(self)
        return

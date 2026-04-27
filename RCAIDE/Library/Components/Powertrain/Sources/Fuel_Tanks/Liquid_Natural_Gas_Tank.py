# RCAIDE/Library/Components/Powertrain/Energy/Sources/Fuel_Tanks/Liquid_Natural_Gas_Tank.py
# 
# Created: April 2026, S. Shekar, S. Sharma
#
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from .Non_Integral_Tank  import Non_Integral_Tank 
import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Integral_Tank.compute_integral_tank_volume               import *
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank.compute_non_integral_tank_volume       import *
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.compute_cryogenic_cylindrical_tank_volume import compute_cryogenic_cylindrical_tank_volume
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity                                                      import compute_cylinder_center_of_gravity
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia                                                      import compute_rounded_end_cylinder_moment_of_inertia, compute_cuboid_moment_of_inertia

# ----------------------------------------------------------------------------------------------------------------------
#  Liquid Natural Gas Tank
# ---------------------------------------------------------------------------------------------------------------------    
class Liquid_Natural_Gas_Tank(Non_Integral_Tank):
    """
    A class representing a non-integral liquid natural gas (LNG) fuel tank.  

    Attributes
    ----------
    tag : str
        Identifier for the fuel tank (default: 'Liquid_Natural_Gas_Tank').  
    material : Solid
        Primary tank material (default: None).  
    insulation_material : Solid
        Insulation material used to reduce heat leak (default: None).  
    design_inlet_temperature : float
        Nominal inlet temperature of liquid natural gas [K] (default: 100 K).  
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
    The liquid natural gas tank is modeled as a non-integral tank, meaning it is 
    not structurally part of the wing or fuselage but rather a dedicated 
    cryogenic vessel. The class supports thermal and structural analyses to 
    capture boil-off, heat leak, and load response.  

    **Major Assumptions**
        * Tank shape is cylindrical with optional hemispherical end caps.  
        * Ullage fraction accounts for vapor volume above liquid natural gas.  
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
    RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Liquid_Natural_Gas_Tank.compute_structural_performance  
        Structural solver for cryogenic natural gas tanks.  
    RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Liquid_Natural_Gas_Tank.compute_thermal_performance  
        Thermal solver for cryogenic natural gas tanks.  
    """

    def __defaults__(self):
        """
        Set default values for liquid natural gas tank attributes.  

        Parameters
        ----------
        None  

        Returns
        -------
        None  
        """
        self.tag                           = 'Liquid_Natural_Gas_Tank'
        self.fuel                          = RCAIDE.Library.Attributes.Propellants.Liquid_Natural_Gas()
        self.material                      = None
        self.insulation_material           = None
        self.geometry_type                 = 'cylindrical' # conformal
        self.design_inlet_temperature      = 100
        self.design_altitiude              = 0
        self.acceptable_heat_leak          = 20
        self.acceptable_total_heat_leak    = 2000
        self.design_altitude               = 30000 * Units.ft
        self.design_isa_deviation          = 0
        self.ullage_volume_fraction        = 0.07
        self.design_external_pressure      = 0 
        self.tank_accesories_weight_factor = 1.5
        self.safety_factor                 = 1.6 # structural factor of safety
        self.pressure_factor               = 5   # internal pressure multiplier for sizing

    def compute_volume(self, wings, fuselages,fuel_tanks):
        """
        Compute the internal volume of the liquid natural gas tank.  

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
        RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Liquid_Natural_Gas_Tank.compute_structural_performance  
            Structural solver for cryogenic natural gas tanks.  
        RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Liquid_Natural_Gas_Tank.compute_thermal_performance  
            Thermal solver for cryogenic natural gas tanks.  
        """
        if self.geometry_type == 'cylindrical':
            if self.wing_tag != None and self.bwb_aft_tank is False:
                wing = wings[self.wing_tag]  
                compute_wing_non_integral_tank_volume(self, wing,fuel_tanks)
                if hasattr(fuel_tanks,self.tag):
                    compute_cryogenic_cylindrical_tank_volume(self,fuel_tanks)
                  
            else:
                if self.bwb_aft_tank == True:
                    if self.wing_tag != None:
                        wing = wings[self.wing_tag]  
                        compute_bwb_aft_tank_volume(self, wing,fuel_tanks)
                        if hasattr(fuel_tanks,self.tag):
                            compute_cryogenic_cylindrical_tank_volume(self,fuel_tanks)        
        
        return
  
    def compute_moments_of_inertia(self,vehicle,center_of_gravity=[[0, 0, 0]]): 
        """
        Computes the moment of inertia tensor for a fuel tank.

        Parameters
        ----------
        center_of_gravity : list, optional
            Reference point coordinates for moment calculation, defaults to [[0, 0, 0]]

        Returns
        -------
        I : ndarray
            3x3 moment of inertia tensor in kg*m^2
 
        """
        
        outer_length = self.lengths.external
        outer_radius = self.diameters.external/2
        inner_length = self.inner_structure.inner_length 
         
        if  self.geometry_type == 'cylindrical':
            inner_radius = self.inner_structure.inner_diameter/2
            _, _ = compute_rounded_end_cylinder_moment_of_inertia(self, outer_length,outer_radius,inner_length=inner_length, inner_radius=inner_radius, center_of_gravity=center_of_gravity, fuel_tank=True) 
        
        return
    

    def compute_center_of_gravity(self,vehicle): 
        """
        Computes the center of gravity for a fuel tank.

        Parameters
        ----------
        center_of_gravity : list, optional
            Reference point coordinates for moment calculation, defaults to [[0, 0, 0]]

        Returns
        -------
        I : ndarray
            3x3 moment of inertia tensor in kg*m^2 
        """
        if self.geometry_type == 'cylindrical':        
            length = self.lengths.external +  self.diameters.external
            _      = compute_cylinder_center_of_gravity(self, length )
        return
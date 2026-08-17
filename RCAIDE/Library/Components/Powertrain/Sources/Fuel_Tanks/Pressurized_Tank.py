# RCAIDE/Library/Components/Powertrain/Sources/Fuel_Tanks/Pressurized_Tank.py
#
# Created: Jul 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from .Non_Integral_Tank import Non_Integral_Tank
import RCAIDE
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank.compute_wing_non_integral_tank_volume            import compute_wing_non_integral_tank_volume
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank.compute_wing_transverse_non_integral_tank_volume import compute_wing_transverse_non_integral_tank_volume
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank.compute_rounded_end_cylindrical_tank_volume      import compute_rounded_end_cylindrical_tank_volume
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank.compute_prismatic_tank_volume                    import compute_prismatic_tank_volume
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Pressurized_Tank.compute_pressurized_cylindrical_tank_volume       import compute_pressurized_cylindrical_tank_volume
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity  import compute_cylinder_center_of_gravity
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia  import compute_rounded_end_cylinder_moment_of_inertia, compute_cuboid_moment_of_inertia

# ----------------------------------------------------------------------------------------------------------------------
#  Pressurized Tank
# ----------------------------------------------------------------------------------------------------------------------
class Pressurized_Tank(Non_Integral_Tank):
    """
    A non-integral pressurized fuel tank for gaseous or liquid propellants
    stored above atmospheric pressure (e.g. compressed natural gas, gaseous
    hydrogen, propane, or other pressurized fuels).

    Unlike ``Cryogenic_Tank``, this class has no insulation layer.  The outer
    envelope is the pressure-vessel wall itself.  Wall thickness is sized by
    thick-walled cylinder theory (Lamé equations, von Mises criterion).

    The user must set ``fuel``, ``design_pressure``, and external dimensions
    (``diameters.external``, ``lengths.external`` for standalone cylindrical
    tanks, or attach to a wing via ``wing_tag``).

    Attributes
    ----------
    geometry_type : str
        Tank shape: ``'cylindrical'`` or ``'prismatic'`` (default: ``'cylindrical'``).
    design_pressure : float or None
        Operating internal gauge pressure [Pa] (default: None).
    design_external_pressure : float
        External pressure for structural sizing [Pa] (default: 0).
    design_altitude : float or None
        Design altitude used to compute differential pressure if needed [m]
        (default: None).
    design_isa_deviation : float
        ISA temperature offset at design altitude [K] (default: 0).
    ullage_volume_fraction : float
        Fraction of internal volume reserved for ullage / gas phase
        (default: 0.05).
    safety_factor : float
        Structural factor of safety (default: 1.5).
    tank_accesories_weight_factor : float
        Multiplier applied to structural shell mass to account for fittings,
        valves, and mounting hardware (default: 1.3).
    """

    def __defaults__(self):
        self.tag                           = 'pressurized_tank'
        self.geometry_type                 = 'cylindrical'
        self.design_pressure               = None
        self.design_external_pressure      = 0
        self.design_altitude               = None
        self.design_isa_deviation          = 0
        self.ullage_volume_fraction        = 0.05
        self.inner_structure.material      = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
        self.tank_accesories_weight_factor = 1.3
        self.safety_factor                 = 1.5

    def compute_volume(self, wings, fuselages, fuel_tanks):
        """Computes net fuel volume and structural wall sizing.

        Cylindrical
        -----------
        - Wing spanwise:   outer envelope from ``compute_wing_non_integral_tank_volume``
        - Wing transverse: outer envelope from ``compute_wing_transverse_non_integral_tank_volume``
        - Standalone:      outer envelope from ``compute_rounded_end_cylindrical_tank_volume``
                           using user-set ``lengths.external`` and ``diameters.external``
        - Then: ``compute_pressurized_cylindrical_tank_volume`` sizes the structural
                wall inward and computes net fuel volume.

        Prismatic
        ---------
        - Uses ``compute_prismatic_tank_volume`` directly with the user-set
          ``wall_thickness`` (no automatic structural sizing for prismatic tanks).
        """
        if self.geometry_type == 'cylindrical':
            if self.wing_tag is not None and self.transverse_tank is False:
                compute_wing_non_integral_tank_volume(self, wings[self.wing_tag], fuel_tanks)
            elif self.wing_tag is not None and self.transverse_tank is True:
                compute_wing_transverse_non_integral_tank_volume(self, wings[self.wing_tag], fuel_tanks)
            else:
                compute_rounded_end_cylindrical_tank_volume(self)
            if hasattr(fuel_tanks, self.tag):
                compute_pressurized_cylindrical_tank_volume(self, fuel_tanks)
        elif self.geometry_type == 'prismatic':
            compute_prismatic_tank_volume(self)
        else:
            raise NotImplementedError(
                f"geometry_type '{self.geometry_type}' is not supported for Pressurized_Tank.")
        return

    def compute_moments_of_inertia(self, vehicle, center_of_gravity=[[0, 0, 0]]):
        if self.geometry_type == 'cylindrical':
            _, _ = compute_rounded_end_cylinder_moment_of_inertia(
                self,
                self.lengths.external,
                self.diameters.external / 2,
                inner_length=self.inner_structure.lengths.internal,
                inner_radius=self.inner_structure.diameters.internal / 2,
                center_of_gravity=center_of_gravity,
                fuel_tank=True)
        elif self.geometry_type == 'prismatic':
            t = self.wall_thickness
            _, _ = compute_cuboid_moment_of_inertia(
                self,
                outer_length=self.lengths.external,
                outer_width=self.widths.external,
                outer_height=self.heights.external,
                inner_length=self.lengths.external - 2 * t,
                inner_width=self.widths.external   - 2 * t,
                inner_height=self.heights.external - 2 * t,
                center_of_gravity=center_of_gravity,
                fuel_tank=True)
        return

    def compute_center_of_gravity(self, vehicle):
        if self.geometry_type == 'cylindrical':
            length = self.lengths.external + self.diameters.external
            _ = compute_cylinder_center_of_gravity(self, length)
        return

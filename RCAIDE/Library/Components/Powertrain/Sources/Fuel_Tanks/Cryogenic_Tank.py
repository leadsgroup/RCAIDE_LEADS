# RCAIDE/Library/Components/Powertrain/Sources/Fuel_Tanks/Cryogenic_Tank.py
#
# Created:  Jun 2026, M. Clarke
#   (consolidated from Liquid_Hydrogen_Tank.py and Liquid_Natural_Gas_Tank.py)

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from .Non_Integral_Tank  import Non_Integral_Tank
import RCAIDE
from RCAIDE.Framework.Core import Units, Data
from RCAIDE.Library.Components import Component
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Integral_Tank.compute_wing_transverse_integral_tank_volume          import compute_wing_transverse_integral_tank_volume
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Integral_Tank.compute_wing_integral_tank_volume                     import compute_wing_integral_tank_volume
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank.compute_wing_non_integral_tank_volume             import compute_wing_non_integral_tank_volume
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank.compute_prismatic_tank_volume                     import compute_prismatic_tank_volume
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank.compute_rounded_end_cylindrical_tank_volume       import compute_rounded_end_cylindrical_tank_volume
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank.compute_wing_transverse_non_integral_tank_volume  import compute_wing_transverse_non_integral_tank_volume
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.compute_cryogenic_cylindrical_tank_volume            import compute_cryogenic_cylindrical_tank_volume
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.compute_cryogenic_conformal_tank_volume         import compute_cryogenic_conformal_tank_volume
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity  import compute_cylinder_center_of_gravity
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia  import compute_rounded_end_cylinder_moment_of_inertia, compute_cuboid_moment_of_inertia

# ----------------------------------------------------------------------------------------------------------------------
#  Cryogenic Tank
# ----------------------------------------------------------------------------------------------------------------------
class Cryogenic_Tank(Non_Integral_Tank):
    """
    A non-integral cryogenic fuel tank for liquid hydrogen, liquid natural gas,
    or other cryogenic propellants.

    The user must set ``fuel`` and ``design_inlet_temperature`` for the specific
    cryogenic propellant being stored.

    Attributes
    ----------
    geometry_type : str
        Tank shape: 'cylindrical', 'conformal', or 'prismatic' (default: 'cylindrical').
    design_inlet_temperature : float or None
        Nominal inlet temperature of the cryogen [K] (default: None).
    design_altitude : float
        Design altitude for thermal/structural sizing [m].
    design_heat_flux : float
        Maximum allowable heat leak per unit area [W/m²] (default: 20).
    design_total_heat_transfer : float
        Maximum allowable total heat leak [W] (default: 2000).
    ullage_volume_fraction : float
        Fraction of internal volume reserved for ullage (default: 0.07).
    safety_factor : float
        Structural factor of safety (default: 1.6).
    pressure_factor : float
        Internal pressure multiplier for sizing (default: 5).
    """

    def __defaults__(self):
        self.tag                            = 'cryogenic_tank'
        self.geometry_type                  = 'cylindrical'
        self.design_inlet_temperature       = None
        self.design_altitude                = None
        self.design_isa_deviation           = 0
        self.design_heat_flux               = None
        self.design_total_heat_transfer     = None
        self.ullage_volume_fraction         = None
        self.inner_structure.material       = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
        self.insulation.material            = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
        self.design_external_pressure       = 0
        self.tank_accesories_weight_factor  = 1.5
        self.safety_factor                  = 1.6
        self.pressure_factor                = 5

    def compute_volume(self, wings, fuselages, fuel_tanks):
        """Computes the net fuel volume and cryogenic structure/insulation sizing.

        The method operates in two stages. First, the outer envelope is determined
        from the wing geometry (or from user-supplied dimensions for standalone
        tanks). Second, cryogenic-specific sizing works inward from that envelope
        to compute insulation thickness, structural wall thickness, and the
        resulting net fuel volume.

        Cylindrical
        -----------
        - Wing spanwise:   outer envelope from ``compute_wing_non_integral_tank_volume``
        - Wing transverse: outer envelope from ``compute_wing_transverse_non_integral_tank_volume``
        - Standalone:      outer envelope from ``compute_rounded_end_cylindrical_tank_volume``
                           using user-set ``lengths.external`` and ``diameters.external``
        - Then:            ``compute_cryogenic_cylindrical_tank_volume`` sizes
                           insulation and structure inward

        Conformal
        ---------
        - Wing spanwise:   outer envelope from ``compute_wing_integral_tank_volume``
        - Wing transverse: outer envelope from ``compute_wing_transverse_integral_tank_volume``
        - Then:            ``compute_cryogenic_conformal_tank_volume`` sizes
                           insulation and structure inward

        Prismatic
        ---------
        - Uses user-set ``lengths.external``, ``widths.external``, ``heights.external``
        - Then: ``compute_cryogenic_conformal_tank_volume`` sizes
                insulation and structure inward
        """
        if self.geometry_type == 'cylindrical':
            if self.wing_tag is not None and self.transverse_tank is False:
                compute_wing_non_integral_tank_volume(self, wings[self.wing_tag], fuel_tanks)
            elif self.wing_tag is not None and self.transverse_tank is True:
                compute_wing_transverse_non_integral_tank_volume(self, wings[self.wing_tag], fuel_tanks)
            else:
                compute_rounded_end_cylindrical_tank_volume(self)
            if hasattr(fuel_tanks, self.tag):
                compute_cryogenic_cylindrical_tank_volume(self, fuel_tanks)
        elif self.geometry_type == 'conformal':
            if self.wing_tag is not None and self.transverse_tank is False:
                compute_wing_integral_tank_volume(self, wings[self.wing_tag])
            elif self.wing_tag is not None and self.transverse_tank is True:
                compute_wing_transverse_integral_tank_volume(self, wings[self.wing_tag], fuel_tanks)
            if hasattr(fuel_tanks, self.tag):
                compute_cryogenic_conformal_tank_volume(self, fuel_tanks)
        elif self.geometry_type == 'prismatic':
            compute_cryogenic_conformal_tank_volume(self, fuel_tanks)
        else:
            raise NotImplementedError
        return

    def compute_moments_of_inertia(self, vehicle, center_of_gravity=[[0, 0, 0]]):
        outer_length = self.lengths.external
        outer_radius = self.diameters.external / 2
        inner_length = self.inner_structure.lengths.internal

        if self.geometry_type == 'cylindrical':
            inner_radius = self.inner_structure.diameters.internal / 2
            _, _ = compute_rounded_end_cylinder_moment_of_inertia(
                self, outer_length, outer_radius,
                inner_length=inner_length, inner_radius=inner_radius,
                center_of_gravity=center_of_gravity, fuel_tank=True)
        elif self.geometry_type == 'conformal' and self.transverse_tank:
            pass
        elif self.geometry_type == 'conformal' and self.transverse_tank is False:
            thickness = self.inner_structure.thickness + self.insulation_thickness
            _, _ = compute_cuboid_moment_of_inertia(
                self,
                outer_length=self.lengths.external,
                outer_width=self.widths.external,
                outer_height=self.heights.external,
                inner_length=self.lengths.external - 2 * thickness,
                inner_width=self.widths.external - 2 * thickness,
                inner_height=self.heights.external - 2 * thickness,
                center_of_gravity=center_of_gravity, fuel_tank=True)
        return

    def compute_center_of_gravity(self, vehicle):
        if self.geometry_type == 'cylindrical':
            length = self.lengths.external + self.diameters.external
            _ = compute_cylinder_center_of_gravity(self, length)
        return

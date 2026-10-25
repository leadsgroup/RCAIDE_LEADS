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
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.compute_cryogenic_tank_performance               import compute_cryogenic_tank_performance
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.append_cryogenic_tank_unknown_and_residual       import append_cryogenic_tank_unknown_and_residual
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.append_cryogenic_tank_conditions                 import append_cryogenic_tank_conditions, append_cryogenic_tank_segment_conditions
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.append_fuel_tank_conditions                                     import append_fuel_tank_conditions, append_fuel_tank_segment_conditions
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.compute_fuel_tank_performance                                   import compute_fuel_tank_performance
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity  import compute_cylinder_center_of_gravity
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia  import compute_rounded_end_cylinder_moment_of_inertia, compute_cuboid_moment_of_inertia

# ----------------------------------------------------------------------------------------------------------------------
#  Cryogenic Tank
# ----------------------------------------------------------------------------------------------------------------------
class Cryogenic_Tank(Non_Integral_Tank):
    """
    Non-integral tank for liquid hydrogen, liquid natural gas or other cryogenic propellants. The user sets
    ``fuel`` and ``design_inlet_temperature``.

    Attributes
    ----------
    geometry_type : str
        'cylindrical' (default), 'conformal' or 'prismatic'.
    design_inlet_temperature : float
        Storage temperature of the liquid [K].
    design_altitude : float
        Altitude for thermal and structural sizing [m].
    design_heat_flux : float
        Allowable heat leak per unit area used to size the insulation [W/m^2].
    design_total_heat_transfer : float
        Allowable total heat leak [W].
    ullage_volume_fraction : float
        Fraction of the internal volume left as ullage at full load [-].
    safety_factor : float
        Structural factor of safety on yield, sigma_vm <= sigma_y / safety_factor (default: 1.6).
    boil_off_model : str
        'quasi_steady' (default) solves the two-phase tank model (ullage and liquid mass and temperature)
        as an ODE within each tank evaluation. 'none' uses the plain fuel tank: fuel leaves only by engine
        draw, with no boil-off.
    heater_power_margin : float
        Multiple of the takeoff pressurant demand the heater is sized to when ``Heater.design_power``
        is not set (default: 1.5).
    heater_direct_boiloff_fraction : float
        Fraction of heater power that boils liquid directly; the rest warms the liquid (default: 0.1).
        This is eta_h of Adler & Martins (2025), Eqs. 13, 17 and 24, tuned on LH2 ground tests (Sec. IV.B).
        It has not been validated for LNG.
    pressure_margin : float
        Pressure above P_sat(design_inlet_temperature) that sets ``design_pressure``, the maximum operating
        pressure used for wall sizing and as the vent pressure [Pa] (default: 2 bar).
    minimum_operating_pressure : float
        Pressure below which the heater adds boil-off to keep tank pressure from falling [Pa]. Set to
        P_sat(design_inlet_temperature) during sizing when not given. Between this pressure and
        ``design_pressure`` the tank self-pressurizes from heat leak, following Adler & Martins (2025),
        Sec. I: the heater prevents pressure from decreasing and the tank vents at its limit.
    pressure_regulation_time_constant : float
        Response time of the heater and vent regulation [s] (default: 400). Within the range of the heater
        response constant C_h tuned by Adler & Martins (2025), Sec. IV.B.
    vent_time_constant : float or None
        Response time of the relief vent [s] (default: 10). Assumed value approximating a relief valve that holds
        design_pressure; None uses pressure_regulation_time_constant.
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
        self.boil_off_model                 = 'quasi_steady'
        self.heater_direct_boiloff_fraction = 0.1
        self.heater_power_margin            = 1.5
        self.pressure_margin                = 2 * Units.bar
        self.design_pressure                = None
        self.minimum_operating_pressure     = None
        self.pressure_regulation_time_constant = 400.0
        self.vent_time_constant                = 10.0

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
            thickness = self.inner_structure.thickness + self.insulation.thickness
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

    def append_operating_conditions(self, segment):
        if self.boil_off_model == 'quasi_steady':
            append_cryogenic_tank_conditions(self, segment)
        else:
            # Exactly the 'none' behavior: no boil-off physics at all
            # (boil_off_flow_rate stays at append_fuel_tank_conditions' own
            # default of 0), not a simplified estimate.
            append_fuel_tank_conditions(self, segment)
        return

    def append_segment_conditions(self, segment):
        append_fuel_tank_segment_conditions(self, segment)
        if self.boil_off_model == 'quasi_steady':
            append_cryogenic_tank_segment_conditions(self, segment)
        return

    def append_unknowns_and_residuals(self, segment):
        if self.boil_off_model == 'quasi_steady':
            append_cryogenic_tank_unknown_and_residual(self, segment)
        return

    def compute_performance(self, state, network):
        if self.boil_off_model == 'quasi_steady':
            return compute_cryogenic_tank_performance(self, state, network)
        else:
            return compute_fuel_tank_performance(self, state, network)

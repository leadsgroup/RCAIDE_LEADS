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
        Structural factor of safety (default: 1.6). Sole structural safety
        margin on the yield criterion (``sigma_vm <= sigma_y/safety_factor``);
        no separate burst/proof pressure factor is applied on top of it -- an
        earlier fixed ``pressure_factor`` (~5x) multiplier stacked
        multiplicatively on the same yield check as this factor, was never
        itself sampled/varied, and had no distinct ultimate-strength criterion
        to justify as a separate margin, so it was removed as redundant.
    boil_off_model : str
        'quasi_steady' (default) solves the full 6-state implicit two-phase
        boil-off model (ullage/liquid mass, temperature, volume) as
        mission-level unknowns -- physically detailed (pressure/temperature
        evolution, active-heater/vent response, fill-level-dependent wetted
        area), but adds real coupling to the shared mission-level solve; a
        vehicle with several tanks can strain solver robustness. 'none'
        falls back to the plain (non-cryogenic) fuel tank behavior -- no
        boil-off physics at all, matching ``develop``/``network_refactor``:
        fuel burns down at the engine's offtake rate only, no unknowns, no
        per-point coupling. Named to match the quasi-steady two-phase
        thermodynamic model of the underlying report's Section V.B, rather
        than as a bare "detailed/not detailed" toggle, so that a future
        additional model doesn't have to be shoehorned into a boolean.
    heater_direct_boiloff_fraction : float
        Fraction of the pressure-builder heater's power that goes directly to
        flash-boiling liquid at the interface, with the remainder instead
        raising the bulk liquid temperature (default: 0.1). Matches Adler &
        Martins (2025) eta_h, Eq. 13/17/24, tuned against real LH2 ground-test
        data (Section 4.2: "the vast majority of the heat from the heater
        goes into the liquid... we select a value of 0.1"). Validated for
        LH2 only; unvalidated for LNG, used here as the best available
        anchor.
    pressure_margin : float
        Operating ullage pressure margin above the saturation pressure at
        ``design_inlet_temperature`` [Pa] (default: 2 bar = 2e5 Pa). Sets
        ``design_pressure = P_rated = P_sat(design_inlet_temperature) +
        pressure_margin``, matching the AST paper's Eq. 4 (sampled 2-6 bar in
        their Table 5). This is both the in-flight runtime target the
        detailed boil-off model regulates the ullage to, AND the internal
        design pressure (``P_internal = P_rated``) fed directly into the
        structural wall-thickness sizing -- the only additional structural
        margin beyond the physical rated pressure is ``safety_factor``.
    pressure_regulation_time_constant : float
        Characteristic response time [s] of the tank's heater/vent regulation
        system correcting a ullage pressure deviation from ``design_pressure``
        (default: 60 s). Drives the regulation flow explicitly, m_dot_reg =
        V_g/(R_specific*T_g*tau) * (design_pressure - P) -- a finite-gain
        feedback law, not an exact/instantaneous constraint, matching the
        finite authority of a real heater (c.f. Adler & Martins (2025)'s own
        heater thermal-inertia constant C_h, Eq. 33-34) rather than assuming
        infinite regulation authority. Smaller values regulate pressure more
        tightly at the cost of a stiffer system; not yet validated against
        real hardware response times.
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
        self.pressure_margin                = 2 * Units.bar
        self.design_pressure                = None  # set from pressure_margin during design-time sizing
        self.pressure_regulation_time_constant = 60.0

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

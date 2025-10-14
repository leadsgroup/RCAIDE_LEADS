# RCAIDE/Library/Components/Propulsors/Converters/Motor.py
#
# Created:  Mar 2024, M. Clarke
# Modified: Sep 2025, M. Guidotti (fixes & cleanup by ChatGPT)

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from .Converter import Converter
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Methods.Powertrain.Converters.Motor.append_motor_conditions import (
    append_motor_conditions,
)


# ----------------------------------------------------------------------------------------------------------------------
#  Motor
# ----------------------------------------------------------------------------------------------------------------------
class Motor(Converter):
    """
    Electric motor component model for electric propulsion systems (DC or PMSM).

    Attributes
    ----------
    tag : str
        Identifier for the motor. Default is 'motor'.

    type : str
        Motor type ('DC' or 'PMSM'). Default is 'DC'.

    resistance : float
        Internal electrical resistance of the motor [Ω].

    no_load_current : float
        Current drawn by the motor with no mechanical load [A].

    speed_constant : float
        Motor speed constant (Kv). For DC usage here assumed in [rpm/V] unless
        otherwise documented by a downstream method.

    efficiency : float
        Overall motor efficiency [-].

    gearbox : Data
        Container for gearbox properties.

    gearbox.gear_ratio : float
        Ratio of output shaft speed to motor speed [-].

    power_split_ratio : float
        Ratio of power distribution when driving multiple loads [-].

    design_angular_velocity : float
        Design point angular velocity [rad/s].

    design_torque : float
        Design point torque output [N·m].

    design_current : float
        Design point current [A].

    inverse_calculation : bool
        Flag for inverse sizing/operation flows.

    interpolated_func : callable or None
        Optional motor performance interpolation function.

    Notes
    -----
    This model is used within RCAIDE powertrain workflows and supports:
      * Electrical losses (resistive, no-load current)
      * Gearbox effects
      * Speed–torque relationships
      * Power splitting across loads

    Definitions
    -----------
    'Kv' (speed constant)
        Relates voltage to (approx.) unloaded motor speed.

    'No-load current'
        Current drawn to overcome internal friction & iron losses when unloaded.

    'Power Split Ratio'
        Fraction of total power delivered to the primary load in multi-load use.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Motor
    """

    def __defaults__(self):
        """
        Set default values so the component can be instantiated safely.

        Assumptions: None
        Inputs:      None
        Outputs:     None
        """
        # Always set a tag
        self.tag = "motor"

        # Initialize a gearbox container up-front so it always exists
        self.gearbox = Data()
        self.gearbox.gear_ratio = 1.0  # default unity ratio

        # Some attributes that are common / referenced elsewhere
        self.power_split_ratio = 0.0
        self.interpolated_func = None

        # Route to the type setter to initialize consistent defaults
        # (This will populate resistance, no_load_current, etc.)
        self.type = "DC"

        return

    # -----------------------------
    #  type property
    # -----------------------------
    @property
    def type(self):
        return getattr(self, "_type", "DC")

    @type.setter
    def type(self, value: str):
        self._type = value

        # Ensure gearbox exists before we rely on it
        if not hasattr(self, "gearbox") or self.gearbox is None:
            self.gearbox = Data()
        if not hasattr(self.gearbox, "gear_ratio"):
            self.gearbox.gear_ratio = 1.0

        # Initialize attributes that both branches may rely on
        self.interpolated_func = None
        self.inverse_calculation = False
        self.design_angular_velocity = 0.0  # [rad/s]
        self.design_torque = 0.0            # [N·m]
        self.design_current = 0.0           # [A]

        if value == "DC":
            # DC motor baseline defaults
            self.resistance = 0.0           # [Ω]
            self.no_load_current = 0.0      # [A]
            self.speed_constant = 0.0       # [rpm/V]
            self.efficiency = 1.0           # [-]
            self.gearbox.gear_ratio = 1.0   # [-]

        elif value == "PMSM":
            # PMSM: provide a richer default parameterization
            # Datasheet-like inputs
            self.speed_constant = 6.56                 # [rpm/V]
            self.stator_inner_diameter = 0.16          # [m]
            self.stator_outer_diameter = 0.348         # [m]

            # Literature / configuration
            self.winding_factor = 0.95                 # [-]

            # Assumptions (baseline)
            self.resistance = 0.002                    # [Ω]
            self.motor_stack_length = 0.1140           # [m]
            self.number_of_turns = 80                  # [-]
            self.length_of_path = 0.4                  # [m]
            self.mu_0 = 1.256637061e-6                 # [N/A^2]
            self.mu_r = 1005                           # [-]
            self.thermal_conductivity = 200            # [W/m·K]
            self.Delta_T = 10                          # [K]
            self.characteristic_length_of_flow = 0.01  # [m]
            self.thermal_conductivity_fluid = 0.026    # [W/m·K]
            self.length_of_conductive_path = 0.4       # [m]
            self.Re_cooling_flow = 100000              # [-]
            self.Re_airgap = 100000                    # [-]
            self.Prandtl_number = 0.708                # [-]
            self.height_of_duct = 0.005                # [m]
            self.width_of_duct = 0.005                 # [m]
            self.hydraulic_diameter_of_duct = 0.005    # [m]
            self.length_of_channel = 0.005             # [m]
            self.volume_flow_rate_of_fluid = 0.005     # [m^3/s]
            self.density_of_fluid = 1000               # [kg/m^3]
            self.velocity_of_fluid = 0.005             # [m/s]
            self.Taylor_number = 20                    # [-]
            self.axial_gap_to_radius_of_rotor = 0.01   # [-]
            self.Conduction_laminar_flow = True        # [-]
            self.Convection_laminar_flow = True        # [-]

            # Reasonable gearbox default still unity unless set by user
            self.gearbox.gear_ratio = 1.0

            # PMSM overall efficiency can be computed downstream; omit hard default
            self.efficiency = None
            self.no_load_current = None

        else:
            # Fallback to DC if an unknown type is provided (fail-safe)
            self._type = "DC"
            self.resistance = 0.0
            self.no_load_current = 0.0
            self.speed_constant = 0.0
            self.efficiency = 1.0
            self.gearbox.gear_ratio = 1.0

        return

    # -----------------------------
    #  operating conditions
    # -----------------------------
    def append_operating_conditions(self, segment):
        """Attach motor operating conditions to the segment's energy conditions."""
        append_motor_conditions(self, segment)
        return


    
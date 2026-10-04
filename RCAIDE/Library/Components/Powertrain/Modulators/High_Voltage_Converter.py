# RCAIDE/Library/Components/Powertrain/Modulators/High_Voltage_Converter.py
#
# Created:  Oct 2026, RCAIDE EHD MVP

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Library.Components import Component
from RCAIDE.Library.Methods.Powertrain.Modulators.High_Voltage_Converter.append_hvpc_conditions import append_hvpc_conditions

# ----------------------------------------------------------------------------------------------------------------------
#  High_Voltage_Converter
# ----------------------------------------------------------------------------------------------------------------------
class High_Voltage_Converter(Component):
    """
    Fixed-efficiency high-voltage power converter (HVPC) that steps bus voltage up to the EHD electrode
    voltage (spec 3.3; R17).

    Attributes
    ----------
    tag : str
        Default 'high_voltage_converter'
    efficiency : float
        eta_HV [-], default 0.85 (first-generation MIT converter, Shevgaonkar 2025)
    specific_power : float
        [W/kg], default 1150 (Shevgaonkar 2025; He et al. 2017 report 1200)
    rated_power : float or None
        [W], user input; sets converter mass
    input_voltage_window : list
        [V_low, V_high] [V], default [160, 225] (MIT design)
    bus_voltage : float or None
        Supply bus voltage [V]; compared against input_voltage_window

    Notes
    -----
    The ESC was not reused: it carries only an efficiency and maps throttle to throttle·V_bus, with no
    specific power, rated power or input window. Like the ESC, the HVPC is owned by its propulsor, so its
    losses are folded into the power the propulsor reports to the bus.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller
    RCAIDE.Library.Components.Powertrain.Propulsors.EHD_Thruster
    """

    def __defaults__(self):
        self.tag                  = 'high_voltage_converter'
        self.efficiency           = 0.85
        self.specific_power       = 1150.
        self.rated_power          = None
        self.input_voltage_window = [160., 225.]
        self.bus_voltage          = None

    def append_operating_conditions(self, segment, energy_conditions, noise_conditions=None):
        """
        Allocates per-control-point HVPC results.
        """
        append_hvpc_conditions(self, segment, energy_conditions)
        return

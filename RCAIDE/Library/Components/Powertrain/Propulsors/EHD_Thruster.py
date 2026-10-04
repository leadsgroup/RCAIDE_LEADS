# RCAIDE/Library/Components/Powertrain/Propulsors/EHD_Thruster.py
#
# Created:  Oct 2026, RCAIDE EHD MVP

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from .Propulsor import Propulsor
from RCAIDE.Library.Methods.Powertrain.Propulsors.EHD_Thruster.append_ehd_thruster_conditions   import append_ehd_thruster_conditions
from RCAIDE.Library.Methods.Powertrain.Propulsors.EHD_Thruster.compute_ehd_thruster_performance import compute_ehd_thruster_performance, reuse_stored_ehd_thruster_data

# ----------------------------------------------------------------------------------------------------------------------
#  EHD_Thruster
# ----------------------------------------------------------------------------------------------------------------------
class EHD_Thruster(Propulsor):
    """
    Electrohydrodynamic (EHD) thruster made of N wire-to-NACA 0010 units fed by a high-voltage power
    converter (spec 3.3).

    Attributes
    ----------
    tag : str
        Default 'ehd_thruster'
    electrode_array : EHD_Electrode_Array
        Electrode geometry and physics inputs
    high_voltage_converter : High_Voltage_Converter
        HVPC between the bus and the electrodes
    additional_mass : float
        Spacers and wiring lump mass [kg], default 0.0

    Notes
    -----
    Throttle maps directly to electrode voltage, so the thruster adds no unknowns or residuals to the
    mission solver. Collector and wire drag are computed inside the propulsor; do not also model the
    collectors as wings. Net thrust acts along body +x. compute_performance returns the electrode power
    as the propulsor power and the HVPC input power as the electrical (bus) power.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters.EHD_Electrode_Array
    RCAIDE.Library.Components.Powertrain.Modulators.High_Voltage_Converter
    RCAIDE.Library.Methods.Powertrain.Propulsors.EHD_Thruster
    """

    def __defaults__(self):
        self.tag                    = 'ehd_thruster'
        self.electrode_array        = None
        self.high_voltage_converter = None
        self.additional_mass        = 0.0

    def append_operating_conditions(self, segment, energy_conditions, noise_conditions=None):
        """
        Appends operating conditions of the segment.
        """
        append_ehd_thruster_conditions(self, segment, energy_conditions, noise_conditions)
        return

    def append_propulsor_unknowns_and_residuals(self, segment):
        """
        No internal unknowns or residuals: throttle maps directly to voltage.
        """
        return

    def unpack_propulsor_unknowns(self, segment):
        """
        No internal unknowns.
        """
        return

    def pack_propulsor_residuals(self, segment):
        """
        No internal residuals.
        """
        return

    def compute_performance(self, state, center_of_gravity=[[0, 0, 0]]):
        """
        Computes thrust, moment, electrode power and bus power.
        """
        thrust, moment, power, power_elec, stored_results_flag, stored_propulsor_tag = compute_ehd_thruster_performance(self, state, center_of_gravity)
        return thrust, moment, power, power_elec, stored_results_flag, stored_propulsor_tag

    def reuse_stored_data(self, state, network, stored_propulsor_tag=None, center_of_gravity=[[0, 0, 0]]):
        """
        Reuses stored thruster data for identical thrusters.
        """
        thrust, moment, power, power_elec = reuse_stored_ehd_thruster_data(self, state, network, stored_propulsor_tag, center_of_gravity)
        return thrust, moment, power, power_elec

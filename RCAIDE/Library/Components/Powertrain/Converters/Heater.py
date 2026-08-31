# RCAIDE/Library/Components/Powertrain/Converters/Heater.py
#
# Created: Aug 2026, M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
from RCAIDE.Library.Components.Powertrain.Converters import Converter
from RCAIDE.Library.Methods.Powertrain.Converters.Heater.append_heater_conditions import append_heater_conditions
from RCAIDE.Library.Methods.Powertrain.Converters.Heater.compute_heater_performance import compute_heater_performance

# ----------------------------------------------------------------------
#  Heater
# ----------------------------------------------------------------------
class Heater(Converter):
    """
    Electric pressure-builder heater for a cryogenic fuel tank.

    Draws electrical power from its assigned bus (typically an
    ``Electrical_Bus`` fed by an engine's ``integrated_drive_generator``,
    the same architecture ``Pump`` draws from) to supply the thermal power
    an in-flight cryogenic tank's boil-off model (``Cryogenic_Tank`` with
    ``boil_off_model = 'quasi_steady'``) has already determined it needs
    to hold ullage pressure near ``design_pressure``. See
    ``compute_heater_performance`` for the read-source and ordering/
    staleness details.
    """

    def __defaults__(self):
        """

        """
        self.tag            = 'Heater'
        self.efficiency     = 0.95   # electrical -> thermal conversion efficiency of the heating element
        self.assigned_tank  = None   # tag of the Cryogenic_Tank source this heater supplies
        self.rated_power    = None   # max thermal power [W]; None -> tank sizes a default cap (see compute_cryogenic_tank_performance)

        return

    def append_operating_conditions(self, segment):
        """Attach heater operating conditions to the segment's energy conditions."""
        append_heater_conditions(self, segment)
        return

    def compute_performance(self,state,network=None):

        inputs, outputs, stored_results_flag, stored_converter_tag =  compute_heater_performance(self,state,network)
        return inputs, outputs, stored_results_flag, stored_converter_tag

# RCAIDE/Library/Components/Powertrain/Converters/Reformer_Fuel_Cell.py
#
#
# Created:  Aug 2026, RCAIDE Team

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from .Converter                             import Converter
from RCAIDE.Library.Methods.Powertrain.Converters.Reformer_Fuel_Cell.append_reformer_fuel_cell_conditions      import append_reformer_fuel_cell_conditions
from RCAIDE.Library.Methods.Powertrain.Converters.Reformer_Fuel_Cell.compute_reformer_fuel_cell_performance    import compute_reformer_fuel_cell_performance

# ----------------------------------------------------------------------
#  Reformer_Fuel_Cell
# ----------------------------------------------------------------------
class Reformer_Fuel_Cell(Converter):
    """
    A reformer/fuel-cell composite propulsion system model: a fuel cell that draws its
    hydrogen from an onboard reformer converting a hydrocarbon fuel, rather than from a
    hydrogen fuel tank.

    Attributes
    ----------
    tag : str
        Identifier for the composite. Default is 'reformer_fuel_cell'.

    reformer : Component
        Reformer component. Default is None; assign a Reformer instance.

    fuel_cell : Component
        Fuel cell component. Default is None; assign a Generic_Fuel_Cell_Stack or
        Proton_Exchange_Membrane_Fuel_Cell instance.

    power_split_ratio : float
        Fraction of the network's electrical demand this unit supplies. Default is 1.0;
        set to e.g. 0.5 on each of two identical units sharing a bus.

    design_voltage : float
        Target stack voltage [V]. Mirrored onto fuel_cell.design_voltage and used to
        size fuel_cell automatically -- see initialize() (default: None)

    design_power : float
        Target stack power [W], mirrored onto fuel_cell.design_power the same way
        (default: None)

    Notes
    -----
    Modeled on Turboelectric_Generator: the composite is assigned to both a fuel
    (chemical) distributor, for the reformer's hydrocarbon feed, and an electrical
    distributor, for the fuel cell's power delivery (e.g.
    ``reformer_fuel_cell.assigned_distributors = [[fuel_line.tag, bus.tag]]``).

    ``reformer.working_fluid`` has no default and must be set explicitly (e.g.
    ``reformer_fuel_cell.reformer.working_fluid = RCAIDE.Library.Attributes.Propellants.Jet_A()``)
    to the hydrocarbon fuel drawn from that fuel distributor.

    ``identical_converters`` is not set here, so it stays at the base Converter default
    of False -- correct for now since no ``reuse_stored_data`` is implemented. Set it to
    True (matching Turboelectric_Generator) only once a ``reuse_stored_data`` implementation
    is added, if a vehicle ever has multiple identical units sharing a bus.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters.Reformer
    RCAIDE.Library.Components.Powertrain.Converters.Generic_Fuel_Cell_Stack
    RCAIDE.Library.Components.Powertrain.Converters.Turboelectric_Generator
    """
    def __defaults__(self):
        # setting the default values
        self.tag                       = 'reformer_fuel_cell'
        self.provides_domain           = 'electrical'
        self.reformer                  = None
        self.fuel_cell                 = None
        self.power_split_ratio         = 1.0    # fraction of the electrical demand this unit supplies, for multiple identical units sharing a bus
        self.design_voltage            = None
        self.design_power              = None

    def initialize(self, network):
        """
        Mirrors design_voltage/design_power onto fuel_cell and sizes it via
        design_fuel_cell(). Runs automatically once per mission (see
        Generic_Fuel_Cell_Stack.initialize()), so vehicle scripts only need to set
        reformer_fuel_cell.design_voltage/design_power -- no explicit design_fuel_cell()
        call needed.
        """
        if self.fuel_cell is not None:
            if self.design_voltage is not None:
                self.fuel_cell.design_voltage = self.design_voltage
            if self.design_power is not None:
                self.fuel_cell.design_power = self.design_power
            self.fuel_cell.initialize(network)
        return

    def append_operating_conditions(self,segment):
        """
        Appends operating conditions of the segment.
        """
        append_reformer_fuel_cell_conditions(self,segment)
        return

    def compute_performance(self,state,network):
        """
        Computes Reformer_Fuel_Cell performance including power.
        """
        inputs, outputs, stored_results_flag, stored_converter_tag = compute_reformer_fuel_cell_performance(self,state,network)
        return inputs, outputs, stored_results_flag, stored_converter_tag

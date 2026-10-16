# RCAIDE/Library/Components/Powertrain/Converters/Reformer_Fuel_Cell.py
#
#
# Created:  Aug 2026, RCAIDE Team

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from .Converter                             import Converter
from RCAIDE.Framework.Core                  import Units
from RCAIDE.Library.Methods.Powertrain.Converters.Reformer_Fuel_Cell.append_reformer_fuel_cell_conditions      import append_reformer_fuel_cell_conditions
from RCAIDE.Library.Methods.Powertrain.Converters.Reformer_Fuel_Cell.compute_reformer_fuel_cell_performance    import compute_reformer_fuel_cell_performance

# ----------------------------------------------------------------------
#  Reformer_Fuel_Cell
# ----------------------------------------------------------------------
class Reformer_Fuel_Cell(Converter):
    """
    A reformer/fuel-cell system: a fuel cell that draws its hydrogen from an onboard
    reformer converting a hydrocarbon fuel, rather than from a hydrogen fuel tank. It is
    modeled as one reformer + solid oxide fuel cell + balance-of-plant system with a fixed
    fuel-to-electricity efficiency and specific power [1]. A hydrogen-fed fuel cell is
    modeled with Generic_Fuel_Cell_Stack instead.

    Attributes
    ----------
    tag : str
        Identifier for the system. Default is 'reformer_fuel_cell'.

    working_fluid : RCAIDE.Library.Attributes.Propellants.Propellant
        Hydrocarbon fuel drawn from the fuel line; its lower_heating_value sets the fuel
        flow. No default; must be set.

    power_split_ratio : float
        Fraction of the network's electrical demand this unit supplies. Default is 1.0;
        set to e.g. 0.5 on each of two identical units sharing a bus.

    design_voltage : float
        Output voltage [V] used to size the electrical bus (default: None)

    design_power : float
        Rated net electrical output [W]; sets the system mass (default: None)

    system_efficiency : float
        Net electrical output over fuel LHV input [-]. Default 0.46, normal cruise of the
        regional-jet SOFC APU in [1], Table 27 (0.45 to 0.48 across cruise power levels;
        0.32 to 0.36 on the ground).

    system_specific_power : float
        design_power per unit system mass [W/kg]. Default 185 kW / 805.4 lb, Case 2 with the
        1.0 kW/kg BSC-2 stack in [1], Tables 27 and 28 (about 0.35 kW/kg with the 0.56 kW/kg
        2005 stack).

    Notes
    -----
    The unit is assigned to both a fuel (chemical) distributor, for the hydrocarbon feed,
    and an electrical distributor, for its power delivery (e.g.
    ``reformer_fuel_cell.assigned_distributors = [[fuel_line.tag, bus.tag]]``).

    References
    ----------
    [1] Mak, A., and Meier, J., "Fuel Cell Auxiliary Power Study, Volume 1: RASER Task Order 5,"
        NASA/CR-2007-214461/VOL1, Honeywell Engines, Systems & Services, 2007 (autothermal jet
        fuel reformer and solid oxide fuel cell APU for a regional jet).

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters.Generic_Fuel_Cell_Stack
    RCAIDE.Library.Components.Powertrain.Converters.Turboelectric_Generator
    """
    def __defaults__(self):
        # setting the default values
        self.tag                       = 'reformer_fuel_cell'
        self.provides_domain           = 'electrical'
        self.working_fluid             = None
        self.power_split_ratio         = 1.0    # fraction of the electrical demand this unit supplies, for multiple identical units sharing a bus
        self.design_voltage            = None
        self.design_power              = None
        self.system_efficiency         = 0.46                          # [-] fuel LHV to net electricity at cruise, [1] Table 27
        self.system_specific_power     = 185E3 / (805.4 * Units.lb)    # [W/kg] about 506 W/kg, [1] Tables 27 and 28, Case 2

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

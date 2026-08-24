# RCAIDE/Library/Components/Powertrain/Converters/Converter.py 
# 
# Created:  Feb 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Components                      import Component   

# ---------------------------------------------------------------------------------------------------------------------- 
#  Converter Component
# ----------------------------------------------------------------------------------------------------------------------
class Converter(Component):
    """
    A generatic converter class object used to build all converters. Inherits from the Component class.

    ``provides_domain`` is the energy domain this converter is a PROVIDER
    of (e.g. ``'electrical'`` for a Generator/Turboelectric_Generator/fuel
    cell), self-declared by each concrete subclass. ``None`` means it is
    not a provider of any domain -- it is registered as a plain consumer on
    whatever distributors it is assigned to (motors, pumps, compressors,
    ...). This lets topology analysis
    (``RCAIDE.Library.Mission.Common.Pre_Process.energy.analyze_topology``)
    classify converters without a hardcoded isinstance list.
    """

    def __defaults__(self):
        """This sets the default values for the component to function.

        Assumptions:
            None 
        """
        # set the deafult values
        self.tag                                = 'tag' 
        self.working_fluid                      = Data()
        self.active                             = True
        self.assigned_converters                = None
        self.assigned_modulators                = None
        self.assigned_distributors              = None 
        self.identical_converters               = False
        self.efficiency                         = 1.0
        self.provides_domain                    = None

    def initialize(self, network):
        return

    def append_segment_conditions(self, segment):
        converter_conditions = segment.state.conditions.energy.converters[self.tag]
        converter_conditions.inputs.power.propulsive[:,0]  = 0.0
        converter_conditions.inputs.power.mechanical[:,0]  = 0.0
        converter_conditions.inputs.power.electrical[:,0]  = 0.0
        converter_conditions.inputs.power.chemical[:,0]    = 0.0
        converter_conditions.inputs.power.pneumatic[:,0]   = 0.0
        converter_conditions.inputs.power.hydraulic[:,0]   = 0.0
        converter_conditions.inputs.power.thermal[:,0]     = 0.0
        converter_conditions.outputs.power.propulsive[:,0] = 0.0
        converter_conditions.outputs.power.mechanical[:,0] = 0.0
        converter_conditions.outputs.power.electrical[:,0] = 0.0
        converter_conditions.outputs.power.chemical[:,0]   = 0.0
        converter_conditions.outputs.power.pneumatic[:,0]  = 0.0
        converter_conditions.outputs.power.hydraulic[:,0]  = 0.0
        converter_conditions.outputs.power.thermal[:,0]    = 0.0

    def reuse_stored_data(self, state, network, stored_conveter_tag):
        """Copies another identical converter's inputs/outputs onto this one.

        Generic fallback for identical_converters=True: subclasses with internal
        sub-component conditions to propagate (e.g. Turboshaft, Turboelectric_Generator)
        override this with their own version; this one is sufficient for converters
        whose conditions are just the standard inputs/outputs.power.* fields.
        """
        stored_conditions = state.conditions.energy.converters[stored_conveter_tag]
        converter_conditions = state.conditions.energy.converters[self.tag]

        for power_type in stored_conditions.inputs.power.keys():
            converter_conditions.inputs.power[power_type][:,0]  = stored_conditions.inputs.power[power_type][:,0]
        for power_type in stored_conditions.outputs.power.keys():
            converter_conditions.outputs.power[power_type][:,0] = stored_conditions.outputs.power[power_type][:,0]

        if 'fuel_mass_flow_rate' in stored_conditions:
            converter_conditions.fuel_mass_flow_rate[:,0] = stored_conditions.fuel_mass_flow_rate[:,0]

        return converter_conditions.inputs, converter_conditions.outputs
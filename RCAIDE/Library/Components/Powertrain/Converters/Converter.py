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
        self.identical_converters               = True 
        self.efficiency                         = 1.0

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
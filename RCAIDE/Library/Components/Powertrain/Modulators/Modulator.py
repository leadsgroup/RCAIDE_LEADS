# RCAIDE/Library/Components/Powertrain/Modulators/Modulator.py 
# 
# Created:  Oct 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Components                      import Component    
# ---------------------------------------------------------------------------------------------------------------------- 
#  Converter Component
# ----------------------------------------------------------------------------------------------------------------------
class Modulator(Component):
    """
    A generatic modulator class object used to build all modulators. Inherits from the Component class.
    """

    def __defaults__(self):
        """This sets the default values for the component to function.

        Assumptions:
            None 
        """
        # set the deafult values
        self.tag                      = 'tag' 
        self.working_fluid            = Data()
        self.active                   = True
        self.assigned_distributors    = None 
        self.efficiency               = 1.0 
        
    def initialize(self, network):
        return
    
    def unpack_unknowns(self,segment):
        return

    def pack_residuals(self,segment):
        return

    def append_unknowns_and_residuals(self,segment):
        return

    def append_segment_conditions(self, segment):
        modulator_conditions = segment.state.conditions.energy.modulators[self.tag]
        modulator_conditions.inputs.power.propulsive[:,0]  = 0.0
        modulator_conditions.inputs.power.mechanical[:,0]  = 0.0
        modulator_conditions.inputs.power.electrical[:,0]  = 0.0
        modulator_conditions.inputs.power.chemical[:,0]    = 0.0
        modulator_conditions.inputs.power.pneumatic[:,0]   = 0.0
        modulator_conditions.inputs.power.hydraulic[:,0]   = 0.0
        modulator_conditions.inputs.power.thermal[:,0]     = 0.0
        modulator_conditions.outputs.power.propulsive[:,0] = 0.0
        modulator_conditions.outputs.power.mechanical[:,0] = 0.0
        modulator_conditions.outputs.power.electrical[:,0] = 0.0
        modulator_conditions.outputs.power.chemical[:,0]   = 0.0
        modulator_conditions.outputs.power.pneumatic[:,0]  = 0.0
        modulator_conditions.outputs.power.hydraulic[:,0]  = 0.0
        modulator_conditions.outputs.power.thermal[:,0]    = 0.0
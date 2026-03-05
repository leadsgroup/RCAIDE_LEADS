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
        self.efficiency               = Data()
        self.efficiency.electrical    = 1.0
        self.efficiency.mechanical    = 1.0
        self.efficiency.chemical      = 1.0
        self.efficiency.hydraulic     = 1.0
        self.efficiency.pneumatic     = 1.0
        self.efficiency.thermal       = 1.0 

    def append_segment_conditions(self,segment): 
        energy_conditions  = segment.state.conditions.energy    
        energy_conditions.modulators[self.tag].inputs.power.electrical[:,0]  = 0.0
        energy_conditions.modulators[self.tag].inputs.power.chemical[:,0]    = 0.0
        energy_conditions.modulators[self.tag].inputs.power.thermal[:,0]     = 0.0
        energy_conditions.modulators[self.tag].outputs.power.electrical[:,0] = 0.0
        energy_conditions.modulators[self.tag].outputs.power.chemical[:,0]   = 0.0
        energy_conditions.modulators[self.tag].outputs.power.thermal[:,0]    = 0.0
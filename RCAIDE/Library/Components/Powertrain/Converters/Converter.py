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
        self.tag                      = 'tag' 
        self.working_fluid            = Data()
        self.active                   = True
        self.assigned_converters      = None
        self.assigned_modulators      = None
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
        energy_conditions.converters[self.tag].inputs.power.electrical[0,:]  = 0  # initial_conditions.propulsors[propulsor.tag].inputs.power.electrical[-1,0] 
        energy_conditions.converters[self.tag].inputs.power.chemical[0,:]    = 0  # initial_conditions.propulsors[propulsor.tag].inputs.power.chemical[-1,0]   
        energy_conditions.converters[self.tag].inputs.power.thermal[0,:]     = 0  # initial_conditions.propulsors[propulsor.tag].inputs.power.thermal[-1,0]    
        energy_conditions.converters[self.tag].outputs.power.electrical[0,:] = 0  # initial_conditions.propulsors[propulsor.tag].outputs.power.electrical[-1,0]
        energy_conditions.converters[self.tag].outputs.power.chemical[0,:]   = 0  # initial_conditions.propulsors[propulsor.tag].outputs.power.chemical[-1,0]  
        energy_conditions.converters[self.tag].outputs.power.thermal[0,:]    = 0  # initial_conditions.propulsors[propulsor.tag].outputs.power.thermal[-1,0]                 
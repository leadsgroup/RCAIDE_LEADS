# RCAIDE/Library/Components/Powertrain/Sources/Source
#  
# 
# Created:  Oct. 2025, M. Guidotti 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
from   RCAIDE.Framework.Core               import Data
from   RCAIDE.Library.Components           import Component 
from   RCAIDE.Framework.Mission.Common     import Conditions

# ----------------------------------------------------------------------------------------------------------------------
#  Source
# ---------------------------------------------------------------------------------------------------------------------- 
class Source(Component):
    """
    """   
    def __defaults__(self):
        """ This sets the default values.
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            None
    
            Outputs:
            None
    
            Properties Used:
            None
        """          
        self.tag                          = 'source' 
        self.active                       = True 
        self.assigned_distributors        = None
        self.efficiency                   = 0.0
        self.power_split_ratio            = 1.0
        self.identical_sources            = False 

    #def append_segment_conditions(self,segment): 
        #energy_conditions  = segment.state.conditions.energy    
        #energy_conditions.sources[self.tag].inputs.power.electrical[:,0]  = 0.0   
        #energy_conditions.sources[self.tag].inputs.power.chemical[:,0]    = 0.0   
        #energy_conditions.sources[self.tag].inputs.power.thermal[:,0]     = 0.0   
        #energy_conditions.sources[self.tag].outputs.power.electrical[:,0] = 0.0   
        #energy_conditions.sources[self.tag].outputs.power.chemical[:,0]   = 0.0   
        #energy_conditions.sources[self.tag].outputs.power.thermal[:,0]    = 0.0   
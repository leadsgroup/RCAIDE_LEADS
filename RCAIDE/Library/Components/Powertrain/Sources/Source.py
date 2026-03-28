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
        self.power_split_ratio            = None
        self.identical_sources            = False
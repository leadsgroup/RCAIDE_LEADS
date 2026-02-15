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
        self.efficiency                   = Data()
        self.efficiency.propulsive        = 1.0
        self.efficiency.electrical        = 1.0
        self.efficiency.mechanical        = 1.0
        self.efficiency.thermal           = 1.0
        self.efficiency.chemical          = 1.0
        self.efficiency.hydraulic         = 1.0
        self.efficiency.pneumatic         = 1.0
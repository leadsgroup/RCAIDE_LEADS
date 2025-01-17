# Planet.py
#
# Created:  
# Modified: Feb 2016, Andrew Wendorff

# ----------------------------------------------------------------------
#  Imports
# ---------------------------------------------------------------------- 
from RCAIDE.Framework.Core import Data 
from RCAIDE.Framework.Analyses import Analysis

# ----------------------------------------------------------------------
#  Analysis
# ----------------------------------------------------------------------
class Planet(Analysis):
    """ RCAIDE.Framework.Analyses.Planet()
    """
    
    def __defaults__(self):
        
        """This sets the default values and methods for the analysis.
    
            Assumptions:
            Planet is Earth.
    
            Source:
            N/A
    
            Inputs:
            None
    
            Outputs:
            None
    
            Properties Used:
            N/A
            """                  
        
        
        self.tag      = 'planet'
        self.settings = Data()
        
        
        
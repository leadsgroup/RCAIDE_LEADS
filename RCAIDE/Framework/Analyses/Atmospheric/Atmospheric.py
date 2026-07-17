#RCAIDE/Frameworks/Analyses/Atmospheric/Atmospheric.py
#
# Created: Dec 2024, M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
from RCAIDE.Library.Attributes.Atmospheres.Atmosphere import Atmosphere
from RCAIDE.Framework.Analyses import Analysis


# ----------------------------------------------------------------------
#  Analysis
# ---------------------------------------------------------------------- 
class Atmospheric(Analysis):
    """This is the base class for atmospheric analyses. It contains functions
    that are built into the default class.
    
    Assumptions:
    None
    
    Source:
    N/A
    """
    def __defaults__(self):
        """This sets the default values for the analysis to function.
        
        Assumptions:
        None
        
        Source:
        N/A
        
        Inputs:
        None
        
        Outputs:
        None
        
        Properties Used:
        N/A
        """          
        atmo_data = Atmosphere()
        self.update(atmo_data)
        
        
    def compute_values(self,altitude):
        """This function is not implemented for the base class."""
        raise NotImplementedError

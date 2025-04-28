# Planet.py
#
# Created:  
# Modified: Apr 2025, S Shekar

# ----------------------------------------------------------------------
#  Imports
# ---------------------------------------------------------------------- 
from RCAIDE.Framework.Core import Data 
from RCAIDE.Framework.Analyses import Analysis

# ----------------------------------------------------------------------
#  Analysis
# ----------------------------------------------------------------------
class Geometry(Analysis):
    """ RCAIDE.Framework.Analyses.Planet()
    """
    
    def __defaults__(self):
        
        """This sets the default values and methods for the analysis.
    
            Assumptions:
            N/A
    
            Source:
            N/A
    
            Inputs:
            None
    
            Outputs:
            None
    
            Properties Used:
            N/A
            """                  
        
        
        self.tag                                 = 'geometry'
        self.vehicle                             = None
        self.settings                            = Data()
        self.settings.update_fuselage_properties = False
        self.settings.overwrite_reference        = False
        self.settings.update_wing_properties     = False
        self.settings.update_fuel_volume         = False

        
        
        
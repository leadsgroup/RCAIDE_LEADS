# RCAIDE/Library/Attributes/Solids/Solid.py
#  
 
#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------

from RCAIDE.Framework.Core import Data

#-------------------------------------------------------------------------------
# Solid Data Class
#------------------------------------------------------------------------------- 
class Solid(Data):
    """ Default Template for Solid Attribute Classes  
    """

    def __defaults__(self):
        """Default Instantiation of Physical Property Values
        
        Assumptions:
            None
        
        Source:
            None
        """

        self.ultimate_tensile_strength  = None
        self.ultimate_shear_strength    = None
        self.ultimate_bearing_strength  = None
        self.yield_tensile_strength     = None
        self.yield_shear_strength       = None
        self.yield_bearing_strength     = None
        self.minimum_gage_thickness     = None
        self.density                    = None

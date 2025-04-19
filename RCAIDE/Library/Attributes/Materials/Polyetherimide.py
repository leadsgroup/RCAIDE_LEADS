# RCAIDE/Library/Attributes/Solids/Polyetherimide.py
# 
# 
#
# Created: Sep 2024 S. Shekar

#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------

from .Solid import Solid
from RCAIDE.Framework.Core import Units

#-------------------------------------------------------------------------------
# Polyetherimide for Reservoir Casing
#-------------------------------------------------------------------------------
class Polyetherimide(Solid):

    """ Physical Constants Specific to Polyetherimide   
    """
    def __defaults__(self):
        """Sets material properties at instantiation.

        Assumptions:
        None

        Source:
        https://www.matweb.com/search/DataSheet.aspx?MatGUID=65baf7a4f90c4a54a6ace03e16b1125b&amp%3bckck=1&ckck=1

        Inputs:
        N/A

        Outputs:
        N/A

        Properties Used:
        None
        """ 
        self.conductivity              = 0.2    # [W/m-K]
        self.emissivity                = 0.96   # [uniteless]
        self.specific_heat             = 1100  

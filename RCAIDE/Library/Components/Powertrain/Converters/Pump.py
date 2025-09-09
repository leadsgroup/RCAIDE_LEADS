# RCAIDE/Library/Components/Powertrain/Converters/Pump.py
# 
# Created: Sep. 2025  M. Guidotti

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
from RCAIDE.Framework.Core import Data

# ----------------------------------------------------------------------
#  Pump
# ----------------------------------------------------------------------
class Pump(Data):
    """
    """

    def __defaults__(self):
        """
        
        """
        self.tag        = 'Pump'
        self.efficiency = 1.0
        return
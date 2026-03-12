# RCAIDE/Library/Components/Powertrain/Distributors/Distributor.py 
# 
# Created:  Oct 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Components                      import Component   

# ---------------------------------------------------------------------------------------------------------------------- 
#  Distributor
# ----------------------------------------------------------------------------------------------------------------------
class Distributor(Component):
    """

    """

    def __defaults__(self):
        """This sets the default values for the component to function.

        Assumptions:
            None 
        """
        # set the deafult values
        self.tag                      = 'tag' 
        self.type                     = 'distributor'
        self.working_fluid            = Data()
        self.active                   = True
        self.assigned_distributors    = None
        self.efficiency               = 0.0
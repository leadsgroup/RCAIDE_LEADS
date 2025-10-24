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
        self.working_fluid            = Data()
        self.active                   = True
        self.assigned_distributors    = []
        self.efficiency               = Data()
        self.efficiency.electrical    = 1.0
        self.efficiency.mechanical    = 1.0
        self.efficiency.chemical      = 1.0
        self.efficiency.hydraulic     = 1.0
        self.efficiency.pneumatic     = 1.0
        self.efficiency.thermal       = 1.0
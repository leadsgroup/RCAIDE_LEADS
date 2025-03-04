# RCAIDE/Framework/Analyses/Weights/Electric.py
#
# Created:  Feb 2025, S. Shekar
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

import RCAIDE
from RCAIDE.Framework.Core import Data 
from .Weights import Weights

# ----------------------------------------------------------------------------------------------------------------------
#  
# ----------------------------------------------------------------------------------------------------------------------
class Electric(Weights):
    """ This is class that evaluates the weight of Transport class aircraft

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

    def __defaults__(self):
        """This sets the default values and methods for the tube and wing
        aircraft weight analysis.

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
        self.method        = 'Physics_Based'
        self.aircraft_type = 'General_Aviation'
        self.propulsion_architecture = 'Electric'

        self.settings.use_max_fuel_weight = True 
        self.settings.advanced_composites = False
        self.settings.fuselage_mounted_landing_gear_factor = 1.12 # assumes fuselage mounted landing gear. Change to 1 if False
        self.settings.cargo_doors_number = 1 # 0 if no cargo doors, 1 if 1 cargo door, 2 if 2 cargo doors
        self.settings.cargo_doors_clamshell = False # True if clamshell cargo doors, False if not
        
        self.settings.weight_reduction_factors           = Data()  
        self.settings.weight_reduction_factors.main_wing = 0.  # Reduction factors are proportional (.1 is a 10% weight reduction)
        self.settings.weight_reduction_factors.fuselage  = 0.  # Reduction factors are proportional (.1 is a 10% weight reduction)
        self.settings.weight_reduction_factors.empennage = 0.  # applied to horizontal and vertical stabilizers
        
        # FLOPS settings
        self.settings.FLOPS = Data() 
        self.settings.FLOPS.complexity  = 'Simple' 
        self.settings.FLOPS.aeroelastic_tailoring_factor = 0.   # Aeroelastic tailoring factor [0 no aeroelastic tailoring, 1 maximum aeroelastic tailoring] 
        self.settings.FLOPS.strut_braced_wing_factor     = 0.   # Wing strut bracing factor [0 for no struts, 1 for struts]
        

        
     
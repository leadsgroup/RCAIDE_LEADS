# RCAIDE/Framework/Analyses/Weights/Conventional_General_Aviationpy
#
# Created:  Jun 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Data 
from .Weights import Weights

# ----------------------------------------------------------------------------------------------------------------------
#  General Aviation Transport Weights Analysis
# ----------------------------------------------------------------------------------------------------------------------
class Conventional_General_Aviation(Weights):
    """ This is class that evaluates the weight of a conventional general aviation class aircraft

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
        self.aircraft_type                                 = 'General_Aviation' 
        self.method                                        = 'FLOPS'
        self.propulsion_architecture                       = 'Conventional' 
        self.settings.advanced_composites                  = False
        self.settings.fuselage_mounted_landing_gear_factor = 1.12 # assumes fuselage mounted landing gear. Change to 1 if False
        self.settings.cargo_doors_number                   = 1 # 0 if no cargo doors, 1 if 1 cargo door, 2 if 2 cargo doors
        self.settings.cargo_doors_clamshell                = False # True if clamshell cargo doors, False if not
        
        # FLOPS settings
        self.settings.FLOPS                                = Data() 
        self.settings.FLOPS.fidelity                       = 'Simple' 
        self.settings.FLOPS.aeroelastic_tailoring_factor   = 0   # Aeroelastic tailoring factor [0 no aeroelastic tailoring, 1 maximum aeroelastic tailoring] 
        self.settings.FLOPS.strut_braced_wing_factor       = 0.   # Wing strut bracing factor [0 for no struts, 1 for struts]
        
     
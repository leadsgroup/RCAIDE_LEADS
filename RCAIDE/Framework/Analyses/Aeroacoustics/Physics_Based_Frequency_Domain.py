# RCAIDE/Framework/Analyses/Aeroacoustics/Physics_Based.py
# 
# 
# Created:   Jul 2023, M. Clarke
# Modified:  Oct 2024, A. Molloy

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
# RCAIDE Imports
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Library.Methods.Aeroacoustics.Physics_Based_Frequency_Domain.evaluate_aeroacoustics import evaluate_aeroacoustics
from .Aeroacoustics      import Aeroacoustics

# ----------------------------------------------------------------------------------------------------------------------
#  Physics_Based
# ----------------------------------------------------------------------------------------------------------------------
class Physics_Based_Frequency_Domain(Aeroacoustics):
    """This is an acoustic analysis based on a collection of frequency domain methods

     Assumptions:

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

        """ This sets the default values for the analysis.

            Assumptions:
            Ground microphone angles start in front of the aircraft (0 deg) and sweep in a lateral direction
            to the starboard wing and around to the tail (180 deg)

            Source:
            N/A

            Inputs:
            None

            Output:
            None

            Properties Used:
            N/A
        """

        # Initialize quantities
        self.tag                                             =  "Physics_Based"
        self.method                                          =  "Frequency_Domain_Buildup"
        self.settings.fidelity                               = 'line_source'
        self.settings.use_plane_loading_surrogate            =  True
        self.settings.wing_wake_interactional_dB_adjustment  =  15

    def evaluate_aeroacoustics(self,segment, vehicle):
        """ Process vehicle to setup vehicle, condititon and configuration

        Assumptions:
        None

        Source:
        N/4

        Inputs:
        self.settings.
            center_frequencies  - 1/3 octave band frequencies   [unitless]

        Outputs:
        None

        Properties Used:
        self.vehicle
        """
        evaluate_aeroacoustics(segment, self.settings, vehicle)
        return

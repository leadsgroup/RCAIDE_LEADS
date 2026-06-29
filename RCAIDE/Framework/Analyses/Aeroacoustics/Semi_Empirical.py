# RCAIDE/Framework/Analyses/Aeroacoustics/Semi_Empirical.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
# RCAIDE Imports
import  RCAIDE
#from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.compute_aircraft_noise              import airframe_noise
#from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.compute_engine_noise       import turbofan_engine_noise   
from RCAIDE.Library.Methods.Aeroacoustics.Common.decibel_arithmetic                           import SPL_arithmetic  
from RCAIDE.Library.Methods.Aeroacoustics.Common.generate_hemisphere_microphone_locations     import generate_hemisphere_microphone_locations  
from .Aeroacoustics      import Aeroacoustics

# package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Semi_Empirical
# ---------------------------------------------------------------------------------------------------------------------- 
class Semi_Empirical(Aeroacoustics): 
    """This is an acoustic analysis based on a collection of correlative modes

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
        self.tag                              = "Semi_Empirical"
        self.settings.noise_hemisphere_radius = 50
        
        return
            
    def evaluate_aeroacoustics(self,segment, vehicle):
        """ Process vehicle to setup geometry, condititon and configuration
    
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
        """         
    
        # unpack  
        settings      = self.settings     
        conditions    = segment.state.conditions  
        dim_cf        = len(settings.center_frequencies ) 
        ctrl_pts      = int(segment.state.numerics.number_of_control_points) 
         
        microphone_locations = generate_hemisphere_microphone_locations(settings)      
        N_hemisphere_mics    = len(microphone_locations)
        
        # create empty arrays for results      
        total_SPL_dBA        = np.ones((ctrl_pts,N_hemisphere_mics))*1E-16 
        total_SPL_spectra    = np.ones((ctrl_pts,N_hemisphere_mics,dim_cf))*1E-16

        conditions.aeroacoustics.hemisphere_SPL_dBA              = total_SPL_dBA *  (1 - settings.noise_reduction_factors.SPL_dbA)
        conditions.aeroacoustics.hemisphere_SPL_1_3_spectrum_dBA = total_SPL_spectra   *  (1 - settings.noise_reduction_factors.SPL_dbA)                                                    
        return   


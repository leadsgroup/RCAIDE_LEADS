# RCAIDE/Framework/Analyses/Noise/Correlation_Buildup.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
# RCAIDE Imports 
from RCAIDE.Library.Methods.Noise.Correlation_Buildup.Airframe.airframe_noise         import airframe_noise
from RCAIDE.Library.Methods.Noise.Correlation_Buildup.Turbofan.turbofan_engine_noise  import turbofan_engine_noise   
from RCAIDE.Library.Methods.Noise.Common.decibel_arithmetic                           import SPL_arithmetic  
from RCAIDE.Library.Methods.Noise.Common.generate_hemisphere_microphone_locations     import generate_hemisphere_microphone_locations  
from .Noise      import Noise   

# package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Correlation_Buildup
# ---------------------------------------------------------------------------------------------------------------------- 
class Correlation_Buildup(Noise): 
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
        self.tag =  "Correlation_Buildup"
        return
            
    def evaluate_noise(self,segment):
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
        config        = segment.analyses.noise.vehicle 
        settings      = self.settings     
        conditions    = segment.state.conditions  
        dim_cf        = len(settings.center_frequencies ) 
        ctrl_pts      = int(segment.state.numerics.number_of_control_points) 
         
        microphone_locations = generate_hemisphere_microphone_locations(settings)      
        N_hemisphere_mics    = len(microphone_locations)
        
        # create empty arrays for results      
        total_SPL_dBA        = np.ones((ctrl_pts,N_hemisphere_mics))*1E-16 
        total_SPL_spectra    = np.ones((ctrl_pts,N_hemisphere_mics,dim_cf))*1E-16
    
        # flap noise - only applicable for turbofan aircraft
        if 'flap' in config.wings.main_wing.control_surfaces:    
            airframe_noise_res        = airframe_noise(microphone_locations,segment,config,settings) 
            total_SPL_dBA             = SPL_arithmetic(np.concatenate((total_SPL_dBA[:,None,:],airframe_noise_res.SPL_dBA[:,None,:]),axis =1),sum_axis=1)
            total_SPL_spectra[:,:,5:] = SPL_arithmetic(np.concatenate((total_SPL_spectra[:,None,:,5:],airframe_noise_res.SPL_1_3_spectrum[:,None,:,:]),axis =1),sum_axis=1)
              
          # iterate through sources  
        for network in config.networks:  
            for propulsor in network.propulsors:        
                engine_noise                = turbofan_engine_noise(microphone_locations,propulsor,conditions.noise.propulsors[propulsor.tag],segment,settings)    
                total_SPL_dBA               = SPL_arithmetic(np.concatenate((total_SPL_dBA[:,None,:],engine_noise.SPL_dBA[:,None,:]),axis =1),sum_axis=1)
                total_SPL_spectra[:,:,5:]   = SPL_arithmetic(np.concatenate((total_SPL_spectra[:,None,:,5:],engine_noise.SPL_1_3_spectrum[:,None,:,:]),axis =1),sum_axis=1)
                     
        conditions.noise.hemisphere_SPL_dBA              = total_SPL_dBA
        conditions.noise.hemisphere_SPL_1_3_spectrum_dBA = total_SPL_spectra                                                      
        return   


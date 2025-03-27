# RCAIDE/Framework/Analyses/Noise/Noise.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
from RCAIDE.Framework.Core import Data
from RCAIDE.Framework.Analyses import Analysis

# package imports
import numpy as np 
# ----------------------------------------------------------------------
#  Noise
# ----------------------------------------------------------------------
class Noise(Analysis):
    """ RCAIDE.Framework.Analyses.Noise.Noise()
    
        The Top Level Noise Analysis Class
        
            Assumptions:
            None
            
            Source:
            N/A
    """
    def __defaults__(self):
        """This sets the default values and methods for the analysis.
    
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
        self.tag                                             = 'Noise'        
                                         
        self.vehicle                                         = Data()
        self.settings                                        = Data()

        self.settings                                        = self.settings
        self.settings.harmonics                              = np.arange(1,30) 
        self.settings.flyover                                = False    
        self.settings.approach                               = False
        self.settings.sideline                               = False
        self.settings.sideline_x_position                    = 0 
        self.settings.print_noise_output                     = False  
        self.settings.mean_sea_level_altitude                = True 
        self.settings.aircraft_origin_location               = np.array([0,0,0])
        self.settings.aircraft_destination_location          = np.array([0,0,0])
        self.settings.aircraft_origin_coordinates            = np.array([0.0,0.0])
        self.settings.aircraft_destination_coordinates       = np.array([0.0,0.0])
        self.settings.noise_reduction_factors                = Data()
        self.settings.noise_reduction_factors.SPL_dbA        = 0.0     # Reduction factors are proportional (.1 is a 10% weight reduction)
        self.settings.topography_file                        = None
        self.settings.microphone_locations                   = None   
        self.settings.microphone_coordinates                 = None
        self.settings.microphone_x_resolution                = 11 
        self.settings.microphone_y_resolution                = 11 
        self.settings.noise_times_steps                      = 101 
        self.settings.number_of_microphone_in_stencil        = 10
        self.settings.microphone_min_x                       = 0  
        self.settings.microphone_max_x                       = 1000 
        self.settings.microphone_min_y                       = -100  
        self.settings.microphone_max_y                       = 100  
        
        self.settings.noise_hemisphere                       = False 
        self.settings.noise_hemisphere_radius                = 20  
        epsilon                                              = 1E-5
        self.settings.noise_hemisphere_phi_angles            = np.linspace(epsilon + (np.pi/2),   np.pi-epsilon,6)   
        self.settings.noise_hemisphere_theta_angles          = np.linspace(epsilon + 0        ,   2*np.pi-epsilon,12)  
                
        # settings for acoustic frequency resolution
        self.settings.center_frequencies                     = np.array([16,20,25,31.5,40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, \
                                                                    500, 630, 800, 1000, 1250, 1600, 2000, 2500, 3150,
                                                                    4000, 5000, 6300, 8000, 10000])        
        self.settings.lower_frequencies                      = np.array([14,18,22.4,28,35.5,45,56,71,90,112,140,180,224,280,355,450,560,710,\
                                                                    900,1120,1400,1800,2240,2800,3550,4500,5600,7100,9000 ])
        self.settings.upper_frequencies                      = np.array([18,22.4,28,35.5,45,56,71,90,112,140,180,224,280,355,450,560,710,900,1120,\
                                                                 1400,1800,2240,2800,3550,4500,5600,7100,9000,11200 ])
        
        return
    
        
        
    def evaluate(self,state):
        """The default evaluate function.

        Assumptions:
        None

        Source:
        N/A

        Inputs:
        None

        Outputs:
        results   <Results class> (empty)

        Properties Used:
        N/A
        """           
        
        results = Data()
        
        return results 
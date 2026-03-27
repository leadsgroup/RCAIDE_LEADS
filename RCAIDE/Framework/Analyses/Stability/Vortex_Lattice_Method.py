# RCAIDE/Framework/Analyses/Stability/Vortex_Lattice_Method.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports   
from RCAIDE.Framework.Core                                     import Data, Units
from RCAIDE.Framework.Analyses                                 import Process 
from RCAIDE.Library.Methods.Stability                          import Common
from .Stability                                                import Stability     
from RCAIDE.Library.Methods.Stability.Vortex_Lattice_Method    import *  
import  numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Vortex_Lattice_Method
# ---------------------------------------------------------------------------------------------------------------------- 
class Vortex_Lattice_Method(Stability):
    """This is a subsonic aerodynamic buildup analysis based on the vortex lattice method

     Assumptions:
     Stall effects are negligible 
 
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
        self.tag                                                    = 'Vortex_Lattice_Method'  
        self.vehicle                                                = Data()  
        self.process                                                = Process()
        self.process.initialize                                     = Process()  
                   
        # correction factors  
        self.settings.use_surrogate                                 = True  
        self.settings.propeller_wake_model                          = False  
        self.settings.model_fuselage                                = False    
        self.settings.number_of_spanwise_vortices                   = 15
        self.settings.number_of_chordwise_vortices                  = 5
        self.settings.wing_spanwise_vortices                        = None
        self.settings.wing_chordwise_vortices                       = None
        self.settings.fuselage_spanwise_vortices                    = None
        self.settings.fuselage_chordwise_vortices                   = None  
        self.settings.spanwise_cosine_spacing                       = True
        self.settings.vortex_distribution                           = Data()
        self.settings.leading_edge_suction_multiplier               = 1.0  
        self.settings.use_VORLAX_matrix_calculation                 = False
        self.settings.floating_point_precision                      = np.float32 
        
        # conditions table, used for surrogate model training
        self.training                                               = Data()  
        self.training.angle_of_attack                               = np.array([-5., -2. , 1E-20 , 2.0, 5.0, 8.0, 12., 45., 75.]) * Units.deg 
        self.training.Mach                                          = np.array([0.1  ,0.3,  0.5,  0.65 , 0.85 , 0.9, 1.3, 1.35 , 1.5 , 2.0, 2.25 , 2.5  , 3.5])             
                      
        self.training.subsonic                                      = None
        self.training.supersonic                                    = None
        self.training.transonic                                     = None  
                         
        # blending function                  
        self.hsub_min                                               = 0.85
        self.hsub_max                                               = 0.95
        self.hsup_min                                               = 1.05
        self.hsup_max                                               = 1.15 
                     
        # surrogoate models                 
        self.surrogates                                             = Data()  

        # build the evaluation process
        compute                                                     = Process() 
        compute.static_stability                                    = evaluate
        compute.dynamic_stability                                   = Common.compute_dynamic_flight_modes    
        self.process.compute                                        = compute 

    def initialize(self, vehicle): 
        
        # compute neutral point 
        if self.settings.compute_neutral_point:
            print("Computing neutral point (This might take a while)")
            compute_neutral_point(self, vehicle)  
        elif vehicle.neutral_point == None:
            raise AttributeError('Neutral point not defined.')
        return 
    
         
    def evaluate(self,state, vehicle):
        """The default evaluate function.

        Assumptions:
        None

        Source:
        N/A

        Inputs:
        None

        Outputs:
        results   <RCAIDE data class>

        Properties Used:
        self.settings
        self.vehicle
        """          
        settings = self.settings 
        results  = self.process.compute(state,settings,vehicle)
        
        return results
    
     
 
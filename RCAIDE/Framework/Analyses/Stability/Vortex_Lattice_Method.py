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
        self.training.angle_purtubation                             = 10 * Units.deg          
        self.training.center_of_gravity_purtubation                 = 0.1   
                         
        # blending function                  
        self.hsub_min                                               = 0.85
        self.hsub_max                                               = 0.95
        self.hsup_min                                               = 1.05
        self.hsup_max                                               = 1.15 
                     
        # surrogoate models                 
        self.surrogates                                             = Data()  

        # build the evaluation process
        compute                                                     = Process() 
        compute.static_stability                                    = None 
        compute.dynamic_stability                                   = Common.compute_dynamic_flight_modes    
        self.process.compute                                        = compute 

    def initialize(self):  
        use_surrogate   = self.settings.use_surrogate   
    
        # build the evaluation process
        compute   =  self.process.compute                  
        if use_surrogate == True: 
            compute.static_stability  = evaluate_surrogate
        else:
            compute.static_stability  = evaluate_no_surrogate  
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
        results   <RCAIDE data class>

        Properties Used:
        self.settings
        self.vehicle
        """          
        settings = self.settings
        vehicle  = self.vehicle 
        results  = self.process.compute(state,settings,vehicle)
        
        return results
    
     
 
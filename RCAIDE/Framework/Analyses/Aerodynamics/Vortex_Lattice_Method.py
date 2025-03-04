# RCAIDE/Framework/Analyses/Aerodynamics/Vortex_Lattice_Method.py
#  
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports   
from RCAIDE.Framework.Core                             import Data, Units
from RCAIDE.Framework.Analyses                         import Process 
from RCAIDE.Library.Methods.Aerodynamics               import Common
from .Aerodynamics                                     import Aerodynamics 
from RCAIDE.Framework.Analyses.Common.Process_Geometry import Process_Geometry 
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method import *   

# package imports 
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  Vortex_Lattice_Method
# ---------------------------------------------------------------------------------------------------------------------- 
class Vortex_Lattice_Method(Aerodynamics):
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
        self.tag                                                         = 'Vortex_Lattice_Method'  
        self.vehicle                                                     = Data()  
        self.process                                                     = Process()
        self.process.initialize                                          = Process()  
                   
        # correction factors            
        self.settings.fuselage_lift_correction                           = 1.14
        self.settings.trim_drag_correction_factor                        = 1.0
        self.settings.wing_parasite_drag_form_factor                     = 1.1
        self.settings.fuselage_parasite_drag_form_factor                 = 2.3
        self.settings.maximum_lift_coefficient_factor                    = 1.0        
        self.settings.lift_to_drag_adjustment                            = 0.0  
        self.settings.oswald_efficiency_factor                           = None
        self.settings.span_efficiency                                    = None
        self.settings.viscous_lift_dependent_drag_factor                 = 0.38
        self.settings.drag_coefficient_increment                         = 0.0
        self.settings.spoiler_drag_increment                             = 0.0
        self.settings.maximum_lift_coefficient                           = np.inf 
        self.settings.use_surrogate                                      = True
        self.settings.recalculate_total_wetted_area                      = False
        self.settings.propeller_wake_model                               = False 
        self.settings.discretize_control_surfaces                        = True
        self.settings.model_fuselage                                     = False
        self.settings.trim_aircraft                                      = True
        self.settings.aileron_flag                                       = False
        self.settings.rudder_flag                                        = False
        self.settings.flap_flag                                          = False
        self.settings.elevator_flag                                      = False
        self.settings.slat_flag                                          = False             

        # correction factors
        self.settings.supersonic                                         = Data()
        self.settings.supersonic.peak_mach_number                        = 1.04  
        self.settings.supersonic.begin_drag_rise_mach_number             = 0.95
        self.settings.supersonic.end_drag_rise_mach_number               = 1.2
        self.settings.supersonic.transonic_drag_multiplier               = 1.25  
        self.settings.supersonic.volume_wave_drag_scaling                = 3.2  
        self.settings.supersonic.fuselage_parasite_drag_begin_blend_mach = 0.91
        self.settings.supersonic.fuselage_parasite_drag_end_blend_mach   = 0.99    
        self.settings.supersonic.cross_sectional_area_calculation_type   = 'Fixed'     
        self.settings.supersonic.wave_drag_type                          = 'Raymer'    
    
        self.settings.number_of_spanwise_vortices                        = 15
        self.settings.number_of_chordwise_vortices                       = 5
        self.settings.wing_spanwise_vortices                             = None
        self.settings.wing_chordwise_vortices                            = None
        self.settings.fuselage_spanwise_vortices                         = None
        self.settings.fuselage_chordwise_vortices                        = None  
        self.settings.spanwise_cosine_spacing                            = True
        self.settings.vortex_distribution                                = Data()  
        self.settings.leading_edge_suction_multiplier                    = 1.0  
        self.settings.use_VORLAX_matrix_calculation                      = False
        self.settings.floating_point_precision                           = np.float32     
    
        # conditions table, used for surrogate model training
        self.training                                               = Data()
        self.training.angle_of_attack                               = np.array([-5., -2. , 1E-20 , 2.0, 5.0, 8.0, 12., 45., 75.]) * Units.deg 
        self.training.Mach                                          = np.array([0.1  ,0.3,  0.5,  0.65 , 0.85 , 0.9, 1.3, 1.35 , 1.5 , 2.0, 2.25 , 2.5  , 3.5])             
                      
        self.training.subsonic                                      = None
        self.training.supersonic                                    = None
        self.training.transonic                                     = None

        self.training.altitude                                      = 0
        self.training.speed_of_sound                                = 343 
        self.training.angle_purtubation                             = 10 * Units.deg          
        self.training.speed_purtubation                             = 5  
        self.training.rate_purtubation                              = 10 * Units.deg / Units.sec   
        self.training.control_surface_purtubation                   = 10 * Units.deg   
        self.training.sideslip_angle                                = np.array([10  , 5.0 ]) * Units.deg
        self.training.aileron_deflection                            = np.array([10  , 5.0 ]) * Units.deg
        self.training.elevator_deflection                           = np.array([10  , 5.0 ]) * Units.deg   
        self.training.rudder_deflection                             = np.array([10  , 1E-3 ]) * Units.deg
        self.training.flap_deflection                               = np.array([10  , 1E-3 ]) * Units.deg 
        self.training.slat_deflection                               = np.array([10  , 1E-3 ]) * Units.deg                      
        self.training.u                                             = np.array([10 , 5 ])  
        self.training.v                                             = np.array([10 , 5 ])  
        self.training.w                                             = np.array([10 , 5 ])    
        self.training.pitch_rate                                    = np.array([3 ,1.5 ])  * Units.deg / Units.sec
        self.training.roll_rate                                     = np.array([3 ,1.5 ])  * Units.deg / Units.sec
        self.training.yaw_rate                                      = np.array([3 ,1.5 ])  * Units.deg / Units.sec
    
        self.reference_values                                       = Data()
        self.reference_values.S_ref                                 = 0
        self.reference_values.c_ref                                 = 0
        self.reference_values.b_ref                                 = 0
        self.reference_values.X_ref                                 = 0
        self.reference_values.Y_ref                                 = 0
        self.reference_values.Z_ref                                 = 0
        
        # control surface flags                  
        self.aileron_flag                                           = False 
        self.flap_flag                                              = False 
        self.rudder_flag                                            = False 
        self.elevator_flag                                          = False 
        self.slat_flag                                              = False
                         
        # blending function                  
        self.hsub_min                                               = 0.85
        self.hsub_max                                               = 0.95
        self.hsup_min                                               = 1.05
        self.hsup_max                                               = 1.15 
                     
        # surrogoate models                 
        self.surrogates                                             = Data() 

        # build the evaluation process
        compute                                    = Process() 
        compute.lift                               = Process() 
        compute.lift.inviscid_wings                = None 
        compute.lift.fuselage                      = Common.Lift.fuselage_correction 
        compute.drag                               = Process()
        compute.drag.parasite                      = Process()
        compute.drag.parasite.wings                = Process_Geometry('wings')
        compute.drag.parasite.wings.wing           = Common.Drag.parasite_drag_wing 
        compute.drag.parasite.fuselages            = Process_Geometry('fuselages')
        compute.drag.parasite.fuselages.fuselage   = Common.Drag.parasite_drag_fuselage
        compute.drag.parasite.booms                = Process_Geometry('booms')
        compute.drag.parasite.booms.boom           = Common.Drag.parasite_drag_fuselage 
        compute.drag.parasite.nacelles             = Common.Drag.parasite_drag_nacelle
        compute.drag.parasite.pylons               = Common.Drag.parasite_drag_pylon
        compute.drag.parasite.total                = Common.Drag.parasite_total
        compute.drag.induced                       = Common.Drag.induced_drag
        compute.drag.cooling                       = Process()
        compute.drag.cooling.total                 = Common.Drag.cooling_drag        
        compute.drag.compressibility               = Process() 
        compute.drag.compressibility.total         = Common.Drag.compressibility_drag
        compute.drag.miscellaneous                 = Common.Drag.miscellaneous_drag 
        compute.drag.spoiler                       = Common.Drag.spoiler_drag
        compute.drag.total                         = Common.Drag.total_drag
        compute.stability                          = Process()
        compute.stability.dynamic_modes            = RCAIDE.Library.Methods.Stability.compute_dynamic_flight_modes  
        self.process.compute                       = compute
        

    def initialize(self):  
        use_surrogate   = self.settings.use_surrogate  

        # If we are using the surrogate
        if use_surrogate == True: 
            # sample training data
            train_VLM_surrogates(self)

            # build surrogate
            build_VLM_surrogates(self)  
    
        # build the evaluation process
        compute   =  self.process.compute                  
        if use_surrogate == True: 
            compute.lift.inviscid_wings  = evaluate_surrogate
        else:
            compute.lift.inviscid_wings  = evaluate_no_surrogate
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
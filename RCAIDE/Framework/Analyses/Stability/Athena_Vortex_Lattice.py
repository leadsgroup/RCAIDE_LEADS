# RCAIDE/Framework/Analyses/Stability/Athena_Vortex_Lattice.py 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports   
from RCAIDE.Framework.Core                                     import Data, Units
from RCAIDE.Framework.Analyses                                 import Process 
from RCAIDE.Library.Methods.Aerodynamics                       import Common
from .Stability                                                import Stability  
from RCAIDE.Framework.Analyses.Common.Process_Geometry         import Process_Geometry 
from RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice import *   
from RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.AVL_Objects.Run_Case     import Run_Case

# package imports 
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  Athena_Vortex_Lattice
# ----------------------------------------------------------------------------------------------------------------------
class Athena_Vortex_Lattice(Stability):
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
        
        self.tag                                                    = 'Athena_Vortex_Lattice'  
        self.vehicle                                                = Data()  
        self.process                                                = Process()
        self.process.initialize                                     = Process()  
                   
        # correction factors           
        settings                                                     = self.settings 
        settings.fuselage_lift_correction                            = 1.14
        settings.trim_drag_correction_factor                         = 1.1
        settings.wing_parasite_drag_form_factor                      = 1.2
        settings.fuselage_parasite_drag_form_factor                  = 2.4  
        settings.drag_reduction_factors                              = Data()
        settings.drag_reduction_factors.parasite_drag                = 0.0  # Reduction factors are proportional (.1 is a 10% weight reduction)
        settings.drag_reduction_factors.induced_drag                 = 0.0  # Reduction factors are proportional (.1 is a 10% weight reduction)
        settings.drag_reduction_factors.compressibility_drag         = 0.0  # Reduction factors are proportional (.1 is a 10% weight reduction) 
        settings.maximum_lift_coefficient_factor                     = 1.0        
        settings.lift_to_drag_adjustment                             = 0.0   
        settings.viscous_lift_dependent_drag_factor                  = 0.38
        settings.drag_coefficient_increment                          = 0.0
        settings.spoiler_drag_increment                              = 0.0
        settings.maximum_lift_coefficient                            = np.inf  
        settings.recalculate_total_wetted_area                       = False
        settings.oswald_efficiency_factor                            = None
        settings.span_efficiency                                     = None
        
        settings.run_cases                                           = Run_Case.Container()
        
        settings.flow_symmetry                                       = Data()
        settings.flow_symmetry.xz_plane                              = 0    # Symmetry across the xz-plane, y=0
        settings.flow_symmetry.xy_parallel                           = 0    # Symmetry across the z=z_symmetry_plane plane
        settings.flow_symmetry.z_symmetry_plane                      = 0.0
         
        settings.number_of_control_surfaces                          = 0
        
        settings.filenames                                           = Data()
        settings.filenames.avl_bin_name                              = 'avl' # to call avl from command line. If avl is not on the system path, include absolute path to the avl binary i.e. '/your/path/to/avl'
        settings.filenames.run_folder                                = 'avl_files'  
        settings.filenames.features                                  = 'aircraft.avl'
        settings.filenames.mass_file                                 = 'aircraft.mass'
        settings.filenames.batch_template                            = 'batch_{0:04d}.run'
        settings.filenames.deck_template                             = 'commands_{0:04d}.deck' 
        settings.filenames.aero_output_template_1                    = 'stability_axis_derivatives_{}.txt' 
        settings.filenames.aero_output_template_2                    = 'surface_forces_{}.txt'
        settings.filenames.aero_output_template_3                    = 'strip_forces_{}.txt'
        settings.filenames.aero_output_template_4                    = 'body_axis_derivatives_{}.txt'
        settings.filenames.dynamic_output_template_1                 = 'eigen_mode_{}.txt'
        settings.filenames.dynamic_output_template_2                 = 'system_matrix_{}.txt'
        settings.filenames.case_template                             = 'case_{0:04d}_{1:04d}'
        settings.filenames.log_filename                              = 'avl_log.txt'
        settings.filenames.err_filename                              = 'avl_err.txt'   
        settings.number_of_spanwise_vortices                         = 20
        settings.number_of_chordwise_vortices                        = 10
        settings.use_surrogate                                       = True 
        settings.trim_aircraft                                       = False
        settings.model_fuselage                                      = False 
        settings.print_output                                        = False 
        settings.keep_files                                          = False  
        settings.new_regression_results                              = False  
        settings.side_slip_angle                                     = 0.0
        settings.roll_rate_coefficient                               = 0.0
        settings.pitch_rate_coefficient                              = 0.0 
        settings.lift_coefficient                                    = None 

        # correction factors
        settings.supersonic                                         = Data()
        settings.supersonic.peak_mach_number                        = 1.04  
        settings.supersonic.begin_drag_rise_mach_number             = 0.95
        settings.supersonic.end_drag_rise_mach_number               = 1.2
        settings.supersonic.transonic_drag_multiplier               = 1.25  
        settings.supersonic.volume_wave_drag_scaling                = 3.2  
        settings.supersonic.fuselage_parasite_drag_begin_blend_mach = 0.91
        settings.supersonic.fuselage_parasite_drag_end_blend_mach   = 0.99    
        settings.supersonic.cross_sectional_area_calculation_type   = 'Fixed'     
        settings.supersonic.wave_drag_type                          = 'Raymer'

        self.reference_values                       = Data()
        self.reference_values.S_ref                 = 0
        self.reference_values.c_ref                 = 0
        self.reference_values.b_ref                 = 0
        self.reference_values.X_ref                 = 0
        self.reference_values.Y_ref                 = 0
        self.reference_values.Z_ref                 = 0          
    
        # conditions table, used for surrogate model training
        self.training                                               = Data()
        self.training.angle_of_attack               = np.array([-2.,0., 2.,5., 7., 10.])*Units.degrees
        self.training.Mach                          = np.array([0.05,0.15,0.25, 0.45,0.65,0.85]) 
        self.training.lift_coefficient              = None
        self.training.drag_coefficient              = None
        self.training.span_efficiency_factor        = None
        self.training.moment_coefficient            = None
        self.training.Cm_alpha_moment_coefficient   = None
        self.training.Cn_beta_moment_coefficient    = None
        self.training.neutral_point                 = None
        self.training_file                          = None 

        self.current_status                         = Data()        
        self.current_status.batch_index             = 0
        self.current_status.batch_file              = None
        self.current_status.deck_file               = None
        self.current_status.cases                   = None 
        
        # surrogoate models                 
        self.surrogates                             = Data()
        self.surrogates.moment_coefficient          = None
        self.surrogates.Cm_alpha_moment_coefficient = None
        self.surrogates.Cn_beta_moment_coefficient  = None      
        self.surrogates.neutral_point               = None  

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
        self.process.compute                       = compute
         

    def initialize(self):  
        use_surrogate   = self.settings.use_surrogate  

        # If we are using the surrogate
        if use_surrogate == True: 
            # sample training data
            train_AVL_surrogates(self)

            # build surrogate
            build_AVL_surrogates(self)  
    
        # build the evaluation process
        compute   =  self.process.compute                  
        if use_surrogate == True: 
            compute.lift.inviscid_wings  = evaluate_AVL_surrogate
        else:
            compute.lift.inviscid_wings  = evaluate_AVL_no_surrogate  
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
    
     
 
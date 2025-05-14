 
from RCAIDE.Framework.Core import Data
from RCAIDE.Framework.Analyses import Analysis


# ----------------------------------------------------------------------
#  Analysis
# ---------------------------------------------------------------------- 
class Stability(Analysis):
    """This is the base class for stability analyses. It contains functions
    that are built into the default class.
    
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
        self.tag      = 'stability'
        self.vehicle  = Data()
        self.settings = Data()
        
    
        self.reference_values                                            = Data()
        self.reference_values.S_ref                                      = 0
        self.reference_values.c_ref                                      = 0
        self.reference_values.b_ref                                      = 0
        self.reference_values.X_ref                                      = 0
        self.reference_values.Y_ref                                      = 0
        self.reference_values.Z_ref                                      = 0
        
        self.settings                                                    = Data()
        self.settings.unique_segment_surrogate                           = False
        self.settings.maximum_lift_coefficient                           = np.inf 
        self.settings.fuselage_lift_correction                           = 1.14
        self.settings.trim_drag_correction_factor                        = 1.05
        self.settings.wing_parasite_drag_form_factor                     = 1.2 
        self.settings.pylon_parasite_drag_factor                         = 0.0
        self.settings.fuselage_parasite_drag_form_factor                 = 2.3  

        self.settings.drag_reduction_factors                             = Data()
        self.settings.drag_reduction_factors.parasite_drag               = 0.0  # Reduction factors are proportional (.1 is a 10% weight reduction)
        self.settings.drag_reduction_factors.induced_drag                = 0.0  # Reduction factors are proportional (.1 is a 10% weight reduction)
        self.settings.drag_reduction_factors.compressibility_drag        = 0.0  # Reduction factors are proportional (.1 is a 10% weight reduction) 
        self.settings.maximum_lift_coefficient_factor                    = 1.0        
        self.settings.lift_to_drag_adjustment                            = 0.0   
        self.settings.viscous_lift_dependent_drag_factor                 = 0.38
        self.settings.drag_coefficient_increment                         = 0.0
        self.settings.spoiler_drag_increment                             = 0.0
        self.settings.maximum_lift_coefficient                           = np.inf  
        self.settings.recalculate_total_wetted_area                      = False
        self.settings.oswald_efficiency_factor                           = None
        self.settings.span_efficiency                                    = None

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
    
        # Stability derivatives. If these are not user defined, then they will be calculated. 
        self.stability_derivatives                                       = Data()   
        self.stability_derivatives.M_0                                   = None
        self.stability_derivatives.Clift_alpha                           = None
        self.stability_derivatives.CX_alpha                              = None
        self.stability_derivatives.CX_u                                  = None
        self.stability_derivatives.CY_beta                               = None
        self.stability_derivatives.CY_r                                  = None
        self.stability_derivatives.CZ_alpha                              = None
        self.stability_derivatives.CZ_u                                  = None
        self.stability_derivatives.CZ_q                                  = None
        self.stability_derivatives.CL_beta                               = None
        self.stability_derivatives.CL_p                                  = None
        self.stability_derivatives.CL_r                                  = None
        self.stability_derivatives.CM_alpha                              = None
        self.stability_derivatives.CM_u                                  = None
        self.stability_derivatives.CM_q                                  = None
        self.stability_derivatives.CN_beta                               = None
        self.stability_derivatives.CN_p                                  = None
        self.stability_derivatives.CN_r                                  = None
        self.stability_derivatives.CY_delta_a                            = None
        self.stability_derivatives.CL_delta_a                            = None
        self.stability_derivatives.CN_delta_a                            = None
        self.stability_derivatives.CM_delta_e                            = None
        self.stability_derivatives.Clift_delta_e                         = None
        self.stability_derivatives.CY_delta_r                            = None
        self.stability_derivatives.CL_delta_r                            = None
        self.stability_derivatives.CN_delta_r                            = None
        self.stability_derivatives.CM_delta_f                            = None
        self.stability_derivatives.Clift_delta_f                         = None      
 
        return



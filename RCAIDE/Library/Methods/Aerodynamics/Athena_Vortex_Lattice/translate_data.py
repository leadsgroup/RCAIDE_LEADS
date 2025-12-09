# RCAIDE/Library/Methods/Aerodynamics/Athena_Vortex_Lattice/translate_data.py
#
# Created: Oct 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
from RCAIDE.Framework.Core import Units
from .AVL_Objects.Run_Case import Run_Case

# package imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  translate_data
# ---------------------------------------------------------------------------------------------------------------------- 
def translate_conditions_to_cases(avl ,conditions, vehicle):
    """ Takes RCAIDE Conditions() data structure and translates to a Container of
    avl Run_Case()s.

    Assumptions:
        None
        
    Source:
        Drela, M. and Youngren, H., AVL, http://web.mit.edu/drela/Public/web/avl

    Inputs:
        conditions.aerodynamics.angles.alpha  [radians] 
        conditions.freestream.mach_number        [-]
        conditions.freestream.density            [kilograms per meters**3]
        conditions.freestream.gravity            [meters per second**2]

    Outputs:
        cases                                    [data structur]

    Properties Used:
        N/A
    """    
    # set up aerodynamic Conditions object
    cases    = Run_Case.Container()
    for i in range(len(conditions.aerodynamics.angles.alpha)):      
        case                                                  = Run_Case()
        case.tag                                              = avl.settings.filenames.case_template.format(avl.current_status.batch_index,i+1)
        case.mass                                             = conditions.weights.total_mass
        case.conditions.freestream.mach                       = conditions.freestream.mach_number[i, 0]
        case.conditions.freestream.density                    = conditions.freestream.density[i, 0]
        case.conditions.freestream.gravitational_acceleration = conditions.freestream.gravity[i, 0]      
        case.conditions.aerodynamics.angles.alpha             = conditions.aerodynamics.angles.alpha[i, 0]/Units.deg
        case.conditions.aerodynamics.angles.beta              = conditions.aerodynamics.angles.beta[i, 0]/Units.deg 
        if type(conditions.aerodynamics.coefficients.lift.total) == np.ndarray: 
            case.conditions.aerodynamics.coefficients.lift.total= conditions.aerodynamics.coefficients.lift.total[i, 0]        
        else:      
            case.conditions.aerodynamics.coefficients.lift.total = None
        case.conditions.static_stability.coefficients.roll    = conditions.static_stability.coefficients.roll[i, 0] 
        case.conditions.static_stability.coefficients.pitch   = conditions.static_stability.coefficients.pitch[i, 0] 
        
        # determine the number of wings 
        n_wings = 0 
        for wing in vehicle.wings:
            n_wings += 1
            if wing.xz_plane_symmetric == True:
                n_wings += 1                
        case.num_wings                                        = n_wings
        case.n_sw                                             = avl.settings.number_of_spanwise_vortices  
                
        cases.append_case(case)
    
    return cases

def translate_results_to_conditions(cases,res,results):
    """ Takes avl results structure containing the results of each run case stored
        each in its own Data() object. Translates into the Conditions() data structure.

    Assumptions:
        None
        
    Source:
        Drela, M. and Youngren, H., AVL, http://web.mit.edu/drela/Public/web/avl

    Inputs:
        case_res = results 
            
     Outputs:
        cases                        [data_structure]
   
    Properties Used:
        N/A
    """   
   
    num_wings = cases[0].num_wings 
    n_sw      = cases[0].n_sw
    dim       = len(cases)
         
    # aero results 1: total surface forces and coefficeints
    res.aerodynamics.coefficients.lift.inviscid.total = np.zeros((dim,1))
    res.aerodynamics.wing_areas                       = np.zeros((dim,num_wings)) 
    res.aerodynamics.wing_CLs                         = np.zeros_like(res.aerodynamics.wing_areas) 
    res.aerodynamics.wing_CDs                         = np.zeros_like(res.aerodynamics.wing_areas) 

    # aero results 2 : sectional forces and coefficients 
    res.aerodynamics.wing_local_spans              = np.zeros((dim,num_wings,n_sw))
    res.aerodynamics.wing_section_chords           = np.zeros_like(res.aerodynamics.wing_local_spans)
    res.aerodynamics.wing_section_cls              = np.zeros_like(res.aerodynamics.wing_local_spans)
    res.aerodynamics.wing_section_induced_angle    = np.zeros_like(res.aerodynamics.wing_local_spans)
    res.aerodynamics.wing_section_cds              = np.zeros_like(res.aerodynamics.wing_local_spans) 
    res.static_stability.control_surfaces_cases   = {}
    
    mach_case = list(results.keys())[0][5:9]   
    for i in range(len(results.keys())):
        aoa_case = '{:04d}'.format(i+1)
        tag = 'case_' + mach_case + '_' + aoa_case
        case_res = results[tag]       
        
        # stability file       
        res.aerodynamics.angles.alpha[i][0]                                 = case_res.aerodynamics.AoA * Units.degree
        res.aerodynamics.angles.beta[i][0]                                  = case_res.aerodynamics.beta * Units.degree
        res.static_stability.coefficients.X[i][0]                           = case_res.aerodynamics.CX 
        res.static_stability.coefficients.Y[i][0]                           = case_res.aerodynamics.CY  
        res.static_stability.coefficients.Z[i][0]                           = case_res.aerodynamics.CZ    
        res.static_stability.coefficients.L[i][0]                           = case_res.aerodynamics.Cltot 
        res.static_stability.coefficients.M[i][0]                           = case_res.aerodynamics.Cmtot 
        res.static_stability.coefficients.N[i][0]                           = case_res.aerodynamics.Cntot      
        res.static_stability.coefficients.roll[i][0]                        = case_res.aerodynamics.roll_moment_coefficient
        res.static_stability.coefficients.pitch[i][0]                       = case_res.aerodynamics.pitch_moment_coefficient
        res.static_stability.coefficients.yaw[i][0]                         = case_res.aerodynamics.yaw_moment_coefficient
        res.aerodynamics.coefficients.lift.inviscid.total[i][0]             = case_res.aerodynamics.total_lift_coefficient
        res.aerodynamics.coefficients.drag.induced.inviscid[i][0]           = case_res.aerodynamics.induced_drag_coefficient 
        res.aerodynamics.coefficients.drag.induced.efficiency_factor[i][0]  = case_res.aerodynamics.oswald_efficiency 
        res.aerodynamics.oswald_efficiency[i][0]                            = case_res.aerodynamics.oswald_efficiency
        
        res.static_stability.derivatives.Clift_alpha[i][0]                  = case_res.stability.alpha_derivatives.lift_curve_slope
        res.static_stability.derivatives.CY_alpha[i][0]                     = case_res.stability.alpha_derivatives.side_force_derivative
        res.static_stability.derivatives.CL_alpha[i][0]                     = case_res.stability.alpha_derivatives.roll_moment_derivative
        res.static_stability.derivatives.CM_alpha[i][0]                     = case_res.stability.alpha_derivatives.pitch_moment_derivative
        res.static_stability.derivatives.CN_alpha[i][0]                     = case_res.stability.alpha_derivatives.yaw_moment_derivative
        res.static_stability.derivatives.Clift_beta[i][0]                   = case_res.stability.beta_derivatives.lift_coefficient_derivative
        res.static_stability.derivatives.CY_beta[i][0]                      = case_res.stability.beta_derivatives.side_force_derivative
        res.static_stability.derivatives.CL_beta[i][0]                      = case_res.stability.beta_derivatives.roll_moment_derivative
        res.static_stability.derivatives.CM_beta[i][0]                      = case_res.stability.beta_derivatives.pitch_moment_derivative
        res.static_stability.derivatives.CN_beta[i][0]                      = case_res.stability.beta_derivatives.yaw_moment_derivative        
    
        res.static_stability.derivatives.Clift_p[i][0]                      = case_res.stability.CL_p 
        res.static_stability.derivatives.Clift_q[i][0]                      = case_res.stability.CL_q
        res.static_stability.derivatives.Clift_r[i][0]                      = case_res.stability.CL_r 
        res.static_stability.derivatives.CY_p[i][0]                         = case_res.stability.CY_p 
        res.static_stability.derivatives.CY_q[i][0]                         = case_res.stability.CY_q 
        res.static_stability.derivatives.CY_r[i][0]                         = case_res.stability.CY_r
        res.static_stability.derivatives.CL_p[i][0]                         = case_res.stability.Cl_p 
        res.static_stability.derivatives.CL_q[i][0]                         = case_res.stability.Cl_q 
        res.static_stability.derivatives.CL_r[i][0]                         = case_res.stability.Cl_r 
        res.static_stability.derivatives.CM_p[i][0]                         = case_res.stability.Cm_p 
        res.static_stability.derivatives.CM_q[i][0]                         = case_res.stability.Cm_q 
        res.static_stability.derivatives.CM_r[i][0]                         = case_res.stability.Cm_r 
        res.static_stability.derivatives.CN_p[i][0]                         = case_res.stability.Cn_p 
        res.static_stability.derivatives.CN_q[i][0]                         = case_res.stability.Cn_q 
        res.static_stability.derivatives.CN_r[i][0]                         = case_res.stability.Cn_r
        res.static_stability.derivatives.CX_u[i][0]                         = case_res.stability.CX_u
        res.static_stability.derivatives.CX_v[i][0]                         = case_res.stability.CX_v
        res.static_stability.derivatives.CX_w[i][0]                         = case_res.stability.CX_w
        res.static_stability.derivatives.CY_u[i][0]                         = case_res.stability.CY_u
        res.static_stability.derivatives.CY_v[i][0]                         = case_res.stability.CY_v
        res.static_stability.derivatives.CY_w[i][0]                         = case_res.stability.CY_w
        res.static_stability.derivatives.CZ_u[i][0]                         = case_res.stability.CZ_u
        res.static_stability.derivatives.CZ_v[i][0]                         = case_res.stability.CZ_v
        res.static_stability.derivatives.CZ_w[i][0]                         = case_res.stability.CZ_w
        res.static_stability.derivatives.CL_u[i][0]                         = case_res.stability.Cl_u
        res.static_stability.derivatives.CL_v[i][0]                         = case_res.stability.Cl_v
        res.static_stability.derivatives.CL_w[i][0]                         = case_res.stability.Cl_w
        res.static_stability.derivatives.CM_u[i][0]                         = case_res.stability.Cm_u
        res.static_stability.derivatives.CM_v[i][0]                         = case_res.stability.Cm_v
        res.static_stability.derivatives.CM_w[i][0]                         = case_res.stability.Cm_w
        res.static_stability.derivatives.CN_u[i][0]                         = case_res.stability.Cn_u
        res.static_stability.derivatives.CN_v[i][0]                         = case_res.stability.Cn_v
        res.static_stability.derivatives.CN_w[i][0]                         = case_res.stability.Cn_w
            
        res.static_stability.derivatives.CX_p[i][0]                         = case_res.stability.CX_p
        res.static_stability.derivatives.CX_q[i][0]                         = case_res.stability.CX_q
        res.static_stability.derivatives.CX_r[i][0]                         = case_res.stability.CX_r
        res.static_stability.derivatives.CY_p[i][0]                         = case_res.stability.CY_p
        res.static_stability.derivatives.CY_q[i][0]                         = case_res.stability.CY_q
        res.static_stability.derivatives.CY_r[i][0]                         = case_res.stability.CY_r
        res.static_stability.derivatives.CZ_p[i][0]                         = case_res.stability.CZ_p
        res.static_stability.derivatives.CZ_q[i][0]                         = case_res.stability.CZ_q
        res.static_stability.derivatives.CZ_r[i][0]                         = case_res.stability.CZ_r   
        res.static_stability.neutral_point[i][0]                            = case_res.stability.neutral_point
        res.static_stability.spiral_criteria[i][0]                          = case_res.stability.spiral_criteria
        
        # aero surface forces file 
        res.aerodynamics.wing_areas[i][:]                   = case_res.aerodynamics.wing_areas   
        res.aerodynamics.wing_CLs[i][:]                     = case_res.aerodynamics.wing_CLs    
        res.aerodynamics.wing_CDs[i][:]                     = case_res.aerodynamics.wing_CDs    
        
        # aero sectional forces file
        res.aerodynamics.wing_local_spans[i][:]             = case_res.aerodynamics.wing_local_spans
        res.aerodynamics.wing_section_chords[i][:]          = case_res.aerodynamics.wing_section_chords  
        res.aerodynamics.wing_section_cls[i][:]             = case_res.aerodynamics.wing_section_cls    
        res.aerodynamics.wing_section_induced_angle[i][:]   = case_res.aerodynamics.wing_section_aoa_i
        res.aerodynamics.wing_section_cds[i][:]             = case_res.aerodynamics.wing_section_cds   
        
        res.static_stability.control_surfaces_cases[tag]    = case_res.stability.control_surfaces
        
    if len(res.static_stability.coefficients.X) > 1:
        res.static_stability.derivatives.CX_alpha[:, 0] =  np.gradient( res.static_stability.coefficients.X[:, 0],res.aerodynamics.angles.alpha[:, 0] )
        res.static_stability.derivatives.CZ_alpha[:, 0] =  np.gradient( res.static_stability.coefficients.Z[:, 0],res.aerodynamics.angles.alpha[:, 0] )
    return  

# RCAIDE/Library/Methods/Stability/Vortex_Lattice_Method/evaluate_VLM.py
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports  
import RCAIDE 
from RCAIDE.Framework.Core                                               import Data 
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.VLM       import VLM
from RCAIDE.Library.Methods.Utilities                                    import Cubic_Spline_Blender  

# package imports
import numpy   as np
from copy      import  deepcopy 

# ----------------------------------------------------------------------------------------------------------------------
#  Vortex_Lattice
# ---------------------------------------------------------------------------------------------------------------------- 
def evaluate_surrogate(state,settings,vehicle):
    """Evaluates static margin and neutral point using built surrogates 
    
    Assumptions: 
        
    Source:
        None

    Args:
        stability    : VLM analysis  [unitless]
        state        : flight conditions     [unitless]
        settings     : VLM analysis settings [unitless]
        vehicle      : vehicle configuration [unitless] 
        
    Returns: 
        None  
    """          
    conditions    = state.conditions
    stability     = state.analyses.stability   
    delta_cg      = stability.training.center_of_gravity_purtubation  
    sub_sur       = stability.surrogates.subsonic
    sup_sur       = stability.surrogates.supersonic
    trans_sur     = stability.surrogates.transonic  
    hsub_min      = stability.hsub_min
    hsub_max      = stability.hsub_max
    hsup_min      = stability.hsup_min
    hsup_max      = stability.hsup_max
    AoA           = np.atleast_2d(conditions.aerodynamics.angles.alpha)    
    Mach          = np.atleast_2d(conditions.freestream.mach_number)   

    # Spline for Subsonic-to-Transonic-to-Supersonic Regimes
    sub_trans_spline = Cubic_Spline_Blender(hsub_min,hsub_max)
    h_sub            = lambda M:sub_trans_spline.compute(M)          
    sup_trans_spline = Cubic_Spline_Blender(hsup_max, hsup_min) 
    h_sup            = lambda M:sup_trans_spline.compute(M) 

    # --------------------------------------------------------------------------------------------    
    # Vehicle Properties  
    # --------------------------------------------------------------------------------------------  
    CG            = vehicle.mass_properties.center_of_gravity[0][0]
    c_ref         = vehicle.reference_chord

    # --------------------------------------------------------------------------------------------            
    # CM at 0 aoa 
    # --------------------------------------------------------------------------------------------  
    CM_0        = conditions.static_stability.coefficients.M_0

    # --------------------------------------------------------------------------------------------         
    # Alpha Purtubation       
    # --------------------------------------------------------------------------------------------  
    delta_angle       = AoA  # let the AoA be the shift in AoA so that the previously comuted CM is the response 
    CM_alpha_prime    = conditions.static_stability.coefficients.M  

    # --------------------------------------------------------------------------------------------      
    # Center of Gravity Purtubation
    # --------------------------------------------------------------------------------------------   
    CM_cg_prime  = compute_stability_derivative(sub_sur.CM_0_shifted_CG   ,trans_sur.CM_0_shifted_CG    ,sup_sur.CM_0_shifted_CG    ,h_sub,h_sup,Mach)

    # --------------------------------------------------------------------------------------------      
    # Neutral Point and Static Margin Calculation 
    # --------------------------------------------------------------------------------------------
    shifted_CG     = CG + delta_cg  
    dCM_dalpha_cg  = (CM_cg_prime     - CM_0) / (delta_angle)    
    dCM_dalpha     = (CM_alpha_prime  - CM_0) / (delta_angle)      
    m              =  (dCM_dalpha_cg[0] - dCM_dalpha[0]) /delta_cg 
    b              =  dCM_dalpha_cg[0]  - (m * shifted_CG)
    NP             =  -b / m  

    # --------------------------------------------------------------------------------------------      
    # Store Results 
    # --------------------------------------------------------------------------------------------           
    conditions.static_stability.neutral_point[:,0] = NP 
    conditions.static_stability.static_margin[:,0] = (NP - CG) / c_ref
    
    return

def evaluate_no_surrogate(state,settings,vehicle):
    """Evaluates static marging and neutral point directly using VLM.
    
    Assumptions:
        
    Source:
        None

    Args:
        stability  : VLM analysis  [unitless]
        state      : flight conditions     [unitless]
        settings   : VLM analysis settings [unitless]
        vehicle    : vehicle configuration [unitless] 
        
    Returns: 
        None  
    """          

    # unpack 
    conditions    = state.conditions 
    stability     = state.analyses.stability   
    delta_angle   = stability.training.angle_purtubation
    delta_cg      = stability.training.center_of_gravity_purtubation  
    n_cpts        = len(conditions.aerodynamics.angles.alpha)

    # --------------------------------------------------------------------------------------------      
    # Vehicle Properties 
    # --------------------------------------------------------------------------------------------      
    CG            = vehicle.mass_properties.center_of_gravity[0][0]
    c_ref         = vehicle.reference_chord  
            
    # --------------------------------------------------------------------------------------------      
    # Equilibrium Condition 
    # --------------------------------------------------------------------------------------------  
    atmosphere                                                         = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data                                                          = atmosphere.compute_values(altitude = conditions.freestream.altitude)  
    equilibrium_conditions                                             = RCAIDE.Framework.Mission.Common.Results()
    equilibrium_conditions.expand_rows(n_cpts,override=False)
    equilibrium_conditions.energy                                      = deepcopy(conditions.energy)
    equilibrium_conditions.freestream.density[:,0]                     = atmo_data.density[:,0]
    equilibrium_conditions.freestream.gravity[:,0]                     = conditions.freestream.gravity[:,0]
    equilibrium_conditions.freestream.speed_of_sound[:,0]              = atmo_data.speed_of_sound[:,0]
    equilibrium_conditions.freestream.dynamic_viscosity[:,0]           = atmo_data.dynamic_viscosity[:,0]
    equilibrium_conditions.aerodynamics.angles.alpha[:,0]              = 1E-12
    equilibrium_conditions.freestream.temperature[:,0]                 = atmo_data.temperature[:,0]
    equilibrium_conditions.freestream.velocity[:,0]                    = conditions.freestream.velocity[:,0]          
    equilibrium_conditions.frames.inertial.velocity_vector[:,0]        = conditions.frames.inertial.velocity_vector[:,0]
    equilibrium_conditions.freestream.mach_number                      = equilibrium_conditions.freestream.velocity/equilibrium_conditions.freestream.speed_of_sound
    equilibrium_conditions.freestream.dynamic_pressure                 = 0.5 * equilibrium_conditions.freestream.density *  (equilibrium_conditions.freestream.velocity ** 2)
    equilibrium_conditions.freestream.reynolds_number                  = equilibrium_conditions.freestream.density * equilibrium_conditions.freestream.velocity * c_ref/ equilibrium_conditions.freestream.dynamic_viscosity  
    
    VLM_results = VLM(equilibrium_conditions,settings,vehicle)  
    CM_0        = VLM_results.CM  
 
    # --------------------------------------------------------------------------------------------      
    # Alpha Purtubation  
    # --------------------------------------------------------------------------------------------   
    pertubation_conditions                             = deepcopy(equilibrium_conditions)   
    pertubation_conditions.aerodynamics.angles.alpha   += delta_angle 
    VLM_results       = VLM(pertubation_conditions,settings,vehicle) 
    CM_alpha_prime    = VLM_results.CM 

    # --------------------------------------------------------------------------------------------      
    # Center of Gravity Purtubation
    # -------------------------------------------------------------------------------------------- 
    pertubation_conditions                             = deepcopy(equilibrium_conditions)  
    pertubation_conditions.aerodynamics.angles.alpha   += delta_angle 
    vehicle_shifted_CG = deepcopy(vehicle) 
    vehicle_shifted_CG.mass_properties.center_of_gravity[0][0] +=delta_cg  
    VLM_results        = VLM(pertubation_conditions,settings,vehicle_shifted_CG)  
    CM_cg_prime        = VLM_results.CM       
      
    # --------------------------------------------------------------------------------------------      
    # Neutral Point and Static Margin Calculation 
    # --------------------------------------------------------------------------------------------   
    shifted_CG     = CG + delta_cg   
    dCM_dalpha_cg  = (CM_cg_prime   - CM_0) / (delta_angle)    
    dCM_dalpha     = (CM_alpha_prime     - CM_0) / (delta_angle)      
    m              =  (dCM_dalpha_cg[0] - dCM_dalpha[0]) /delta_cg 
    b              =  dCM_dalpha_cg[0]  - (m * shifted_CG)
    NP             =  -b / m  

    # --------------------------------------------------------------------------------------------      
    # Store Results 
    # --------------------------------------------------------------------------------------------           
    conditions.static_stability.neutral_point[:,0] = NP 
    conditions.static_stability.static_margin[:,0] = (NP - CG) / c_ref      
        
    return

def compute_stability_derivative(sub_sur,trans_sur,sup_sur,h_sub,h_sup,Mach):
    if trans_sur ==  None and  sup_sur == None:
        derivative = h_sub(Mach)*sub_sur(Mach) 
        return derivative
        
    derivative = h_sub(Mach)*sub_sur(Mach) +   (1 - (h_sup(Mach) + h_sub(Mach)))*trans_sur(Mach)  + h_sup(Mach)*sup_sur(Mach) 
    return derivative



def compute_coefficients(sub_sur_Clift,sub_sur_Cdrag,sub_sur_CX,sub_sur_CY,sub_sur_CZ,sub_sur_CL,sub_sur_CM,sub_sur_CN,
                         trans_sur_Clift,trans_sur_Cdrag,trans_sur_CX,trans_sur_CY,trans_sur_CZ,trans_sur_CL,trans_sur_CM,trans_sur_CN,
                         sup_sur_Clift,sup_sur_Cdrag,sup_sur_CX,sup_sur_CY,sup_sur_CZ,sup_sur_CL,sup_sur_CM,sup_sur_CN,
                         h_sub,h_sup,Mach, pts): 
    

     #  subsonic 
    sub_Clift     = np.atleast_2d(sub_sur_Clift(pts)).T  
    sub_Cdrag     = np.atleast_2d(sub_sur_Cdrag(pts)).T  
    sub_CX        = np.atleast_2d(sub_sur_CX(pts)).T 
    sub_CY        = np.atleast_2d(sub_sur_CY(pts)).T     
    sub_CZ        = np.atleast_2d(sub_sur_CZ(pts)).T     
    sub_CL        = np.atleast_2d(sub_sur_CL(pts)).T     
    sub_CM        = np.atleast_2d(sub_sur_CM(pts)).T     
    sub_CN        = np.atleast_2d(sub_sur_CN(pts)).T
    
    
    if trans_sur_Clift ==  None and  sup_sur_Clift == None:
    
        results       = Data() 
        results.Clift = h_sub(Mach) * sub_Clift
        results.Cdrag = h_sub(Mach) * sub_Cdrag
        results.CX    = h_sub(Mach) * sub_CX   
        results.CY    = h_sub(Mach) * sub_CY   
        results.CZ    = h_sub(Mach) * sub_CZ   
        results.CL    = h_sub(Mach) * sub_CL   
        results.CM    = h_sub(Mach) * sub_CM   
        results.CN    = h_sub(Mach) * sub_CN   
        
        return results
   
    
    # transonic   
    trans_Clift   = np.atleast_2d(trans_sur_Clift(pts)).T  
    trans_Cdrag   = np.atleast_2d(trans_sur_Cdrag(pts)).T  
    trans_CX      = np.atleast_2d(trans_sur_CX(pts)).T 
    trans_CY      = np.atleast_2d(trans_sur_CY(pts)).T     
    trans_CZ      = np.atleast_2d(trans_sur_CZ(pts)).T     
    trans_CL      = np.atleast_2d(trans_sur_CL(pts)).T     
    trans_CM      = np.atleast_2d(trans_sur_CM(pts)).T     
    trans_CN      = np.atleast_2d(trans_sur_CN(pts)).T

    # supersonic 
    sup_Clift     = np.atleast_2d(sup_sur_Clift(pts)).T  
    sup_Cdrag     = np.atleast_2d(sup_sur_Cdrag(pts)).T  
    sup_CX        = np.atleast_2d(sup_sur_CX(pts)).T 
    sup_CY        = np.atleast_2d(sup_sur_CY(pts)).T     
    sup_CZ        = np.atleast_2d(sup_sur_CZ(pts)).T     
    sup_CL        = np.atleast_2d(sup_sur_CL(pts)).T     
    sup_CM        = np.atleast_2d(sup_sur_CM(pts)).T     
    sup_CN        = np.atleast_2d(sup_sur_CN(pts)).T            

    # apply 
    results       = Data() 
    results.Clift = h_sub(Mach)*sub_Clift + (1 - (h_sup(Mach) + h_sub(Mach)))*trans_Clift  + h_sup(Mach)*sup_Clift
    results.Cdrag = h_sub(Mach)*sub_Cdrag + (1 - (h_sup(Mach) + h_sub(Mach)))*trans_Cdrag  + h_sup(Mach)*sup_Cdrag
    results.CX    = h_sub(Mach)*sub_CX    + (1 - (h_sup(Mach) + h_sub(Mach)))*trans_CX     + h_sup(Mach)*sup_CX   
    results.CY    = h_sub(Mach)*sub_CY    + (1 - (h_sup(Mach) + h_sub(Mach)))*trans_CY     + h_sup(Mach)*sup_CY   
    results.CZ    = h_sub(Mach)*sub_CZ    + (1 - (h_sup(Mach) + h_sub(Mach)))*trans_CZ     + h_sup(Mach)*sup_CZ   
    results.CL    = h_sub(Mach)*sub_CL    + (1 - (h_sup(Mach) + h_sub(Mach)))*trans_CL     + h_sup(Mach)*sup_CL   
    results.CM    = h_sub(Mach)*sub_CM    + (1 - (h_sup(Mach) + h_sub(Mach)))*trans_CM     + h_sup(Mach)*sup_CM   
    results.CN    = h_sub(Mach)*sub_CN    + (1 - (h_sup(Mach) + h_sub(Mach)))*trans_CN     + h_sup(Mach)*sup_CN

    return results


def compute_coefficient(sub_sur_coef,trans_sur_coef, sup_sur_coef, h_sub,h_sup,Mach, pts): 

    #  subsonic 
    sub_coef  = np.atleast_2d(sub_sur_coef(pts)).T     
   
    if trans_sur_coef == None and sup_sur_coef == None:
        coef = h_sub(Mach) 
        return  coef
    
    # transonic 
    trans_coef  = np.atleast_2d(trans_sur_coef(pts)).T    

    # supersonic 
    sup_coef  = np.atleast_2d(sub_sur_coef(pts)).T             

    # apply  
    coef = h_sub(Mach)*sub_coef +   (1 - (h_sup(Mach) + h_sub(Mach)))*trans_coef  + h_sub(Mach)*sup_coef 

    return coef 
# RCAIDE/Library/Methods/Aerodynamics/Low_Fidelity_Empiracle_Correlation/empiracle_lift.py
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports  
import RCAIDE  

# package imports
import numpy   as np 

# ----------------------------------------------------------------------------------------------------------------------
#  Vortex_Lattice
# ---------------------------------------------------------------------------------------------------------------------- 
def empiracle_lift(state,settings,vehicle):
    """Evaluates surrogates forces and moments using built surrogates 
    
    Assumptions:
        
    Source:
        None

    Args:
        aerodynamics : VLM analysis  [unitless]
        state        : flight conditions     [unitless]
        settings     : VLM analysis settings [unitless]
        vehicle      : vehicle configuration [unitless] 
        
    Returns: 
        None  
    """          
    conditions    = state.conditions 
    AoA           = np.atleast_2d(conditions.aerodynamics.angles.alpha)    
    Mach          = np.atleast_2d(conditions.freestream.mach_number)
    
    b     =  0
    d     =  0
    e     =  0.9
    sweep =  0
    Cl_alpha =  np.pi * 2 
    for wing in vehicle.wings:
        if type(wing) == RCAIDE.Library.Components.Wings.Main_Wing: 
            b = wing.spans.projected
            S_ref = wing.areas.reference
            sweep = wing.sweeps.leading_edge 
            if  wing.areas.exposed == 0: 
                S_exp =  wing.areas.reference * 2
            AR = wing.aspect_ratio 
            
    for fuselage in vehicle.fuselages: 
        d =  np.maximum(d,fuselage.width)
        
    delta_max_t    = sweep
    F              =  1.07 * (1 + d / b) ** 2
    beta           = np.sqrt(1-Mach**2)
    eta            = Cl_alpha *beta  / (2 * np.pi)
    demonominator  =  2 + np.sqrt(4 + ((AR*beta/eta)**2)* (1+ (np.tan(delta_max_t)/beta)**2 ))
    CL_alpha       =  (2 * np.pi * AR / (demonominator)) * (S_exp /S_ref) * F
    
    CL =  CL_alpha * AoA
    CDi  = (CL ** 2) / (np.pi * AR * e)
    
    conditions.static_stability.coefficients.lift              =  CL 
    conditions.static_stability.coefficients.drag              =  CDi
    conditions.aerodynamics.coefficients.lift.total            = conditions.static_stability.coefficients.lift 
    conditions.aerodynamics.coefficients.drag.induced.inviscid = conditions.static_stability.coefficients.drag
    
    return
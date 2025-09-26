# RCAIDE/Library/Methods/Stability/Vortex_Lattice_Method/compute_neutral_point.py
#  
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports  
import RCAIDE 
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.VLM   import VLM 
from copy import deepcopy 

# package imports
import numpy  as np
from scipy.optimize import minimize 

# ----------------------------------------------------------------------------------------------------------------------
#  compute_neutral_point
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_neutral_point(stability):
    """
    Computes the neutral point of an aircraft using Vortex Lattice Method optimization.

    Parameters
    ----------
    stability : RCAIDE.Framework.Analyses.Stability
        Stability analysis object containing:
            - vehicle : RCAIDE.Library.Components.Vehicle
                Vehicle object with geometry and mass properties
            - settings : RCAIDE.Framework.Analyses.Stability.Settings
                Stability analysis settings
            - training : Data
                Training data for analysis
                - angle_of_attack : numpy.ndarray
                    Array of angle of attack values [radians]
                - Mach : numpy.ndarray
                    Array of Mach numbers [unitless]

    Returns
    -------
    None
        The neutral point is stored in stability.vehicle.neutral_point

    Notes
    -----
    This function computes the neutral point of an aircraft by finding the center
    of gravity location where the pitching moment coefficient derivative with respect
    to angle of attack is zero. It uses an optimization approach with the Vortex
    Lattice Method to evaluate aerodynamic characteristics. It specifically uses the
    Scipy SLSQP solver.
    
    **Major Assumptions**
        * Linear aerodynamics within the analysis range
        * Control surfaces are removed for clean configuration analysis
        * Vortex Lattice Method accurately represents the aerodynamics
        * Small angle approximations apply
    
    **Theory**

    The neutral point is defined as the center of gravity location where:
    :math:`\\frac{\\partial C_M}{\\partial \\alpha} = 0`

    **Definitions**

    'Neutral Point'
        Center of gravity location where the aircraft has neutral static stability (i.e. Cm_alpha = 0).

    """
  
    vehicle        = deepcopy(stability.vehicle)
    settings       = stability.settings
    AoA            = stability.training.angle_of_attack 
    Mach           = stability.training.Mach   
    len_Mach       = len(Mach)        
    len_AoA        = len(AoA)
     
    AoAs                                            = np.atleast_2d(np.tile(AoA,len_Mach).T.flatten()).T 
    Machs                                           = np.atleast_2d(np.repeat(Mach,len_AoA)).T      
    conditions                                      = RCAIDE.Framework.Mission.Common.Results() 
    conditions.freestream.mach_number               = Machs
    conditions.freestream.velocity                  = np.ones_like(Machs) * 1e-6
    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs)*AoAs  
    
    # --------------------------------------------------------------------------------------------------------------
    # Neutral Point 
    # --------------------------------------------------------------------------------------------------------------   
    clean_wing_vehicle_np = deepcopy(vehicle) # Double check this is correct
    for wing in clean_wing_vehicle_np.wings:
        wing.control_surfaces = []
    # use center of gravity as inital guess
    cg     =  vehicle.mass_properties.center_of_gravity[0][0]
    bnds   = [[0, 100]]
    sol = minimize(neutral_point_objective, [cg], args=(conditions,settings,clean_wing_vehicle_np,Mach,AoA) , method='SLSQP', bounds=bnds, tol=1e-4)
     
    stability.vehicle.neutral_point  =  sol.x[0]  
   
    return 

def neutral_point_objective(cg_location,conditions,settings,clean_wing_vehicle_np,Mach,AoA):
    """
    Objective function for neutral point optimization.

    Parameters
    ----------
    cg_location : list
        Center of gravity location [m]
    conditions : RCAIDE.Framework.Mission.Common.Results
        Flight conditions for VLM analysis
    settings : RCAIDE.Framework.Analyses.Stability.Settings
        Stability analysis settings
    clean_wing_vehicle_np : RCAIDE.Library.Components.Vehicle
        Clean wing vehicle configuration (no control surfaces)
    Mach : numpy.ndarray
        Array of Mach numbers [unitless]
    AoA : numpy.ndarray
        Array of angle of attack values [radians]

    Returns
    -------
    float
        Absolute value of pitching moment coefficient derivative with respect to angle of attack

    Notes
    -----
    This function serves as the objective function for the neutral point optimization.
    It updates the vehicle center of gravity, runs VLM analysis, and computes the
    pitching moment coefficient derivative to minimize.
    
    **Major Assumptions**
        * VLM analysis is valid for the given conditions
        * Pitching moment coefficient varies linearly with angle of attack
        * Clean wing configuration represents the neutral point condition
    """
    len_Mach       = len(Mach)        
    len_AoA        = len(AoA)
    
    # update neutral point
    clean_wing_vehicle_np.mass_properties.center_of_gravity[0][0] =  cg_location[0]
    
    # run VLM 
    VLM_results = VLM(conditions,settings,clean_wing_vehicle_np)    
    
    AoA       = conditions.aerodynamics.angles.alpha
    CM_res    = VLM_results.CM
    CM        = np.reshape(CM_res,(len_Mach,len_AoA)).T 
    
    # compute dCM_dalpha 
    dCM_dalpha = ( CM[2, 0] - CM[1, 0]) /( AoA[2] - AoA[1])
     
    # find abs 
    return  abs(dCM_dalpha)
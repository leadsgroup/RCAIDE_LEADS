# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/evaluate_AVL_surrogate.py
#  
# Created: Oct 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports   
from RCAIDE.Framework.Core import  Data   
from RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.run_AVL_analysis  import run_AVL_analysis  

# package imports
import numpy   as np  

# ----------------------------------------------------------------------------------------------------------------------
#  Vortex_Lattice
# ---------------------------------------------------------------------------------------------------------------------- 
def evaluate_AVL_surrogate(state,settings,vehicle):
    """
    Evaluates aerodynamic forces and moments using pre-trained AVL surrogate models.

    Parameters
    ----------
    state : Data
        Aircraft state data containing flight conditions and analysis containers
            - conditions.freestream.mach_number : array_like
                Flight Mach numbers
            - conditions.aerodynamics.angles.alpha : array_like
                Angles of attack in radians
            - analyses.aerodynamics : Data
                Aerodynamics analysis containing surrogate models
    settings : Data
        AVL analysis settings and configuration parameters
    vehicle : Vehicle
        Vehicle configuration containing mass properties and wing geometry
            - mass_properties.center_of_gravity : array_like
                Center of gravity coordinates in meters
            - wings.main_wing.chords.mean_aerodynamic : float
                Mean aerodynamic chord in meters

    Returns
    -------
    None
        Results are stored directly in state.conditions

    Notes
    -----
    This function provides rapid aerodynamic coefficient evaluation using
    surrogate models trained from AVL analysis data. The surrogates enable
    efficient computation during trajectory optimization and mission analysis
    without requiring repeated calls to the AVL solver. 

    Static stability derivatives and neutral point calculations are included
    for complete flight dynamics modeling. Static margin is computed relative
    to the mean aerodynamic chord.

    **Major Assumptions**
        * Surrogate models adequately represent flight envelope
        * Linear interpolation provides sufficient accuracy
        * Static stability calculations are valid for current configuration

    **Theory**

    Static margin calculation:

    .. math::
        SM = \\frac{x_{np} - x_{cg}}{\\bar{c}}

    where :math:`x_{np}` is neutral point, :math:`x_{cg}` is center of gravity,
    and :math:`\\bar{c}` is mean aerodynamic chord.

    **Definitions**

    'Static Margin'
        Distance between neutral point and center of gravity, indicating stability

    'Neutral Point'
        Aircraft center of pressure location where pitching moment is independent of angle of attack

    See Also
    --------
    RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.build_AVL_surrogates : Surrogate model training function
    """
    conditions          = state.conditions
    aerodynamics        = state.analyses.aerodynamics  
    Mach                = conditions.freestream.mach_number
    AoA                 = conditions.aerodynamics.angles.alpha
    lift_model          = aerodynamics.surrogates.lift_coefficient            
    drag_model          = aerodynamics.surrogates.drag_coefficient            
    e_model             = aerodynamics.surrogates.span_efficiency_factor      
    moment_model        = aerodynamics.surrogates.moment_coefficient          
    Cm_alpha_model      = aerodynamics.surrogates.Cm_alpha_moment_coefficient 
    Cn_beta_model       = aerodynamics.surrogates.Cn_beta_moment_coefficient       
    neutral_point_model = aerodynamics.surrogates.neutral_point               
    cg                  = vehicle.mass_properties.center_of_gravity[0]
    MAC                 = vehicle.wings.main_wing.chords.mean_aerodynamic
  
    pts   = np.hstack((AoA,Mach))     
    conditions.aerodynamics.coefficients.lift.total                   = np.atleast_2d(lift_model(pts)).T  
    conditions.aerodynamics.coefficients.drag.induced.inviscid        = np.atleast_2d(drag_model(pts)).T  
    conditions.aerodynamics.span_efficiency                           = np.atleast_2d(e_model(pts)).T  
    conditions.control_surfaces.slat.static_stability.coefficients.M  = np.atleast_2d(moment_model(pts)).T  
    conditions.static_stability.derivatives.CM_alpha                  = np.atleast_2d(Cm_alpha_model(pts)).T  
    conditions.static_stability.derivatives.CN_beta                   = np.atleast_2d(Cn_beta_model(pts)).T  
    conditions.static_stability.neutral_point                         = np.atleast_2d(neutral_point_model(pts)).T    
    conditions.static_stability.static_margin                         = (conditions.static_stability.neutral_point - cg)/MAC     
    aerodynamics.settings.span_efficiency                             = conditions.aerodynamics.span_efficiency   
    return


def evaluate_AVL_no_surrogate(state,settings,vehicle):
    """
    Evaluates aerodynamic forces and moments directly using AVL vortex lattice analysis.

    Parameters
    ----------
    state : Data
        Aircraft state data containing flight conditions and analysis containers
            - conditions : Data
                Flight condition data for AVL analysis
            - analyses.aerodynamics : Data
                Aerodynamics analysis configuration
    settings : Data
        AVL analysis settings and configuration parameters
    vehicle : Vehicle
        Vehicle configuration for geometric and mass property data

    Returns
    -------
    None
        Results are stored directly in state.conditions

    Notes
    -----
    This function performs full AVL vortex lattice analysis for each flight
    condition. The function automatically handles AVL input file generation, analysis
    execution, and result parsing. All aerodynamic coefficients and stability
    derivatives are computed and stored in the flight conditions.

    Use this method when surrogate models are not available or when higher
    fidelity AVL analysis is required for specific flight conditions. Note, that is
    requires AVL to be installed and accessible.

    **Definitions**

    'Vortex Lattice Method'
        Numerical technique for solving inviscid flow around lifting surfaces

    References
    ----------
    [1] Drela, M. and Youngren, H., "AVL 3.36 User Primer", MIT, 2017

    See Also
    --------
    RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.run_AVL_analysis : Core AVL analysis execution function
    """

    # unpack 
    conditions     = state.conditions
    aerodynamics   = state.analyses.aerodynamics   
    run_AVL_analysis(aerodynamics,conditions)
                       
    return

 
 
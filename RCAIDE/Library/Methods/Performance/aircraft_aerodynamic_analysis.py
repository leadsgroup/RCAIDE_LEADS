# RCAIDE/Methods/Performance/aircraft_aerodynamic_analysis.py
# 
# 
# Created:  Dec 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import  Data 
 
# Pacakge imports 
import numpy as np  

#------------------------------------------------------------------------------
# aircraft_aerodynamic_analysis
#------------------------------------------------------------------------------  
def aircraft_aerodynamic_analysis(aerodynamics_analysis = None,
                                  angle_of_attack_range = None,
                                  Mach_number_range= None,
                                  control_surface_deflection_range = np.array([[0]]),
                                  altitude = 0,
                                  delta_ISA=0):
    """
    Computes aerodynamic coefficients across ranges of angle of attack and Mach numbers using vortex lattice methods.
 
 
    Parameters
    --------
    vehicle : Vehicle
        The vehicle instance to be analyzed
    angle_of_attack_range : ndarray
        Array of angle of attack values to evaluate [radians]
    Mach_number_range : ndarray
        Array of Mach numbers to evaluate
    control_surface_deflection_range : ndarray, optional
        Array of control surface deflection angles [radians], default [[0]]
    altitude : float, optional
        Altitude for atmospheric properties [m], default 0
    delta_ISA : float, optional
        Temperature offset from ISA conditions [K], default 0
    use_surrogate : bool, optional
        Flag for using surrogate model in analysis, default True
    model_fuselage : bool, optional
        Flag for including fuselage effects, default True. Of note, fuselage modeling can 
        sometimes be difficult for VLM solvers.
 
    Returns
    --------
    results : Data
        Container of analysis results including:
            * Mach : ndarray
                Evaluated Mach numbers
            * alpha : ndarray
                Evaluated angles of attack [rad]
            * lift_coefficient : ndarray
                Computed lift coefficients
            * drag_coefficient : ndarray
                Computed drag coefficients
 
    Notes
    -----
    The function uses the US Standard Atmosphere 1976 model for atmospheric properties
    and evaluates aerodynamic coefficients using vortex lattice methods. Can use a surrogate model
    for faster evaluation or just direct evaluation of the aerodynamics. 
 
    **Major Assumptions**
        * Flow is steady and inviscid
        * Small angle approximations apply
        * Linear aerodynamics
        * Atmospheric properties follow US Standard Atmosphere 1976
 
    See Also
    --------
    RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method
    RCAIDE.Library.Attributes.Atmospheres.Earth.US_Standard_1976
    """

    #------------------------------------------------------------------------
    # setup flight conditions
    #------------------------------------------------------------------------   
    atmosphere     = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data      = atmosphere.compute_values(altitude,delta_ISA)
    P              = atmo_data.pressure 
    T              = atmo_data.temperature 
    rho            = atmo_data.density 
    a              = atmo_data.speed_of_sound 
    mu             = atmo_data.dynamic_viscosity
       
    # -----------------------------------------------------------------
    # Evaluate Without Surrogate
    # ----------------------------------------------------------------- 
    ctrl_pts = len(angle_of_attack_range[:, 0] )
    state                                         = RCAIDE.Framework.Mission.Common.State()
    state.conditions                              = RCAIDE.Framework.Mission.Common.Results() 
    state.conditions.freestream.density           = rho * np.ones_like(angle_of_attack_range)
    state.conditions.freestream.dynamic_viscosity = mu  * np.ones_like(angle_of_attack_range)
    state.conditions.freestream.temperature       = T   * np.ones_like(angle_of_attack_range)
    state.conditions.freestream.pressure          = P   * np.ones_like(angle_of_attack_range)
    state.conditions.aerodynamics.angles.alpha    = angle_of_attack_range  
    state.conditions.aerodynamics.angles.beta     = angle_of_attack_range *0  
    state.conditions.freestream.u                 = angle_of_attack_range *0       
    state.conditions.freestream.v                 = angle_of_attack_range *0       
    state.conditions.freestream.w                 = angle_of_attack_range *0       
    state.conditions.static_stability.roll_rate   = angle_of_attack_range *0       
    state.conditions.static_stability.pitch_rate  = angle_of_attack_range *0 
    state.conditions.static_stability.yaw_rate    = angle_of_attack_range *0  
    state.conditions.expand_rows(ctrl_pts)
 
    CL_vals    = np.zeros((len(angle_of_attack_range),len(Mach_number_range)))  
    CD_vals    = np.zeros((len(angle_of_attack_range),len(Mach_number_range))) 
 
    state.analyses                                  =  Data()
    aerodynamics_analysis.initialize()            
    state.analyses.aerodynamics = aerodynamics_analysis 
    
    for i in range (len(Mach_number_range)):  
        state.conditions.freestream.mach_number                 = Mach_number_range[i, 0] * np.ones_like(angle_of_attack_range)
        state.conditions.freestream.velocity                    = Mach_number_range[i, 0] * a   * np.ones_like(angle_of_attack_range)   
        state.conditions.freestream.reynolds_number             = state.conditions.freestream.density * state.conditions.freestream.velocity / state.conditions.freestream.dynamic_viscosity 
        state.conditions.frames.inertial.velocity_vector[:,0]   = Mach_number_range[i, 0] * a[0, 0]   *  angle_of_attack_range[:, 0] 
        
     
        # ---------------------------------------------------------------------------------------
        # Evaluate With Surrogate
        # ---------------------------------------------------------------------------------------  
        _                 = state.analyses.aerodynamics.evaluate(state)        
        CL_vals[:,i]      = state.conditions.aerodynamics.coefficients.lift.total[:, 0]
        CD_vals[:,i]      = state.conditions.aerodynamics.coefficients.drag.total[:, 0] 

  
    results = Data(
        Mach              = Mach_number_range, 
        alpha             = angle_of_attack_range, 
        lift_coefficient  = CL_vals, 
        drag_coefficient  = CD_vals, 
    )  
          
    return results  

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
from RCAIDE.Library.Mission.Common.Pre_Process  import geometry_preprocess_routine 
 
# Pacakge imports 
import numpy as np  
import os, sys 

#------------------------------------------------------------------------------
# aircraft_aerodynamic_analysis
#------------------------------------------------------------------------------  
def aircraft_aerodynamic_analysis(analyses                         = None, 
                                  angle_of_attacks                 = None,
                                  mach_numbers                     = None,
                                  non_dimensional_reynolds_numbers = None,
                                  temperatures                     = None, 
                                  overwrite_reference              = True,  
                                  altitude = None ):
    """
    Computes aerodynamic coefficients across ranges of angle of attack and Mach numbers using vortex lattice methods.
 
 
    Parameters
    --------
    vehicle : Vehicle
        The vehicle instance to be analyzed
    angle_of_attacks : ndarray
        Array of angle of attack values to evaluate [radians]
    mach_numbers : ndarray
        Array of Mach numbers to evaluate 
    altitude : float, optional
        Altitude for atmospheric properties [m], default 0 
 
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
            * moment_coefficient : ndarray
                Computed Y-moment coefficients
 
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
    # Preprocess Geometry 
    #------------------------------------------------------------------------ 
    geometry_preprocess_routine(analyses)
    
    #------------------------------------------------------------------------  
    # Check size of arrays 
    #------------------------------------------------------------------------
    if angle_of_attacks is None:
        raise ValueError("Angle of attack range must be defined as nx1 2d-array")
    if mach_numbers is None:
        raise ValueError("Mach number range must be defined as nx1 2d-array ")
    
    dim_AoA   = len(angle_of_attacks[:, 0] )
    dim_Mach  = len(mach_numbers[:, 0] )
    
    if dim_Mach != dim_AoA:
        raise ValueError("Angle of attack and Mach number range must same dimension") 

    #------------------------------------------------------------------------
    # setup flight conditions
    #------------------------------------------------------------------------
    # if altitude is specified 
    if altitude is not None:   
        atmosphere     = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
        atmo_data      = atmosphere.compute_values(altitude)
        P              = atmo_data.pressure 
        T              = atmo_data.temperature 
        rho            = atmo_data.density 
        a              = atmo_data.speed_of_sound  
        mu             = atmo_data.dynamic_viscosity
        V              = mach_numbers * a 
        non_dimensional_reynolds_numbers  = V * rho / mu 
    
    # if non_dimensional_reynolds_numbers and temperatures are specified 
    elif non_dimensional_reynolds_numbers is not  None and temperatures is not None:  
        dim_Re   = len(non_dimensional_reynolds_numbers[:, 0] )
        dim_T    = len(temperatures[:, 0] ) 
        if dim_Re != dim_T:
            raise ValueError("Reynolds number and temperature range must same dimension") 
        elif dim_AoA != dim_T: 
            raise ValueError("Angle of attack and temperature range must same dimension")      
        

        atmosphere     = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
        atmo_data      = atmosphere.compute_values(0)
        P              = atmo_data.pressure 
        T              = temperatures
        a              = RCAIDE.Library.Attributes.Gases.Air().compute_speed_of_sound(T=T) 
        V              = mach_numbers * a 
        mu             = RCAIDE.Library.Attributes.Gases.Air().compute_absolute_viscosity(T=T)
        rho            = non_dimensional_reynolds_numbers * mu / V
    else:
        raise ValueError("Specify either 1) altitude or combination or 2) non dimensional reynolds numbers and temperature arrays")            
       
    # -----------------------------------------------------------------
    # Evaluate Without Surrogate
    # ----------------------------------------------------------------- 
    ctrl_pts = len(angle_of_attacks[:, 0] )
    state                                              = RCAIDE.Framework.Mission.Common.State()
    state.conditions                                   = RCAIDE.Framework.Mission.Common.Results() 
    state.conditions.freestream.density                = rho * np.ones_like(angle_of_attacks)
    state.conditions.freestream.dynamic_viscosity      = mu  * np.ones_like(angle_of_attacks)
    state.conditions.freestream.temperature            = T   * np.ones_like(angle_of_attacks)
    state.conditions.freestream.pressure               = P   * np.ones_like(angle_of_attacks)
    state.conditions.freestream.dynamic_pressure       = 0.5 * rho * V**2  
    state.conditions.aerodynamics.angles.alpha         = angle_of_attacks  
    state.conditions.aerodynamics.angles.beta          = angle_of_attacks *0  
    state.conditions.freestream.u                      = angle_of_attacks *0       
    state.conditions.freestream.v                      = angle_of_attacks *0       
    state.conditions.freestream.w                      = angle_of_attacks *0       
    state.conditions.static_stability.roll_rate        = angle_of_attacks *0       
    state.conditions.static_stability.pitch_rate       = angle_of_attacks *0 
    state.conditions.static_stability.yaw_rate         = angle_of_attacks *0 
    state.conditions.frames.wind.transform_to_inertial = np.tile( np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0.,  1.]]]) , ( ctrl_pts,  1, 1)  ) 
    state.conditions.expand_rows(ctrl_pts)
  
    state.analyses  = analyses 
    state.analyses.aerodynamics.filename =  os.path.join(os.path.dirname(os.path.abspath(sys.argv[0])), "aerodynamic_training_data.pkl" )
    state.analyses.aerodynamics.initialize(state.analyses.vehicle)    
    state.conditions.freestream.mach_number                 = mach_numbers
    state.conditions.freestream.velocity                    = V
    state.conditions.freestream.reynolds_number             = non_dimensional_reynolds_numbers
    state.conditions.frames.inertial.velocity_vector        = np.tile(np.array([[0, 0, 0]]), ( ctrl_pts,  1))
    state.conditions.frames.inertial.velocity_vector[:,0]   = V[:,0]  
 
    # ---------------------------------------------------------------------------------------
    # Evaluate With Surrogate
    # ---------------------------------------------------------------------------------------  
    _                 = state.analyses.aerodynamics.evaluate(state,state.analyses.vehicle)   
    results = Data(
        Mach                             = mach_numbers, 
        alpha                            = angle_of_attacks, 
        lift_coefficient                 = state.conditions.aerodynamics.coefficients.lift.total, 
        drag_coefficient                 = state.conditions.aerodynamics.coefficients.drag.total,
        parasite_drag_coefficient        = state.conditions.aerodynamics.coefficients.drag.parasite.total,
        form_drag_coefficient            = state.conditions.aerodynamics.coefficients.drag.form.total,
        wave_drag_coefficient            = state.conditions.aerodynamics.coefficients.drag.wave.total,
        induced_drag_coefficient         = state.conditions.aerodynamics.coefficients.drag.induced.total,
        miscellaneous_drag_coefficient   = state.conditions.aerodynamics.coefficients.drag.miscellaneous.total,
        compressibility_drag_coefficient = state.conditions.aerodynamics.coefficients.drag.compressible.total,
        cooling_drag_coefficient         = state.conditions.aerodynamics.coefficients.drag.cooling.total,
        trim_drag_coefficient            = state.conditions.aerodynamics.coefficients.drag.trim.total,
        moment_coefficient               = state.conditions.static_stability.coefficients.M, 
        state_conditions                 = state.conditions,
        
    )  
          
    return results  
 
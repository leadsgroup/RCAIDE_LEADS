# RCAIDE/Methods/Performance/aircraft_aerostructural_analysis.py
# 
# 
# Created:  Apr 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import  Data  
from RCAIDE.Library.Mission.Common.Pre_Process  import geometry_preprocess_routine 
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.VLM       import VLM 
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.FEA   import FEA 
 
# Pacakge imports 
import numpy as np   

#------------------------------------------------------------------------------
# aircraft_aerostructural_analysis
#------------------------------------------------------------------------------  
def aircraft_aerostructural_analysis(analyses                         = None, 
                                     angle_of_attacks                 = None,
                                     mach_numbers                     = None,
                                     non_dimensional_reynolds_numbers = None,
                                     temperatures                     = None, 
                                     overwrite_reference              = True,  
                                     altitude                         = None ):
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
    conditions                                        = RCAIDE.Framework.Mission.Common.Results() 
    conditions.freestream.density                     = rho * np.ones_like(angle_of_attacks)
    conditions.freestream.dynamic_viscosity           = mu  * np.ones_like(angle_of_attacks)
    conditions.freestream.temperature                 = T   * np.ones_like(angle_of_attacks)
    conditions.freestream.pressure                    = P   * np.ones_like(angle_of_attacks)
    conditions.freestream.dynamic_pressure            = 0.5 * rho * V**2  
    conditions.freestream.velocity                    = V
    conditions.freestream.gravitational_acceleration  = 9.81 * np.ones_like(angle_of_attacks)
    conditions.freestream.mach_number                 = mach_numbers
    conditions.aerodynamics.angles.alpha              = angle_of_attacks
    conditions.aerodynamics.angles.beta               = angle_of_attacks *0  
    conditions.freestream.u                           = angle_of_attacks *0       
    conditions.freestream.v                           = angle_of_attacks *0       
    conditions.freestream.w                           = angle_of_attacks *0       
    conditions.static_stability.roll_rate             = angle_of_attacks *0       
    conditions.static_stability.pitch_rate            = angle_of_attacks *0 
    conditions.static_stability.yaw_rate              = angle_of_attacks *0 
    conditions.frames.wind.transform_to_inertial      = np.tile( np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0.,  1.]]]) , ( ctrl_pts,  1, 1)  ) 
    conditions.expand_rows(ctrl_pts)
    conditions.control_surfaces                       = Data()
    conditions.aerostructures                         = Data()

    # Initialise fuel masses so FEA can compute fuel tank structural loads.
    # In a direct analysis (no mission), fuel is held at its current vehicle-level mass.
    conditions.weights                       = Data()
    conditions.weights.components            = Data()
    conditions.weights.components.mass       = Data()
    for network in analyses.vehicle.networks:
        for source in network.sources:
            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank):
                fuel_tag = source.fuel.tag
                conditions.weights.components.mass[fuel_tag] = source.mass_properties.mass * np.ones((ctrl_pts, 1))

    for wing in analyses.vehicle.wings: 
        for control_surface in wing.control_surfaces: 
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron:
                analyses.aerodynamics.aileron_flag  = True 
                conditions.control_surfaces.aileron = Data()
                conditions.control_surfaces.aileron.deflection = control_surface.deflection * np.ones_like(angle_of_attacks)
                conditions.control_surfaces.aileron.static_stability = Data()
                conditions.control_surfaces.aileron.static_stability.coefficients = Data()
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator:
                analyses.aerodynamics.elevator_flag = True 
                conditions.control_surfaces.elevator = Data()
                conditions.control_surfaces.elevator.deflection = control_surface.deflection * np.ones_like(angle_of_attacks)
                conditions.control_surfaces.elevator.static_stability = Data()
                conditions.control_surfaces.elevator.static_stability.coefficients = Data()
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder:
                analyses.aerodynamics.rudder_flag   = True
                conditions.control_surfaces.rudder = Data()
                conditions.control_surfaces.rudder.deflection = control_surface.deflection * np.ones_like(angle_of_attacks)
                conditions.control_surfaces.rudder.static_stability = Data()
                conditions.control_surfaces.rudder.static_stability.coefficients = Data()
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Flap:
                analyses.aerodynamics.flap_flag      = True
                conditions.control_surfaces.flap = Data()
                conditions.control_surfaces.flap.deflection = control_surface.deflection * np.ones_like(angle_of_attacks)
                conditions.control_surfaces.flap.static_stability = Data()
                conditions.control_surfaces.flap.static_stability.coefficients = Data()
            
    # run aerodynamics analyses
    VLM_results =  VLM(conditions,
                      analyses.aerodynamics.settings,
                      analyses.vehicle)
    
    # run aerostructures analyses
    FEA_results = FEA(conditions,
                      VLM_results,
                      analyses.aerodynamics.settings.vortex_distribution,
                      analyses.aerostructures.settings,analyses.vehicle)  
          
    return FEA_results 

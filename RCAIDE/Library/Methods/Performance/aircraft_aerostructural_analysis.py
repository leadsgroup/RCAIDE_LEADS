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
            
    coupled = (analyses.aerostructures.settings.aeroelastic_coupling == 'coupled')

    if coupled:
        FEA_results = _run_coupled_rows(conditions, analyses.aerodynamics.settings,
                                        analyses.vehicle, analyses.aerostructures)
    else:
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


def _run_coupled_rows(full_conditions, settings, vehicle, aerostructural_analyses, tol=1e-4, max_iter=15):
    """Per-(AoA,Mach)-row VLM<->FEA convergence for a direct (no-surrogate,
    no-mission) analysis, e.g. a benchmark comparison case. Reuses the same
    per-row iteration used for surrogate training (train_VLM_surrogates.py's
    _converge_aeroelastic) so both paths solve the identical fixed-point
    problem; only the row-slicing differs, since this function's conditions
    carries fields (frames.wind, freestream.u/v/w) the surrogate trainer
    never populates.

    Returns a Data keyed by wing.tag (deflection, elastic_twist, stresses -
    same shape FEA() returns), plus a .convergence.delta (n_cases, max_iter)
    NaN-padded relative-deflection trace for plotting iteration vs. deflection.
    """
    from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.train_VLM_surrogates import _converge_aeroelastic
    from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.generate_vortex_distribution import generate_vortex_distribution

    num_cases  = len(full_conditions.aerodynamics.angles.alpha)
    jig_vd     = None
    conv_delta = np.full((num_cases, max_iter), np.nan)
    structural_results = None

    for i in range(num_cases):
        conditions = RCAIDE.Framework.Mission.Common.Results()
        conditions.freestream.density                     = np.atleast_2d(full_conditions.freestream.density[i,:])
        conditions.freestream.dynamic_viscosity            = np.atleast_2d(full_conditions.freestream.dynamic_viscosity[i,:])
        conditions.freestream.temperature                  = np.atleast_2d(full_conditions.freestream.temperature[i,:])
        conditions.freestream.pressure                     = np.atleast_2d(full_conditions.freestream.pressure[i,:])
        conditions.freestream.dynamic_pressure             = np.atleast_2d(full_conditions.freestream.dynamic_pressure[i,:])
        conditions.freestream.velocity                     = np.atleast_2d(full_conditions.freestream.velocity[i,:])
        conditions.freestream.gravitational_acceleration   = np.atleast_2d(full_conditions.freestream.gravitational_acceleration[i,:])
        conditions.freestream.mach_number                  = np.atleast_2d(full_conditions.freestream.mach_number[i,:])
        conditions.aerodynamics.angles.alpha                = np.atleast_2d(full_conditions.aerodynamics.angles.alpha[i,:])
        conditions.aerodynamics.angles.beta                 = np.atleast_2d(full_conditions.aerodynamics.angles.beta[i,:])
        conditions.static_stability.roll_rate               = np.atleast_2d(full_conditions.static_stability.roll_rate[i,:])
        conditions.static_stability.pitch_rate              = np.atleast_2d(full_conditions.static_stability.pitch_rate[i,:])
        conditions.static_stability.yaw_rate                = np.atleast_2d(full_conditions.static_stability.yaw_rate[i,:])
        conditions.control_surfaces                         = full_conditions.control_surfaces
        conditions.aerostructures                           = Data()
        conditions.weights = Data()
        conditions.weights.components = Data()
        conditions.weights.components.mass = Data()
        for tag in full_conditions.weights.components.mass.keys():
            conditions.weights.components.mass[tag] = np.atleast_2d(
                full_conditions.weights.components.mass[tag][i,:])

        if jig_vd is None:
            jig_vd = generate_vortex_distribution(conditions, settings, vehicle)

        _, structural_i, history_i = _converge_aeroelastic(
            conditions, settings, vehicle, aerostructural_analyses, jig_vd, tol, max_iter)

        conv_delta[i, :len(history_i)] = history_i

        if structural_results is None:
            structural_results = Data()
            for wing_tag in structural_i.keys():
                structural_results[wing_tag] = Data(
                    structural_node_data = structural_i[wing_tag].structural_node_data,
                    load                  = structural_i[wing_tag].load,
                    deflection            = structural_i[wing_tag].deflection,
                    elastic_twist         = structural_i[wing_tag].elastic_twist,
                    normal_stress         = structural_i[wing_tag].normal_stress,
                    shear_stress          = structural_i[wing_tag].shear_stress,
                    margin_of_safety      = structural_i[wing_tag].margin_of_safety)
        else:
            for wing_tag in structural_i.keys():
                s = structural_results[wing_tag]
                s.load             = np.vstack((s.load            , structural_i[wing_tag].load))
                s.deflection       = np.vstack((s.deflection      , structural_i[wing_tag].deflection))
                s.elastic_twist    = np.vstack((s.elastic_twist   , structural_i[wing_tag].elastic_twist))
                s.normal_stress    = np.vstack((s.normal_stress   , structural_i[wing_tag].normal_stress))
                s.shear_stress     = np.vstack((s.shear_stress    , structural_i[wing_tag].shear_stress))
                s.margin_of_safety = np.vstack((s.margin_of_safety, structural_i[wing_tag].margin_of_safety))

    structural_results.convergence       = Data()
    structural_results.convergence.delta = conv_delta
    return structural_results

# estimate_take_off_field_length.py
#
# Created:  Jun 2014, T. Orra, C. Ilario, Celso, 
# Modified: Apr 2015, M. Vegh 
#           Jan 2016, E. Botero
#           Mar 2020, M. Clarke
#           May 2020, E. Botero
#           Jul 2020, E. Botero 


# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# RCAIDE Imports
import RCAIDE
from RCAIDE.Framework.Core            import Data, Units     
from RCAIDE.Library.Methods.Aerodynamics.Common.Drag import * 
from RCAIDE.Library.Methods.Aerodynamics.Common.Lift import *

# package imports
import numpy as np

# ----------------------------------------------------------------------
#  Compute field length required for takeoff
# ----------------------------------------------------------------------
def estimate_take_off_field_length(vehicle,analyses,altitude = 0, delta_isa = 0, compute_2nd_seg_climb = False):
    """ Computes the takeoff field length for a given vehicle configuration in a given airport.
    Also optionally computes the second segment climb gradient.

    Assumptions:
    For second segment climb gradient:
    One engine inoperative
    Only validated for two engine aircraft

    Source:
    http://adg.stanford.edu/aa241/AircraftDesign.html

    Inputs:
    analyses.base.atmosphere               [RCAIDE data type]
    airport.
      altitude                             [m]
      delta_isa                            [K]
    vehicle.
      mass_properties.takeoff              [kg]
      reference_area                       [m^2]
      V2_VS_ratio (optional)               [Unitless]
      maximum_lift_coefficient (optional)  [Unitless]
      networks.*.number_of_engines       [Unitless]

    Outputs:
    takeoff_field_length                   [m]

    Properties Used:
    N/A
    """        

    # ==============================================
        # Unpack
    # ==============================================
    atmo            = analyses.atmosphere 
    weight          = vehicle.mass_properties.takeoff
    reference_area  = vehicle.reference_area
    try:
        V2_VS_ratio = vehicle.V2_VS_ratio
    except:
        V2_VS_ratio = 1.20
        
        
    

    # ==============================================
    # Computing atmospheric conditions
    # ==============================================
    atmo_values       = atmo.compute_values(altitude,delta_isa)
    conditions        = RCAIDE.Framework.Mission.Common.Results()
    
    p   = atmo_values.pressure
    T   = atmo_values.temperature
    rho = atmo_values.density
    a   = atmo_values.speed_of_sound
    mu  = atmo_values.dynamic_viscosity
    sea_level_gravity = atmo.planet.sea_level_gravity
    
    # ==============================================
    # Determining vehicle maximum lift coefficient
    # ==============================================
    # Condition to CLmax calculation: 90KTAS @ airport
    state = Data()
    state.conditions = RCAIDE.Framework.Mission.Common.Results()
    state.conditions.freestream = Data()
    state.conditions.freestream.density           = rho
    state.conditions.freestream.velocity          = 90. * Units.knots
    state.conditions.freestream.dynamic_viscosity = mu
    
    settings = analyses.aerodynamics.settings

    maximum_lift_coefficient, induced_drag_high_lift = compute_max_lift_coeff(state,settings,vehicle)

    # ==============================================
    # Computing speeds (Vs, V2, 0.7*V2)
    # ==============================================
    stall_speed       = (2 * weight * sea_level_gravity / (rho * reference_area * maximum_lift_coefficient)) ** 0.5
    V2_speed          = V2_VS_ratio * stall_speed
    speed_for_thrust  = 0.70 * V2_speed

    # ==============================================
    # Determining vehicle number of engines
    # ==============================================
    engine_number = 0.
    for network in vehicle.networks : # may have than one network
        engine_number += len(network.propulsors)
    if engine_number == 0:
        raise ValueError("No engine found in the vehicle")

    # ==============================================
    # Getting engine thrust
    # ==============================================
    

    # Step 28: Static Sea Level Thrust  
    planet                                            = RCAIDE.Library.Attributes.Planets.Earth()
    atmosphere_sls                                    = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data                                         = atmosphere_sls.compute_values(0.0,0.0)
                                                      
    p                                                 = atmo_data.pressure          
    T                                                 = atmo_data.temperature       
    rho                                               = atmo_data.density          
    a                                                 = atmo_data.speed_of_sound    
    mu                                                = atmo_data.dynamic_viscosity 
    
    conditions                                        = RCAIDE.Framework.Mission.Common.Results() 
    conditions.freestream.altitude                    = np.atleast_1d(0)
    conditions.freestream.mach_number                 = np.atleast_1d(0.01)
    conditions.freestream.pressure                    = np.atleast_1d(p)
    conditions.freestream.temperature                 = np.atleast_1d(T)
    conditions.freestream.density                     = np.atleast_1d(rho)
    conditions.freestream.dynamic_viscosity           = np.atleast_1d(mu)
    conditions.freestream.gravity                     = np.atleast_2d(planet.sea_level_gravity) 
    conditions.freestream.speed_of_sound              = np.atleast_1d(a)
    conditions.freestream.velocity                    = np.atleast_1d(a*0.01)   

    # setup conditions   
    segment                                           = RCAIDE.Framework.Mission.Segments.Segment()  
    segment.state.conditions                          = conditions
    
    thrust =  np.array([[0.0, 0.0, 0.0]])
     
    analysis = RCAIDE.Framework.Analyses.Vehicle() 
    energy   = RCAIDE.Framework.Analyses.Energy.Energy()
    energy.vehicle  = vehicle 
    analysis.append(energy)            
    segment.analyses = analysis      
    
    for network in vehicle.networks:
        network.add_unknowns_and_residuals_to_segment(segment)
        
        for propulsor in  network.propulsors: 
            segment.state.conditions.energy[propulsor.tag].throttle = np.array([[1]])
        
        network.evaluate(segment.state,center_of_gravity = vehicle.mass_properties.center_of_gravity)
        
        thrust += conditions.energy.thrust_force_vector
         
    # ==============================================
    # Calculate takeoff distance
    # ==============================================

    # Defining takeoff distance equations coefficients 
    takeoff_constants = np.zeros(3)
    if engine_number == 2:
        takeoff_constants[0] =   857.4
        takeoff_constants[1] =   2.476
        takeoff_constants[2] =   0.00014
    elif engine_number == 3:
        takeoff_constants[0] = 667.9
        takeoff_constants[1] =   2.343
        takeoff_constants[2] =   0.000093
    elif engine_number == 4:
        takeoff_constants[0] = 486.7
        takeoff_constants[1] =   2.282
        takeoff_constants[2] =   0.0000705
    elif engine_number >  4:
        takeoff_constants[0] = 486.7
        takeoff_constants[1] =   2.282
        takeoff_constants[2] =   0.0000705
        print('The vehicle has more than 4 engines. Using 4 engine correlation. Result may not be correct.')
    else:
        takeoff_constants[0] = 857.4
        takeoff_constants[1] =   2.476
        takeoff_constants[2] =   0.00014
        print('Incorrect number of engines: {0:.1f}. Using twin engine correlation.'.format(engine_number))

    # Define takeoff index   (V2^2 / (T/W)
    takeoff_index = V2_speed**2. / (thrust[0][0] / weight)
    
    # Calculating takeoff field length
    takeoff_field_length = 0.
    for idx,constant in enumerate(takeoff_constants):
        takeoff_field_length += constant * takeoff_index**idx
    takeoff_field_length = takeoff_field_length * Units.ft
    
    # calculating second segment climb gradient, if required by user input
    if compute_2nd_seg_climb:
        
        # Getting engine thrust at V2 (update only speed related conditions)
        state.conditions.freestream.dynamic_pressure  = np.array(np.atleast_1d(0.5 * rho * V2_speed**2))
        state.conditions.freestream.velocity          = np.array(np.atleast_1d(V2_speed))
        state.conditions.freestream.mach_number       = np.array(np.atleast_1d(V2_speed/ a))
        state.conditions.freestream.dynamic_viscosity = np.array(np.atleast_1d(mu))
        state.conditions.freestream.density           =  np.array(np.atleast_1d(rho))
        
        # engine condition
        num_propulsors =  0
        for network in vehicle.networks:
            num_propulsors += len(network.propulsors) 
            for propulsor in  network.propulsors: 
                engine_out_location = propulsor.origin[0][1] 
        thrust  = thrust * (num_propulsors -1 )/num_propulsors
        single_engine_thrust =  np.linalg.norm(thrust /num_propulsors)

        # Compute windmilling drag
        windmilling_drag_coefficient = windmilling_drag(vehicle,state)

        # Compute asymmetry drag   
        asymmetry_drag_coefficient = asymmetry_drag(state, vehicle,engine_out_location, single_engine_thrust, windmilling_drag_coefficient)
           
        # Compute l over d ratio for takeoff condition, NO engine failure
        l_over_d = estimate_2ndseg_lift_drag_ratio(state,settings,vehicle) 
        
        # Compute L over D ratio for takeoff condition, WITH engine failure
        clv2            = maximum_lift_coefficient / (V2_VS_ratio) **2
        cdv2_all_engine = clv2 / l_over_d
        cdv2            = cdv2_all_engine + asymmetry_drag_coefficient + windmilling_drag_coefficient
        l_over_d_v2     = clv2 / cdv2
    
        # Compute 2nd segment climb gradient
        second_seg_climb_gradient = thrust / (weight*sea_level_gravity) - 1. / l_over_d_v2

        return takeoff_field_length[0][0], second_seg_climb_gradient[0][0]

    else:
        # return only takeoff_field_length
        return takeoff_field_length[0][0],0
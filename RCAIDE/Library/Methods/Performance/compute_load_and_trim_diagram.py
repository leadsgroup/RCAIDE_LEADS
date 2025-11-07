# RCAIDE/Methods/Performance/compute_load_and_trim_diagram.py
# 
# 
# Created:  Dec 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import  Data,  Units 
from RCAIDE.Library.Methods.Mass_Properties.estimate_maximum_landing_weight import estimate_maximum_landing_weight
from RCAIDE.Library.Mission.Common.Pre_Process import  geometry, mass_properties

# Pacakge imports 
import numpy as np   

#------------------------------------------------------------------------------
# compute_load_and_trim_diagram
#------------------------------------------------------------------------------  
def compute_load_and_trim_diagram(vehicle, number_of_points = 5, aerodynamic_analysis = None, stability_analysis = None,  weights_analysis = None, altitude = None, airspeed = None):
    """
    Computes the loading dragram of an aircraft 
 
 
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
            * loading_lift_coefficient : ndarray
                Computed lift coefficients
            * loading_drag_coefficient : ndarray
                Computed drag coefficients
            * loading_moment_coefficient : ndarray
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
    vehicle.mass_properties.takeoff  = None
    #------------------------------------------------------------------------  
    # Check Input Args
    #------------------------------------------------------------------------
    if altitude == None:
        raise AttributeError('Altitude not set') 
    
    if airspeed  == None:
        raise AttributeError('Airspeed not set')
    
    if aerodynamic_analysis == None:
        raise AttributeError('Aerodynamic analysis not set') 
    
    if weights_analysis  == None:
        raise AttributeError('Weights analysis not set')
    
    if stability_analysis  == None:
        raise AttributeError('Stability analysis not set') 

    # check that cabins are defined with at least one class
    cabin_class_check = False
    
    for fuselage in  vehicle.fuselages: 
        for cabin in fuselage.cabins:
            for _ in cabin.classes:
                cabin_class_check = True
    for wing in vehicle.wings: 
        if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
            for cabin in wing.cabins:    
                for _ in cabin.classes:
                    cabin_class_check = True
                    
    if cabin_class_check == False:
        raise AttributeError('At least one cabin class must be defined to create Aircraft mass-C.G. envelope ') 

    # Set up vehicle configs
    configs  = configs_setup(vehicle)

    # create analyses
    analyses = analyses_setup(configs, aerodynamic_analysis,  stability_analysis, weights_analysis)

    # mission analyses 
    mission = mission_setup(analyses, altitude, airspeed)
    
    # create mission instances (for multiple types of missions)
    missions = missions_setup(mission)
    
    results  = missions.base_mission.evaluate() 
         
    #------------------------------------------------------------------------  
    # Compute Loading Points 
    #------------------------------------------------------------------------
    percent_cargo        =  np.hstack((np.zeros(number_of_points), np.linspace(0, 1, number_of_points)))
    percent_pax          =  np.hstack((np.linspace(0, 1, number_of_points), np.ones(number_of_points)))
    percent_cargo        =  np.tile(percent_cargo, 2)
    percent_pax          =  np.tile(percent_pax, 2)
    fill_order           =  sorted( [ 'ascending', 'descending']*int(len(percent_pax)/2))
    percent_fuel         =  np.linspace(0, 1, number_of_points) 
    
    # create empty data structures 
    loading_mass                     = np.zeros((len(percent_cargo),len(percent_fuel)))
    loading_CG_location              = np.zeros((len(percent_cargo),len(percent_fuel)))
    loading_LEMAC_location           = np.zeros((len(percent_cargo),len(percent_fuel)))
    
    # compute mass properties of aircraft to get weight distribution
    vehicle_0         = mission.segments[0].analyses.weights.vehicle
    x_cg_0            = mission.segments[0].analyses.weights.vehicle.mass_properties.center_of_gravity
    weight_breakdown  = mission.segments[0].analyses.weights.vehicle.mass_properties.weight_breakdown 
    neutral_point_0   = mission.segments[0].analyses.stability.vehicle.neutral_point
                        
     
    CARGO =  weight_breakdown.payload.cargo 
    BAG   =  weight_breakdown.payload.baggage 
    PAX   =  weight_breakdown.payload.passengers 
    MTOW  =  vehicle_0.mass_properties.max_takeoff
    OEW   =  weight_breakdown.empty.total
    MLW   =  estimate_maximum_landing_weight(MTOW)   

    # -------------------------------------------------------------------------
    # Load Diagram Data 
    # -------------------------------------------------------------------------     
    total_sims = len(percent_cargo) * len(percent_fuel)
    counter    = 0
    for i in range(len(percent_cargo)):
        for j in range(len(percent_fuel)):
 
            # Aircraft-Level Properties  
            vehicle.mass_properties.takeoff  = None # this ensures that the takeoff weight is computed
            vehicle.mass_properties.payload  = (BAG + PAX) * percent_pax[i] +  percent_cargo[i] *CARGO 
            vehicle.mass_properties.cargo    = percent_cargo[i] * CARGO
            pax                              = int(vehicle_0.number_of_passengers * percent_pax[i])
            vehicle.number_of_passengers     = np.maximum(1,pax)
 
            # Update Passengers           
            for fuselage in  vehicle.fuselages: 
                for cabin in fuselage.cabins:
                    cabin.filled_seats_arrangement  = fill_order[i]
                    for cabin_class in cabin.classes:
                        pax =  1 if i == 0 else int(percent_pax[i] * vehicle_0.fuselages[fuselage.tag].cabins[cabin.tag].classes[cabin_class.tag].number_of_passengers) 
                        cabin_class.number_of_passengers =  np.maximum(1,pax)
            for wing in vehicle.wings: 
                if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
                    for cabin in wing.cabins:    
                        cabin.filled_seats_arrangement  = fill_order[i]
                        for cabin_class in cabin.classes: 
                            pax =  1 if i == 0 else int(percent_pax[i] *  vehicle_0.fuselages[fuselage.tag].cabins[cabin.tag].classes[cabin_class.tag].number_of_passengers)                             
                            cabin_class.number_of_passengers  = np.maximum(1,pax)
             
            # Update Fuel            
            for network in  vehicle.networks:
                for fuel_line in  network.fuel_lines: 
                    for fuel_tank in fuel_line.fuel_tanks:
                        fuel_tank.fuel.mass_properties.mass = percent_fuel[j] * vehicle_0.networks[network.tag].fuel_lines[fuel_line.tag].fuel_tanks[fuel_tank.tag].fuel.mass_properties.mass
        
            #  run mission
            configs  = configs_setup(vehicle) 
            analyses = analyses_setup(configs, aerodynamic_analysis, stability_analysis,  weights_analysis, overwrite_fuel_volume = False) 
            mission  = mission_setup(analyses, altitude, airspeed) 
            missions = missions_setup(mission)
            
            geometry(missions.base_mission)
            mass_properties(missions.base_mission) 
            
            # store results 
            loading_CG_location[i,j]         = mission.segments[0].analyses.weights.vehicle.mass_properties.center_of_gravity[0][0] 
            loading_mass[i,j]                = mission.segments[0].analyses.weights.vehicle.mass_properties.takeoff 
            loading_LEMAC_location[i,j]      = 100 * (loading_CG_location[i,j] - mission.segments[0].analyses.weights.vehicle.LEMAC) / mission.segments[0].analyses.aerodynamics.vehicle.reference_chord
            
            print('***************************************')
            print('Loading Diagram Data: ' + str(counter+1) + ' of ' +  str(total_sims))
            print('Mass                : ', loading_mass[i,j])
            print('Percent Fuel        : ', percent_fuel[j]*100 )
            print('Percent Cargo       : ', percent_cargo[i]*100 )
            print('Percent Pax         : ', percent_pax[i]*100 )
            print('LEMAC               : ', loading_LEMAC_location[i,j] )
            print('***************************************')
             

            counter += 1
          
    # -------------------------------------------------------------------------
    # Trim Diagram Data
    # ------------------------------------------------------------------------- 
    percent_mass                     = np.linspace(0,1, number_of_points)
    percent_cg_shift                 = np.linspace(0.8,1.2, number_of_points)
    aerodynamic_lift_coefficient     = np.zeros((len(percent_mass),len(percent_cg_shift)))
    aerodynamic_drag_coefficient     = np.zeros((len(percent_mass),len(percent_cg_shift)))
    aerodynamic_moment_coefficient   = np.zeros((len(percent_mass),len(percent_cg_shift)))
    aerodynamic_neutral_point        = np.zeros((len(percent_mass),len(percent_cg_shift)))
    aerodynamic_static_margin        = np.zeros((len(percent_mass),len(percent_cg_shift)))
    aerodynamic_moment               = np.zeros((len(percent_mass),len(percent_cg_shift))) 
    aerodynamic_mass                 = np.zeros((len(percent_mass),len(percent_cg_shift))) 
    aerodynamic_LEMAC_location       = np.zeros((len(percent_mass),len(percent_cg_shift))) 

    total_sims = len(percent_mass) * len(percent_cg_shift)
    counter    = 0        
    for k in range(len(percent_mass)):
        for l in range(len(percent_cg_shift)):
    
            # Aircraft-Level Properties  
            vehicle.mass_properties.takeoff                 = None # this ensures that the takeoff weight is computed 
            vehicle.mass_properties.payload                 = (BAG + PAX) * percent_mass[k] +  percent_mass[k] *CARGO 
            vehicle.mass_properties.cargo                   = percent_mass[k] * CARGO
            vehicle.mass_properties.center_of_gravity[0][0] = x_cg_0[0][0] * percent_cg_shift[l] 
            
            pax = int(vehicle_0.number_of_passengers *  percent_mass[k])
            vehicle.number_of_passengers     = np.maximum(1,pax)
    
            # Update Passengers           
            for fuselage in  vehicle.fuselages: 
                for cabin in fuselage.cabins:
                    cabin.filled_seats_arrangement  = fill_order[i]
                    for cabin_class in cabin.classes:
                        pax =  1 if i == 0 else int( percent_mass[k] * vehicle_0.fuselages[fuselage.tag].cabins[cabin.tag].classes[cabin_class.tag].number_of_passengers) 
                        cabin_class.number_of_passengers =  np.maximum(1,pax)
            for wing in vehicle.wings: 
                if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
                    for cabin in wing.cabins:    
                        cabin.filled_seats_arrangement  = fill_order[i]
                        for cabin_class in cabin.classes: 
                            pax =  1 if i == 0 else int(percent_mass[k] *  vehicle_0.fuselages[fuselage.tag].cabins[cabin.tag].classes[cabin_class.tag].number_of_passengers)                             
                            cabin_class.number_of_passengers  = np.maximum(1,pax)
             
            # Update Fuel            
            for network in  vehicle.networks:
                for fuel_line in  network.fuel_lines: 
                    for fuel_tank in fuel_line.fuel_tanks:
                        fuel_tank.fuel.mass_properties.mass = percent_mass[k] * vehicle_0.networks[network.tag].fuel_lines[fuel_line.tag].fuel_tanks[fuel_tank.tag].fuel.mass_properties.mass
            
            #  run mission
            configs  = configs_setup(vehicle) 
            analyses = analyses_setup(configs, aerodynamic_analysis, stability_analysis, weights_analysis, overwrite_fuel_volume = False, update_center_of_gravity = False,neutral_point= neutral_point_0) 
            mission  = mission_setup(analyses, altitude, airspeed) 
            missions = missions_setup(mission) 
            results  = missions.base_mission.evaluate()
            
            # store results
            segment = results.segments['cruise']
            
            # store results 
            aerodynamic_lift_coefficient[k,l]    = segment.state.conditions.aerodynamics.coefficients.lift.total[0][0]  
            aerodynamic_drag_coefficient[k,l]    = segment.state.conditions.aerodynamics.coefficients.drag.total[0][0]  
            aerodynamic_moment_coefficient[k,l]  = segment.state.conditions.static_stability.coefficients.M[0][0]         
            aerodynamic_moment[k,l]              = segment.state.conditions.frames.inertial.total_moment_vector[0][1]
            aerodynamic_neutral_point[k,l]       = segment.state.conditions.static_stability.neutral_point[0][0]  
            aerodynamic_static_margin[k,l]       = segment.state.conditions.static_stability.static_margin[0][0]    
            aerodynamic_mass[k,l]                = mission.segments[0].analyses.weights.vehicle.mass_properties.takeoff 
            aerodynamic_LEMAC_location[k,l]      = 100 * (vehicle.mass_properties.center_of_gravity[0][0] - mission.segments[0].analyses.weights.vehicle.LEMAC) / mission.segments[0].analyses.aerodynamics.vehicle.reference_chord
            
            counter += 1
            print('***************************************')
            print('Trim Diagram Data: ' + str(counter) + ' of ' +  str(total_sims))
            print('Center of Gravity : ',vehicle.mass_properties.center_of_gravity[0][0])
            print('Neutral Point     : ',aerodynamic_neutral_point[k,l])
            print('Static Margin     : ',aerodynamic_static_margin[k,l])
            print('Percent Mass      : ',percent_mass[k]*100 ) 
            print('Mass              : ', aerodynamic_mass[k,l] ) 
            print('***************************************')
 
  
    RES = Data(
               number_of_points                = number_of_points,
               loading_CG_location             = loading_CG_location, 
               loading_mass                    = loading_mass,
               loading_LEMAC_location          = loading_LEMAC_location, 
               aerodynamic_lift_coefficient    = aerodynamic_lift_coefficient,
               aerodynamic_drag_coefficient    = aerodynamic_drag_coefficient,  
               aerodynamic_moment_coefficient  = aerodynamic_moment_coefficient, 
               aerodynamic_moment              = aerodynamic_moment,            
               aerodynamic_neutral_point       = aerodynamic_neutral_point,     
               aerodynamic_static_margin       = aerodynamic_static_margin,
               aerodynamic_mass                = aerodynamic_mass,           
               aerodynamic_LEMAC_location      = aerodynamic_LEMAC_location,   
               MTOW                            = MTOW,          
               OEW                             = OEW, 
               MLW                             = MLW,  
               )
    
    return RES  
 
 
def configs_setup(vehicle): 
    configs     = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base'  
    configs.append(base_config) 
    return configs
  
def analyses_setup(configs, aerodynamics,stability, weights,overwrite_fuel_volume=True, update_center_of_gravity=True, neutral_point = None,):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config, aerodynamics,stability, weights, overwrite_fuel_volume, update_center_of_gravity,neutral_point)
        analyses[tag] = analysis

    return analyses
 
def base_analysis(vehicle, aerodynamics,stability, weights,overwrite_fuel_volume, update_center_of_gravity,neutral_point):
    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle() 
    
    #  Geometry
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.vehicle = vehicle  
    geometry.settings.overwrite_fuel_volume = overwrite_fuel_volume     
    analyses.append(geometry)

     # ------------------------------------------------------------------
    #  Weights 
    weights.vehicle                              = vehicle 
    weights.settings.FLOPS.fidelity              = 'Complex' 
    weights.settings.update_moment_of_inertia    = update_center_of_gravity 
    weights.settings.update_center_of_gravity    = update_center_of_gravity
    weights.print_weight_analysis_report         = False
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis  
    aerodynamics.vehicle  = vehicle 
    aerodynamics.vehicle.neutral_point = neutral_point     
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis  
    stability.vehicle                            = vehicle
    stability.vehicle.neutral_point              = neutral_point
    stability.settings.update_center_of_gravity  = update_center_of_gravity
    analyses.append(stability)       

    # ------------------------------------------------------------------
    #  Energy
    energy          = RCAIDE.Framework.Analyses.Energy.Energy()
    energy.vehicle  = vehicle 
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   

    # done!
    return analyses    

def mission_setup(analyses, altitude, airspeed): 
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'
  
    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()
    base_segment.state.numerics.solver.type = 'root_finder'

    # ------------------------------------------------------------------    
    #   Cruise Segment 
    # ------------------------------------------------------------------    

    segment = Segments.Single_Point.Set_Speed_Set_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.base )  
    segment.altitude  =  35000 *  Units.ft
    segment.air_speed =  450 * Units['knots'] 
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]   
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment) 
 

    return mission

def missions_setup(mission): 
 
    missions     = RCAIDE.Framework.Mission.Missions() 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions   
 
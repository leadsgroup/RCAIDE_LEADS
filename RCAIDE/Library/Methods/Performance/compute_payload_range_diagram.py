# RCAIDE/Library/Methods/Performance/compute_payload_range_diagram.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core import Units , Data  
from RCAIDE.Library.Plots.Common import set_axes, plot_style    
 
# Pacakge imports 
import numpy as np
from matplotlib import pyplot as plt
 
# ----------------------------------------------------------------------
#  Calculate vehicle Payload Range Diagram
# ----------------------------------------------------------------------  
def compute_payload_range_diagram(vehicle,assigned_propulsors,weights_analysis,aerodynamics_analysis,cruise_airspeed = 515*Units.mph, cruise_altitude= 35000*Units.feet,max_range_guess = 1000*Units.nmi, reserves=0., plot_diagram = True, fuel_name=None):  
    """Calculates and plots the payload range diagram for an aircraft by modifying the cruise segment and weights of the aicraft .

        Sources:
        N/A

        Assumptions:
        None 

        Inputs:
            vehicle             data structure for aircraft                  [-]
            mission             data structure for mission                   [-] 
            cruise_segment_tag  string of cruise segment                     [string]
            reserves            reserve fuel                                 [unitless] 
            
        Outputs: 
            payload_range       data structure of payload range properties   [m/s]
    """ 

    configs  =  configs_setup(vehicle)

    # create analyses
    analyses =  analyses_setup(configs)              
    
    for network in vehicle.networks:
        if type(network) == RCAIDE.Framework.Networks.Fuel:
            electric_aircraft =  False
            
            # mission analyses 
            mission =  mission_setup(analyses,electric_aircraft,assigned_propulsors,cruise_airspeed, cruise_altitude, max_range_guess)
            
            # create mission instances (for multiple types of missions)
            missions = missions_setup(mission) 
            cruise_segment_tag = 'cruise'
            payload_range  =  conventional_payload_range_diagram(vehicle, missions.base_mission,cruise_segment_tag,reserves,plot_diagram,fuel_name) 
        elif type(network) == RCAIDE.Framework.Networks.Electric:
            electric_aircraft =  True
             
            # mission analyses 
            mission =  mission_setup(analyses,electric_aircraft,assigned_propulsors,cruise_airspeed, cruise_altitude, max_range_guess)
            
            # create mission instances (for multiple types of missions)
            missions = missions_setup(mission) 
            cruise_segment_tag = 'cruise'
            payload_range  =  electric_payload_range_diagram(vehicle,missions.base_mission,cruise_segment_tag,plot_diagram)
    return payload_range 
             
def conventional_payload_range_diagram(vehicle,mission,cruise_segment_tag,reserves,plot_diagram, fuel_name): 
    """Calculates and plots the payload range diagram for a fuel-bases aircraft by modifying the
    cruise segment range and weights of the aicraft .

        Sources:
        N/A

        Assumptions:
        None 

        Inputs:
            vehicle             data structure for aircraft                  [-]
            mission             data structure for mission                   [-] 
            cruise_segment_tag  string of cruise segment                     [string]
            reserves            reserve fuel                                 [unitless] 
            
        Outputs: 
            payload_range       data structure of payload range properties   [m/s]
    """ 
    #unpack
    mass = vehicle.mass_properties
    if not mass.operating_empty:
        raise AttributeError("Error calculating Payload Range Diagram: Vehicle Operating Empty not defined") 
    else:
        OEW = mass.operating_empty

    if not mass.max_zero_fuel:
        raise AttributeError("Error calculating Payload Range Diagram: Vehicle MZFW not defined") 
    else:
        MZFW = vehicle.mass_properties.max_zero_fuel

    if not mass.max_takeoff:
        raise AttributeError("Error calculating Payload Range Diagram: Vehicle MTOW not defined") 
    else:
        MTOW = vehicle.mass_properties.max_takeoff

    if mass.max_payload == 0:
        MaxPLD = MZFW - OEW  
    else:
        MaxPLD = vehicle.mass_properties.max_payload
        MaxPLD = min(MaxPLD , MZFW - OEW) #limit in structural capability

    if mass.max_fuel == 0:
        MaxFuel = MTOW - OEW # If not defined, calculate based in design weights
    else:
        MaxFuel = vehicle.mass_properties.max_fuel  # If max fuel capacity not defined
        MaxFuel = min(MaxFuel, MTOW - OEW)


    # Define payload range points
    #Point  = [ RANGE WITH MAX. PLD   , RANGE WITH MAX. FUEL , FERRY RANGE   ]
    TOW     = [ MTOW                               , MTOW                   , OEW + MaxFuel ]
    FUEL    = [ min(TOW[1] - OEW - MaxPLD,MaxFuel) , MaxFuel                , MaxFuel       ]
    PLD     = [ MaxPLD                             , min(MTOW - MaxFuel - OEW, MaxPLD)   , 0.   ]
    
    # allocating Range array
    R       = [0,0,0]

    # loop for each point of Payload Range Diagram
    for i in range(len(TOW)):
        ##    for i in [2]: 
        # Define takeoff weight
        mission.segments[0].analyses.weights.vehicle.mass_properties.takeoff = TOW[i]

        # Evaluate mission with current TOW
        results = mission.evaluate()
        segment = results.segments[cruise_segment_tag]

        # Distance convergency in order to have total fuel equal to target fuel
        #
        # User don't have the option of run a mission for a given fuel. So, we
        # have to iterate distance in order to have total fuel equal to target fuel
        #

        maxIter = 10    # maximum iteration limit
        tol     = 1.    # fuel convergency tolerance
        err     = 9999. # error to be minimized
        iter    = 0     # iteration count

        while abs(err) > tol and iter < maxIter:
            iter = iter + 1

            # Current total fuel burned in mission
            TotalFuel  = TOW[i] - results.segments[-1].conditions.weights.total_mass[-1,0]

            # Difference between burned fuel and target fuel
            missingFuel = FUEL[i] - TotalFuel - reserves

            # Current distance and fuel consuption in the cruise segment
            CruiseDist = np.diff( segment.conditions.frames.inertial.position_vector[[0,-1],0] )[0]        # Distance [m]
            CruiseFuel = segment.conditions.weights.total_mass[0,0] - segment.conditions.weights.total_mass[-1,0]    # [kg]
            # Current specific range (m/kg)
            CruiseSR    = CruiseDist / CruiseFuel        # [m/kg]

            # Estimated distance that will result in total fuel burn = target fuel
            DeltaDist  =  CruiseSR *  missingFuel
            mission.segments[cruise_segment_tag].distance = (CruiseDist + DeltaDist)

            # running mission with new distance
            results = mission.evaluate()
            segment = results.segments[cruise_segment_tag]

            # Difference between burned fuel and target fuel
            err = ( TOW[i] - results.segments[-1].conditions.weights.total_mass[-1,0] ) - FUEL[i] + reserves 

        # Allocating resulting range in ouput array.
        R[i] =  results.segments[-1].conditions.frames.inertial.position_vector[-1,0]   

    # Inserting point (0,0) in output arrays
    R.insert(0,0)
    PLD.insert(0,MaxPLD)
    FUEL.insert(0,0)
    TOW.insert(0,0)

    # packing results
    payload_range                = Data()
    payload_range.range          = np.array(R)
    payload_range.payload        = np.array(PLD)
    payload_range.fuel           = np.array(FUEL)
    payload_range.takeoff_weight = np.array(TOW)
    payload_range.reserves       = reserves
     
    if plot_diagram:  
        # get plotting style 
        ps      = plot_style()  
    
        parameters = {'axes.labelsize': ps.axis_font_size,
                      'xtick.labelsize': ps.axis_font_size,
                      'ytick.labelsize': ps.axis_font_size,
                      'axes.titlesize': ps.title_font_size}
        plt.rcParams.update(parameters)
        
        if fuel_name ==  None: 
            fig  = plt.figure('Fuel_Payload_Range_Diagram')
        else:
            fig  = plt.figure('Fuel_Payload_Range_Diagram for ' + fuel_name)
        axis = fig.add_subplot(1,1,1)
        axis.plot(payload_range.range /Units.nmi,payload_range.payload,color = 'k', linewidth = ps.line_width )
        axis.set_xlabel('Range (nautical miles)')
        axis.set_ylabel('Payload (kg)')
        axis.set_title("Fuel Payload Range Diagram") 
        fig.tight_layout()
        set_axes(axis) 

    return payload_range 
 
def electric_payload_range_diagram(vehicle,mission,cruise_segment_tag,plot_diagram):
    """Calculates and plots the payload range diagram for an electric aircraft by modifying the
    cruise segment distance and payload weight of the aicraft .

        Sources:
        N/A

        Assumptions:
        None 

        Inputs:
            vehicle             data structure for aircraft                  [-]
            mission             data structure for mission                   [-] 
            cruise_segment_tag  string of cruise segment                     [string]
            reserves            reserve fuel                                 [unitless] 
            
        Outputs: 
            payload_range       data structure of payload range properties   [m/s]
    """ 
    mass = vehicle.mass_properties
    if not mass.operating_empty:
        print("Error calculating Payload Range Diagram: vehicle Operating Empty Weight is undefined.")
        return True
    else:
        OEW = mass.operating_empty

    if not mass.max_payload:
        print("Error calculating Payload Range Diagram: vehicle Maximum Payload Weight is undefined.")
        return True
    else:
        MaxPLD = mass.max_payload

    if not mass.max_takeoff:
        print("Error calculating Payload Range Diagram: vehicle Maximum Payload Weight is undefined.")
        return True
    else:
        MTOW = mass.max_takeoff

    # Define Diagram Points
    # Point = [Value at Maximum Payload Range,  Value at Ferry Range]
    TOW =   [MTOW,      OEW]    # Takeoff Weights
    PLD =   [MaxPLD,    0.]     # Payload Weights

    # Initialize Range Array
    R = np.zeros(2)

    # Calculate Vehicle Range for Max Payload and Ferry Conditions
    for i in range(2):
        mission.segments[0].analyses.weights.vehicle.mass_properties.takeoff = TOW[i]
        results = mission.evaluate()
        segment = results.segments[cruise_segment_tag]
        R[i]    = segment.conditions.frames.inertial.position_vector[-1,0] 

    # Insert Starting Point for Diagram Construction
    R   = np.insert(R, 0, 0)
    PLD = np.insert(PLD, 0, MaxPLD)
    TOW = np.insert(TOW, 0, 0)

    # Pack Results
    payload_range = Data()
    payload_range.range             = np.array(R)
    payload_range.payload           = np.array(PLD)
    payload_range.takeoff_weight    = np.array(TOW)

    if plot_diagram: 
        # get plotting style 
        ps      = plot_style()  
    
        parameters = {'axes.labelsize': ps.axis_font_size,
                      'xtick.labelsize': ps.axis_font_size,
                      'ytick.labelsize': ps.axis_font_size,
                      'axes.titlesize': ps.title_font_size}
        plt.rcParams.update(parameters)

        fig  = plt.figure('Electric_Payload_Range_Diagram')
        axis = fig.add_subplot(1,1,1)        
        axis.plot(payload_range.range /Units.nmi, payload_range.payload,color = 'k', linewidth = ps.line_width )
        axis.set_xlabel('Range (nautical miles)')
        axis.set_ylabel('Payload (kg)')
        axis.set_title('Payload Range Diagram')
        set_axes(axis) 
        fig.tight_layout()

    return payload_range

# ---------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    """This function sets up vehicle configurations for use in different parts of the mission.
    Here, this is mostly in terms of high lift settings."""

    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs     = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base' 
    configs.append(base_config)

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'cruise'
    configs.append(config) 

    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'takeoff'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 30. * Units.deg 
    config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity =  3470. * Units.rpm
    config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity      =  3470. * Units.rpm 
    config.V2_VS_ratio = 1.21
    configs.append(config)


    # ------------------------------------------------------------------
    #   Cutback Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'cutback'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 20. * Units.deg
    config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity =  2780. * Units.rpm
    config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity      =  2780. * Units.rpm 
    configs.append(config)   



    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'landing'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 30. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg
    config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity =  2030. * Units.rpm
    config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity      =  2030. * Units.rpm
    config.landing_gears.main_gear.gear_extended    = True
    config.landing_gears.nose_gear.gear_extended    = True  
    config.Vref_VS_ratio = 1.23
    configs.append(config)   

    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------ 

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'short_field_takeoff'    
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg
    config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity =  3470. * Units.rpm
    config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity      =  3470. * Units.rpm 
    config.landing_gears.main_gear.gear_extended    = True
    config.landing_gears.nose_gear.gear_extended    = True  
    config.V2_VS_ratio = 1.21 
    configs.append(config)

    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------  

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'reverse_thrust'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 30. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg 
    config.landing_gears.main_gear.gear_extended    = True
    config.landing_gears.nose_gear.gear_extended    = True  
    configs.append(config)    



    return configs

  
def analyses_setup(configs):
    """Set up analyses for each of the different configurations."""

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # Build a base analysis for each configuration. Here the base analysis is always used, but
    # this can be modified if desired for other cases.
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

def base_analysis(vehicle):
    """This is the baseline set of analyses to be used with this vehicle. Of these, the most
    commonly changed are the weights and aerodynamics methods."""

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Weights
    weights = RCAIDE.Framework.Analyses.Weights.Conventional()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.vehicle = vehicle
    aerodynamics.settings.number_of_spanwise_vortices   = 40
    aerodynamics.settings.number_of_chordwise_vortices  = 4 
    aerodynamics.settings.fuselage_lift_correction      = 1
    aerodynamics.settings.trim_drag_correction_factor                        = 1.2
    aerodynamics.settings.wing_parasite_drag_form_factor                     = 1.5
    aerodynamics.settings.fuselage_parasite_drag_form_factor                 = 3.0
    aerodynamics.settings.viscous_lift_dependent_drag_factor                 = 1.0
    aerodynamics.training.angle_of_attack                     = np.array([-2. , 1E-20 , 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0]) * Units.deg 
    #aerodynamics.training.Mach                                          = np.array([0.1  ,0.3,  0.5,  0.65 , 0.85 , 0.9]) 
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    energy = RCAIDE.Framework.Analyses.Energy.Energy()
    energy.vehicle = vehicle 
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

    return analyses    

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
 
def mission_setup(analyses,electric_aircraft,assigned_propulsors, cruise_airspeed, cruise_altitude, max_range_guess): 
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'
  
    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()
    base_segment.state.numerics.number_of_control_points  = 3   


    #------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Inital_Climb" 
    segment.analyses.extend( analyses.cutback ) 
    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 1000  * Units['feet']
    segment.air_speed      = 200.0 * Units['knots']
    segment.climb_rate     = 1800   * Units['fpm']  

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_1" 
    segment.analyses.extend( analyses.cutback ) 
    segment.altitude_end   = 8000   * Units['ft']
    segment.air_speed_end  = 300 * Units['knots']
    segment.climb_rate     = 1700   * Units['fpm']  

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_2" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end   = 16000   * Units['ft']
    segment.air_speed  = 350 * Units['knots']
    segment.climb_rate     = 1300   * Units['fpm']  

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)


    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_3" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end   = 33000   * Units['ft']
    segment.air_speed  = 450 * Units['knots']
    segment.climb_rate     = 1000   * Units['fpm']  

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_4" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end   = cruise_altitude
    segment.air_speed      = cruise_airspeed
    segment.climb_rate     = 280   * Units['fpm']  

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)
    # ------------------------------------------------------------------    
    #   Cruise Segment 
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.cruise )
    if electric_aircraft: 
        segment.initial_battery_state_of_charge               = 1.0
     
    segment.altitude  = cruise_altitude
    segment.air_speed = cruise_airspeed
    segment.distance  = max_range_guess
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = assigned_propulsors
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment) 
 

 # ------------------------------------------------------------------
    #   First Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_1" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                  = 10000   * Units.ft
    segment.air_speed                                     = 380 * Units['knots']
    segment.descent_rate                                  = 1850   * Units['fpm']  

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag  = "approach" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                  = 2000 * Units.ft
    segment.air_speed                                     = 225.0 * Units['knots']
    segment.descent_rate                                  = 650  * Units['fpm']  

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "final_approach"  
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                  = 50.0   * Units.ft
    segment.air_speed                                     = 175.0 * Units['knots']
    segment.descent_rate                                  = 600.0   * Units['fpm']  

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    return mission

def missions_setup(mission): 
 
    missions     = RCAIDE.Framework.Mission.Missions() 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions   

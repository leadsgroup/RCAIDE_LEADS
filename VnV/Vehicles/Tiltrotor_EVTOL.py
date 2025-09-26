''' 
# Tiltrotor.py
# 
# Created: May 2019, M Clarke
#          Sep 2020, M. Clarke 

'''
#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units  
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Rotor  import design_electric_rotor
from RCAIDE.Library.Plots                                         import * 
from RCAIDE import  load 
from RCAIDE import  save  
from RCAIDE.Framework.External_Interfaces.OpenVSP.export_vsp_vehicle  import export_vsp_vehicle 

import os
import numpy as np 
from copy import deepcopy
import matplotlib.pyplot as plt 
import  pickle
# ----------------------------------------------------------------------------------------------------------------------
#  REGRESSION
# ----------------------------------------------------------------------------------------------------------------------  
def main(redesign_rotors=True):           
         
    # vehicle data
    new_geometry    = True
    if new_geometry :
        vehicle  = vehicle_setup(redesign_rotors)
        save_aircraft_geometry(vehicle , 'Tiltrotor')
    else: 
        vehicle = load_aircraft_geometry('Tiltrotor') 

    # Set up configs
    configs  = configs_setup(vehicle)

    # vehicle analyses
    analyses = analyses_setup(configs)

    # mission analyses
    mission  = mission_setup(analyses) 
    missions = missions_setup(mission) 
     
    results = missions.base_mission.evaluate() 
  
    plot_results(results)   

    return
 
def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        if config.networks.electric.propulsors['prop_rotor_propulsor_1'].rotor.orientation_euler_angles[1] > 45*Units.degrees: 
            analysis.aerodynamics.settings.drag_coefficient_increment =  0.25
        elif config.networks.electric.propulsors['prop_rotor_propulsor_1'].rotor.orientation_euler_angles[1] > 15*Units.degrees: 
            analysis.aerodynamics.settings.drag_coefficient_increment =  0.25
        analyses[tag] = analysis

    return analyses

def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle() 

    # ------------------------------------------------------------------
    #  Geometry
    # ------------------------------------------------------------------
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.vehicle                               = vehicle 
    geometry.settings.update_center_of_gravity     = True 
    analyses.append(geometry)

    # ------------------------------------------------------------------
    #  Weights
    weights         = RCAIDE.Framework.Analyses.Weights.Electric()
    weights.aircraft_type = "VTOL"
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics         = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.settings.maximum_lift_coefficient =  1.5
    aerodynamics.vehicle = vehicle 
    analyses.append(aerodynamics)
     
    # ------------------------------------------------------------------
    #  Stability Analysis
    stability         = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method() 
    stability.vehicle = vehicle 
    analyses.append(stability)    

    # ------------------------------------------------------------------
    #  Energy 
    energy          = RCAIDE.Framework.Analyses.Energy.Energy()
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

    # done!
    return analyses    



# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------
def vehicle_setup(redesign_rotors=True) : 

    ospath      = os.path.abspath(__file__)
    separator   = os.path.sep
    airfoil_path    = os.path.dirname(ospath) + separator
    local_path  = os.path.dirname(ospath) + separator          
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    vehicle                                   = RCAIDE.Vehicle()
    vehicle.tag                               = 'Tiltrotor'
    vehicle.configuration                     = 'eVTOL'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    # mass properties
    vehicle.mass_properties.takeoff           = 2100
    vehicle.mass_properties.operating_empty   = 2100      
    vehicle.mass_properties.max_takeoff       = 2100  
    vehicle.mass_properties.max_payload       = 350           
    vehicle.mass_properties.center_of_gravity = [[2.0144,   0.  ,  0. ]]      
    vehicle.mass_properties.moments_of_inertia.tensor = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    vehicle.reference_area                        = 10.39
    vehicle.flight_envelope.ultimate_load         = 5.7   
    vehicle.flight_envelope.positive_limit_load   = 3.  
    vehicle.passengers                            = 5


    #------------------------------------------------------------------------------------------------------------------------------------
    # ##################################################### Landing Gear ################################################################    
    #------------------------------------------------------------------------------------------------------------------------------------ 
    main_gear                                = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear() 
    main_gear.tire_diameter                  = 6  *  Units.inches 
    main_gear.rim_diameter                   = 3  *  Units.inches 
    main_gear.tire_width                     = 6  *  Units.inches 
    main_gear.strut_length                   = 12  * Units.ft 
    main_gear.wheels                         = 4   
    main_gear.number_of_gear_types_in_tandem = 1
    main_gear.number_of_wheels_in_gear_type  = 2
    main_gear.fairing                        = True
    main_gear.xz_plane_symmetric             = True
    main_gear.gear_extended                  = True
    vehicle.append_component(main_gear)  

    nose_gear                                = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()   
    nose_gear.tire_diameter                  =  5 *  Units.inches   
    nose_gear.rim_diameter                   =  3 *  Units.inches 
    nose_gear.tire_width                     =  5 *  Units.inches 
    nose_gear.strut_length                   =  6.* Units.ft 
    nose_gear.wheels                         = 2  
    nose_gear.fairing                        = True 
    nose_gear.gear_extended                  = True
    nose_gear.number_of_gear_types_in_tandem = 1
    nose_gear.number_of_wheels_in_gear_type  = 2    
    vehicle.append_component(nose_gear)

    #------------------------------------------------------------------------------------------------------------------------------------
    # ##################################################### Wings ################################################################    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # WING PROPERTIES           
    wing                                      = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                                  = 'main_wing'  
    wing.aspect_ratio                         = 9.11 
    wing.sweeps.quarter_chord                 = 0.0  
    wing.thickness_to_chord                   = 0.10
    wing.taper                                = 0.42 
    wing.spans.projected                      = 9.736 
    wing.chords.root                          = 1.57 
    # wing.total_length                         = 1.57  
    wing.chords.tip                           = 0.66 
    wing.chords.mean_aerodynamic              = 1.069 
    wing.dihedral                             = 0   * Units.degrees  
    wing.areas.reference                      = 10.39  
    wing.areas.wetted                         = 10.39 * 2.1   
    wing.areas.exposed                        = 10.39 * 0.9  
    wing.twists.root                          = 0   * Units.degrees  
    wing.twists.tip                           = 0   * Units.degrees   
    wing.origin                               = [[ 1.778,0 , 1.0 ]]
    wing.aerodynamic_center                   = [ 1.8 ,0 , 1.0 ]    
    wing.winglet_fraction                     = 0.0  
    wing.xz_plane_symmetric                   = True
    wing.vertical                             = False

    ospath                                = os.path.abspath(__file__)
    separator                             = os.path.sep
    rel_path                              = os.path.dirname(ospath) + separator

    tip_airfoil                           = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    tip_airfoil.NACA_4_Series_code        = '6410'      
    tip_airfoil.coordinate_file           = rel_path + 'Airfoils' + separator + 'NACA_6410.txt' 
                                              
    # Segment                                              
    segment                                   = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                               = 'Section_1'   
    segment.percent_span_location             = 0.0
    segment.twist                             = 4.0  * Units.degrees
    segment.root_chord_percent                = 1 
    segment.dihedral_outboard                 = 8.  * Units.degrees
    segment.sweeps.quarter_chord              = 0. * Units.degrees 
    segment.thickness_to_chord                = 0.10 
    segment.append_airfoil(tip_airfoil)
    wing.append_segment(segment)                           
                                              
    # Segment                                               
    segment                                   = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                               = 'Section_2'    
    segment.percent_span_location             = 0.4875
    segment.twist                             = 4.0  * Units.degrees
    segment.root_chord_percent                = 0.6496
    segment.dihedral_outboard                 = 0. * Units.degrees
    segment.sweeps.quarter_chord              = 0. * Units.degrees
    segment.thickness_to_chord                = 0.10
    segment.append_airfoil(tip_airfoil)
    wing.append_segment(segment)                                 
                                              
    # Segment                                              
    segment                                   = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                               = 'Section_5'   
    segment.percent_span_location             = 1.0
    segment.twist                             = 0. 
    segment.root_chord_percent                = 0.42038
    segment.dihedral_outboard                 = 0.  * Units.degrees 
    segment.sweeps.quarter_chord              = 0.  * Units.degrees 
    segment.thickness_to_chord                = 0.10
    segment.append_airfoil(tip_airfoil)
    wing.append_segment(segment)    
    

    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.7
    aileron.span_fraction_end     = 0.9 
    aileron.deflection            = 0.0 * Units.degrees
    aileron.chord_fraction        = 0.2
    wing.append_control_surface(aileron)     
    
        
    # add to vehicle 
    vehicle.append_component(wing)  
                  
                                              
    # WING PROPERTIES                         
    wing                                      = RCAIDE.Library.Components.Wings.Horizontal_Tail() 
    wing.aspect_ratio                         = 4.27172 
    wing.sweeps.quarter_chord                 = 22.46  * Units.degrees 
    wing.thickness_to_chord                   = 0.15 
    wing.spans.projected                      = 3.6
    wing.chords.root                          = 1.193 
    wing.total_length                         = 1.193 
    wing.chords.tip                           = 0.535 
    wing.taper                                = 0.44  
    wing.chords.mean_aerodynamic              = 0.864 
    wing.dihedral                             = 45.0 * Units.degrees 
    wing.areas.reference                      = 4.25
    wing.areas.wetted                         = 4.25 * 2.1 
    wing.areas.exposed                        = 4.25 * 2 
    wing.twists.root                          = 0 * Units.degrees 
    wing.twists.tip                           = 0 * Units.degrees 
    wing.origin                               = [[ 5.167, 0.0 ,0.470 ]]
    wing.aerodynamic_center                   = [  5.267,  0., 0.470  ]  
    wing.winglet_fraction                     = 0.0 
    wing.xz_plane_symmetric                   = True    

    elevator                              = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
    elevator.tag                          = 'elevator'
    elevator.span_fraction_start          = 0.6
    elevator.span_fraction_end            = 0.9
    elevator.deflection                   = 0.0  * Units.deg
    elevator.chord_fraction               = 0.4
    wing.append_control_surface(elevator)       
    

    rudder                                = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    rudder.tag                            = 'rudder'
    rudder.span_fraction_start            = 0.1
    rudder.span_fraction_end              = 0.5
    rudder.deflection                     = 0.0  * Units.deg
    rudder.chord_fraction                 = 0.4
    wing.append_control_surface(rudder) 

    # add to vehicle
    vehicle.append_component(wing)   
    
     
  
    # ---------------------------------------------------------------   
    # FUSELAGE                
    # ---------------------------------------------------------------   
    # FUSELAGE PROPERTIES
    fuselage                                    = RCAIDE.Library.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage' 
    fuselage.seats_abreast                      = 2.  
    fuselage.seat_pitch                         = 3.  
    fuselage.fineness.nose                      = 0.88   
    fuselage.fineness.tail                      = 2.09  
    fuselage.lengths.nose                       = 0.5  
    fuselage.lengths.tail                       = 1.5
    fuselage.lengths.cabin                      = 4.46 
    fuselage.lengths.total                      = 6.46
    fuselage.width                              = 4.65 * Units.feet
    fuselage.heights.maximum                    = 5.31 * Units.feet      # change 
    fuselage.heights.at_quarter_length          = 5.31 * Units.feet      # change 
    fuselage.heights.at_wing_root_quarter_chord = 5.31 * Units.feet      # change 
    fuselage.heights.at_three_quarters_length   = 2.559* Units.feet      # change 
    fuselage.areas.wetted                       = 236. * Units.feet**2   # change 
    fuselage.areas.front_projected              = 16.98 * Units.feet**2   # change 
    fuselage.effective_diameter                 = 4.65 * Units.feet     # change 
    fuselage.differential_pressure              = 0. 
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_0'    
    segment.percent_x_location                  = 0.0 
    segment.percent_z_location                  = 0.     # change  
    segment.height                              = 0.049 
    segment.width                               = 0.032 
    fuselage.append_segment(segment)                     
                                                
    # Segment                                             
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_1'   
    segment.percent_x_location                  = 0.026  
    segment.percent_z_location                  = 0.00849
    segment.height                              = 0.481 
    segment.width                               = 0.553 
    fuselage.append_segment(segment)           
                                                
    # Segment                                             
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.074
    segment.percent_z_location                  = 0.02874
    segment.height                              = 1.00
    segment.width                               = 0.912 
    fuselage.append_segment(segment)                     
                                                
    # Segment                                            
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.161  
    segment.percent_z_location                  = 0.04348   
    segment.height                              = 1.41
    segment.width                               = 1.174  
    fuselage.append_segment(segment)                     
                                                
    # Segment                                             
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.284 
    segment.percent_z_location                  = 0.05435 
    segment.height                              = 1.62
    segment.width                               = 1.276  
    fuselage.append_segment(segment)              
                                                
    # Segment                                             
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 0.531 
    segment.percent_z_location                  = 0.0510 
    segment.height                              = 1.409
    segment.width                               = 1.121 
    fuselage.append_segment(segment)                     
                                                
    # Segment                                             
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 0.651
    segment.percent_z_location                  = 0.05636 
    segment.height                              = 1.11
    segment.width                               = 0.833
    fuselage.append_segment(segment)                  
                                                
    # Segment                                             
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.773
    segment.percent_z_location                  = 0.06149 
    segment.height                              = 0.78
    segment.width                               = 0.512 
    fuselage.append_segment(segment)                  
                                                
    # Segment                                             
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 1.
    segment.percent_z_location                  = 0.07352  
    segment.height                              = 0.195  
    segment.width                               = 0.130 
    fuselage.append_segment(segment)                   
                                                
    vehicle.append_component(fuselage)  



    #------------------------------------------------------------------------------------------------------------------------------------
    # ########################################################  Energy Network  ######################################################### 
    #------------------------------------------------------------------------------------------------------------------------------------
    # define network
    network                                                = RCAIDE.Framework.Networks.Electric() 
    network.charging_power                                 = 1000
   
    #==================================================================================================================================== 
    # Tilt Rotor Bus 
    #====================================================================================================================================          
    bus                           = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    bus.tag                       = 'bus'
    bus.number_of_battery_modules =  4

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus Battery
    #------------------------------------------------------------------------------------------------------------------------------------ 
    battery_module                                                    = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC() 
    battery_module.tag                                                = 'bus_battery'
    battery_module.electrical_configuration.series                    = 60
    battery_module.electrical_configuration.parallel                  = 60 #100                
    battery_module.geometrtic_configuration.normal_count              = 140
    battery_module.geometrtic_configuration.parallel_count            = 25
    battery_module.geometrtic_configuration.stacking_rows             = 2
    
                       # starboard   | port        | front  | rear 
    modules_origins = [[1.8, 2.0,1.0 ],[1.8, -2.0, 1.0  ],[0.5, 0.0, 0.0 ],[3.5, 0.0, 0.0]]  
    orientation     = [[0, 0.0, np.pi],[0, 0.0, np.pi ],[0, 0.0, 0 ],[0, 0.0,0 ]]   
    for m_i in range(bus.number_of_battery_modules):
        module =  deepcopy(battery_module)
        module.tag = 'nmc_module_' + str(m_i+1) 
        module.origin = [modules_origins[m_i]] 
        module.orientation_euler_angles  = orientation[m_i]  
        bus.battery_modules.append(module) 
    bus.initialize_bus_properties()    
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Lift Propulsors 
    #------------------------------------------------------------------------------------------------------------------------------------    
     
    # Define Lift Propulsor Container 
    propulsor                                     = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()
    propulsor.tag                                 = 'propulsor'       
              
    # Electronic Speed Controller           
    prop_rotor_esc                                = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
    prop_rotor_esc.efficiency                     = 0.95
    prop_rotor_esc.bus_voltage                    = bus.voltage
    prop_rotor_esc.tag                            = 'prop_rotor_esc_1'  
    propulsor.electronic_speed_controller         = prop_rotor_esc  
    
    # Lift Rotor Design
    g                                             = 9.81                                    # gravitational acceleration   
    Hover_Load                                    = vehicle.mass_properties.max_takeoff*g *1.1  # hover load   

    prop_rotor                                    = RCAIDE.Library.Components.Powertrain.Converters.Prop_Rotor()   
    prop_rotor.tag                                = 'prop_rotor'   
    prop_rotor.tip_radius                         = 3/2
    prop_rotor.hub_radius                         = 0.15 * prop_rotor.tip_radius
    prop_rotor.number_of_blades                   = 4

    prop_rotor.hover.design_altitude              = 40 * Units.feet  
    prop_rotor.hover.design_thrust                = Hover_Load/6
    prop_rotor.hover.design_freestream_velocity   = np.sqrt(prop_rotor.hover.design_thrust/(2*1.2*np.pi*(prop_rotor.tip_radius**2)))
    
    prop_rotor.oei.design_altitude                = 40 * Units.feet  
    prop_rotor.oei.design_thrust                  = Hover_Load/5  
    prop_rotor.oei.design_freestream_velocity     = np.sqrt(prop_rotor.oei.design_thrust/(2*1.2*np.pi*(prop_rotor.tip_radius**2)))
    
    prop_rotor.cruise.design_altitude             = 1500 * Units.feet
    prop_rotor.cruise.design_thrust               = 3150 / 6 # Assumed a lift to drag ratio of 6.78
    prop_rotor.cruise.design_freestream_velocity  = 200.* Units['mph']   
    airfoil                                       = RCAIDE.Library.Components.Airfoils.Airfoil()   
    airfoil.coordinate_file                       =  local_path + 'Airfoils' + separator + 'NACA_4412.txt'
    airfoil.polar_files                           = [local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_50000.txt' ,
                                                     local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_100000.txt' ,
                                                     local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_200000.txt' ,
                                                     local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_500000.txt' ,
                                                     local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_1000000.txt',
                                                     local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_3500000.txt',
                                                     local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_5000000.txt',
                                                     local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_7500000.txt' ]
    prop_rotor.append_airfoil(airfoil)                
    prop_rotor.airfoil_polar_stations             = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    propulsor.rotor = prop_rotor   
    
    
 
    #------------------------------------------------------------------------------------------------------------------------------------               
    # Lift Rotor Motor  
    #------------------------------------------------------------------------------------------------------------------------------------    
    prop_rotor_motor                         = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor()
    prop_rotor_motor.efficiency              = 0.98
    prop_rotor_motor.nominal_voltage         = bus.voltage 
    prop_rotor_motor.no_load_current         = 0.01  
    propulsor.motor                          = prop_rotor_motor

    #------------------------------------------------------------------------------------------------------------------------------------               
    # Lift Rotor Nacelle
    #------------------------------------------------------------------------------------------------------------------------------------     
    nacelle                           = RCAIDE.Library.Components.Nacelles.Nacelle() 
    nacelle.length                    = 0.45
    nacelle.diameter                  = 0.3 
    nacelle.areas.wetted              = 10.0 * Units.feet**2
    nacelle.flow_through              = False 
    propulsor.nacelle                 = nacelle  

    current_dir = os.path.abspath(os.path.dirname(__file__))
    test_dir = os.path.abspath(os.path.join(current_dir, '..' + separator + 'Verification' + separator + 'mission_segments'))
     
            
    if redesign_rotors:
        design_electric_rotor(propulsor)
        save_propulsor(propulsor, os.path.join(test_dir, 'proprotor_propulsor.res'))
    else:
        regression_prop_rotor_propulsor = deepcopy(propulsor)        
        design_electric_rotor(regression_prop_rotor_propulsor, iterations=2)
        loaded_propulsor = load_propulsor(os.path.join(test_dir, 'proprotor_propulsor.res'))  
        for key,item in propulsor.rotor.items(): 
            propulsor.rotor[key] = loaded_propulsor.rotor[key] 
               
        propulsor.rotor.airfoils.airfoil.coordinate_file  =  local_path + 'Airfoils' + separator + 'NACA_4412.txt'
        propulsor.rotor.airfoils.airfoil.polar_files      = [local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_50000.txt' ,
                                                                        local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_100000.txt' ,
                                                                        local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_200000.txt' ,
                                                                        local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_500000.txt' ,
                                                                        local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_1000000.txt',
                                                                        local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_3500000.txt',
                                                                        local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_5000000.txt',
                                                                        local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_7500000.txt' ]
       
        for key,item in propulsor.motor.items(): 
            propulsor.motor[key] = loaded_propulsor.motor[key] 
         
    # Front Rotors Locations 
    origins =[[0.208, -1.848,  1.195],[0.208, 1.848,  1.195],
              [1.505,5.000,1.320],[1.505,-5.000,1.320],
              [  5.318, 1.848,   2.282],[   5.318, -1.848,   2.282]] 
     
    assigned_propulsor_list = []    
    for i in range(len(origins)): 
        propulsor_i                                       = deepcopy(propulsor)
        propulsor_i.tag                                   = 'prop_rotor_propulsor_' + str(i + 1)
        propulsor_i.rotor.tag                             = 'prop_rotor_' + str(i + 1) 
        propulsor_i.rotor.origin                          = [origins[i]]  
        propulsor_i.motor.tag                             = 'prop_rotor_motor_' + str(i + 1)   
        propulsor_i.motor.origin                          = [origins[i]]  
        propulsor_i.electronic_speed_controller.tag       = 'prop_rotor_esc_' + str(i + 1)  
        propulsor_i.electronic_speed_controller.origin    = [origins[i]]  
        propulsor_i.nacelle.tag                           = 'prop_rotor_nacelle_' + str(i + 1)  
        propulsor_i.nacelle.origin                        = [origins[i]]   
        network.propulsors.append(propulsor_i)   
        assigned_propulsor_list.append(propulsor_i.tag) 
    bus.assigned_propulsors = [assigned_propulsor_list]


    # Avionics                            
    avionics                        = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.power_draw             = 10. # Watts  
    avionics.mass_properties.mass   = 1.0 * Units.kg
    bus.avionics                    = avionics    
   
    network.busses.append(bus)
     
    # append energy network 
    vehicle.append_energy_network(network)     

    return vehicle 
             
 
def configs_setup(vehicle):
    '''
    The configration set up below the scheduling of the nacelle angle and vehicle speed.
    Since one prop_rotor operates at varying flight conditions, one must perscribe  the 
    pitch command of the prop_rotor which us used in the variable pitch model in the analyses
    Note: low pitch at take off & low speeds, high pitch at cruise
    '''
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------ 
    configs = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config                                                       = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag                                                   = 'base'     
    configs.append(base_config) 
     
 
    # ------------------------------------------------------------------
    #   Hover Climb Configuration
    # ------------------------------------------------------------------
    config                                                 = RCAIDE.Library.Components.Configs.Config(vehicle)
    config.tag                                             = 'vertical_flight'
    vector_angle                                           = 90.0 * Units.degrees    
    for network in  config.networks:  
        for propulsor in  network.propulsors:
            propulsor.rotor.orientation_euler_angles =  [0, vector_angle, 0] 
    configs.append(config)  
    

    # ------------------------------------------------------------------
    #  Transition Setting 1
    # ------------------------------------------------------------------
    config                                            = RCAIDE.Library.Components.Configs.Config(vehicle)
    vector_angle                                      = 85.0  * Units.degrees   
    config.tag                                        = 'transition_setting_1' 
    for network in  config.networks:  
        for propulsor in  network.propulsors:
            propulsor.rotor.orientation_euler_angles = [0, vector_angle, 0] 
            propulsor.rotor.blade_pitch_command      = propulsor.rotor.cruise.design_blade_pitch_command * 0.25  
    configs.append(config) 
    
    # ------------------------------------------------------------------
    #  Transition Setting 2
    # ------------------------------------------------------------------
    config                                            = RCAIDE.Library.Components.Configs.Config(vehicle)
    vector_angle                                      = 75.0  * Units.degrees   
    config.tag                                        = 'transition_setting_2' 
    for network in  config.networks:  
        for propulsor in  network.propulsors:
            propulsor.rotor.orientation_euler_angles =  [0, vector_angle, 0] 
    configs.append(config)   

    # ------------------------------------------------------------------
    #  Transition Setting 4
    # ------------------------------------------------------------------
    config                                            = RCAIDE.Library.Components.Configs.Config(vehicle)
    config.tag                                        = 'transition_setting_3'   
    vector_angle                                      = 0.0 * Units.degrees   
    for network in  config.networks:  
        for propulsor in  network.propulsors:
            propulsor.rotor.orientation_euler_angles =  [0, vector_angle, 0]
            propulsor.rotor.blade_pitch_command   = propulsor.rotor.cruise.design_blade_pitch_command * 0.5   
    configs.append(config)
        

    # ------------------------------------------------------------------
    # Low Speed Transition  
    # ------------------------------------------------------------------
    config                                            = RCAIDE.Library.Components.Configs.Config(vehicle)
    vector_angle                                      = 70.0  * Units.degrees   
    config.tag                                        = 'low_speed_transition' 
    for network in  config.networks:  
        for propulsor in  network.propulsors:
            propulsor.rotor.orientation_euler_angles =  [0, vector_angle, 0]  
    configs.append(config)
    


    # ------------------------------------------------------------------
    # Medium Speed Transition 
    # ------------------------------------------------------------------
    config                                            = RCAIDE.Library.Components.Configs.Config(vehicle) 
    config.tag                                        = 'medium_speed_transition' 
    vector_angle                                      = 20.0  * Units.degrees   
    for network in  config.networks:  
        for propulsor in  network.propulsors:
            propulsor.rotor.orientation_euler_angles =  [0, vector_angle, 0]  
    configs.append(config)
    

    # ------------------------------------------------------------------
    # High Speed Transition 
    # ------------------------------------------------------------------
    config                                            = RCAIDE.Library.Components.Configs.Config(vehicle) 
    config.tag                                        = 'high_speed_transition'  
    for network in  config.networks:  
        for propulsor in  network.propulsors:
            propulsor.rotor.orientation_euler_angles =  [0, vector_angle, 0]  
    configs.append(config)       

 
    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------
    config                                            = RCAIDE.Library.Components.Configs.Config(vehicle)
    config.tag                                        = 'cruise'   
    vector_angle                                      = 0.0 * Units.degrees   
    for network in  config.networks:  
        for propulsor in  network.propulsors:
            propulsor.rotor.orientation_euler_angles =  [0, vector_angle, 0]
            propulsor.rotor.blade_pitch_command   = propulsor.rotor.cruise.design_blade_pitch_command  
    configs.append(config)
             

    return configs


# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
def mission_setup(analyses):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments  
    base_segment = Segments.Segment() 
    base_segment.state.numerics.solver.type = 'optimize' 
    

    beta_cruise = analyses.low_speed_transition.energy.vehicle.networks.electric.propulsors.prop_rotor_propulsor_1.rotor.cruise.design_blade_pitch_command
    
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------ 
    segment                                            = Segments.Vertical_Flight.Climb(base_segment)
    segment.tag                                        = "Vertical_Climb"   
    segment.analyses.extend(analyses.vertical_flight) 
    segment.altitude_start                             = 0.0  * Units.ft  
    segment.altitude_end                               = 50.  * Units.ft   
    segment.climb_rate                                 = 300. * Units['ft/min'] 
    segment.initial_battery_state_of_charge            = 1.0 
    segment.true_course                                = 0   * Units.degree  
    segment.state.numerics.solver.type = 'root_finder' 

    # define flight dynamics to model  
    segment.flight_dynamics.force_z                    = True 

    # define flight controls  
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    segment.assigned_control_variables.throttle.initial_guess_values = [[0.8]]
    
    mission.append_segment(segment)   
     
    # ------------------------------------------------------------------
    #  First Transition Segment
    # ------------------------------------------------------------------ 
    segment                                               = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag                                           = "departure_transition_1"  
    segment.analyses.extend( analyses.transition_setting_1)   
    segment.air_speed_start                               = 10 * Units['knots']     
    segment.air_speed_end                                 = 35 * Units['mph']     
    segment.acceleration                                  = 1
    segment.true_course                                   = 0 * Units.degree
    segment.state.numerics.solver.type                    = 'root_finder'
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active                                = True           
    segment.assigned_control_variables.throttle.assigned_propulsors                   = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                                          'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    segment.assigned_control_variables.body_angle.active             = True  
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   First Cruise Segment: Constant Acceleration, Constant Altitude
    # ------------------------------------------------------------------ 
    segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                      = "departure_transition_2"  
    segment.analyses.extend(analyses.transition_setting_2)   
    segment.climb_rate               = 728. * Units['ft/min']   
    segment.air_speed_start          = 35 * Units['mph']   
    segment.air_speed_end            = 100 * Units['mph']   
    segment.altitude_end             = 500.0 * Units.ft
    segment.true_course              = 0 * Units.degree 
    segment.state.numerics.solver.type = 'root_finder' 

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    segment.assigned_control_variables.body_angle.active             = True 
                                                                             
    mission.append_segment(segment)   
 
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    # Circular departure pattern 
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment                                               = Segments.Cruise.Curved_Constant_Radius_Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                                           = "Departure_Pattern_Curve"   
    segment.analyses.extend( analyses.transition_setting_2 )            
    segment.air_speed   = 90 * Units['knots']  
    segment.turn_radius = 4000 * Units.feet  
    segment.true_course = 0 * Units.degree     
    segment.turn_angle  = 90 * Units.degree
    segment.altitude    = 500 * Units.feet
    segment.state.numerics.solver.type = 'optimize'
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                             = True    
    segment.flight_dynamics.force_z                                             = True    
    segment.flight_dynamics.force_y                                             = True     
   
    # define flight controls              
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3','prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]   
    segment.assigned_control_variables.body_angle.active                        = True    
    segment.assigned_control_variables.bank_angle.active                        = True    
    segment.assigned_control_variables.bank_angle.initial_guess_values          = [[20.0 * Units.degree]]
    
    mission.append_segment(segment)  
     
    # ------------------------------------------------------------------
    #  Second Transition Segment
    # ------------------------------------------------------------------ 
    segment                           = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag                       = "departure_transition_3"  
    segment.analyses.extend(analyses.high_speed_transition)   
    segment.air_speed_end             = 125.  * Units['mph']  
    segment.acceleration              = 9.81/5 
    segment.true_course               = 90 * Units.degree
    

    segment.state.numerics.solver.step_size                 = 1E-2 
    segment.state.numerics.solver.tolerance_solution        = 1E-6 
    segment.state.numerics.solver.objective                 = None
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active                                = True           
    segment.assigned_control_variables.throttle.assigned_propulsors                   = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                                          'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    
    segment.assigned_control_variables.thrust_vector_angle.active                     = True        
    segment.assigned_control_variables.thrust_vector_angle.assigned_propulsors        =  [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                                        'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]   
    
    segment.assigned_control_variables.blade_pitch_command.active                     = True        
    segment.assigned_control_variables.blade_pitch_command.assigned_rotors            =  [['prop_rotor_1','prop_rotor_2','prop_rotor_3',
                                                                                        'prop_rotor_4','prop_rotor_5','prop_rotor_6']]   
    segment.assigned_control_variables.blade_pitch_command.bounds                     = [[0,beta_cruise ]] 
     
    mission.append_segment(segment)
     
    # ------------------------------------------------------------------
    #   First Cruise Segment: Constant Acceleration, Constant Altitude
    # ------------------------------------------------------------------ 
    segment                           = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                       = "climb"  
    segment.analyses.extend(analyses.cruise) 
    segment.altitude_start            = 500.0 * Units.ft
    segment.climb_rate                = 500. * Units['ft/min']
    segment.air_speed_start           = 125.  * Units['mph']   
    segment.air_speed_end             = 170.  * Units['mph']   
    segment.altitude_end              = 1000.0 * Units.ft 
    segment.true_course               = 90 * Units.degree  
    segment.state.numerics.solver.type = 'root_finder' 
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  =[['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]   
        
    segment.assigned_control_variables.body_angle.active             = True
    
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Cruise Segment: Constant Acceleration, Constant Altitude
    # ------------------------------------------------------------------ 
    segment                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                      = "cruise"  
    segment.analyses.extend(analyses.cruise) 
    segment.altitude                 = 1000.0 * Units.ft
    segment.air_speed                = 170.  * Units['mph']   
    segment.distance                 = 160*Units.nmi
    segment.true_course              = 90 * Units.degree  
    segment.state.numerics.solver.type = 'root_finder'  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                             = True    
    segment.flight_dynamics.force_z                                             = True    
    
    # define flight controls              
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3','prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]   
    segment.assigned_control_variables.body_angle.active                        = True    
    mission.append_segment(segment)    
    
    
    # ------------------------------------------------------------------
    #    Descent Segment: Constant Acceleration, Constant Altitude
    # ------------------------------------------------------------------ 
    segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                      = "descent"  
    segment.analyses.extend(analyses.transition_setting_3)
    segment.climb_rate               = -300. * Units['ft/min']
    segment.air_speed_start          = 170.  * Units['mph']   
    segment.air_speed_end            = 90 * Units.kts 
    segment.altitude_start           = 1000.0 * Units.ft
    segment.altitude_end             = 500.0 * Units.ft
    segment.true_course              = 90 * Units.degree  

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    segment.assigned_control_variables.body_angle.active             = True 
    mission.append_segment(segment)      
    
    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Circular departure pattern 
    #------------------------------------------------------------------------------------------------------------------------------------ 
    segment                                               = Segments.Cruise.Curved_Constant_Radius_Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                                           = "Approach_Pattern_Curve"   
    segment.analyses.extend( analyses.transition_setting_2 )             
    segment.air_speed   = 90 * Units.kts 
    segment.turn_radius = 4000 * Units.feet  
    segment.true_course = 90 * Units.degree     
    segment.turn_angle  = 90 * Units.degree
    segment.altitude    = 500 * Units.feet
    segment.state.numerics.solver.type = 'optimize' 
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                             = True    
    segment.flight_dynamics.force_z                                             = True    
    segment.flight_dynamics.force_y                                             = True    

    # define flight controls              
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                                    'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    segment.assigned_control_variables.body_angle.active                        = True    
    segment.assigned_control_variables.bank_angle.active                        = True    
    #segment.assigned_control_variables.bank_angle.initial_guess_values          = [[20.0 * Units.degree]]
    
    mission.append_segment(segment)  
    
    # ------------------------------------------------------------------
    #   First Cruise Segment: Constant Acceleration, Constant Altitude
    # ------------------------------------------------------------------ 
    segment                          = Segments.Descent.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                      = "arriving_transition_1" 
    segment.analyses.extend(analyses.low_speed_transition) 
    segment.descent_rate               = 728. * Units['ft/min'] 
    segment.air_speed_start            = 90 * Units.kts 
    segment.air_speed_end              = 35 * Units['mph']  
    segment.altitude_start             = 500.0 * Units.ft
    segment.altitude_end               = 50.0 * Units.ft
    segment.true_course                = 0 * Units.degree      
    segment.state.numerics.solver.step_size                 = 1E-2 
    segment.state.numerics.solver.tolerance_solution        = 1E-6 
    segment.state.numerics.solver.objective                 = None

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active                                = True           
    segment.assigned_control_variables.throttle.assigned_propulsors                   = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                                          'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    
    segment.assigned_control_variables.thrust_vector_angle.active                     = True        
    segment.assigned_control_variables.thrust_vector_angle.assigned_propulsors        =  [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                                        'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]   
    
    segment.assigned_control_variables.blade_pitch_command.active                     = True        
    segment.assigned_control_variables.blade_pitch_command.assigned_rotors            =  [['prop_rotor_1','prop_rotor_2','prop_rotor_3',
                                                                                        'prop_rotor_4','prop_rotor_5','prop_rotor_6']]   
    segment.assigned_control_variables.blade_pitch_command.bounds                     = [[0,beta_cruise ]]
                                                                             
    mission.append_segment(segment)   
    
    # ------------------------------------------------------------------
    #  Landing Transition Segment
    # ------------------------------------------------------------------ 
    segment                                               = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag                                           = "arriving_transition_2" 
    segment.analyses.extend( analyses.low_speed_transition)   
    segment.air_speed_start            = 35 * Units['mph'] 
    segment.air_speed_end              = 10 * Units['knots']     
    segment.acceleration               = -1.0
    segment.true_course                = 0 * Units.degree  
    segment.altitude                   = 50.0 * Units.ft
    segment.state.numerics.solver.type = 'optimize' 
 
    segment.state.numerics.solver.step_size                 = 1E-2 
    segment.state.numerics.solver.tolerance_solution        = 1E-6 
    segment.state.numerics.solver.objective                 = None

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active                                = True           
    segment.assigned_control_variables.throttle.assigned_propulsors                   = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                                          'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    
    segment.assigned_control_variables.thrust_vector_angle.active                     = True        
    segment.assigned_control_variables.thrust_vector_angle.assigned_propulsors        =  [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                                        'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]   
    
    segment.assigned_control_variables.blade_pitch_command.active                     = True        
    segment.assigned_control_variables.blade_pitch_command.assigned_rotors            =  [['prop_rotor_1','prop_rotor_2','prop_rotor_3',
                                                                                        'prop_rotor_4','prop_rotor_5','prop_rotor_6']]   
    segment.assigned_control_variables.blade_pitch_command.bounds                     = [[0,beta_cruise ]]
        
    mission.append_segment(segment)
    
    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Vertical Descent 
    #------------------------------------------------------------------------------------------------------------------------------------ 
    segment                                                         = Segments.Vertical_Flight.Descent(base_segment)
    segment.tag                                                     = "Vertical_Descent" 
    segment.analyses.extend( analyses.vertical_flight)                
    segment.altitude_end                                            = 0.   * Units.ft  
    segment.descent_rate                                            = 300. * Units['ft/min'] 
    segment.true_course                                             = 180 * Units.degree  
    segment.state.numerics.solver.type = 'root_finder' 
                  
    # define flight dynamics to model              
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
            
    mission.append_segment(segment)      
 
    return mission


def noise_mission_setup(number_of_cpts, analyses, radius_Vert1=4000*Units.ft, radius_Vert2=4000*Units.ft, dep_heading=200*Units.degrees, app_heading=90*Units.degrees, dep_sector=90*Units.degrees, app_sector=90*Units.degrees, path_heading = 100, level_cruise_distance=205*Units.miles,cruise_altitude=1000*Units.ft): 
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments  
    base_segment = Segments.Segment()
    base_segment.state.numerics.number_of_control_points    = number_of_cpts
    base_segment.state.numerics.solver.type = 'optimize' 
    
    # ------------------------------------------------------------------
    #   Mission Constants
    # ------------------------------------------------------------------
    hover_altitude = 50.0 * Units.ft
    
    pattern_speed    = 100. * Units['mph'] 
    cruise_speed     = 125. * Units['mph']
    transition_speed = 35.  * Units['mph']
    
    transition_climb_rate = 728.0 * Units['ft/min']
    
    cruise_climb_rate =  500. * Units['ft/min']
    pattern_altitude  =  500.0 * Units.ft
    
    # ------------------------------------------------------------------
    #   Vertical Climb from vertiport A
    # ------------------------------------------------------------------ 
    segment                                            = Segments.Vertical_Flight.Climb(base_segment)
    segment.tag                                        = "Vertical_Climb"   
    segment.analyses.extend(analyses.vertical_flight) 
    segment.altitude_start                             = 0.0  * Units.ft  
    segment.altitude_end                               = hover_altitude   
    segment.climb_rate                                 = 300. * Units['ft/min'] 
    segment.initial_battery_state_of_charge            = 1.0 
    segment.true_course                                = 0   * Units.degree # this is the true couse of the starting value  
    segment.state.numerics.solver.type = 'root_finder' 

    # define flight dynamics to model  
    segment.flight_dynamics.force_z                    = True 

    # define flight controls  
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    segment.assigned_control_variables.throttle.initial_guess_values = [[0.8]]
    
    mission.append_segment(segment)    
    
    # ------------------------------------------------------------------
    #  Horizontal Transition
    # ------------------------------------------------------------------ 
    segment                                               = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag                                           = "Horizontal_Transition"  
    segment.analyses.extend( analyses.transition_setting_2)   
    segment.air_speed_start                                 = 10 * Units['knots'] 
    segment.air_speed_end                                 = transition_speed    
    segment.acceleration                                  = 1.0
    segment.true_course                                   = dep_heading  
    segment.state.numerics.solver.type = 'optimize' 

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    segment.assigned_control_variables.body_angle.active             = True 
    
    mission.append_segment(segment)    
    
    # ------------------------------------------------------------------
    #  Transition and climb to departure pattern High-speed
    # ------------------------------------------------------------------ 
    segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                      = "Climb_Transition_1" 
    segment.analyses.extend(analyses.transition_setting_2) 
    segment.climb_rate               = transition_climb_rate 
    segment.air_speed_end            = (pattern_speed - transition_speed)/2 + transition_speed
    segment.altitude_end             = (pattern_altitude - hover_altitude)/2 + hover_altitude
    segment.true_course              = dep_heading
    segment.state.numerics.solver.type = 'optimize'

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    segment.assigned_control_variables.body_angle.active             = True 
                                                                             
    mission.append_segment(segment)     

    # ------------------------------------------------------------------
    #  Transition and climb to departure pattern High-speed
    # ------------------------------------------------------------------ 
    segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                      = "Climb_Transition_2" 
    segment.analyses.extend(analyses.transition_setting_4) 
    segment.climb_rate               = transition_climb_rate 
    segment.air_speed_end            = pattern_speed
    segment.altitude_end             = pattern_altitude
    segment.true_course              = dep_heading
    segment.state.numerics.solver.type = 'optimize'

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    segment.assigned_control_variables.body_angle.active             = True 
                                                                             
    mission.append_segment(segment)   
    
    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Circular departure pattern 
    #------------------------------------------------------------------------------------------------------------------------------------ 
    segment                                               = Segments.Cruise.Curved_Constant_Radius_Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                                           = "Departure_Pattern_Curve"   
    segment.analyses.extend( analyses.transition_setting_2 )           
    segment.air_speed   = pattern_speed
    segment.turn_radius = radius_Vert1
    segment.true_course = dep_heading + (90 * Units.degree)
    segment.turn_angle  = dep_sector
    segment.altitude    = pattern_altitude
    segment.state.numerics.solver.type = 'optimize'
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                             = True    
    segment.flight_dynamics.force_z                                             = True    
    segment.flight_dynamics.force_y                                             = True     
   
    # define flight controls              
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                                    'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]   
    segment.assigned_control_variables.body_angle.active                        = True    
    segment.assigned_control_variables.bank_angle.active                        = True    
    segment.assigned_control_variables.bank_angle.initial_guess_values          = [[20.0 * Units.degree]]
    
    mission.append_segment(segment)  
   
    # ------------------------------------------------------------------
    #   Climb to cruise altitude from pattern
    # ------------------------------------------------------------------ 
    segment                           = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                       = "Cruise_Climb"  
    segment.analyses.extend(analyses.cruise) 
    segment.climb_rate                = cruise_climb_rate
    segment.air_speed_start           = pattern_speed
    segment.air_speed_end             = cruise_speed
    segment.altitude_end              = cruise_altitude
    segment.true_course               = path_heading    
    segment.state.numerics.solver.type = 'root_finder'
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  =[['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]   
    segment.assigned_control_variables.body_angle.active             = True
    
    mission.append_segment(segment)
     
    # ------------------------------------------------------------------
    #  Cruise 
    # ------------------------------------------------------------------ 
    segment                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                      = "Cruise"  
    segment.analyses.extend(analyses.cruise) 
    segment.altitude                 = cruise_altitude
    segment.air_speed                = cruise_speed
    segment.distance                 = level_cruise_distance
    segment.true_course              = path_heading
    segment.state.numerics.solver.type = 'root_finder' 
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    segment.assigned_control_variables.body_angle.active             = True
    mission.append_segment(segment)     
    
    # ------------------------------------------------------------------
    #  Descent from cruise to approach pattern
    # ------------------------------------------------------------------ 
    segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                      = "Cruise_Descent"  
    segment.analyses.extend(analyses.cruise)
    segment.climb_rate               = -cruise_climb_rate 
    segment.air_speed_start          = cruise_speed
    segment.air_speed_end            = pattern_speed
    segment.altitude_start           = cruise_altitude
    segment.altitude_end             = pattern_altitude
    segment.true_course              = path_heading
    segment.state.numerics.solver.type = 'optimize' 

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    segment.assigned_control_variables.body_angle.active             = True
    
        
    mission.append_segment(segment)      
    
    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Circular approach pattern 
    #------------------------------------------------------------------------------------------------------------------------------------ 
    segment                                               = Segments.Cruise.Curved_Constant_Radius_Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                                           = "Approach_Pattern_Curve"   
    segment.analyses.extend( analyses.transition_setting_2 )             
    segment.air_speed   = pattern_speed
    segment.turn_radius = radius_Vert2
    segment.true_course = path_heading - (90 *Units.degrees)  
    segment.turn_angle  = app_sector
    segment.altitude    = pattern_altitude
    segment.state.numerics.solver.type = 'optimize' 
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                             = True    
    segment.flight_dynamics.force_z                                             = True    
    segment.flight_dynamics.force_y                                             = True     
    
    # define flight controls              
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                                    'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    segment.assigned_control_variables.body_angle.active                        = True    
    segment.assigned_control_variables.bank_angle.active                        = True    
    segment.assigned_control_variables.bank_angle.initial_guess_values          = [[20.0 * Units.degree]]
    
    mission.append_segment(segment)  
    
    
    # ------------------------------------------------------------------
    #  Transition and descent to vertiport
    # ------------------------------------------------------------------ 
    segment                          = Segments.Descent.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                      = "Transition_Descend_1" 
    segment.analyses.extend(analyses.transition_setting_2) 
    segment.descent_rate             = transition_climb_rate
    segment.air_speed_end            = transition_speed + (pattern_speed - transition_speed)/2      
    segment.air_speed_start          = pattern_speed
    segment.altitude_end             = (pattern_altitude - hover_altitude)/2 + hover_altitude
    segment.altitude_start           = pattern_altitude
    segment.true_course              = app_heading
    segment.state.numerics.solver.type = 'optimize' 

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    segment.assigned_control_variables.body_angle.active             = True 
                                                                             
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Transition and descent to vertiport
    # ------------------------------------------------------------------ 
    segment                          = Segments.Descent.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                      = "Transition_Descend_2" 
    segment.analyses.extend(analyses.transition_setting_2) 
    segment.descent_rate             = transition_climb_rate
    segment.air_speed_end            = transition_speed   
    segment.altitude_end             = hover_altitude
    segment.true_course              = app_heading
    segment.state.numerics.solver.type = 'optimize' 

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    segment.assigned_control_variables.body_angle.active             = True 
                                                                             
    mission.append_segment(segment)
   
    # ------------------------------------------------------------------
    #  Horizontal Transition
    # ------------------------------------------------------------------ 
    segment                                               = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag                                           = "Horizontal_Transition_Approach"  
    segment.analyses.extend( analyses.transition_setting_2)   
    segment.air_speed_end              = 10 * Units['knots'] 
    segment.acceleration                                  = -1.0
    segment.true_course                                   = app_heading  

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    segment.assigned_control_variables.body_angle.active             = True 
    
    mission.append_segment(segment)
    
    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Vertical Descent 
    #------------------------------------------------------------------------------------------------------------------------------------ 
    segment                                                         = Segments.Vertical_Flight.Descent(base_segment)
    segment.tag                                                     = "Vertical_Descent" 
    segment.analyses.extend( analyses.vertical_flight)                
    segment.altitude_end                                            = 0.   * Units.ft  
    segment.descent_rate                                            = 300. * Units['ft/min'] 
    segment.true_course                                             = app_heading
    segment.state.numerics.solver.type                              = 'root_finder'

    # define flight dynamics to model              
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
            
    mission.append_segment(segment)      
    
    return mission


def missions_setup(mission): 
 
    missions         = RCAIDE.Framework.Mission.Missions()
    
    # base mission 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions

def plot_results(results):
    # Plots fligh conditions 
    plot_flight_conditions(results) 
    
    # Plot arcraft trajectory
    plot_flight_trajectory(results)
    
    # Plot Aerodynamic Coefficients
    plot_aerodynamic_coefficients(results)  
     
    # Plot Aircraft Stability
    plot_longitudinal_stability(results) 
    plot_lateral_stability(results)
    
    # Plot Aircraft Electronics 
    plot_battery_temperature(results)
    plot_battery_cell_conditions(results) 
    plot_battery_degradation(results) 
    plot_electric_propulsor_efficiencies(results) 
    
    # Plot Propeller Conditions 
    plot_rotor_conditions(results)  
    plot_propulsor_throttles(results)
    return

def save_aircraft_geometry(geometry,filename): 
    pickle_file  = filename + '.pkl'
    with open(pickle_file, 'wb') as file:
        pickle.dump(geometry, file) 
    return 


def load_aircraft_geometry(filename):  
    load_file = filename + '.pkl' 
    with open(load_file, 'rb') as file:
        results = pickle.load(file) 
    return results


def load_propulsor(filename):
    propulsor =  load(filename)
    return propulsor

def save_propulsor(propulsor, filename):
    save(propulsor, filename)
    return

def display_stability_derivatives(segment):
    """This function displays the stability derivatives of the aircraft."""

    # Get the stability derivatives
    stability_derivatives = segment.conditions.static_stability.derivatives

    # Display the stability derivatives
    print(f"CLift_alpha: {stability_derivatives.Clift_alpha[0,0]:.3f}")
    print(f"CY_beta: {stability_derivatives.CY_beta[0,0]:.3f}")
    print(f"CL_beta: {stability_derivatives.CL_beta[0,0]:.4f}")
    print(f"CM_alpha: {stability_derivatives.CM_alpha[0,0]:.3f}")
    print(f"CN_beta: {stability_derivatives.CN_beta[0,0]:.3f}")
    print(f"CL_p: {stability_derivatives.CL_p[0,0]:.5f}")
    print(f"CL_r: {stability_derivatives.CL_r[0,0]:.5f}")
    print(f"CM_q: {stability_derivatives.CM_q[0,0]:.5f}")
    print(f"CN_p: {stability_derivatives.CN_p[0,0]:.5f}")
    print(f"CN_r: {stability_derivatives.CN_r[0,0]:.5f}")
    print(f"CM_delta_e: {stability_derivatives.CM_delta_e[0,0]:.5f}")
    print(f"CL_delta_a: {stability_derivatives.CL_delta_a[0,0]:.5f}")
    print(f"CN_delta_a: {stability_derivatives.CN_delta_a[0,0]:.5f}")
    print(f"CN_delta_r: {stability_derivatives.CN_delta_r[0,0]:.5f}")
    return

if __name__ == '__main__': 
    main()    
    plt.show()
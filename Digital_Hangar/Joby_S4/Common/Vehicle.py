''' 
# Vehicle.py
# 
# Created: May 2019, M Clarke
#          Sep 2020, M. Clarke 

'''
#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import RCAIDE
from RCAIDE.Core import Units, Data   
import pickle
from RCAIDE.Visualization.Performance.Aerodynamics.Vehicle import *  
from RCAIDE.Visualization.Performance.Mission import *   
from RCAIDE.Visualization.Performance.Energy.Battery import *   
from RCAIDE.Visualization.Performance.Noise import *  
from RCAIDE.Visualization.Geometry import *
from RCAIDE.Components.Energy.Networks.Battery_Electric_Rotor                 import Battery_Electric_Rotor
from RCAIDE.Methods.Power.Battery.Sizing                                      import initialize_from_mass 
from RCAIDE.Methods.Geometry.Two_Dimensional.Planform                         import segment_properties
from RCAIDE.Methods.Power.Battery.Sizing                                      import initialize_from_circuit_configuration 
from RCAIDE.Methods.Weights.Correlations.Propulsion                           import nasa_motor
from RCAIDE.Methods.Propulsion.electric_motor_sizing                          import size_optimal_motor
from RCAIDE.Methods.Propulsion                                                import prop_rotor_design
from RCAIDE.Methods.Weights.Buildups.eVTOL.empty                              import empty
from RCAIDE.Methods.Center_of_Gravity.compute_component_centers_of_gravity    import compute_component_centers_of_gravity
from RCAIDE.Methods.Geometry.Two_Dimensional.Planform.wing_segmented_planform import wing_segmented_planform 
from RCAIDE.Methods.Weights.Buildups.eVTOL.converge_evtol_weight              import converge_evtol_weight  
 
import os
import numpy as np 
from copy import deepcopy 

# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------
def vehicle_setup(resize_aircraft,vehicle_name = 'Joby_S4_CRM') :
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------  
    
    if resize_aircraft:
        
        # ------------------------------------------------------------------
        #   Initialize the Vehicle
        # ------------------------------------------------------------------    
        vehicle                                   = RCAIDE.Vehicle()
        vehicle.tag                               = 'Joby_S4'
        vehicle.configuration                     = 'eVTOL'
    
        # ------------------------------------------------------------------
        #   Vehicle-level Properties
        # ------------------------------------------------------------------    
        # mass properties
        vehicle.mass_properties.takeoff           = 2177
        vehicle.mass_properties.operating_empty   = 2177      
        vehicle.mass_properties.max_takeoff       = 2177             
        vehicle.mass_properties.center_of_gravity = [[2.0144,   0.  ,  0. ]]      
        vehicle.reference_area                    = 10.39
        vehicle.flight_envelope.ultimate_load            = 5.7   
        vehicle.flight_envelope.positive_limit_load               = 3.  
        vehicle.number_of_passengers                        = 5
    
        # ------------------------------------------------------------------    
        # WINGS                                    
        # ------------------------------------------------------------------    
        # WING PROPERTIES           
        wing                                      = RCAIDE.Components.Wings.Main_Wing()
        wing.tag                                  = 'main_wing'  
        wing.aspect_ratio                         = 9.11 
        wing.sweeps.quarter_chord                 = 0.0  
        wing.thickness_to_chord                   = 0.15
        wing.taper                                = 0.650 
        wing.spans.projected                      = 9.736 
        wing.chords.root                          = 1.57 
        wing.total_length                         = 1.57  
        wing.chords.tip                           = 0.66 
        wing.chords.mean_aerodynamic              = 1.069 
        wing.dihedral                             = 0   * Units.degrees  
        wing.areas.reference                      = 10.39 * 2  
        wing.areas.wetted                         = 10.39 * 2   
        wing.areas.exposed                        = 10.39 * 2   
        wing.twists.root                          = 0   * Units.degrees  
        wing.twists.tip                           = 0   * Units.degrees   
        wing.origin                               = [[ 1.778,0 , 1.0 ]]
        wing.aerodynamic_center                   = [ 1.8 ,0 , 1.0 ]    
        wing.winglet_fraction                     = 0.0  
        wing.xz_plane_symmetric                   = True
        wing.vertical                             = False
                                                  
        # Segment                                              
        segment                                   = RCAIDE.Components.Wings.Segment()
        segment.tag                               = 'Section_1'   
        segment.percent_span_location             = 0.0
        segment.twist                             = 0.0 
        segment.root_chord_percent                = 1 
        segment.dihedral_outboard                 = 8.  * Units.degrees
        segment.sweeps.quarter_chord              = 0. * Units.degrees 
        segment.thickness_to_chord                = 0.15 
        wing.append_segment(segment)                           
                                                  
        # Segment                                               
        segment                                   = RCAIDE.Components.Wings.Segment()
        segment.tag                               = 'Section_2'    
        segment.percent_span_location             = 0.4875
        segment.twist                             = 0.0
        segment.root_chord_percent                = 0.6496
        segment.dihedral_outboard                 = 0. * Units.degrees
        segment.sweeps.quarter_chord              = 0. * Units.degrees
        segment.thickness_to_chord                = 0.135
        wing.append_segment(segment)                                 
                                                  
        # Segment                                              
        segment                                   = RCAIDE.Components.Wings.Segment()
        segment.tag                               = 'Section_5'   
        segment.percent_span_location             = 1.0
        segment.twist                             = 0. 
        segment.root_chord_percent                = 0.42038
        segment.dihedral_outboard                 = 0.  * Units.degrees 
        segment.sweeps.quarter_chord              = 0.  * Units.degrees 
        segment.thickness_to_chord                = 0.12
        wing.append_segment(segment)             
         
        # add to vehicle 
        vehicle.append_component(wing)  
                      
                                                  
        # WING PROPERTIES                         
        wing                                      = RCAIDE.Components.Wings.Wing()
        wing.tag                                  = 'v_tail'  
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
        wing.areas.reference                      = 4.25 * 2 
        wing.areas.wetted                         = 4.25 * 2 
        wing.areas.exposed                        = 4.25 * 2 
        wing.twists.root                          = 0 * Units.degrees 
        wing.twists.tip                           = 0 * Units.degrees 
        wing.origin                               = [[ 5.167, 0.0 ,0.470 ]]
        wing.aerodynamic_center                   = [  5.267,  0., 0.470  ]  
        wing.winglet_fraction                     = 0.0 
        wing.xz_plane_symmetric                   = True    
    
        # add to vehicle
        vehicle.append_component(wing)    
      
        # ---------------------------------------------------------------   
        # FUSELAGE                
        # ---------------------------------------------------------------   
        # FUSELAGE PROPERTIES
        fuselage                                    = RCAIDE.Components.Fuselages.Fuselage()
        fuselage.tag                                = 'fuselage' 
        fuselage.seats_abreast                      = 2.  
        fuselage.seat_pitch                         = 3.  
        fuselage.fineness.nose                      = 0.88   
        fuselage.fineness.tail                      = 1.13   
        fuselage.lengths.nose                       = 0.5  
        fuselage.lengths.tail                       = 1.5
        fuselage.lengths.cabin                      = 4.46 
        fuselage.lengths.total                      = 6.46
        fuselage.width                              = 5.85 * Units.feet      # change 
        fuselage.heights.maximum                    = 4.65 * Units.feet      # change 
        fuselage.heights.at_quarter_length          = 3.75 * Units.feet      # change 
        fuselage.heights.at_wing_root_quarter_chord = 4.65 * Units.feet      # change 
        fuselage.heights.at_three_quarters_length   = 4.26 * Units.feet      # change 
        fuselage.areas.wetted                       = 236. * Units.feet**2   # change 
        fuselage.areas.front_projected              = 0.14 * Units.feet**2   # change 
        fuselage.effective_diameter                 = 1.276     # change 
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
     
        
         
         
        #------------------------------------------------------------------
        # PROPULSOR
        #------------------------------------------------------------------
        net                              = Battery_Electric_Rotor()
        net.rotor_group_indexes          = [0,0,0,0,0,0]
        net.motor_group_indexes          = [0,0,0,0,0,0]  
        net.esc_group_indexes            = [0,0,0,0,0,0]     
        net.active_propulsor_groups      = [True]
         
        # Battery         
        bat                                                    = RCAIDE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNiMnCoO2_18650()  
        bat.pack.electrical_configuration.series               = 140   
        bat.pack.electrical_configuration.parallel             = 100
        bat.module_config.number_of_modules                    = 14 
        bat.module.geometrtic_configuration.total              = bat.pack.electrical_configuration.total
        bat.module_config.voltage                              = bat.pack.max_voltage/bat.module_config.number_of_modules # assumes modules are connected in parallel, must be less than max_module_voltage (~50) /safety_factor (~ 1.5)  
        bat.module.geometrtic_configuration.normal_count       = 25
        bat.module.geometrtic_configuration.parallel_count     = 40
        net.battery                                            = bat    
        net.voltage                                            = bat.pack.max_voltage 
     
        # 1. Electronic Speed Controller  
        prop_rotor_esc              = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
        prop_rotor_esc.efficiency   = 0.95
        for i in range(6):
            prop_rotor_ESC          = deepcopy(prop_rotor_esc)
            prop_rotor_ESC.tag      = 'prop_rotor_esc' + str(i + 1)  
            net.electronic_speed_controllers.append(prop_rotor_ESC)    
        
        # 2. Prop Rotors    
        g               = 9.81                                   # gravitational acceleration  
        Drag            = estimate_cruise_drag(vehicle,altitude = 1500. * Units.ft,speed= 130.* Units['mph'] ,lift_coefficient = 0.5 ,profile_drag = 0.06)
        Hover_Load      = vehicle.mass_properties.takeoff*g      # hover load          
     
     
        prop_rotor                                    = RCAIDE.Library.Components.Powertrain.Converters.Prop_Rotor() 
        prop_rotor.tag                                = 'prop_rotor'     
        prop_rotor.tip_radius                         = 3/2
        prop_rotor.hub_radius                         = 0.15 * prop_rotor.tip_radius
        prop_rotor.number_of_blades                   = 4
        
           
        prop_rotor.hover.design_altitude             = 40 * Units.feet  
        prop_rotor.hover.design_thrust               = Hover_Load/(net.number_of_rotor_engines) 
        prop_rotor.hover.design_freestream_velocity  = np.sqrt(prop_rotor.hover.design_thrust/(2*1.2*np.pi*(prop_rotor.tip_radius**2))) 
    
        prop_rotor.oei.design_altitude               = 40 * Units.feet  
        prop_rotor.oei.design_thrust                 = Hover_Load/(net.number_of_rotor_engines-2)  
        prop_rotor.oei.design_freestream_velocity    = np.sqrt(prop_rotor.oei.design_thrust/(2*1.2*np.pi*(prop_rotor.tip_radius**2)))  
    
        # CRUISE                   
        prop_rotor.cruise.design_altitude             = 1500 * Units.feet                      
        prop_rotor.cruise.design_thrust               = Drag*1.1/8
        prop_rotor.cruise.design_freestream_velocity  = 130*Units.mph  
        
         
        airfoil                                       = RCAIDE.Library.Components.Airfoils.Airfoil()    
        ospath                                        = os.path.abspath(__file__)
        separator                                     = os.path.sep
        rel_path                                      = os.path.dirname(ospath) + separator    
        airfoil.coordinate_file                       = rel_path + '..' + separator + 'Airfoils' + separator + 'NACA_4412.txt'
        airfoil.polar_files                           = [rel_path + '..' + separator + 'Airfoils' + separator + 'Polars' + separator +'NACA_4412_polar_Re_50000.txt' ,
                                                         rel_path + '..' + separator + 'Airfoils' + separator + 'Polars' + separator +'NACA_4412_polar_Re_100000.txt' ,
                                                         rel_path + '..' + separator + 'Airfoils' + separator + 'Polars' + separator +'NACA_4412_polar_Re_200000.txt' ,
                                                         rel_path + '..' + separator + 'Airfoils' + separator + 'Polars' + separator +'NACA_4412_polar_Re_500000.txt' ,
                                                         rel_path + '..' + separator + 'Airfoils' + separator + 'Polars' + separator +'NACA_4412_polar_Re_1000000.txt',
                                                         rel_path + '..' + separator + 'Airfoils' + separator + 'Polars' + separator +'NACA_4412_polar_Re_3500000.txt',
                                                         rel_path + '..' + separator + 'Airfoils' + separator + 'Polars' + separator +'NACA_4412_polar_Re_5000000.txt',
                                                         rel_path + '..' + separator + 'Airfoils' + separator + 'Polars' + separator +'NACA_4412_polar_Re_7500000.txt' ]
        
        prop_rotor.append_airfoil(airfoil)   
        prop_rotor.airfoil_polar_stations            = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]   
        prop_rotor                                   = prop_rotor_design(prop_rotor)   
                
        # Prop Rotors                                          
        prop_rotor_origins           = [[0.208, -1.848,  1.195],[0.208, 1.848,  1.195],
                                        [1.505,5.000,1.320],[1.505,-5.000,1.320],
                                        [  5.318, 1.848,   2.282],[   5.318, -1.848,   2.282]]    
        for ii in range(6):
            pr                        = deepcopy(prop_rotor)
            pr.tag                    = 'prop_rotor_' + str(ii+1) 
            pr.origin                 = [prop_rotor_origins[ii]] 
            net.rotors.append(pr)  
    
        # Component 7 the Motors  
        prop_rotor_motor                         = RCAIDE.Library.Components.Powertrain.Converters.Motor()
        prop_rotor_motor.efficiency              = 0.9
        prop_rotor_motor.nominal_voltage         = bat.pack.max_voltage*0.75 
        prop_rotor_motor.propeller_radius        = prop_rotor.tip_radius   
        prop_rotor_motor.no_load_current         = 0.1   
        prop_rotor_motor.rotor_radius            = prop_rotor.tip_radius
        prop_rotor_motor.design_torque           = prop_rotor.hover.design_torque
        prop_rotor_motor.angular_velocity        = prop_rotor.hover.design_angular_velocity/prop_rotor_motor.gear_ratio  
        prop_rotor_motor                         = size_optimal_motor(prop_rotor_motor)
        prop_rotor_motor.mass_properties.mass    = nasa_motor(prop_rotor_motor.design_torque)     
      
     
        # Appending motors with different origins
        for ii in range(6):
            motor        = deepcopy(prop_rotor_motor)
            motor.tag    = 'prop_rotor_motor_' + str(ii+1)
            motor.origin =  [prop_rotor_origins[ii]]                     
            net.motors.append(motor)   
                 
        # Component 3: Avionics    
        avionics                       = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
        avionics.power_draw            = 20. # Watts  
        net.avionics                   = avionics   
        
        # Component 4: Miscellaneous Systems 
        sys                            = RCAIDE.Library.Components.Powertrain.Systems.Systems()
        sys.mass_properties.mass       = 5 # kg      
         
     
        vehicle.append_component(net) 
        
        main_wing_motor_origins                 = np.array([[-0.109,2.283 ,1.630],[-0.109,-2.283 ,1.630] ,[ 2.283,4.891,2.391],[ 2.283,-4.891,2.391]]) 
        tail_motor_origins                      = np.array([[6.522,2.174,2.065],[6.522,-2.174,2.065]]) 
        vehicle.wings['main_wing'].motor_spanwise_locations = main_wing_motor_origins[:,1]/vehicle.wings['main_wing'].spans.projected
        vehicle.wings['v_tail'].motor_spanwise_locations    = tail_motor_origins[:,1]/vehicle.wings['main_wing'].spans.projected
       
     
        settings = Data()
        converge_evtol_weight(vehicle,settings,contingency_factor = 1.0) 
        breakdown = empty(vehicle,settings,contingency_factor = 1.0 )
        print(breakdown)
        
        vehicle.weight_breakdown  = breakdown
        compute_component_centers_of_gravity(vehicle)
        vehicle.center_of_gravity() 
        
        save_aircraft_geometry(vehicle,vehicle.tag)        
        
    else: 
        vehicle = load_aircraft_geometry(vehicle_name) 
    
    return vehicle 
 
# ---------------------------------------------------------------------
#   Define the Configurations
# --------------------------------------------------------------------- 
def configs_setup(vehicle):
    '''
    The configration set up below the scheduling of the nacelle angle and vehicle speed.
    Since one propeller operates at varying flight conditions, one must perscribe  the 
    pitch command of the propeller which us used in the variable pitch model in the analyses
    Note: low pitch at take off & low speeds, high pitch at cruise
    '''
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------ 
    configs                                     = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config                                 = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag                             = 'base'
    configs.append(base_config)
 
    # ------------------------------------------------------------------
    #   Hover Configuration
    # ------------------------------------------------------------------
    config                                            = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                        = 'vertical_climb'
    vector_angle                                      = 90.0 * Units.degrees
    for prop in config.networks.Battery_Electric_Rotor.rotors: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.blade_pitch_command                     = prop.inputs.pitch_command_hover
    configs.append(config) 

    # ------------------------------------------------------------------
    #  Vertical Transition 1 
    # ------------------------------------------------------------------  
    config                                            = RCAIDE.Library.Components.Configs.Config(base_config)
    vector_angle                                      = 35.0  * Units.degrees
    config.tag                                        = 'vertical_transition_1'
    for prop in config.networks.Battery_Electric_Rotor.rotors: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.blade_pitch_command                     = -5.  * Units.degrees 
    configs.append(config)    


    # ------------------------------------------------------------------
    # Vertical Transition 2    
    # ------------------------------------------------------------------ 
    config                                            = RCAIDE.Library.Components.Configs.Config(base_config)
    vector_angle                                      = 25.0  * Units.degrees
    config.tag                                        = 'vertical_transition_2'
    for prop in config.networks.Battery_Electric_Rotor.rotors: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.blade_pitch_command                     = -5.  * Units.degrees 
    configs.append(config)    
    

    # ------------------------------------------------------------------
    #   Hover-to-Cruise Configuration
    # ------------------------------------------------------------------ 
    config                                            = RCAIDE.Library.Components.Configs.Config(base_config)
    vector_angle                                      = 15.0  * Units.degrees
    config.tag                                        = 'climb_transition'
    for prop in config.networks.Battery_Electric_Rotor.rotors: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.blade_pitch_command                     = -3.  * Units.degrees 
    configs.append(config)    

     
    # ------------------------------------------------------------------
    #   Climb Configuration
    # ------------------------------------------------------------------
    config                                            = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                        = 'climb'
    vector_angle                                      = 0.0 * Units.degrees 
    for prop in config.networks.Battery_Electric_Rotor.rotors: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.blade_pitch_command                     = 0.  * Units.degrees  
    configs.append(config)
   

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------      
    config                                            = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                        = 'cruise'
    vector_angle                                      = 0.0 * Units.degrees 
    for prop in config.networks.Battery_Electric_Rotor.rotors: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.blade_pitch_command                     = prop.inputs.pitch_command_cruise
    configs.append(config)
  

    # ------------------------------------------------------------------
    #   Approach Configuration
    # ------------------------------------------------------------------  
    config                                            = RCAIDE.Library.Components.Configs.Config(base_config)
    vector_angle                                      = 15.0  * Units.degrees
    config.tag                                        = 'approach'  
    for prop in config.networks.Battery_Electric_Rotor.rotors: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.blade_pitch_command                     = 0.  * Units.degrees 
    configs.append(config)    

    
    
    # ------------------------------------------------------------------
    #  Descent Transition
    # ------------------------------------------------------------------  
    config                                            = RCAIDE.Library.Components.Configs.Config(base_config)
    vector_angle                                      = 45.0  * Units.degrees
    config.tag                                        = 'descent_transition'  
    for prop in config.networks.Battery_Electric_Rotor.rotors: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.blade_pitch_command                     = -5.  * Units.degrees 
    configs.append(config)    

    

    # ------------------------------------------------------------------
    #   Hover Configuration
    # ------------------------------------------------------------------ 
    config                                            = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                        = 'vertical_descent'
    vector_angle                                      = 90.0  * Units.degrees 
    for prop in config.networks.Battery_Electric_Rotor.rotors: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.blade_pitch_command                     = -10.  * Units.degrees  
    configs.append(config) 
    configs.append(config)     

    return configs 

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
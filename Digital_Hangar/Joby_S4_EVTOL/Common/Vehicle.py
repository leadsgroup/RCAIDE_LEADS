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
from RCAIDE.Framework.Core import Units  
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Rotor  import design_electric_rotor
from RCAIDE.Library.Plots                                         import * 
from RCAIDE import  load 
from RCAIDE import  save  
from RCAIDE.Framework.External_Interfaces.OpenVSP.export_vsp_vehicle  import export_vsp_vehicle 

import sys 
import os
import numpy as np 
from copy import deepcopy 
import matplotlib.pyplot as plt


def main(plot_results=True, plot_vehicle=True) :

# Step 1 design a vehicle
    vehicle = vehicle_setup()

    if plot_vehicle:
        plot_3d_vehicle(vehicle,side_view = True )
    return




# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------
def vehicle_setup():
    import sys 
    local_path = sys.path[0] + os.sep
    #local_path = ""

    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------  
    

        
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
    wing                                      = RCAIDE.Library.Components.Wings.Main_Wing()
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
    segment                                   = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                               = 'Section_1'   
    segment.percent_span_location             = 0.0
    segment.twist                             = 0.0 
    segment.root_chord_percent                = 1 
    segment.dihedral_outboard                 = 8.  * Units.degrees
    segment.sweeps.quarter_chord              = 0. * Units.degrees 
    segment.thickness_to_chord                = 0.15 
    wing.append_segment(segment)                           
                                                
    # Segment                                               
    segment                                   = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                               = 'Section_2'    
    segment.percent_span_location             = 0.4875
    segment.twist                             = 0.0
    segment.root_chord_percent                = 0.6496
    segment.dihedral_outboard                 = 0. * Units.degrees
    segment.sweeps.quarter_chord              = 0. * Units.degrees
    segment.thickness_to_chord                = 0.135
    wing.append_segment(segment)                                 
                                                
    # Segment                                              
    segment                                   = RCAIDE.Library.Components.Wings.Segments.Segment()
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
    wing                                      = RCAIDE.Library.Components.Wings.Wing()
    wing.tag                                  = 'v_tail'  
    wing.aspect_ratio                         = 4.27172 
    wing.sweeps.quarter_chord                 = 22.46  * Units.degrees 
    wing.thickness_to_chord                   = 0.15 
    wing.spans.projected                      = 3.6
    wing.chords.root                          = 0.75
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
    fuselage                                    = RCAIDE.Library.Components.Fuselages.Fuselage()
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
    battery_module.electrical_configuration.parallel                  = 60          
    battery_module.geometrtic_configuration.normal_count              = 60
    battery_module.geometrtic_configuration.parallel_count            = 60
    battery_module.geometrtic_configuration.stacking_rows             = 2
    
                       # starboard   | port        | front  | rear 
    modules_origins = [[1.8, 2.0,1.0 ],[1.8, -2.0, 1.0  ],[0.3, 0.0, 0.0 ],[2, 0.0, 0.0]]  
    orientation     = [[0, 0.0, np.pi],[0, 0.0, np.pi ],[0, 0.0, 0 ],[0, 0.0,0 ]]   
    for m_i in range(bus.number_of_battery_modules):
        module =  deepcopy(battery_module)
        module.tag = 'nmc_module_' + str(m_i+1) 
        module.origin = [modules_origins[m_i]] 
        module.orientation_euler_angles  = orientation[m_i]  
        bus.battery_modules.append(module) 
    bus.initialize_bus_properties()    
        
   
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsors 
    #------------------------------------------------------------------------------------------------------------------------------------    
     
    # Define  Propulsor 
    propulsor                                     = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()
    propulsor.tag                                 = 'propulsor'       
              
    # Electronic Speed Controller           
    prop_rotor_esc                                = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
    prop_rotor_esc.efficiency                     = 0.95
    prop_rotor_esc.bus_voltage                    = bus.voltage
    prop_rotor_esc.tag                            = 'prop_rotor_esc_1'  
    propulsor.electronic_speed_controller         = prop_rotor_esc  
    
    # Prop-Rotor Design
    g                                             = 9.81                                    # gravitational acceleration   
    Hover_Load                                    = vehicle.mass_properties.max_takeoff*g * 1.1 # hover load   

    prop_rotor                                    = RCAIDE.Library.Components.Powertrain.Converters.Prop_Rotor()   
    prop_rotor.tag                                = 'prop_rotor'   
    prop_rotor.tip_radius                         = 2.85 / 2 
    prop_rotor.hub_radius                         = 0.12 * prop_rotor.tip_radius
    prop_rotor.number_of_blades                   = 5

    prop_rotor.hover.design_altitude              = 40 * Units.feet  
    prop_rotor.hover.design_thrust                = Hover_Load/6
    prop_rotor.hover.design_freestream_velocity   = np.sqrt(prop_rotor.hover.design_thrust/(2*1.2*np.pi*(prop_rotor.tip_radius**2)))
    
    prop_rotor.oei.design_altitude                = 40 * Units.feet  
    prop_rotor.oei.design_thrust                  = Hover_Load/5  
    prop_rotor.oei.design_freestream_velocity     = np.sqrt(prop_rotor.oei.design_thrust/(2*1.2*np.pi*(prop_rotor.tip_radius**2)))
    
    prop_rotor.cruise.design_altitude             = 1500 * Units.feet  
    prop_rotor.cruise.design_thrust               = 1500   
    prop_rotor.cruise.design_freestream_velocity  = 170.  * Units['mph']   
    
    airfoil                                       = RCAIDE.Library.Components.Airfoils.Airfoil()   
    airfoil.coordinate_file                       =  local_path + 'NACA_4412.txt'
    airfoil.polar_files                           = [local_path + 'NACA_4412_polar_Re_50000.txt' ,
                                                     local_path + 'NACA_4412_polar_Re_100000.txt' ,
                                                     local_path + 'NACA_4412_polar_Re_200000.txt' ,
                                                     local_path + 'NACA_4412_polar_Re_500000.txt' ,
                                                     local_path + 'NACA_4412_polar_Re_1000000.txt',
                                                     local_path + 'NACA_4412_polar_Re_3500000.txt',
                                                     local_path + 'NACA_4412_polar_Re_5000000.txt',
                                                     local_path + 'NACA_4412_polar_Re_7500000.txt' ]
    prop_rotor.append_airfoil(airfoil)                
    prop_rotor.airfoil_polar_stations             = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    propulsor.rotor = prop_rotor    
    
    #------------------------------------------------------------------------------------------------------------------------------------               
    # Lift Rotor Motor  
    #------------------------------------------------------------------------------------------------------------------------------------    
    prop_rotor_motor                         = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor()
    prop_rotor_motor.efficiency              = 0.98
    prop_rotor_motor.nominal_voltage         = 0.75 * bus.voltage 
    prop_rotor_motor.no_load_current         = 0.01  
    propulsor.motor                          = prop_rotor_motor  

    #------------------------------------------------------------------------------------------------------------------------------------               
    # Lift Rotor Nacelle
    #------------------------------------------------------------------------------------------------------------------------------------    
    nacelle                                     = RCAIDE.Library.Components.Nacelles.Stack_Nacelle()
    nacelle.tag                                 = 'nacelle_1'
    nacelle.length                              = 2
    nacelle.diameter                            = 0.85 
    nacelle.areas.wetted                        = 1.0    
    nacelle.flow_through                        = False     

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_1'
    nac_segment.percent_x_location              = 0.0 
    nac_segment.height                          = 0.0
    nac_segment.width                           = 0.0
    nacelle.append_segment(nac_segment)   

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_2'
    nac_segment.percent_x_location              = 0.1 
    nac_segment.percent_z_location              = 0 
    nac_segment.height                          = 0.3 
    nac_segment.width                           = 0.3  
    nacelle.append_segment(nac_segment)   

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_3'
    nac_segment.percent_x_location              = 0.15
    nac_segment.percent_z_location              = 0 
    nac_segment.height                          = 0.35
    nac_segment.width                           = 0.35
    nacelle.append_segment(nac_segment)  

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_4'
    nac_segment.percent_x_location              = 0.25
    nac_segment.percent_z_location              = 0
    nac_segment.height                          = 0.45  
    nac_segment.width                           = 0.45 
    nacelle.append_segment(nac_segment)  

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_5'
    nac_segment.percent_x_location              = 0.5
    nac_segment.percent_z_location              = 0
    nac_segment.height                          = 0.4
    nac_segment.width                           = 0.45
    nacelle.append_segment(nac_segment)   

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_6'
    nac_segment.percent_x_location              = 0.75
    nac_segment.percent_z_location              = 0 
    nac_segment.height                          = 0.35	 
    nac_segment.width                           = 0.35   
    nacelle.append_segment(nac_segment)

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_6'
    nac_segment.percent_x_location              = 0.9
    nac_segment.percent_z_location              = 0
    nac_segment.height                          = 0.2
    nac_segment.width                           = 0.25 
    nacelle.append_segment(nac_segment)    

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_7'
    nac_segment.percent_x_location              = 1.0 
    nac_segment.percent_z_location              = 0 	
    nac_segment.height                          = 0.1
    nac_segment.width                           = 0.1 
    nacelle.append_segment(nac_segment) 
    propulsor.nacelle                 = nacelle  
            
    design_electric_rotor(propulsor, print_iterations=True, solver_sense_step=1E-3, solver_tolerance=1E-2)
    
               
         
    # Front Rotors Locations 
    origins =[[0.6 , -2.2,  1.195],[0.6 , 2.2,  1.195],
              [1.3 , -5.000,1.320],[1.3 ,5.000,1.320],
              [  4.55 , -2.1 ,   2 ],[   4.55, 2.1 ,   2 ]] 
    nacelle_origins =[[0.3, -2.2,  1.195],[0.3, 2.2,  1.195],
              [1.00, -5.000,1.320],[1.0,5.000,1.320],
              [  4.25 , -2.1 ,   2. ],[   4.25 , 2.1 ,   2. ]]
    
    tags  = ['front_port_','front_starboard_',
             'outboard_port_','outboard_starboard_', 
             'rear_port_','rear_starboard_']
     
    assigned_propulsor_list = []    
    for i in range(len(origins)): 
        propulsor_i                                       = deepcopy(propulsor)
        propulsor_i.tag                                   = tags[i] +  'propulsor' 
        propulsor_i.rotor.tag                             = tags[i] +'rotor' 
        propulsor_i.rotor.origin                          = [origins[i]]  
        propulsor_i.motor.tag                             = tags[i] +'motor' 
        propulsor_i.motor.origin                          = [origins[i]]  
        propulsor_i.electronic_speed_controller.tag       = tags[i] +'esc' 
        propulsor_i.electronic_speed_controller.origin    = [origins[i]]  
        propulsor_i.nacelle.tag                           = tags[i] +'nacelle' 
        propulsor_i.nacelle.origin                        = [nacelle_origins[i]]   
        network.propulsors.append(propulsor_i)   
        assigned_propulsor_list.append(propulsor_i.tag) 
    bus.assigned_propulsors = [assigned_propulsor_list]


    avionics                        = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.power_draw             = 10. # Watts  
    avionics.mass_properties.mass   = 1.0 * Units.kg
    bus.avionics                    = avionics    
   
    network.busses.append(bus)
     
    vehicle.append_energy_network(network)
        
  
    
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


if __name__ == '__main__':
    main()
    plt.show()
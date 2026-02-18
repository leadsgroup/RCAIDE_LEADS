#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units  
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Rotor  import design_electric_rotor
from RCAIDE.Library.Plots                                         import * 
from RCAIDE import  load 
from RCAIDE import  save   

# python imports   
import numpy as np   
from copy import deepcopy 
import sys 
import os

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    redesign_rotors = True
    
    # Step 1: design a vehicle
    vehicle  = vehicle_setup(redesign_rotors)  

    try:
        import vsp as vsp
        from RCAIDE.Framework.External_Interfaces.OpenVSP import export_vsp_vehicle 
        export_vsp_vehicle(vehicle, 'Vahana')
    except ImportError:
        pass
    
    # Step 2: plot vehicle 
    plot_3d_vehicle(vehicle,export_gltf=True)  
    
    return  

def vehicle_setup(redesign_rotors=True):
    
    airfoil_file_path =  os.path.join(os.path.split(sys.path[0])[0], '_Airfoils_and_Polars') + os.sep 
    polar_file_path   =  os.path.join(os.path.join(os.path.split(sys.path[0])[0], '_Airfoils_and_Polars') , '_Polars') + os.sep  
    
    #------------------------------------------------------------------------------------------------------------------------------------
    # ################################################# Vehicle-level Properties ########################################################  
    #------------------------------------------------------------------------------------------------------------------------------------
    vehicle                                     = RCAIDE.Vehicle()
    vehicle.tag                                 = 'Vahana'
    vehicle.configuration                       = 'eVTOL'
         
    # mass properties
    vehicle.mass_properties.takeoff             = 735. 
    vehicle.mass_properties.operating_empty     = 735.
    vehicle.mass_properties.max_takeoff         = 735.
    vehicle.mass_properties.center_of_gravity   = [[ 2.0144,   0.  ,  0.]] 
    vehicle.number_of_passengers                = 0
    vehicle.flight_envelope.ultimate_load       = 5.7
    vehicle.flight_envelope.positive_limit_load = 3.
    vehicle.number_of_passengers                = 1

    #------------------------------------------------------------------------------------------------------------------------------------
    # ##################################################### Landing Gear ################################################################    
    #------------------------------------------------------------------------------------------------------------------------------------ 
    main_gear                                   = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear() 
    main_gear.tire_diameter                     = 6  *  Units.inches 
    main_gear.rim_diameter                      = 3  *  Units.inches 
    main_gear.tire_width                        = 6  *  Units.inches 
    main_gear.strut_length                      = 12  * Units.ft 
    main_gear.wheels                            = 1   
    main_gear.number_of_gear_types_in_tandem    = 1
    main_gear.number_of_wheels_in_gear_type     = 1
    main_gear.origin                            = [[4.0,0, 0]]
    main_gear.fairing                           = True
    main_gear.xz_plane_symmetric                = True
    main_gear.gear_extended                     = True
    vehicle.append_component(main_gear)  

    nose_gear                                   = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()   
    nose_gear.tire_diameter                     =  5 *  Units.inches   
    nose_gear.rim_diameter                      =  3 *  Units.inches 
    nose_gear.tire_width                        =  5 *  Units.inches 
    nose_gear.strut_length                      =  6.* Units.ft 
    nose_gear.wheels                            = 1
    nose_gear.origin                            = [[0.5,0, 0]]
    nose_gear.fairing                           = True 
    nose_gear.gear_extended                     = True
    nose_gear.number_of_gear_types_in_tandem    = 1
    nose_gear.number_of_wheels_in_gear_type     = 1    
    vehicle.append_component(nose_gear)    

    #------------------------------------------------------------------------------------------------------------------------------------
    # ######################################################## Wings ####################################################################  
    #------------------------------------------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------
    wing                                        = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                                    = 'canard_wing'  
    wing.aspect_ratio                           = 11.37706641  
    wing.sweeps.quarter_chord                   = 0.0
    wing.thickness_to_chord                     = 0.18  
    wing.taper                                  = 1.  
    wing.spans.projected                        = 6.65 
    wing.chords.root                            = 0.95 
    wing.total_length                           = 0.95   
    wing.chords.tip                             = 0.95 
    wing.chords.mean_aerodynamic                = 0.95   
    wing.dihedral                               = 0.0  
    wing.areas.reference                        = wing.chords.root*wing.spans.projected 
    wing.areas.wetted                           = 2*wing.chords.root*wing.spans.projected*0.95  
    wing.areas.exposed                          = 2*wing.chords.root*wing.spans.projected*0.95 
    wing.twists.root                            = 0.  
    wing.twists.tip                             = 0.  
    wing.origin                                 = [[0.1,  0.0 , 0.0]]  
    wing.aerodynamic_center                     = [0., 0., 0.]     
    wing.winglet_fraction                       = 0.0 
    wing.xz_plane_symmetric                     = True 
    airfoil                                     = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                     = airfoil_file_path + 'NACA_63_412.txt' 
    wing.append_airfoil(airfoil)
                                                
    # add to vehicle                                          
    vehicle.append_component(wing)                            
                                                
    wing                                        = RCAIDE.Library.Components.Wings.Wing()
    wing.tag                                    = 'main_wing'  
    wing.aspect_ratio                           = 11.37706641  
    wing.sweeps.quarter_chord                   = 0.0
    wing.thickness_to_chord                     = 0.18  
    wing.taper                                  = 1.  
    wing.spans.projected                        = 6.65 
    wing.chords.root                            = 0.95 
    wing.total_length                           = 0.95   
    wing.chords.tip                             = 0.95 
    wing.chords.mean_aerodynamic                = 0.95   
    wing.dihedral                               = 0.0  
    wing.areas.reference                        = wing.chords.root*wing.spans.projected 
    wing.areas.wetted                           = 2*wing.chords.root*wing.spans.projected*0.95  
    wing.areas.exposed                          = 2*wing.chords.root*wing.spans.projected*0.95 
    wing.twists.root                            = 0.  
    wing.twists.tip                             = 0.  
    wing.origin                                 = [[ 5.138, 0.0  ,  1.323 ]]
    wing.aerodynamic_center                     = [0., 0., 0.]     
    wing.winglet_fraction                       = 0.0  
    wing.xz_plane_symmetric                     = True  
    vehicle.reference_area                      = 2*wing.areas.reference 
    wing.append_airfoil(airfoil)

    # add to vehicle 
    vehicle.append_component(wing)   


    #------------------------------------------------------------------------------------------------------------------------------------
    # ##########################################################  Fuselage ############################################################## 
    #------------------------------------------------------------------------------------------------------------------------------------
    
    fuselage                                    = RCAIDE.Library.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage' 

    # define cabin    
    cabin                                       = RCAIDE.Library.Components.Fuselages.Cabins.Cabin()
    cabin.origin                                = [[1, 0, 0]] 
    economy_class                               = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest        = 1
    economy_class.number_of_rows                = 1 
    economy_class.seat_arm_rest_width           = 2 *  Units.inches 
    economy_class.seat_width                    = 15 *  Units.inches
    economy_class.aisle_width                   = 0  *  Units.inches   
    cabin.append_cabin_class(economy_class)
    fuselage.append_cabin(cabin)

    fuselage.fineness.nose                      = 1.5 
    fuselage.fineness.tail                      = 4.0 
    fuselage.lengths.nose                       = 1.7   
    fuselage.lengths.tail                       = 2.7 
    fuselage.lengths.cabin                      = 1.7  
    fuselage.lengths.total                      = 6.1  
    fuselage.width                              = 1.15  
    fuselage.heights.maximum                    = 1.7 
    fuselage.heights.at_quarter_length          = 1.2  
    fuselage.heights.at_wing_root_quarter_chord = 1.7  
    fuselage.heights.at_three_quarters_length   = 0.75 
    fuselage.areas.wetted                       = 12.97989862  
    fuselage.areas.front_projected              = 1.365211404  
    fuselage.effective_diameter                 = 1.318423736  
    fuselage.differential_pressure              = 0.  

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_0'   
    segment.percent_x_location                  = 0.  
    segment.percent_z_location                  = 0.  
    segment.height                              = 0.09  
    segment.width                               = 0.23473  
    segment.length                              = 0.  
    segment.effective_diameter                  = 0. 
    fuselage.segments.append(segment)             

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_1'   
    segment.percent_x_location                  = 0.97675/6.1 
    segment.percent_z_location                  = 0.21977/6.1
    segment.height                              = 0.9027  
    segment.width                               = 1.01709  
    fuselage.segments.append(segment)             


    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'    
    segment.percent_x_location                  = 1.93556/6.1 
    segment.percent_z_location                  = 0.39371/6.1
    segment.height                              = 1.30558   
    segment.width                               = 1.38871  
    fuselage.segments.append(segment)             


    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'    
    segment.percent_x_location                  = 3.44137/6.1 
    segment.percent_z_location                  = 0.57143/6.1
    segment.height                              = 1.52588 
    segment.width                               = 1.47074 
    fuselage.segments.append(segment)             

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 4.61031/6.1
    segment.percent_z_location                  = 0.10893
    segment.height                              = 1.3906
    segment.width                               = 1.11463  
    fuselage.segments.append(segment)              

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 0.9827
    segment.percent_z_location                  = 0.180
    segment.height                              = 0.6145
    segment.width                               = 0.3838
    fuselage.segments.append(segment)            
    
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 1. 
    segment.percent_z_location                  = 0.18
    segment.height                              = 0.4
    segment.width                               = 0.25
    fuselage.segments.append(segment)        

    # add to vehicle
    vehicle.append_component(fuselage)     

    #------------------------------------------------------------------------------------------------------------------------------------
    # ########################################################  Energy Network  ######################################################### 
    #------------------------------------------------------------------------------------------------------------------------------------
    network                                                = RCAIDE.Framework.Networks.Electric() 
    network.charging_power                                 = 1000
    #==================================================================================================================================== 
    # Lift Bus 
    #====================================================================================================================================          
    bus                                                    = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    bus.tag                                                = 'bus' 

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus Battery
    #------------------------------------------------------------------------------------------------------------------------------------ 
    bat                                                    = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC() 
    bat.tag                                                = 'bus_battery'
    bat.electrical_configuration.series                    = 8 
    bat.electrical_configuration.parallel                  = 60 
    bat.geometrtic_configuration.normal_count              = 20
    bat.geometrtic_configuration.parallel_count            = 24  
    
    for _ in range(10):
        bus.battery_modules.append(deepcopy(bat))   
    bus.initialize_bus_properties()
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Lift Propulsors 
    #------------------------------------------------------------------------------------------------------------------------------------    
     
    # Define Lift Propulsor Container 
    prop_rotor_propulsor                                = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()
    prop_rotor_propulsor.tag                            = 'prop_rotor_propulsor'      
    prop_rotor_propulsor.wing_mounted                   = True 
              
    # Electronic Speed Controller           
    prop_rotor_esc                                      = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
    prop_rotor_esc.efficiency                           = 0.95    
    prop_rotor_esc.tag                                  = 'esc_1'  
    prop_rotor_esc.bus_voltage                          = bus.voltage   
    prop_rotor_propulsor.electronic_speed_controller    = prop_rotor_esc  
    
    # Lift Rotor Design
    g                                                   = 9.81                                
    Hover_Load                                          = vehicle.mass_properties.takeoff*g *1.1   
    prop_rotor                                          = RCAIDE.Library.Components.Powertrain.Converters.Prop_Rotor()   
    prop_rotor.tag                                      = 'prop_rotor'   
    prop_rotor.tip_radius                               = 0.8875
    prop_rotor.hub_radius                               = 0.15 * prop_rotor.tip_radius
    prop_rotor.number_of_blades                         = 3
    prop_rotor.hover.design_altitude                    = 40 * Units.feet   
    prop_rotor.hover.design_thrust                      = Hover_Load/8 
    prop_rotor.hover.design_freestream_velocity         = np.sqrt(prop_rotor.hover.design_thrust/(2*1.2*np.pi*(prop_rotor.tip_radius**2)))  
    prop_rotor.oei.design_altitude                      = 40 * Units.feet  
    prop_rotor.oei.design_thrust                        = Hover_Load/7  
    prop_rotor.oei.design_freestream_velocity           = np.sqrt(prop_rotor.oei.design_thrust/(2*1.2*np.pi*(prop_rotor.tip_radius**2)))   
    prop_rotor.cruise.design_altitude                   = 1500 * Units.feet
    prop_rotor.cruise.design_thrust                     = 500   
    prop_rotor.cruise.design_freestream_velocity        = 130.* Units['mph'] 
    
    
    airfoil                                             = RCAIDE.Library.Components.Airfoils.Airfoil()   
    airfoil.coordinate_file                             =  airfoil_file_path + 'NACA_4412.txt'
    airfoil.polar_files                                 = [polar_file_path + 'NACA_4412_polar_Re_50000.txt' ,
                                                           polar_file_path  + 'NACA_4412_polar_Re_100000.txt' ,
                                                           polar_file_path  + 'NACA_4412_polar_Re_200000.txt' ,
                                                           polar_file_path  + 'NACA_4412_polar_Re_500000.txt' ,
                                                           polar_file_path  + 'NACA_4412_polar_Re_1000000.txt',
                                                           polar_file_path  + 'NACA_4412_polar_Re_3500000.txt',
                                                           polar_file_path  + 'NACA_4412_polar_Re_5000000.txt',
                                                           polar_file_path  + 'NACA_4412_polar_Re_7500000.txt' ]
    prop_rotor.append_airfoil(airfoil)                
    prop_rotor.airfoil_polar_stations                   = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    prop_rotor_propulsor.rotor                          =  prop_rotor

   
    #------------------------------------------------------------------------------------------------------------------------------------               
    # Lift Rotor Motor  
    #------------------------------------------------------------------------------------------------------------------------------------    
    prop_rotor_motor                         = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor()
    prop_rotor_motor.efficiency              = 0.95
    prop_rotor_motor.nominal_voltage         = bus.voltage *0.75
    prop_rotor_motor.tag                     = 'motor_1'
    prop_rotor_motor.no_load_current         = 0.1    
    prop_rotor_propulsor.motor               = prop_rotor_motor

    #------------------------------------------------------------------------------------------------------------------------------------               
    # Lift Rotor Nacelle
    #------------------------------------------------------------------------------------------------------------------------------------     
    nacelle                                  = RCAIDE.Library.Components.Nacelles.Nacelle() 
    nacelle.length                           = 0.9
    nacelle.diameter                         = 0.3 
    nacelle.flow_through                     = False    
    prop_rotor_propulsor.nacelle             = nacelle
    
    if redesign_rotors:
        design_electric_rotor(prop_rotor_propulsor, print_iterations=True)
        save_propulsor(prop_rotor_propulsor, os.path.join(airfoil_file_path, 'vahana_tilt_rotor_propulsor.res'))
    else:
        regression_prop_rotor_propulsor = deepcopy(prop_rotor_propulsor)        
        design_electric_rotor(regression_prop_rotor_propulsor, iterations=2)
        loaded_propulsor = load_propulsor(os.path.join(airfoil_file_path, 'vahana_tilt_rotor_propulsor.res'))  
        for key,item in prop_rotor_propulsor.rotor.items():
            prop_rotor_propulsor.rotor[key] = loaded_propulsor.rotor[key] 
        for key,item in prop_rotor_propulsor.motor.items():
            prop_rotor_propulsor.motor[key] = loaded_propulsor.motor[key] 
         
    # Front Rotors Locations 
    nacelle_origins = [[-0.2, 1.347, 0.0], [-0.2, 3.2969999999999997, 0.0], [-0.2, -1.347, 0.0], [-0.2, -3.2969999999999997, 0.0],\
               [4.938, 1.347, 1.4], [4.938, 3.2969999999999997, 1.4],[4.938, -1.347, 1.5], [4.938, -3.2969999999999997, 1.4]] 
    rotor_origins = [[0., 1.347, 0.0], [0., 3.2969999999999997, 0.0], [0., -1.347, 0.0], [0., -3.2969999999999997, 0.0],\
               [5.0, 1.347, 1.4], [5.0, 3.2969999999999997, 1.4],[5.0, -1.347, 1.5], [5.0, -3.2969999999999997, 1.4]] 
    motor_origins = [[0.5, 1.347, 0.0], [0.5, 3.2969999999999997, 0.0], [0.5, -1.347, 0.0], [0.5, -3.2969999999999997, 0.0],\
               [5.7, 1.347, 1.4], [5.7, 3.2969999999999997, 1.4],[5.7, -1.347, 1.5], [5.7, -3.2969999999999997, 1.4]]
    
    assigned_propulsor_list =  []
    for i in range(8): 
        prop_rotor_propulsor_i                                       = deepcopy(prop_rotor_propulsor)
        prop_rotor_propulsor_i.tag                                   = 'prop_rotor_propulsor_' + str(i + 1)
        prop_rotor_propulsor_i.rotor.tag                             = 'rotor_' + str(i + 1) 
        prop_rotor_propulsor_i.rotor.origin                          = [rotor_origins[i]]  
        prop_rotor_propulsor_i.motor.tag                             = 'motor_' + str(i + 1)  
        if i < 4: 
            prop_rotor_propulsor_i.motor.wing_tag                    = 'canard_wing'
        else:
            prop_rotor_propulsor_i.motor.wing_tag                    = 'main_wing'
        prop_rotor_propulsor_i.motor.origin                          = [motor_origins[i]]  
        prop_rotor_propulsor_i.electronic_speed_controller.tag       = 'esc_' + str(i + 1)  
        prop_rotor_propulsor_i.electronic_speed_controller.origin    = [motor_origins[i]]  
        prop_rotor_propulsor_i.nacelle.tag                           = 'nacelle_' + str(i + 1)  
        prop_rotor_propulsor_i.nacelle.origin                        = [nacelle_origins[i]]
        assigned_propulsor_list.append(prop_rotor_propulsor_i.tag)
        network.propulsors.append(prop_rotor_propulsor_i)  
    bus.assigned_propulsors = [assigned_propulsor_list]       
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Additional Bus Loads
    #------------------------------------------------------------------------------------------------------------------------------------            
    # Payload   
    systems                         = RCAIDE.Library.Components.Powertrain.Systems.Systems()
    systems.power_draw              = 10. # Watts 
    systems.mass_properties.mass    = 1.0 * Units.kg
    bus.systems                     = systems 
                             
    # Avionics                            
    avionics                        = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.power_draw             = 10. # Watts  
    avionics.mass_properties.mass   = 1.0 * Units.kg
    bus.avionics                    = avionics     
    network.busses.append(bus) 
        
    # append energy network 
    vehicle.append_energy_network(network)  

    return vehicle

def load_propulsor(filename):
    propulsor =  load(filename)
    return propulsor

def save_propulsor(propulsor, filename):
    save(propulsor, filename)
    return 
 
if __name__ == '__main__': 
    main()     

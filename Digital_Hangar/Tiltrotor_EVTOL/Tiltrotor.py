# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units    
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Rotor   import design_electric_rotor  
from RCAIDE.Library.Plots                 import *       
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
    
    # Step 1: design a vehicle
    redesign_rotors = False  
    vehicle  = vehicle_setup(redesign_rotors)

    try:
        import vsp as vsp
        from RCAIDE.Framework.External_Interfaces.OpenVSP import export_vsp_vehicle 
        export_vsp_vehicle(vehicle, 'Tiltrotor')
    except ImportError:
        pass
        
    # Step 2: plot vehicle 
    plot_3d_vehicle(vehicle,export_gltf=True)  
    
    return 
 
def vehicle_setup(redesign_rotors = False):

    local_path = sys.path[0] + os.sep 
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
    vehicle.mass_properties.max_takeoff               = 2404  
    vehicle.mass_properties.takeoff                   = 2404  
    vehicle.mass_properties.max_payload               = 350  
    vehicle.mass_properties.min_payload               = 350  
    vehicle.mass_properties.center_of_gravity         = [[2.0144,   0.  ,  0. ]]      
    vehicle.mass_properties.moments_of_inertia.tensor = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    vehicle.reference_area                            = 10.39
    vehicle.flight_envelope.ultimate_load             = 5.7   
    vehicle.flight_envelope.positive_limit_load       = 3.  
    vehicle.number_of_passengers                      = 5
    vehicle.neutral_point                             = 2.600 

    #------------------------------------------------------------------------------------------------------------------------------------
    # ##################################################### Landing Gear ################################################################    
    #------------------------------------------------------------------------------------------------------------------------------------ 
    main_gear                                = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear() 
    main_gear.tire_diameter                  = 6  *  Units.inches 
    main_gear.rim_diameter                   = 3  *  Units.inches 
    main_gear.tire_width                     = 6  *  Units.inches 
    main_gear.strut_length                   = 12  * Units.ft 
    main_gear.wheels                         = 1   
    main_gear.number_of_gear_types_in_tandem = 1
    main_gear.number_of_wheels_in_gear_type  = 1
    main_gear.origin                         = [[3.5,0, 0]]
    main_gear.fairing                        = True
    main_gear.xz_plane_symmetric             = True
    main_gear.gear_extended                  = True
    vehicle.append_component(main_gear)  

    nose_gear                                = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()   
    nose_gear.tire_diameter                  =  5 *  Units.inches   
    nose_gear.rim_diameter                   =  3 *  Units.inches 
    nose_gear.tire_width                     =  5 *  Units.inches 
    nose_gear.strut_length                   =  6.* Units.ft 
    nose_gear.wheels                         = 1
    nose_gear.origin                         = [[0.25,0, 0]]
    nose_gear.fairing                        = True 
    nose_gear.gear_extended                  = True
    nose_gear.number_of_gear_types_in_tandem = 1
    nose_gear.number_of_wheels_in_gear_type  = 1    
    vehicle.append_component(nose_gear)

    #------------------------------------------------------------------------------------------------------------------------------------
    # ##################################################### Wings ################################################################    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # WING PROPERTIES           
    wing                                      = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                                  = 'main_wing'  
    wing.aspect_ratio                         = 9.11 
    wing.sweeps.quarter_chord                 = 0.0  
    wing.thickness_to_chord                   = 0.15
    wing.taper                                = 0.42 
    wing.spans.projected                      = 9.736 
    wing.chords.root                          = 1.57  
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
 
                                              
    # Segment                                              
    segment                                   = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                               = 'root'   
    segment.percent_span_location             = 0.0
    segment.twist                             = 0. * Units.degrees
    segment.root_chord_percent                = 1 
    segment.dihedral_outboard                 = 8.  * Units.degrees
    segment.sweeps.quarter_chord              = 0. * Units.degrees  
    airfoil                                   = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    airfoil.NACA_4_Series_code                = '2312'       
    #segment.append_airfoil(airfoil)
    wing.append_segment(segment)                           
                                              
    # Segment                                               
    segment                                   = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                               = 'mid_section'    
    segment.percent_span_location             = 0.4875
    segment.root_chord_percent                = 0.6496
    segment.twist                             = 0. * Units.degrees
    segment.dihedral_outboard                 = 0. * Units.degrees
    segment.sweeps.quarter_chord              = 0. * Units.degrees 
    airfoil                                   = RCAIDE.Library.Components.Airfoils.Airfoil() 
    airfoil.coordinate_file                   = local_path + 'NACA_63_412.txt'    
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)                                 
                                              
    # Segment                                              
    segment                                   = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                               = 'tip'   
    segment.percent_span_location             = 1.0
    segment.twist                             = 0. 
    segment.root_chord_percent                = 0.42038
    segment.dihedral_outboard                 = 0.  * Units.degrees 
    segment.sweeps.quarter_chord              = 0.  * Units.degrees  
    airfoil                                   = RCAIDE.Library.Components.Airfoils.Airfoil() 
    airfoil.coordinate_file                   = local_path + 'NACA_63_412.txt'    
    segment.append_airfoil(airfoil)
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
    wing.sweeps.quarter_chord                 = -10  * Units.degrees 
    wing.thickness_to_chord                   = 0.15 
    wing.spans.projected                      = 4.2
    wing.chords.root                          = 1.193 
    wing.total_length                         = 1.193 
    wing.chords.tip                           = 0.535 
    wing.taper                                = 0.44  
    wing.chords.mean_aerodynamic              = 0.864 
    wing.dihedral                             = 35.0 * Units.degrees 
    wing.areas.reference                      = 4.25
    wing.areas.wetted                         = 4.25 * 2.1 
    wing.areas.exposed                        = 4.25 * 2 
    wing.twists.root                          = 0 * Units.degrees 
    wing.twists.tip                           = 0 * Units.degrees 
    wing.origin                               = [[ 5.25 , 0.0 ,0.470 ]]
    wing.aerodynamic_center                   = [  5.267,  0., 0.470  ]  
    wing.winglet_fraction                     = 0.0 
    wing.xz_plane_symmetric                   = True

    # Segment                                              
    segment                                   = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                               = 'Section_1'   
    segment.percent_span_location             = 0.0
    segment.twist                             = 0 
    segment.root_chord_percent                = 1 
    segment.dihedral_outboard                 = wing.dihedral
    segment.sweeps.quarter_chord              =  wing.sweeps.quarter_chord 
    airfoil                                   = RCAIDE.Library.Components.Airfoils.Airfoil() 
    airfoil.coordinate_file                   = local_path + 'NACA_63_412.txt'    
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)                           
                                              
    # Segment                                               
    segment                                   = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                               = 'Section_2'    
    segment.percent_span_location             = 1
    segment.twist                             = 0
    segment.root_chord_percent                = wing.taper
    segment.dihedral_outboard                 = 0. * Units.degrees
    segment.sweeps.quarter_chord              = 0 * Units.degrees 
    airfoil                                   = RCAIDE.Library.Components.Airfoils.Airfoil() 
    airfoil.coordinate_file                   = local_path + 'NACA_63_412.txt'    
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)                                 
        

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

    # define cabin    
    cabin                                       = RCAIDE.Library.Components.Fuselages.Cabins.Cabin()
    cabin.origin                                = [[0.5, 0, 0]] 
    economy_class                               = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest        = 2
    economy_class.number_of_rows                = 3 
    economy_class.seat_arm_rest_width           = 2 *  Units.inches 
    economy_class.seat_width                    = 15 *  Units.inches
    economy_class.aisle_width                   = 0  *  Units.inches   
    cabin.append_cabin_class(economy_class)
    fuselage.append_cabin(cabin)
        
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
            
    if redesign_rotors:
        design_electric_rotor(propulsor)
        save_propulsor(propulsor, os.path.join(local_path, 'tilt_rotor_propulsor.res'))
    else:
        regression_prop_rotor_propulsor = deepcopy(propulsor)        
        design_electric_rotor(regression_prop_rotor_propulsor, iterations=2)
        loaded_propulsor = load_propulsor(os.path.join(local_path, 'tilt_rotor_propulsor.res'))  
        for key,item in propulsor.rotor.items(): 
            propulsor.rotor[key] = loaded_propulsor.rotor[key] 
               
        propulsor.rotor.airfoils.airfoil.coordinate_file  =  local_path   + 'NACA_4412.txt'
        propulsor.rotor.airfoils.airfoil.polar_files      = [local_path   + 'NACA_4412_polar_Re_50000.txt' ,
                                                             local_path   + 'NACA_4412_polar_Re_100000.txt' ,
                                                             local_path   + 'NACA_4412_polar_Re_200000.txt' ,
                                                             local_path   + 'NACA_4412_polar_Re_500000.txt' ,
                                                             local_path   + 'NACA_4412_polar_Re_1000000.txt',
                                                             local_path   + 'NACA_4412_polar_Re_3500000.txt',
                                                             local_path   + 'NACA_4412_polar_Re_5000000.txt',
                                                             local_path   + 'NACA_4412_polar_Re_7500000.txt' ]
       
        for key,item in propulsor.motor.items(): 
            propulsor.motor[key] = loaded_propulsor.motor[key] 
         
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
    propulsor = load(filename)
    return propulsor

def save_propulsor(propulsor, filename):
    save(propulsor, filename)
    return 

if __name__ == '__main__': 
    main()     
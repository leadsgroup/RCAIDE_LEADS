#  NASA_X48py 
""" setup file for the NASA X-48 vehicle
"""
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units, Data       
from RCAIDE.Library.Methods.Geometry.Planform                               import segment_properties    
from RCAIDE.Library.Methods.Powertrain.Converters.Ducted_Fan                import design_ducted_fan
from RCAIDE.Library.Methods.Powertrain.Converters.Motor                     import design_DC_motor 
from RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Electric.Common import compute_motor_weight
from RCAIDE.Library.Plots                                                   import *     
 
# python imports 
import numpy as np  
from copy import deepcopy 
import os

# ----------------------------------------------------------------------
#   Define the Vehicle
# ---------------------------------------------------------------------- 
def vehicle_setup(regression_flag, ducted_fan_type):

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    vehicle     = RCAIDE.Vehicle()
    vehicle.tag = 'NASA_X48'    

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    # mass properties
    vehicle.mass_properties.max_takeoff   = 227  
    vehicle.mass_properties.takeoff       = 227  
    vehicle.mass_properties.max_zero_fuel = 227 
    vehicle.mass_properties.cargo         = 0.0 

    # envelope properties
    vehicle.flight_envelope.ultimate_load = 2.5
    vehicle.flight_envelope.limit_load    = 1.5

    # basic parameters
    vehicle.reference_area                = 9.34 
    vehicle.systems.control               = "fully powered" 
    vehicle.systems.accessories           = "medium range"


    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------ 
    wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing' 
    wing.aspect_ratio                     = 4.1
    wing.sweeps.quarter_chord             = 33. * Units.deg
    wing.thickness_to_chord               = 0.1
    wing.taper                            = 0.0138
    wing.spans.projected                  = 6.4
    wing.chords.root                      = 4.6
    wing.chords.tip                       = wing.chords.root * wing.taper      
    wing.chords.mean_aerodynamic          = (wing.chords.root * (1 + wing.taper ) ) /2 
    wing.areas.reference                  = 9.34 
    wing.areas.wetted                     = 9.34 * 2
    wing.twists.root                      = 3.0 * Units.degrees
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[0,0,0]]
    wing.aerodynamic_center               = [0,0,0] 
    wing.vertical                         = False
    wing.symmetric                        = True
    wing.high_lift                        = True 
    wing.dynamic_pressure_ratio           = 1.0
    
    segment = RCAIDE.Library.Components.Wings.Segments.Segment() 
    segment.tag                   = 'section_1'
    segment.percent_span_location = 0.0
    segment.twist                 = 3. * Units.deg
    segment.root_chord_percent    = 1.
    segment.dihedral_outboard     = 0. * Units.degrees
    segment.sweeps.quarter_chord  = 40.0 * Units.degrees
    segment.thickness_to_chord    = 0.165 
    wing.append_segment(segment)
    
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                      = 'section_2'
    segment.percent_span_location    = 0.052
    segment.twist                    = 3. * Units.deg
    segment.root_chord_percent       = 0.921
    segment.dihedral_outboard        = 0.   * Units.degrees
    segment.sweeps.quarter_chord     = 52.5 * Units.degrees
    segment.thickness_to_chord       = 0.167 
    wing.append_segment(segment)

    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                      = 'section_3'
    segment.percent_span_location    = 0.138
    segment.twist                    = 0. * Units.deg
    segment.root_chord_percent       = 0.76
    segment.dihedral_outboard        = 1.85 * Units.degrees
    segment.sweeps.quarter_chord     = 36.9 * Units.degrees  
    segment.thickness_to_chord       = 0.171 
    wing.append_segment(segment)
    
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                      = 'section_4'
    segment.percent_span_location    = 0.221
    segment.twist                    = 2.5 * Units.deg
    segment.root_chord_percent       = 0.624
    segment.dihedral_outboard        = 1.85 * Units.degrees
    segment.sweeps.quarter_chord     = 30.4 * Units.degrees    
    segment.thickness_to_chord       = 0.175 
    wing.append_segment(segment)
    
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'section_5'
    segment.percent_span_location = 0.457
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 1.313
    segment.dihedral_outboard     = 1.85  * Units.degrees
    segment.sweeps.quarter_chord  = 30.85 * Units.degrees
    segment.thickness_to_chord    = 0.118
    wing.append_segment(segment)
    
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'section_6'
    segment.percent_span_location = 0.568
    segment.twist                 = 1. * Units.deg
    segment.root_chord_percent    = 0.197
    segment.dihedral_outboard     = 1.85 * Units.degrees
    segment.sweeps.quarter_chord  = 34.3 * Units.degrees
    segment.thickness_to_chord    = 0.10
    wing.append_segment(segment)
     
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'tip'
    segment.percent_span_location = 1
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 0.0241
    segment.dihedral_outboard     = 0. * Units.degrees
    segment.sweeps.quarter_chord  = 0. * Units.degrees
    segment.thickness_to_chord    = 0.10
    wing.append_segment(segment)
    
    # Fill out more segment properties automatically
    wing = segment_properties(wing)        

    # add to vehicle
    vehicle.append_component(wing)

    #------------------------------------------------------------------------------------------------------------------------- 
    #  Turbofan Network
    #-------------------------------------------------------------------------------------------------------------------------     
    fuselage = RCAIDE.Library.Components.Fuselages.Blended_Wing_Body_Fuselage()   
    vehicle.append_component(fuselage)    
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Electric Network
    #------------------------------------------------------------------------------------------------------------------------------------  
    #initialize the electric network
    net                              = RCAIDE.Framework.Networks.Electric()   

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus
    #------------------------------------------------------------------------------------------------------------------------------------  
    bus                              = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus() 

    #------------------------------------------------------------------------------------------------------------------------------------           
    # Battery
    #------------------------------------------------------------------------------------------------------------------------------------  
    bat                                                    = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_LFP() 
    bat.tag                                                = 'li_ion_battery'
    bat.electrical_configuration.series                    = 40  
    bat.electrical_configuration.parallel                  = 1 
    bat.geometrtic_configuration.normal_count              = 40  
    bat.geometrtic_configuration.parallel_count            = 1
    bus.battery_modules.append(bat)      
    bus.initialize_bus_properties()
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Starboard Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    center_propulsor                              = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Ducted_Fan()  
    center_propulsor.tag                          = 'center_propulsor' 
  
    # Electronic Speed Controller       
    esc                                           = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
    esc.tag                                       = 'esc_1'
    esc.efficiency                                = 0.95 
    center_propulsor.electronic_speed_controller  = esc   
        

    # Ducted_fan                            
    ducted_fan                                   = RCAIDE.Library.Components.Powertrain.Converters.Ducted_Fan()
    ducted_fan.tag                               = 'ducted_fan'
    ducted_fan.number_of_rotor_blades            = 12 
    ducted_fan.number_of_radial_stations         = 20
    ducted_fan.tip_radius                        = 6 * Units.inches  / 2
    ducted_fan.hub_radius                        = 0.25 * ducted_fan.tip_radius 
    ducted_fan.exit_radius                       = 1.1 * ducted_fan.tip_radius
    ducted_fan.blade_clearance                   = 0.001
    ducted_fan.length                            = 10. * Units.inches 
    ducted_fan.fan_effectiveness                 = 1.1  
    ducted_fan.rotor_percent_x_location          = 0.4
    ducted_fan.origin                            = [[2.,  0, 0.95]]
    ducted_fan.stator_percent_x_location         = 0.7
    ducted_fan.fidelity                          = ducted_fan_type  
    ducted_fan.cruise.design_thrust              = 75
    ducted_fan.cruise.design_altitude            = 5000  * Units.ft
    ducted_fan.cruise.design_angular_velocity    = (0.8* 339.709) /  ducted_fan.tip_radius
    ducted_fan.cruise.design_freestream_velocity = 90 *  Units.mph
    ducted_fan.cruise.design_reference_velocity  = 90 *  Units.mph 
     
    if ducted_fan_type == 'Blade_Element_Momentum_Theory':
        airfoil                                      = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil() 
        airfoil.NACA_4_Series_code                   = '2208'
        ducted_fan.append_duct_airfoil(airfoil)  
        airfoil                                      = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
        airfoil.NACA_4_Series_code                   = '0008'    
        ducted_fan.append_hub_airfoil(airfoil)   
        dfdc_bin_name                                = '/Users/matthewclarke/Documents/LEADS/CODES/DFDC/bin/dfdc'  
    else:
        dfdc_bin_name                                = 'dfdc' 
     
    design_ducted_fan(ducted_fan,dfdc_bin_name,regression_flag,keep_files = True) 
    center_propulsor.ducted_fan                  = ducted_fan    
              
    # DC_Motor       
    motor                                         = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor()
    motor.efficiency                              = 0.98
    motor.origin                                  = [[2.,  0, 0.95]]
    motor.nominal_voltage                         = bus.voltage 
    motor.no_load_current                         = 0.001
    motor.rotor_radius                            = ducted_fan.tip_radius
    motor.design_torque                           = ducted_fan.cruise.design_torque
    motor.angular_velocity                        = ducted_fan.cruise.design_angular_velocity 
    design_DC_motor(motor)   
    motor.mass_properties.mass                    = compute_motor_weight(motor) 
    center_propulsor.motor                        = motor  
    net.propulsors.append(center_propulsor) 

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Right Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    starboard_propulsor                             = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Ducted_Fan() 
    starboard_propulsor.tag                         = "starboard_propulsor"  
    esc_2                                           = deepcopy(esc)
    esc_2.origin                                    = [[2., 2.5, 0.95]]      
    starboard_propulsor.electronic_speed_controller = esc_2 
    ducted_fan_2                                    = deepcopy(ducted_fan)
    ducted_fan_2.tag                                = 'ducted_fan_2' 
    ducted_fan_2.origin                             = [[2.,2.5,0.95]]
    ducted_fan_2.clockwise_rotation                 = False        
    starboard_propulsor.ducted_fan                  = ducted_fan_2
    motor_2                                         = deepcopy(motor)
    motor_2.origin                                  = [[2., 2.5, 0.95]]      
    starboard_propulsor.motor                       = motor_2    
    # append propulsor to network 
    net.propulsors.append(starboard_propulsor)     

    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Left Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    port_propulsor                             = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Ducted_Fan() 
    port_propulsor.tag                         = "port_propulsor"  
    esc_3                                      = deepcopy(esc)
    esc_3.origin                               = [[2., -2.5, 0.95]]      
    port_propulsor.electronic_speed_controller = esc_3   
    ducted_fan_3                               = deepcopy(ducted_fan)
    ducted_fan_3.tag                           = 'ducted_fan_3' 
    ducted_fan_3.origin                        = [[2.,-2.5,0.95]]
    ducted_fan_3.clockwise_rotation            = False        
    port_propulsor.ducted_fan                  = ducted_fan_3
    motor_3                                    = deepcopy(motor)
    motor_3.origin                             = [[2., -2.5, 0.95]]      
    port_propulsor.motor                       = motor_2    
    # append propulsor to network
    net.propulsors.append(port_propulsor)     


    #------------------------------------------------------------------------------------------------------------------------------------           
    # Payload 
    #------------------------------------------------------------------------------------------------------------------------------------  
    payload                      = RCAIDE.Library.Components.Payloads.Payload()
    payload.power_draw           = 10. # Watts
    payload.mass_properties.mass = 1.0 * Units.kg
    bus.payload                  = payload

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Avionics
    #------------------------------------------------------------------------------------------------------------------------------------  
    avionics                     = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.power_draw          = 20. # Watts
    bus.avionics                 = avionics   

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to bus       
    bus.assigned_propulsors =  [[center_propulsor.tag, starboard_propulsor.tag, port_propulsor.tag]]

    # append bus   
    net.busses.append(bus)
    
    vehicle.append_energy_network(net)

    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------

    return vehicle


# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):

    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs     = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base'  
    configs.append(base_config)
 
    return configs

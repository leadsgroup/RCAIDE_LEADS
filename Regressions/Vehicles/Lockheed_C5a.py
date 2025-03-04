# Research/Aircraft/Lockheed_C-5a.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units      
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan_Propulsor   import design_turbofan
from RCAIDE.Library.Methods.Geometry.Planform               import wing_planform, segment_properties
from RCAIDE.Library.Plots                 import *     

# python imports 
import numpy as np  
from copy import deepcopy
import matplotlib.pyplot as plt  
import os   

def main():
    
    # Step 1 design a vehicle
    vehicle  = vehicle_setup()    
    
    # Step 2 create aircraft configuration based on vehicle 
    configs  = configs_setup(vehicle)

    # plot vehicle 
    plot_3d_vehicle(vehicle,
                    min_x_axis_limit            = -100,
                    max_x_axis_limit            = 100,
                    min_y_axis_limit            = -100,
                    max_y_axis_limit            = 100,
                    min_z_axis_limit            = -100,
                    max_z_axis_limit            = 100)          
    
    return
# ----------------------------------------------------------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------------------------------------------------------
def vehicle_setup():
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------

    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'Lockheed_C-5a'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties: https://apps.dtic.mil/sti/tr/pdf/ADA145518.pdf 
    # ------------------------------------------------------------------

    # mass properties 
    vehicle.mass_properties.max_takeoff               =   348812  # kg
    vehicle.mass_properties.operating_empty           =   169643 # kg
    vehicle.mass_properties.takeoff                   =   296750 # kg
    vehicle.mass_properties.max_zero_fuel             =   288000 # kg
    vehicle.mass_properties.max_payload               =   118364 # kg
    vehicle.mass_properties.max_fuel                  =   150800 # kg
    vehicle.mass_properties.payload                   =   99770 # kg
    
    fuel_percentage = 1.0

    # envelope properties
    vehicle.flight_envelope.ultimate_load = 3.5
    vehicle.flight_envelope.limit_load    = 1.5
    
    vehicle.flight_envelope.design_range = 3700 * Units.mile
    vehicle.flight_envelope.design_cruise_altitude = 35000 * Units.feet
    vehicle.flight_envelope.design_mach_number  = 0.75
    

    # basic parameters
    vehicle.reference_area         = 565.33
    vehicle.passengers             = 0 # 83
    vehicle.systems.control        = "fully powered"
    vehicle.systems.accessories    = "long range"

    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------
    wing                                   = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                               = 'main_wing'
    wing.areas.reference                   = 565.33
    wing.aspect_ratio                      = 8.122
    wing.chords.root                       = 14.0
    wing.chords.tip                        = 3.667
    wing.sweeps.quarter_chord              = 24.0 * Units.deg
    wing.thickness_to_chord                = 0.131
    wing.chords.mean_aerodynamic           = 8.533 * Units.meter 
    wing.taper                             = 0.262
    wing.dihedral                          = -3.5 * Units.deg
    wing.spans.projected                   = 66.3
    wing.origin                            = [[20.0,0,3.913]]
    wing.vertical                          = False
    wing.symmetric                         = True       
    wing.high_lift                         = True
    wing.areas.exposed                     =  1.0* wing.areas.wetted        
    wing.twists.root                       =  0.0* Units.degrees
    wing.twists.tip                        =  0.0* Units.degrees    
    wing.dynamic_pressure_ratio            = 1.0
    wing.mass_properties.center_of_gravity = [[23.0,0,3.913]] 
    
    
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'root'
    root_airfoil                  = RCAIDE.Library.Components.Airfoils.Airfoil()
    ospath                        = os.path.abspath(__file__)
    separator                     = os.path.sep
    segment.percent_span_location = 0.0
    rel_path                      = os.path.dirname(ospath) + separator  + '..'  + separator 
    segment.twist                 = 0.0 * Units.deg
    root_airfoil.coordinate_file  = rel_path  + 'C-5a' + separator + 'c5a.txt'
    segment.root_chord_percent    = 1.0
    segment.thickness_to_chord    = 0.131
    segment.dihedral_outboard     = -3.5 * Units.degrees
    segment.sweeps.quarter_chord  = 24.0 * Units.degrees
    wing.segments.append(segment)    
    
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    Yehudi_airfoil                  = RCAIDE.Library.Components.Airfoils.Airfoil() 
    Yehudi_airfoil.coordinate_file  = rel_path  + 'C-5a' + separator + 'c5c.txt'    
    segment.tag                   = 'yehudi'
    segment.percent_span_location = 0.471
    segment.twist                 = 0 # (4. - segment.percent_span_location*4.) * Units.deg
    segment.root_chord_percent    = 0.588
    segment.thickness_to_chord    = 0.111
    segment.dihedral_outboard     =  -3.5* Units.degrees
    segment.sweeps.quarter_chord  =  24.0* Units.degrees
    wing.segments.append(segment)

    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    tip_airfoil                   =  RCAIDE.Library.Components.Airfoils.Airfoil()    
    tip_airfoil.coordinate_file   = rel_path + 'C-5a' + separator + 'c5e.txt'
    segment.tag                   = 'Tip'
    segment.percent_span_location = 1.
    segment.twist                 = 0#(4. - segment.percent_span_location*4.) * Units.deg
    segment.root_chord_percent    = 0.262
    segment.thickness_to_chord    = 0.108
    segment.dihedral_outboard     = -3.5 * Units.degrees
    segment.sweeps.quarter_chord  = 24.0 * Units.degrees
    wing.segments.append(segment)       
    
    # Fill out more segment properties automatically
    wing = segment_properties(wing)        

    # control surfaces -------------------------------------------
    flap                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap() 
    flap.tag                   = 'flap' 
    flap.span_fraction_start   = 0.108
    flap.span_fraction_end     = 0.73
    flap.deflection            = 0.0 * Units.deg 
    flap.chord_fraction        = 0.18    
    flap.configuration_type    = 'single_slotted'
    wing.append_control_surface(flap)   
        
    slat                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Slat()
    slat.tag                   = 'slat' 
    slat.span_fraction_start   = 0.12 
    slat.span_fraction_end     = 0.93     
    slat.deflection            = 0.0 * Units.deg 
    slat.chord_fraction        = 0.13   
    wing.append_control_surface(slat)
    
    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.73
    aileron.span_fraction_end     = 0.93
    aileron.deflection            = 0.0 * Units.degrees
    aileron.chord_fraction        = 0.16
    wing.append_control_surface(aileron)     
    wing                          = wing_planform(wing) 

    # add to vehicle
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag = 'horizontal_stabilizer'
    wing.areas.reference         = 90.2
    wing.aspect_ratio            = 4.653
    wing.sweeps.quarter_chord    = 25.0 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.taper                   =0.375
    wing.chords.root             = 6.4 
    wing.chords.tip              = 2.4 
    wing.chords.mean_aerodynamic = 4.4
    wing.areas.reference         = 90.2
    wing.areas.exposed           = 85.0    # Exposed area of the horizontal tail
    wing.areas.wetted            = 185.0     # Wetted area of the horizontal tail    
    wing.anhedral                = 2.0 * Units.degrees
    wing.origin                  = [[64.6,0,14.783]]
    wing.vertical                = False
    wing.symmetric               = True       
    wing.high_lift               = False
    wing                         = wing_planform(wing)
    wing.areas.exposed           = 1.0 * wing.areas.wetted 
    wing.twists.root             = 0 * Units.degrees
    wing.twists.tip              = 0 * Units.degrees    
    wing.dynamic_pressure_ratio  = 0.95 # Double check this number
    
    # Wing Segments
    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'root_segment'
    segment.percent_span_location  = 0.0
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 1.0
    segment.anhedral_outboard      = 2.0 * Units.degrees
    segment.sweeps.quarter_chord   = 28.2250  * Units.degrees 
    segment.thickness_to_chord     = .1
    wing.append_segment(segment)

    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'tip_segment'
    segment.percent_span_location  = 1.
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 0.3333               
    segment.dihedral_outboard      = -2.0 * Units.degrees
    segment.sweeps.quarter_chord   = 0 * Units.degrees  
    segment.thickness_to_chord     = .1
    wing.append_segment(segment)
    
    # Fill out more segment properties automatically
    wing = segment_properties(wing)        

    # control surfaces -------------------------------------------
    elevator                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
    elevator.tag                   = 'elevator'
    elevator.span_fraction_start   = 0.09
    elevator.span_fraction_end     = 0.92
    elevator.deflection            = 0.0  * Units.deg
    elevator.chord_fraction        = 0.3
    wing.append_control_surface(elevator)    

    # add to vehicle
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    
    wing = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag = 'vertical_stabilizer'

    wing.aspect_ratio            = 1.307
    wing.sweeps.quarter_chord    = 35.0  * Units.deg   
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 0.947

    wing.spans.projected         = 11.57
    wing.total_length            = wing.spans.projected 
    
    wing.chords.root             = 9.5 
    wing.chords.tip              = 9.0 
    wing.chords.mean_aerodynamic = 8.78

    wing.areas.reference         = 102.
    wing.areas.wetted            = 215.0 # An approximation
    
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.origin                  = [[58.5,0,3.587]]
    wing.aerodynamic_center      = [0,0,0]

    wing.vertical                = True
    wing.symmetric               = False
    wing.t_tail                  = True

    wing.dynamic_pressure_ratio  = 1.0


    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root'
    segment.percent_span_location         = 0.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 1.
    segment.dihedral_outboard             = 0 * Units.degrees
    segment.sweeps.quarter_chord          = 35.0 * Units.degrees  
    segment.thickness_to_chord            = .1
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_1'
    segment.percent_span_location         = 0.78
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.8745
    segment.dihedral_outboard             = 0. * Units.degrees
    segment.sweeps.quarter_chord          = 0.0 * Units.degrees   
    segment.thickness_to_chord            = .1
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.947
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.quarter_chord          = 0.0    
    segment.thickness_to_chord            = .1  
    wing.append_segment(segment)
    
 
    # control surfaces -------------------------------------------
    rudder                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    rudder.tag                   = 'rudder'
    rudder.span_fraction_start   = 0.05 
    rudder.span_fraction_end     = 0.769 
    rudder.deflection            = 0 
    rudder.chord_fraction        = 0.25  
    wing.append_control_surface(rudder)    
    
    
    # Fill out more segment properties sautomatically
    wing = segment_properties(wing)        

    # add to vehicle
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage                                    = RCAIDE.Library.Components.Fuselages.Tube_Fuselage() 
    fuselage.number_coach_seats                 = vehicle.passengers 
    fuselage.seats_abreast                      = 6
    fuselage.seat_pitch                         = 1     * Units.meter 
    fuselage.fineness.nose                      = 1.6
    fuselage.fineness.tail                      = 2.0
    fuselage.lengths.nose                       = 7.4   * Units.meter
    fuselage.lengths.tail                       = 27.00   * Units.meter
    fuselage.lengths.total                      = 70.0 * Units.meter  
    fuselage.lengths.fore_space                 = 6.5    * Units.meter
    fuselage.lengths.aft_space                  = 21.5    * Units.meter
    fuselage.width                              = 7.4  * Units.meter
    fuselage.heights.maximum                    = 8.1  * Units.meter
    fuselage.effective_diameter                 = 7.75  * Units.meter
    fuselage.areas.side_projected               = fuselage.heights.maximum * fuselage.lengths.total * Units['meters**2'] 
    fuselage.areas.wetted                       = np.pi * fuselage.width/2 * fuselage.lengths.total * Units['meters**2'] 
    fuselage.areas.front_projected              = np.pi * fuselage.width/2      * Units['meters**2']  
    fuselage.differential_pressure              = 5.0e4 * Units.pascal
    fuselage.heights.at_quarter_length          = 8.0 * Units.meter
    fuselage.heights.at_three_quarters_length   = 6.75 * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = 8.1 * Units.meter
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_0'    
    segment.percent_x_location                  = 0.0000
    segment.percent_z_location                  = 0.00144 
    segment.height                              = 0.000 
    segment.width                               = 0.000  
    fuselage.segments.append(segment)   
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_1'    
    segment.percent_x_location                  = 0.0079 
    segment.percent_z_location                  = 0.00183 
    segment.height                              = 1.53636
    segment.width                               = 2.12455
    fuselage.segments.append(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.02279 
    segment.percent_z_location                  = 0.00221 
    segment.height                              = 2.9 
    segment.width                               = 3.7 
    fuselage.segments.append(segment)      
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.03869 
    segment.percent_z_location                  = 0.00416 
    segment.height                              = 4.05 
    segment.width                               = 4.800 
    fuselage.segments.append(segment)   

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.06471 	
    segment.percent_z_location                  = 0.00832 
    segment.height                              = 5.63 
    segment.width                               = 6.07409 
    fuselage.segments.append(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 0.07611 
    segment.percent_z_location                  = 0.01192 
    segment.height                              = 6.56 
    segment.width                               = 6.45 
    fuselage.segments.append(segment)     
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 0.08938 
    segment.percent_z_location                  = 0.01325 
    segment.height                              = 7.07 
    segment.width                               = 6.87273 
    fuselage.segments.append(segment)             
     
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.10569 
    segment.percent_z_location                  = 0.01393 
    segment.height                              = 7.50455 
    segment.width                               = 7.26364 
    fuselage.segments.append(segment)    
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 0.13859 
    segment.percent_z_location                  = 0.01240 
    segment.height                              = 7.8 
    segment.width                               = 7.4 
    fuselage.segments.append(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_9'     
    segment.percent_x_location                  = 0.590 
    segment.percent_z_location                  = 0.00543
    segment.height                              = 8.1
    segment.width                               = 7.4
    fuselage.segments.append(segment)     
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_10'     
    segment.percent_x_location                  = 0.61462 
    segment.percent_z_location                  = 0.00617
    segment.height                              = 8.0
    segment.width                               = 7.4
    fuselage.segments.append(segment)   
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_11'     
    segment.percent_x_location                  = 0.69067 
    segment.percent_z_location                  = 0.01497
    segment.height                              = 7.5
    segment.width                               = 7.4
    fuselage.segments.append(segment)    
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_12'     
    segment.percent_x_location                  = 0.77524
    segment.percent_z_location                  = 0.02446
    segment.height                              = 6.55
    segment.width                               = 7.2
    fuselage.segments.append(segment)             
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_13'     
    segment.percent_x_location                  = 0.85018 
    segment.percent_z_location                  = 0.03133
    segment.height                              = 5.16
    segment.width                               = 5.8
    fuselage.segments.append(segment)               
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_14'     
    segment.percent_x_location                  = 0.92308 
    segment.percent_z_location                  = 0.037
    segment.height                              = 3.25
    segment.width                               = 3.8
    fuselage.segments.append(segment)       
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_15'     
    segment.percent_x_location                  = 0.96625 
    segment.percent_z_location                  = 0.039
    segment.height                              = 1.67
    segment.width                               = 2.0
    fuselage.segments.append(segment)           # Segment
    
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_16'     
    segment.percent_x_location                  = 1.00 
    segment.percent_z_location                  = 0.04076
    segment.height                              = 0.0
    segment.width                               = 0.0
    fuselage.segments.append(segment)       

    # add to vehicle
    vehicle.append_component(fuselage)

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Landing Gear
    #------------------------------------------------------------------------------------------------------------------------------------   
    main_gear               = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear()
    main_gear.tire_diameter       =  1.2* Units.m
    main_gear.strut_length        =  0.80* Units.m
    main_gear.wheels              =  6   #number of wheels on the main landing gear
    main_gear.units               =  4   #number of nose landing gear   
    vehicle.append_component(main_gear)  

    nose_gear               = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()
    nose_gear.tire_diameter       =  1.2* Units.m
    nose_gear.strut_length        =  0.80* Units.m
    nose_gear.units               =  1   #number of nose landing gear
    nose_gear.wheels              =  4   #number of wheels on the nose landing gear  
    vehicle.append_component(nose_gear)
  
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Turbofan Network
    #------------------------------------------------------------------------------------------------------------------------------------  
    #initialize the gas turbine network
    net                                         = RCAIDE.Framework.Networks.Fuel() 
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Fuel Distrubition Line 
    #------------------------------------------------------------------------------------------------------------------------------------  
    fuel_line                                   = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line() 
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Fuel Tank & Fuel
    #------------------------------------------------------------------------------------------------------------------------------------   
    fuel_tank                                   = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Wing_Fuel_Tank()
    fuel_tank.origin                            = [[23.0,0,3.913]] # vehicle.wings.main_wing.origin   
    fuel_tank.mass_properties.center_of_gravity = [[23.0,0,3.913]] #vehicle.wings.main_wing.mass_properties.center_of_gravity    
    
    # fuel 
    fuel                                        = RCAIDE.Library.Attributes.Propellants.Jet_A1()   
    fuel.mass_properties.mass                   = fuel_percentage * vehicle.mass_properties.max_fuel
    fuel.origin                                 = [[23.0,0,3.913]]# vehicle.wings.main_wing.origin    
    fuel.mass_properties.center_of_gravity      = [[23.0,0,3.913]] #vehicle.wings.main_wing.mass_properties.center_of_gravity
    fuel.internal_volume                        = fuel.mass_properties.mass/fuel.density  
    fuel_tank.fuel                              = fuel
    fuel_tank.volume                           = fuel.internal_volume
    fuel_line.fuel_tanks.append(fuel_tank) 
    

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------    
    turbofan                                        = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan() 
    turbofan.tag                                    = 'outer_starboard_propulsor' 
    turbofan.origin                                 = [[26.429, 17.6, 0.2]] 
    turbofan.engine_length                          = 7.92    
    turbofan.bypass_ratio                           = 8  
    turbofan.design_altitude                        = 0*Units.ft
    turbofan.design_mach_number                     = 0.01
    turbofan.design_thrust                          = 193000* Units.N 
     
    # Nacelle 
    nacelle                                         = RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle()
    nacelle.diameter                                = 2.5
    nacelle.length                                  = 3.00
    nacelle.tag                                     = 'nacelle_1'
    nacelle.inlet_diameter                          = 2.46
    nacelle.origin                                  = [[26.429,17.6,0.2]] 
    nacelle.areas.wetted                            = 1.1*np.pi*nacelle.diameter*nacelle.length
    nacelle_airfoil                                 = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    nacelle_airfoil.NACA_4_Series_code              = '2410'
    nacelle.append_airfoil(nacelle_airfoil)
    turbofan.nacelle                                = nacelle
                  
    # fan                     
    fan                                             = RCAIDE.Library.Components.Powertrain.Converters.Fan()   
    fan.tag                                         = 'fan'
    fan.polytropic_efficiency                       = 0.93
    fan.pressure_ratio                              = 1.7   
    turbofan.fan                                    = fan        
                        
    # working fluid                        
    turbofan.working_fluid                          = RCAIDE.Library.Attributes.Gases.Air() 
    ram                                             = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                         = 'ram' 
    turbofan.ram                                    = ram 
               
    # inlet nozzle               
    inlet_nozzle                                    = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                                = 'inlet nozzle'
    inlet_nozzle.polytropic_efficiency              = 0.98
    inlet_nozzle.pressure_ratio                     = 0.98 
    turbofan.inlet_nozzle                           = inlet_nozzle


    # low pressure compressor    
    low_pressure_compressor                        = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    low_pressure_compressor.tag                    = 'lpc'
    low_pressure_compressor.polytropic_efficiency  = 0.91
    low_pressure_compressor.pressure_ratio         = 1.9   
    turbofan.low_pressure_compressor               = low_pressure_compressor

    # high pressure compressor  
    high_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    high_pressure_compressor.tag                   = 'hpc'
    high_pressure_compressor.polytropic_efficiency = 0.91
    high_pressure_compressor.pressure_ratio        = 7.73    
    turbofan.high_pressure_compressor              = high_pressure_compressor

    # low pressure turbine  
    low_pressure_turbine                           = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    low_pressure_turbine.tag                       ='lpt'
    low_pressure_turbine.mechanical_efficiency     = 0.99
    low_pressure_turbine.polytropic_efficiency     = 0.93 
    turbofan.low_pressure_turbine                  = low_pressure_turbine
   
    # high pressure turbine     
    high_pressure_turbine                          = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    high_pressure_turbine.tag                      ='hpt'
    high_pressure_turbine.mechanical_efficiency    = 0.99
    high_pressure_turbine.polytropic_efficiency    = 0.93 
    turbofan.high_pressure_turbine                 = high_pressure_turbine 
   
    # combustor     
    combustor                                      = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                                  = 'Comb'
    combustor.efficiency                           = 0.99 
    combustor.alphac                               = 1.0     
    combustor.turbine_inlet_temperature            = 1370 # celcius
    combustor.pressure_ratio                       = 0.95
    combustor.fuel_data                            = RCAIDE.Library.Attributes.Propellants.Jet_A()  
    turbofan.combustor                             = combustor
           
    # core nozzle           
    core_nozzle                                    = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                = 'core nozzle'
    core_nozzle.polytropic_efficiency              = 0.95
    core_nozzle.pressure_ratio                     = 0.99  
    core_nozzle.diameter                           = 0.92    
    turbofan.core_nozzle                           = core_nozzle
          
    # fan nozzle          
    fan_nozzle                                  = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    fan_nozzle.tag                              = 'fan nozzle'
    fan_nozzle.polytropic_efficiency            = 0.95
    fan_nozzle.pressure_ratio                   = 0.99 
    fan_nozzle.diameter                         = 1.659
    turbofan.fan_nozzle                         = fan_nozzle 
    
    #design turbofan
    design_turbofan(turbofan)  
    
    # append propulsor to distribution line 
    net.propulsors.append(turbofan)


    #------------------------------------------------------------------------------------------------------------------------------------  
    # Inner Starboard Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------     
    
    # copy turbofan
    turbofan_2                             = deepcopy(turbofan)
    turbofan_2.tag                         = 'inner_starboard_propulsor'  
    turbofan_2.origin                      = [[23.214,11.2,0.2]]   # change origin  
    turbofan_2.nacelle.origin              = [[23.214,11.2,0.2]]    
    
    # append propulsor to distribution line 
    net.propulsors.append(turbofan_2)

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Inner Port Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------     
    
    # copy turbofan
    turbofan_3                             = deepcopy(turbofan)
    turbofan_3.tag                         = 'inner_port_propulsor'  
    turbofan_3.origin                      = [[23.214, -11.2,0.2]]   # change origin  
    turbofan_3.nacelle.origin              = [[23.214, -11.2,0.2]]    
    
    # append propulsor to distribution line 
    net.propulsors.append(turbofan_3)
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Outer Port Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------     
    
    # copy turbofan
    turbofan_4                             = deepcopy(turbofan)
    turbofan_4.tag                         = 'outer_port_propulsor'  
    turbofan_4.origin                      = [[26.429, -17.6,0.2]]   # change origin  
    turbofan_4.nacelle.origin              = [[26.429, -17.6,0.2]]    
    
    # append propulsor to distribution line 
    net.propulsors.append(turbofan_4)
 
    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to fuel line   
    fuel_line.assigned_propulsors =  [[turbofan.tag, turbofan_2.tag, turbofan_3.tag, turbofan_4.tag]]
 
    #------------------------------------------------------------------------------------------------------------------------------------   
    # Append fuel line to network      
    net.fuel_lines.append(fuel_line)        
    
    # Append energy network to aircraft 
    vehicle.append_energy_network(net)     
        
    return vehicle

# ---------------------------------------------------------------------
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
    config.wings['main_wing'].control_surfaces.flap.deflection  =  16 * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  =  16 * Units.deg 
    config.networks.fuel.propulsors['outer_starboard_propulsor'].fan.angular_velocity =   3860* Units.rpm
    config.networks.fuel.propulsors['outer_port_propulsor'].fan.angular_velocity      =   3860* Units.rpm 
    config.networks.fuel.propulsors['inner_starboard_propulsor'].fan.angular_velocity =   3860* Units.rpm
    config.networks.fuel.propulsors['inner_port_propulsor'].fan.angular_velocity      =   3860* Units.rpm    
    config.V2_VS_ratio = 1.21
    configs.append(config)

    
    # ------------------------------------------------------------------
    #   Cutback Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'cutback'
    config.wings['main_wing'].control_surfaces.flap.deflection  =  0* Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  =  0* Units.deg
    config.networks.fuel.propulsors['outer_starboard_propulsor'].fan.angular_velocity =   3474* Units.rpm
    config.networks.fuel.propulsors['outer_port_propulsor'].fan.angular_velocity      =   3474* Units.rpm
    config.networks.fuel.propulsors['inner_starboard_propulsor'].fan.angular_velocity =   3474* Units.rpm
    config.networks.fuel.propulsors['inner_port_propulsor'].fan.angular_velocity      =   3474* Units.rpm     
    configs.append(config)   
    
        
    
    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'landing'
    config.wings['main_wing'].control_surfaces.flap.deflection  =  40* Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  =  40* Units.deg
    config.networks.fuel.propulsors['outer_starboard_propulsor'].fan.angular_velocity =  2316 * Units.rpm
    config.networks.fuel.propulsors['outer_port_propulsor'].fan.angular_velocity      =  2316 * Units.rpm
    config.networks.fuel.propulsors['inner_starboard_propulsor'].fan.angular_velocity =  2316 * Units.rpm
    config.networks.fuel.propulsors['inner_port_propulsor'].fan.angular_velocity      =  2316 * Units.rpm    
    config.landing_gears.main_gear.gear_extended    = True
    config.landing_gears.nose_gear.gear_extended    = True  
    config.Vref_VS_ratio = 1.3
    configs.append(config)   
     
    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------ 

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'short_field_takeoff'    
    config.wings['main_wing'].control_surfaces.flap.deflection  =  16* Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  =  16* Units.deg
    config.networks.fuel.propulsors['outer_starboard_propulsor'].fan.angular_velocity =   4091* Units.rpm
    config.networks.fuel.propulsors['outer_port_propulsor'].fan.angular_velocity      =   4091* Units.rpm
    config.networks.fuel.propulsors['inner_starboard_propulsor'].fan.angular_velocity =   4091* Units.rpm
    config.networks.fuel.propulsors['inner_port_propulsor'].fan.angular_velocity      =   4091* Units.rpm    
    config.landing_gears.main_gear.gear_extended    = True
    config.landing_gears.nose_gear.gear_extended    = True  
    config.V2_VS_ratio = 1.21 
    configs.append(config)    

    # done!
    return configs

if __name__ == '__main__': 
    main()
    plt.show()


# RESEARCH/Aircraft/Lockheed_F22/Lockheed_F22.py
# 
# 
# Created:  Jan 2025, A. Molloy, M. Guidotti, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units           
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan                import design_turbofan     
from RCAIDE.Library.Methods.Geometry.Planform                             import segment_properties   
from RCAIDE.Library.Plots                                                 import *     
import RCAIDE.Framework.External_Interfaces.OpenVSP as openvsp
from RCAIDE.Library.Plots.Common import set_axes, plot_style 

# python imports 
import numpy as np  
from   copy import deepcopy
import matplotlib.pyplot as plt  
import os
import sys
import pickle
import matplotlib.cm as cm
import numpy as np 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
 
    # Step 1 design a vehicle 
    vehicle  = vehicle_setup()
   
    # Step 2 create aircraft configuration based on vehicle 
    # configs  = configs_setup(vehicle)

    # # Step 3 set up analysis
    # analyses = analyses_setup(configs)

    # # Step 4 set up a flight mission
    # escort_mission = escort_mission_setup(analyses)
    # dogfight_mission = dogfight_mission_setup(analyses)
    # long_range_mission = long_range_mission_setup(analyses)

    # missions = missions_setup(long_range_mission, escort_mission, dogfight_mission) 

    # # Step 5 execute flight profile
    # # escort_mission_results = missions.escort_mission.evaluate() 
    # # dogfight_mission_results = missions.dogfight_mission.evaluate()  
    # long_range_mission_results = missions.long_range_mission.evaluate()

    # # Step 6 plot results 
    # plot_mission(long_range_mission_results)
    # plot_mission(escort_mission_results)
    # plot_mission(dogfight_mission_results)

    plot_3d_vehicle(vehicle,
                    min_x_axis_limit            = -1,
                    max_x_axis_limit            = 50,
                    min_y_axis_limit            = -25,
                    max_y_axis_limit            = 25,
                    min_z_axis_limit            = -25,
                    max_z_axis_limit            = 25)          

    return

def vehicle_setup(): 
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------      
    vehicle = RCAIDE.Vehicle()
   
    # ################################################# Vehicle-level Properties #################################################   
    vehicle.tag                                       = 'Lockheed_F35C' 
    vehicle.mass_properties.max_takeoff               = 31800. * Units.kilogram
    vehicle.mass_properties.max_zero_fuel             = 23846 * Units.kilogram
    vehicle.mass_properties.max_fuel                  = 8980 * Units.kilogram
    vehicle.mass_properties.fuel                      = 8165 * Units.kilogram
    vehicle.mass_properties.takeoff                   = 31800. * Units.kilogram   
    vehicle.mass_properties.max_payload               = 8160 * Units.kilogram 
    vehicle.mass_properties.payload                   = 6000 *Units.kilogram  
    vehicle.mass_properties.center_of_gravity         = [[10.0, 0, 0]] 
    vehicle.flight_envelope.ultimate_load             = 13.5 
    vehicle.flight_envelope.positive_limit_load       = 7.5 
    vehicle.flight_envelope.negative_limit_load       = -6 
    vehicle.flight_envelope.design_mach_number        = 1.6 
    vehicle.flight_envelope.design_cruise_altitude    = 50000.0*Units.feet 
    vehicle.flight_envelope.design_range              = 1200.0 * Units.nmi # Combat radius is 670 nmi
    vehicle.reference_area                            = 66.04 * Units['meters**2']
    vehicle.number_of_passengers                                = 0
    vehicle.systems.control                           = "fully powered" 

    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------

    wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing'
    wing.aspect_ratio                     = 2.574
    wing.sweeps.quarter_chord             = 35 * Units.deg
    wing.thickness_to_chord               = 0.025
    wing.spans.projected                  = 13.039 * Units.meter
    wing.chords.root                      = 7.87 * Units.meter
    wing.chords.tip                       = 1.484 * Units.meter
    wing.taper                            = wing.chords.tip / wing.chords.root
    wing.chords.mean_aerodynamic          = 8.167 * Units.meter 
    wing.areas.reference                  = 66.04 * Units['meters**2']
    wing.areas.wetted                     = 100.0 * Units['meters**2']
    wing.twists.root                      = 0.0 * Units.degrees 
    wing.twists.tip                       = -3.0 * Units.degrees 
    wing.origin                           = [[5.046,0,0.426]]
    wing.aerodynamic_center               = [10.0,0,0.426] 
    wing.vertical                         = False
    wing.dihedral                         = 0.0 * Units.degrees 
    wing.xz_plane_symmetric               = True 
    wing.dynamic_pressure_ratio           = 1.0
    
    ospath                                = os.path.abspath(__file__)
    separator                             = os.path.sep
    rel_path                              = os.path.dirname(ospath) + separator  + '..'  + separator 
    wing_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    wing_airfoil.coordinate_file          = rel_path + 'Airfoils' + separator + 'NACA65_203.txt' 

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Segment_0'
    segment.percent_span_location         = 0.0
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.leading_edge           = 0.0 * Units.degrees
    segment.thickness_to_chord            = .005
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Segment_1'
    segment.percent_span_location         = 0.1292
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.leading_edge           = -47.04 * Units.degrees
    segment.thickness_to_chord            = .0025
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Segment_2'
    segment.percent_span_location         = 0.16150
    segment.root_chord_percent            = 1.414
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.leading_edge           = -47.04 * Units.degrees
    segment.thickness_to_chord            = .0025
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Segment_3'
    segment.percent_span_location         = 0.1938
    segment.root_chord_percent            = 1.4462
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.leading_edge           = -47.04 * Units.degrees
    segment.thickness_to_chord            = .0025
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Segment_4'
    segment.percent_span_location         = 0.2216
    segment.root_chord_percent            = 1.1529 
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.leading_edge           = -47.04 * Units.degrees
    segment.thickness_to_chord            = .0025
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Segment_5'
    segment.percent_span_location         = 0.258
    segment.root_chord_percent            = 1.1689
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.leading_edge           = 86.6 * Units.degrees
    segment.thickness_to_chord            = .0025
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Segment_6'
    segment.percent_span_location         = 0.2836
    segment.root_chord_percent            = 0.78137
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.leading_edge           = 35.0 * Units.degrees
    segment.thickness_to_chord            = .0025
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)
    
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Tip'
    segment.percent_span_location         = 1.0
    segment.root_chord_percent            = 0.1983
    segment.dihedral_outboard             = 0 * Units.degrees
    segment.sweeps.quarter_chord          = 0 * Units.degrees
    segment.thickness_to_chord            = .0025
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)

    # # control surfaces -------------------------------------------

    # flap                          = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    # flap.tag                      = 'flap'
    # flap.span_fraction_start      = 0.3
    # flap.span_fraction_end        = 0.7
    # flap.deflection               = 0.0 * Units.degrees
    # flap.configuration_type       = 'plain'
    # flap.chord_fraction           = 0.14
    # wing.append_control_surface(flap)

    # aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    # aileron.tag                   = 'aileron'
    # aileron.span_fraction_start   = 0.7
    # aileron.span_fraction_end     = 0.963
    # aileron.deflection            = 0.0 * Units.degrees
    # aileron.chord_fraction        = 0.25
    # wing.append_control_surface(aileron)       

    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------ 
    wing                         = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag                     = 'horizontal_stabilizer' 
    wing.aspect_ratio            = 1.6018
    wing.sweeps.leading_edge     = 32.79 * Units.deg  
    wing.thickness_to_chord      = 0.05
    wing.taper                   = 0.2245
    wing.spans.projected         = 5.85863 * Units.meter # Note that this assumes the tail meets int he middle (but aerodynamically it won't. This will affect weight calcualtions)
    wing.chords.root             = 2.722 * Units.meter
    wing.chords.tip              = 0.6111 * Units.meter
    wing.chords.mean_aerodynamic = 2.05 * Units.meter
    wing.areas.reference         = 10.71349 * Units['meters**2']
    wing.areas.exposed           = 10 * Units['meters**2']    # Exposed area of the horizontal tail
    wing.areas.wetted            = 21.00 * Units['meters**2']     # Wetted area of the horizontal tail
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees 
    wing.origin                  = [[12.75, 1.311, 0.426]]
    wing.aerodynamic_center      = [14.0, 1.311, 0.426] 
    wing.vertical                = False
    wing.xz_plane_symmetric      = True 
    wing.dynamic_pressure_ratio  = 0.9


    tail_airfoil = RCAIDE.Library.Components.Airfoils.Airfoil() 
    tail_airfoil.coordinate_file = rel_path + 'Airfoils' + separator + 'supersonic_tail.txt'      

    # Wing Segments
    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'root_segment'
    segment.percent_span_location  = 0
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 1.0
    segment.dihedral_outboard      = 0.0 * Units.degrees
    segment.sweeps.leading_edge    = 0.0  * Units.degrees 
    segment.thickness_to_chord     = .05
    segment.append_airfoil(tail_airfoil)
    wing.append_segment(segment)

    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'segment_1'
    segment.percent_span_location  = 0.29
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 0.918
    segment.dihedral_outboard      = 0.0 * Units.degrees
    segment.sweeps.leading_edge    = 32.75  * Units.degrees 
    segment.thickness_to_chord     = .05
    segment.append_airfoil(tail_airfoil)
    wing.append_segment(segment)

    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'tip_segment'
    segment.percent_span_location  = 1.
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 0.2245              
    segment.dihedral_outboard      = 0 * Units.degrees
    segment.sweeps.leading_edge   = 32.75  * Units.degrees 
    segment.thickness_to_chord     = .05
    segment.append_airfoil(tail_airfoil)
    wing.append_segment(segment)
    
    # Fill out more segment properties automatically
    wing = segment_properties(wing)
    

    # # control surfaces -------------------------------------------
    # elevator                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator() # F-35 appears to have a stabilator instead of an elevator
    # elevator.tag                   = 'elevator'
    # elevator.span_fraction_start   = 0.09
    # elevator.span_fraction_end     = 0.92
    # elevator.deflection            = 0.0  * Units.deg
    # elevator.chord_fraction        = 0.7
    # wing.append_control_surface(elevator)     

    # add to vehicle
    vehicle.append_component(wing)    

    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------ 
    stabilizer_1 = RCAIDE.Library.Components.Wings.Vertical_Tail()
    stabilizer_1.tag                     = 'vertical_stabilizer' 
    stabilizer_1.aspect_ratio            = 1.1875
    stabilizer_1.sweeps.leading_edge     = 38.67  * Units.deg   
    stabilizer_1.thickness_to_chord      = 0.05
    stabilizer_1.taper                   = 0.66234 
    stabilizer_1.spans.projected         = 2.413 * Units.meter 
    stabilizer_1.total_length            = 2.413 * Units.meter  
    stabilizer_1.chords.root             = 2.444 * Units.meter
    stabilizer_1.chords.tip              = 1.61905 * Units.meter
    stabilizer_1.chords.mean_aerodynamic = 2.03 * Units.meter 
    stabilizer_1.areas.reference         = 9.803 * Units['meters**2']
    stabilizer_1.areas.wetted            = 10.29 * Units['meters**2']
    stabilizer_1.twists.root             = 0.0 * Units.degrees
    stabilizer_1.twists.tip              = 0.0 * Units.degrees 
    stabilizer_1.origin                  = [[11.686, 1.475, 0.426]]
    stabilizer_1.aerodynamic_center      = [12,1.5,1.0] 
    stabilizer_1.vertical                = False
    stabilizer_1.symmetric               = True
    stabilizer_1.t_tail                  = False 
    stabilizer_1.dynamic_pressure_ratio  = 1.0
    
    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root'
    segment.percent_span_location         = 0.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 1.
    segment.dihedral_outboard             = 60 * Units.degrees
    segment.sweeps.leading_edge           = 38.6792 * Units.degrees
    segment.thickness_to_chord            = 0.05 
    segment.append_airfoil(tail_airfoil)
    stabilizer_1.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_1'
    segment.percent_span_location         = 1.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = stabilizer_1.taper
    segment.dihedral_outboard             = 0. * Units.degrees
    segment.sweeps.leading_edge           = 38.6792 * Units.degrees   
    segment.thickness_to_chord            = 0.05  
    segment.append_airfoil(tail_airfoil)
    stabilizer_1.append_segment(segment)

    # # control surfaces -------------------------------------------
    # rudder                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    # rudder.tag                   = 'rudder'
    # rudder.span_fraction_start   = 0.1 
    # rudder.span_fraction_end     = 0.95 
    # rudder.deflection            = 0 
    # rudder.chord_fraction        = 0.33  
    # stabilizer_1.append_control_surface(rudder)    
    
    # # Fill out more segment properties automatically
    # stabilizer_1 = segment_properties(stabilizer_1)        

    # add to vehicle
    vehicle.append_component(stabilizer_1) 
    
    # ################################################# Fuselage ################################################################ 
        
    fuselage                                    = RCAIDE.Library.Components.Fuselages.Fuselage() 
    fuselage.number_coach_seats                 = vehicle.number_of_passengers 
    fuselage.seats_abreast                      = 0
    fuselage.seat_pitch                         = 0.0     * Units.meter 
    fuselage.fineness.nose                      = 2.
    fuselage.fineness.tail                      = 3.5 
    fuselage.lengths.nose                       = 3.3   * Units.meter
    fuselage.lengths.tail                       = 3.0   * Units.meter
    fuselage.lengths.total                      = 13.6 * Units.meter  
    fuselage.lengths.fore_space                 = 0.25    * Units.meter
    fuselage.lengths.aft_space                  = 1.0    * Units.meter
    fuselage.width                              = 1.75  * Units.meter
    fuselage.heights.maximum                    = 2.08  * Units.meter
    fuselage.effective_diameter                 = 1.467    * Units.meter
    fuselage.areas.side_projected               = fuselage.heights.maximum * fuselage.lengths.total * Units['meters**2'] 
    fuselage.areas.wetted                       = np.pi * fuselage.width/2 * fuselage.lengths.total * Units['meters**2'] 
    fuselage.areas.front_projected              = np.pi * fuselage.width/2      * Units['meters**2']  
    fuselage.differential_pressure              = 5.0e4 * Units.pascal
    fuselage.heights.at_quarter_length          = fuselage.heights.maximum * Units.meter
    fuselage.heights.at_three_quarters_length   = fuselage.heights.maximum * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = fuselage.heights.maximum* Units.meter
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_0'    
    segment.percent_x_location                  = 0.0000
    segment.percent_z_location                  = 0.00 
    segment.height                              = 0.000 
    segment.width                               = 0.000  
    fuselage.append_segment(segment)   
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_1'    
    segment.percent_x_location                  = 0.01652
    segment.percent_z_location                  = 0.00231 
    segment.height                              = 0.398
    segment.width                               = 0.31
    segment.curvature                           = 1.3
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.06
    segment.percent_z_location                  = 0.00672 
    segment.height                              = 0.849
    segment.width                               = 0.878
    segment.curvature                           = 1.3
    fuselage.append_segment(segment)      
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.10879
    segment.percent_z_location                  = 0.01089
    segment.height                              = 1.116 
    segment.width                               = 1.192 
    segment.curvature                           = 1.3
    fuselage.append_segment(segment)   

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.15519 	
    segment.percent_z_location                  = 0.01639 
    segment.height                              = 1.33878
    segment.width                               = 1.35204 
    segment.curvature                           = 1.3
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 0.20595 
    segment.percent_z_location                  = 0.03151 
    segment.height                              = 1.824 
    segment.width                               = 1.48469 
    segment.curvature                           = 1.3
    fuselage.append_segment(segment)     
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 0.24666 
    segment.percent_z_location                  = 0.03817 
    segment.height                              = 2.08
    segment.width                               = 1.63776 
    segment.curvature                           = 1.3
    fuselage.append_segment(segment)             
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.2959 
    segment.percent_z_location                  = 0.03746 
    segment.height                              = 2.08 
    segment.width                               = 1.75 
    segment.curvature                           = 1.3
    fuselage.append_segment(segment)  

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 0.3539 
    segment.percent_z_location                  = 0.03093 
    segment.height                              = 2.02 
    segment.width                               = 1.75 
    segment.curvature                           = 1.3
    fuselage.append_segment(segment)    
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_9'   
    segment.percent_x_location                  = 0.49722 
    segment.percent_z_location                  = 0.02613 
    segment.height                              = 1.936 
    segment.width                               = 1.75 
    segment.curvature                           = 1.3
    fuselage.append_segment(segment)    

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_10'   
    segment.percent_x_location                  = 0.6412 
    segment.percent_z_location                  = 0.02209 
    segment.height                              = 1.8 
    segment.width                               = 1.75 
    segment.curvature                           = 1.3
    fuselage.append_segment(segment)    

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_11'   
    segment.percent_x_location                  = 0.75 
    segment.percent_z_location                  = 0.01825
    segment.height                              = 1.8 
    segment.width                               = 1.75 
    segment.curvature                           = 1.3
    fuselage.append_segment(segment)    

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_12'   
    segment.percent_x_location                  = 0.875 
    segment.percent_z_location                  = 0.01825
    segment.height                              = 1.6 
    segment.width                               = 1.75 
    segment.curvature                           = 1.3
    fuselage.append_segment(segment)    

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_13'   
    segment.percent_x_location                  = 0.9421
    segment.percent_z_location                  = 0.01825
    segment.height                              = 1.5
    segment.width                               = 1.75 
    segment.curvature                           = 1.3
    fuselage.append_segment(segment)    

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_14'   
    segment.percent_x_location                  = 1.0
    segment.percent_z_location                  = 0.0 
    segment.height                              = 0
    segment.width                               = 0
    fuselage.append_segment(segment)   

    # add to vehicle
    vehicle.append_component(fuselage)

    # ################################################# Landing Gear #############################################################   
    # ------------------------------------------------------------------        
    #  Landing Gear
    # ------------------------------------------------------------------  
    main_gear               = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear()
    main_gear.tire_diameter = 24 * Units.inches
    main_gear.strut_length  = 4.0 * Units.ft 
    main_gear.units         = 2    # Number of main landing gear
    main_gear.wheels        = 1    # Number of wheels on the main landing gear
    vehicle.append_component(main_gear)  

    nose_gear               = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()       
    nose_gear.tire_diameter = 18. * Units.inches
    nose_gear.units         = 1    # Number of nose landing gear
    nose_gear.wheels        = 1    # Number of wheels on the nose landing gear
    nose_gear.strut_length  = 3.0 * Units.ft 
    vehicle.append_component(nose_gear)

    #------------------------------------------------------------------------------------------------------------------------------------
    # ########################################################## Energy Network ######################################################### 
    #------------------------------------------------------------------------------------------------------------------------------------ 
    #initialize the fuel network
    net                                            = RCAIDE.Framework.Networks.Fuel() 
    net.identical_propulsors                       = True 
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Fuel Distrubition Line 
    #------------------------------------------------------------------------------------------------------------------------------------  
    fuel_line                                     = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line() 

    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Propulsor: Starboard Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------         
    turbofan1                                    = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan()   
    turbofan1.origin                             = [[ 16.0 , 0.0 , 0.0 ]]
    turbofan1.tag                                = 'propulsor_1'    
    turbofan1.length                             = 5.59                     
    turbofan1.diameter                           = 1.17               
    turbofan1.bypass_ratio                       = 0.57                     
    turbofan1.design_altitude                    = 50000*Units.ft             
    turbofan1.design_mach_number                 = 1.6                    
    turbofan1.design_thrust                      = 3000. * Units.lbf  
    turbofan1.afterburner_active                 = False
    
    # working fluid                   
    turbofan1.working_fluid                      = RCAIDE.Library.Attributes.Gases.Air() 
    
    # Ram inlet 
    ram                                          = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                      = 'ram' 
    turbofan1.ram                                = ram 
          
    # inlet nozzle          
    inlet_nozzle                                 = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                             = 'inlet nozzle'
    inlet_nozzle.polytropic_efficiency           = 0.97                                       
    inlet_nozzle.pressure_ratio                  = 1
    inlet_nozzle.compressibility_effects         = False
    turbofan1.inlet_nozzle                       = inlet_nozzle
    
    # fan                
    fan                                          = RCAIDE.Library.Components.Powertrain.Converters.Fan()   
    fan.tag                                      = 'fan'
    fan.polytropic_efficiency                    = 0.91                   
    fan.pressure_ratio                           = 1.4                    
    turbofan1.fan                                = fan        

    # low pressure compressor    
    low_pressure_compressor                      = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    low_pressure_compressor.tag                  = 'lpc'
    low_pressure_compressor.polytropic_efficiency = 0.91                  
    low_pressure_compressor.pressure_ratio       = 1.3                     
    turbofan1.low_pressure_compressor            = low_pressure_compressor

    # high pressure compressor  
    high_pressure_compressor                     = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    high_pressure_compressor.tag                 = 'hpc'
    high_pressure_compressor.polytropic_efficiency = 0.91                        
    high_pressure_compressor.pressure_ratio      = 15.38             
    turbofan1.high_pressure_compressor           = high_pressure_compressor
    
    # combustor  
    combustor                                    = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                                = 'Comb'
    combustor.efficiency                         = 0.997                    
    combustor.turbine_inlet_temperature          = 1980               
    combustor.pressure_ratio                     = 0.94                     
    combustor.fuel_data                          = RCAIDE.Library.Attributes.Propellants.Jet_A()  
    turbofan1.combustor                          = combustor
    
    # high pressure turbine     
    high_pressure_turbine                        = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    high_pressure_turbine.tag                    ='hpt'
    high_pressure_turbine.mechanical_efficiency  = 0.99                     
    high_pressure_turbine.polytropic_efficiency  = 0.93                     
    turbofan1.high_pressure_turbine              = high_pressure_turbine 

    # low pressure turbine  
    low_pressure_turbine                         = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    low_pressure_turbine.tag                     ='lpt'
    low_pressure_turbine.mechanical_efficiency   = 0.99                     
    low_pressure_turbine.polytropic_efficiency   = 0.93                     
    turbofan1.low_pressure_turbine               = low_pressure_turbine
   
    # Afterburner  
    afterburner                                   = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    afterburner.tag                               = 'afterburner' 
    afterburner.efficiency                        = 0.9
    afterburner.alphac                            = 1.0     
    afterburner.turbine_inlet_temperature         = 1980 
    afterburner.pressure_ratio                    = 1.0
    afterburner.fuel_data                         = RCAIDE.Library.Attributes.Propellants.Jet_A()     
    turbofan1.afterburner                         = afterburner     

    # core nozzle
    core_nozzle                                    = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                = 'core_nozzle'
    core_nozzle.polytropic_efficiency              = 0.98                     # CHECKED Ref. [2] Page 9
    core_nozzle.pressure_ratio                     = 0.995
    core_nozzle.diameter                           = 1.1
    core_nozzle.pressure_recovery                  = 0.99
    turbofan1.core_nozzle                          = core_nozzle
     
    # # fan nozzle             
    fan_nozzle                                     = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    fan_nozzle.tag                                 = 'fan_nozzle'
    fan_nozzle.polytropic_efficiency               = 0.98                     # CHECKED Ref. [2] Page 9
    fan_nozzle.pressure_ratio                      = 0.995 
    fan_nozzle.diameter                            = 1.3
    turbofan1.fan_nozzle                           = fan_nozzle
    
    # # design turbofan
    design_turbofan(turbofan1)

    # Nacelle
    nacelle                                     = RCAIDE.Library.Components.Nacelles.Stack_Nacelle()
    nacelle.diameter                            = 1.05
    nacelle.tag                                 = 'nacelle_1'
    nacelle.origin                              = [[4.590,1.148,0.164]] 
    nacelle.length                              = 9.415
    nacelle.inlet_diameter                      = 2.0 # This is double the actual inlet area but this is done on purpose to account for the second inlet. 
    nacelle.areas.wetted                        = 20.0
    
    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_0' 
    nac_segment.orientation_euler_angles        = [-7.71*Units.degrees, 0.0*Units.degrees, 45.0*Units.degrees] 
    nac_segment.percent_x_location              = 0.0  
    nac_segment.percent_y_location              = 0.0
    nac_segment.percent_z_location              = -0.01383
    nac_segment.height                          = 1.03185  
    nac_segment.width                           = 1.5
    nac_segment.curvature                       = 4
    nacelle.append_segment(nac_segment)         

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_1'
    nac_segment.percent_x_location              = 0.20492
    nac_segment.percent_y_location              = 0.0082
    nac_segment.percent_z_location              = -0.0123
    nac_segment.height                          = 1.0
    nac_segment.width                           = 1.117
    nac_segment.curvature                       = 4
    nacelle.append_segment(nac_segment)      

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_2'
    nac_segment.percent_x_location              = 0.5
    nac_segment.percent_y_location              = 0.00713
    nac_segment.percent_z_location              = -0.01434
    nac_segment.height                          = 1.0
    nac_segment.width                           = 1.117
    nac_segment.curvature                       = 4
    nacelle.append_segment(nac_segment)   

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_3'
    nac_segment.percent_x_location              = 0.75
    nac_segment.percent_y_location              = 0.00217
    nac_segment.percent_z_location              = -0.0082
    nac_segment.height                          = 0.822
    nac_segment.width                           = 1.14286
    nac_segment.curvature                       = 4
    nacelle.append_segment(nac_segment)    

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_4'
    nac_segment.percent_x_location              = 1.0
    nac_segment.percent_y_location              = 0.00806
    nac_segment.percent_z_location              = 0.02459
    nac_segment.height                          = 0.0
    nac_segment.width                           = 0.0
    nac_segment.curvature                       = 4
    nacelle.append_segment(nac_segment)       

    turbofan1.nacelle                            = nacelle
    
    net.propulsors.append(turbofan1)

    # THe ghost propulsor, which produces no thrust, exists solely to provide a location for the nacelle mirror
    ghost_propulsor = deepcopy(turbofan1)
    ghost_propulsor.tag = 'ghost_propulsor'
    ghost_propulsor.design_altitude                    = 0*Units.ft             
    ghost_propulsor.design_mach_number                 = 0.5                    
    ghost_propulsor.design_thrust                      = 0. * Units.lbf  

    nacelle_mirror = deepcopy(nacelle)
    nacelle_mirror.tag = 'nacelle_mirror'
    nacelle_mirror.origin = [[4.590, -1.148, 0.164]] 
    nacelle_mirror.segments['segment_0'].percent_y_location = 0.0
    nacelle_mirror.segments['segment_0'].orientation_euler_angles        = [7.71*Units.degrees, 0.0*Units.degrees, -45.0*Units.degrees] 
    nacelle_mirror.segments['segment_1'].percent_y_location = -0.0082
    nacelle_mirror.segments['segment_2'].percent_y_location = -0.00713
    nacelle_mirror.segments['segment_3'].percent_y_location = -0.00217
    nacelle_mirror.segments['segment_4'].percent_y_location = -0.00806
    ghost_propulsor.nacelle = nacelle_mirror

    net.propulsors.append(ghost_propulsor)
    #------------------------------------------------------------------------------------------------------------------------- 
    #  Energy Source: Fuel Tank
    #------------------------------------------------------------------------------------------------------------------------- 
    # fuel tank
    fuel_tank                                        = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Wing_Fuel_Tank()
    fuel_tank.origin                                 = vehicle.wings.main_wing.origin  
    fuel_tank.fuel                              = RCAIDE.Library.Attributes.Propellants.Jet_A() 
    fuel_line.fuel_tanks.append(fuel_tank)

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to fuel line to network   
    #------------------------------------------------------------------------------------------------------------------------------------   
    fuel_line.assigned_propulsors =  [['propulsor_1']]

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Append fuel line to fuel line to network  
    #------------------------------------------------------------------------------------------------------------------------------------   
    net.fuel_lines.append(fuel_line)        

    #------------------------------------------------------------------------------------------------------------------------- 
    # Done ! 
    #-------------------------------------------------------------------------------------------------------------------------      

    # Append energy network to aircraft 
    vehicle.append_energy_network(net)   

    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------      
 
    return vehicle 

# ----------------------------------------------------------------------
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
    #   Initialize Configurations
    # ------------------------------------------------------------------ 
    config     = RCAIDE.Library.Components.Configs.Config(vehicle)
    config.tag = 'idle'      
    configs.append(config) 
    
    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------

    config     = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'cruise'
    configs.append(config)

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------

    config     = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'no_after_burner_cruise'
    config.networks.fuel.propulsors['propulsor_1'].afterburner.active      = False
    config.networks.fuel.propulsors['propulsor_2'].afterburner.active      = False
    configs.append(config)
     
    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------ 
    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'descent' 
    configs.append(config) 

    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'takeoff'     
    config.V2_VS_ratio = 1.21
    configs.append(config)

    # ------------------------------------------------------------------
    #   No Afterburner Takeoff Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'no_afterburner_takeoff'     
    config.networks.fuel.propulsors['propulsor_1'].afterburner.active      = False
    config.networks.fuel.propulsors['propulsor_2'].afterburner.active      = False
    config.V2_VS_ratio = 1.21
    configs.append(config)

    # ------------------------------------------------------------------
    #   Cutback Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'cutback'
    configs.append(config)   

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'landing'
    # config.wings['main_wing'].control_surfaces.flap.deflection  = 30. * Units.deg
    config.landing_gears.main_gear.gear_extended    = True
    config.landing_gears.nose_gear.gear_extended    = True  
    config.Vref_VS_ratio = 1.23
    configs.append(config)   

    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------ 

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'short_field_takeoff'    
    # config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    # config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg
    config.landing_gears.main_gear.gear_extended    = True
    config.landing_gears.nose_gear.gear_extended    = True  
    config.V2_VS_ratio = 1.21 
    configs.append(config)

    return configs

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

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
    analyses.vehicle = vehicle

    # ------------------------------------------------------------------
    #  Weights
    # ------------------------------------------------------------------
    # weights = RCAIDE.Framework.Analyses.Weights.Conventional()
    # weights.vehicle = vehicle
    # analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.settings.number_of_spanwise_vortices   = 40
    aerodynamics.settings.number_of_chordwise_vortices  = 2  
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    energy = RCAIDE.Framework.Analyses.Energy.Energy() 
    analyses.append(energy)
    

    # ------------------------------------------------------------------
    # Emissions 
    emissions = RCAIDE.Framework.Analyses.Emissions.Emission_Index_Correlation_Method()                      
    analyses.append(emissions) 

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)   

    return analyses    


# ----------------------------------------------------------------------
#   Define the Missions
# ----------------------------------------------------------------------

def long_range_mission_setup(analyses):
    """This function defines the baseline mission that will be flown by the aircraft in order
    to compute performance."""

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'long_range_mission'
  
    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Takeoff Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Takeoff(base_segment)
    segment.tag = "Takeoff" 
    segment.analyses.extend( analyses.no_afterburner_takeoff )
    segment.velocity_start           = 10.* Units.knots
    segment.velocity_end             = 150.0 * Units['mph']
    segment.friction_coefficient     = 0.04
    segment.altitude                 = 0.0   
    segment.true_course              = 90.0 * Units.deg
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1" 
    segment.analyses.extend( analyses.no_afterburner_takeoff ) 
    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 10000.0   * Units.ft
    segment.air_speed      = 250.0 * Units['mph']
    segment.climb_rate     = 3000.0   * Units['fpm']  
    segment.true_course              = 45.0 * Units.deg
     
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

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_2" 
    segment.analyses.extend( analyses.no_after_burner_cruise ) 
    segment.altitude_end   = 30000.0   * Units.ft
    segment.air_speed      = 300.0 * Units['mph']
    segment.climb_rate     = 1500.0   * Units['fpm']  
    segment.true_course              = 0.0 * Units.deg

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  
    
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Third Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_3" 
    segment.analyses.extend( analyses.no_after_burner_cruise ) 
    segment.altitude_end = 30000.0   * Units.ft
    segment.air_speed    = 300.0  * Units['mph']
    segment.climb_rate   = 1000.0    * Units['fpm']  
    segment.true_course              = 0.0 * Units.deg
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)


    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Mach_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.no_after_burner_cruise) 
    segment.altitude                                      = 30000.0   * Units.ft  
    segment.mach_number                                   = 0.65
    segment.distance                                      = 1200 * Units.nmi   
    segment.true_course                                   = 0.0 * Units.deg
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_1" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_start                                = 30000.0   * Units.ft 
    segment.altitude_end                                  = 20000.0   * Units.ft
    segment.air_speed                                     = 300.0 * Units['mph']
    segment.descent_rate                                  = 1500.0   * Units['fpm']  
    segment.true_course                                   = 0.0 * Units.deg
    
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
    segment.tag  = "descent_2" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                  = 10000.0   * Units.ft
    segment.air_speed                                     = 300.0 * Units['mph']
    segment.descent_rate                                  = 2000.0   * Units['fpm']  
    segment.true_course                                   = 0.0 * Units.deg
    
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
    segment.tag = "descent_3"  
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                  = 3000.0   * Units.ft
    segment.air_speed                                     = 250.0 * Units['mph']
    segment.descent_rate                                  = 3000.0   * Units['fpm']
    segment.true_course              = 0.0 * Units.deg  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Fourth Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_4" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                  = 500.0   * Units.ft
    segment.air_speed                                     = 200.0 * Units['mph']
    segment.descent_rate                                  = 1000.0   * Units['fpm']
    segment.true_course              = 45.0 * Units.deg  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Fifth Descent Segment:Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_5" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                  = 0.0   * Units.ft
    segment.air_speed                                     = 200.0 * Units['mph']
    segment.descent_rate                                  = 500.0   * Units['fpm']  
    segment.true_course              = 90.0 * Units.deg  
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Landing Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Landing(base_segment)
    segment.tag = "Landing"

    segment.analyses.extend( analyses.landing ) 
    segment.velocity_end                                  = 10 * Units.knots 
    segment.friction_coefficient                          = 0.4
    segment.altitude                                      = 0.0   
    segment.true_course              = 90.0 * Units.deg  
    segment.assigned_control_variables.elapsed_time.active           = True  
    segment.assigned_control_variables.elapsed_time.initial_guess_values   = [[30.]]  
    mission.append_segment(segment)     

    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------

    return mission

def escort_mission_setup(analyses):
    """This function defines the baseline mission that will be flown by the aircraft in order
    to compute performance."""

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'escort_mission'
  
    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Takeoff Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Takeoff(base_segment)
    segment.tag = "Takeoff" 
    segment.analyses.extend( analyses.takeoff )
    segment.velocity_start           = 10.* Units.knots
    segment.velocity_end             = 150.0 * Units['mph']
    segment.friction_coefficient     = 0.04
    segment.altitude                 = 0.0   
    segment.true_course              = 0.0 * Units.deg
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1" 
    segment.analyses.extend( analyses.takeoff ) 
    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 10000.0   * Units.ft
    segment.air_speed      = 250.0 * Units['mph']
    segment.climb_rate     = 3000.0   * Units['fpm']  
    segment.true_course              = 0.0 * Units.deg
     
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

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_2" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end   = 30000.0   * Units.ft
    segment.air_speed      = 400.0 * Units['mph']
    segment.climb_rate     = 1500.0   * Units['fpm']  
    segment.true_course              = 0.0 * Units.deg

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  
    
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Third Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_3" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end = 40000.0   * Units.ft
    segment.air_speed    = 500.0  * Units['mph']
    segment.climb_rate   = 1000.0    * Units['fpm']  
    segment.true_course              = 0.0 * Units.deg
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)


    # ------------------------------------------------------------------    
    #  First Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Mach_Constant_Altitude(base_segment)
    segment.tag = "cruise_1" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude                                                 = 40000 * Units['ft']  
    segment.mach_number                                              = 2.25 # Check L/D is 8.4 around M=1, M=1.5 is 4, M=2.25 is 2
    segment.distance                                                 = 1000 * Units.km   
    segment.true_course                                              = 0.0 * Units.deg
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)

    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Circular departure pattern 
    #------------------------------------------------------------------------------------------------------------------------------------ 
    segment                                               = Segments.Cruise.Curved_Constant_Radius_Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                                           = "Curve"    
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude    = 40000. * Units.ft
    segment.air_speed   = 400 * Units.kts 
    segment.turn_radius = 3600 * Units.feet  
    segment.true_course = 0 * Units.degree # this is the true couse of the starting value     
    segment.turn_angle  = 180 * Units.degree
    
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True  

    # ------------------------------------------------------------------    
    #  Second Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Mach_Constant_Altitude(base_segment)
    segment.tag = "cruise_2" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude                                                 = 40000 * Units['ft']  
    segment.mach_number                                              = 2.25 # Check L/D is 8.4 around M=1, M=1.5 is 4, M=2.25 is 2
    segment.distance                                                 = 1000 * Units.km   
    segment.true_course                                              = 180.0 * Units.deg
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_1" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_start                                = 40000.0   * Units.ft 
    segment.altitude_end                                  = 30000.0   * Units.ft
    segment.air_speed                                     = 400.0 * Units['mph']
    segment.descent_rate                                  = 1500.0   * Units['fpm']  
    segment.true_course              = 180.0 * Units.deg
    
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
    segment.tag  = "descent_2" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                  = 10000.0   * Units.ft
    segment.air_speed                                     = 300.0 * Units['mph']
    segment.descent_rate                                  = 2000.0   * Units['fpm']  
    segment.true_course              = 180.0 * Units.deg
    
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
    segment.tag = "descent_3"  
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                  = 3000.0   * Units.ft
    segment.air_speed                                     = 250.0 * Units['mph']
    segment.descent_rate                                  = 3000.0   * Units['fpm']
    segment.true_course              = 180.0 * Units.deg  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Fourth Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_4" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                  = 500.0   * Units.ft
    segment.air_speed                                     = 200.0 * Units['mph']
    segment.descent_rate                                  = 1000.0   * Units['fpm']
    segment.true_course              = 180.0 * Units.deg  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)



    # ------------------------------------------------------------------
    #   Fifth Descent Segment:Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_5" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                  = 0.0   * Units.ft
    segment.air_speed                                     = 200.0 * Units['mph']
    segment.descent_rate                                  = 500.0   * Units['fpm']  
    segment.true_course              = 180.0 * Units.deg  
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Landing Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Landing(base_segment)
    segment.tag = "Landing"

    segment.analyses.extend( analyses.landing ) 
    segment.velocity_end                                  = 10 * Units.knots 
    segment.friction_coefficient                          = 0.4
    segment.altitude                                      = 0.0   
    segment.true_course              = 180.0 * Units.deg  
    segment.assigned_control_variables.elapsed_time.active           = True  
    segment.assigned_control_variables.elapsed_time.initial_guess_values   = [[30.]]  
    mission.append_segment(segment)     

    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------

    return mission

def dogfight_mission_setup(analyses):
    """This function defines the baseline mission that will be flown by the aircraft in order
    to compute performance."""

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'dogfight_mission'
  
    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Takeoff Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Takeoff(base_segment)
    segment.tag = "Takeoff" 
    segment.analyses.extend( analyses.takeoff )
    segment.velocity_start           = 10.* Units.knots
    segment.velocity_end             = 150.0 * Units['mph']
    segment.friction_coefficient     = 0.04
    segment.altitude                 = 0.0   
    segment.true_course              = 90.0 * Units.deg
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1" 
    segment.analyses.extend( analyses.takeoff ) 
    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 10000.0   * Units.ft
    segment.air_speed      = 250.0 * Units['mph']
    segment.climb_rate     = 3000.0   * Units['fpm']  
    segment.true_course              = 45.0 * Units.deg
     
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

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_2" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end   = 30000.0   * Units.ft
    segment.air_speed      = 400.0 * Units['mph']
    segment.climb_rate     = 1500.0   * Units['fpm']  
    segment.true_course              = 0.0 * Units.deg

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  
    
    mission.append_segment(segment)

    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Circular segment
    #------------------------------------------------------------------------------------------------------------------------------------ 
    segment                                               = Segments.Cruise.Curved_Constant_Radius_Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                                           = "Curve_1"    
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude    = 30000. * Units.ft
    segment.air_speed   = 400 * Units.kts 
    segment.turn_radius = 3600 * Units.feet  
    segment.true_course = 0 * Units.degree # this is the true couse of the starting value     
    segment.turn_angle  = 180 * Units.degree
    
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True  

    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Circular segment
    #------------------------------------------------------------------------------------------------------------------------------------ 
    segment                                               = Segments.Cruise.Curved_Constant_Radius_Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                                           = "Curve_2"    
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude    = 30000. * Units.ft
    segment.air_speed   = 350 * Units.kts 
    segment.turn_radius = 1800 * Units.feet  
    segment.true_course = 180 * Units.degree # this is the true couse of the starting value     
    segment.turn_angle  = 180 * Units.degree
    
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True  

    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Circular segment
    #------------------------------------------------------------------------------------------------------------------------------------ 
    segment                                               = Segments.Cruise.Curved_Constant_Radius_Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                                           = "Curve_3"    
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude    = 30000. * Units.ft
    segment.air_speed   = 450 * Units.kts 
    segment.turn_radius = 4000 * Units.feet  
    segment.true_course = 0 * Units.degree # this is the true couse of the starting value     
    segment.turn_angle  = 180 * Units.degree
    
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True  

    # ------------------------------------------------------------------    
    #  Second Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Mach_Constant_Altitude(base_segment)
    segment.tag = "cruise_2" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude                                                 = 30000 * Units['ft']  
    segment.mach_number                                              = 2.25 # Check L/D is 8.4 around M=1, M=1.5 is 4, M=2.25 is 2
    segment.distance                                                 = 100 * Units.km   
    segment.true_course                                              = 180.0 * Units.deg
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_1" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_start                                = 40000.0   * Units.ft 
    segment.altitude_end                                  = 10000.0   * Units.ft
    segment.air_speed                                     = 400.0 * Units['mph']
    segment.descent_rate                                  = 1500.0   * Units['fpm']  
    segment.true_course              = 180.0 * Units.deg
    
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
    segment.tag = "descent_2"  
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                  = 3000.0   * Units.ft
    segment.air_speed                                     = 250.0 * Units['mph']
    segment.descent_rate                                  = 3000.0   * Units['fpm']
    segment.true_course              = 180.0 * Units.deg  
    
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
    segment.tag = "descent_3" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                  = 500.0   * Units.ft
    segment.air_speed                                     = 200.0 * Units['mph']
    segment.descent_rate                                  = 1000.0   * Units['fpm']
    segment.true_course              = 180.0 * Units.deg  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)



    # ------------------------------------------------------------------
    #   Fourth Descent Segment:Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_4" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                  = 0.0   * Units.ft
    segment.air_speed                                     = 200.0 * Units['mph']
    segment.descent_rate                                  = 500.0   * Units['fpm']  
    segment.true_course              = 180.0 * Units.deg  
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Landing Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Landing(base_segment)
    segment.tag = "Landing"

    segment.analyses.extend( analyses.landing ) 
    segment.velocity_end                                  = 10 * Units.knots 
    segment.friction_coefficient                          = 0.4
    segment.altitude                                      = 0.0   
    segment.true_course              = 180.0 * Units.deg  
    segment.assigned_control_variables.elapsed_time.active           = True  
    segment.assigned_control_variables.elapsed_time.initial_guess_values   = [[30.]]  
    mission.append_segment(segment)     

    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------

    return mission

def missions_setup(long_range_mission, escort_mission, dogfight_mission):
    """This allows multiple missions to be incorporated if desired, but only one is used here."""

    missions     = RCAIDE.Framework.Mission.Missions()

    escort_mission.tag  = 'escort_mission'
    missions.append(escort_mission)

    dogfight_mission.tag  = 'dogfight_mission'
    missions.append(dogfight_mission)

    long_range_mission.tag  = 'long_range_mission'
    missions.append(long_range_mission)

    return missions

def save_aircraft_geometry(geometry,filename,save_dir): 

    # Create full path for pickle file
    pickle_file = os.path.join(save_dir, filename + '.pkl')

    # Save the file
    with open(pickle_file, 'wb') as file:
        pickle.dump(geometry, file) 
    return


def load_aircraft_geometry(filename,load_dir):
    # Create full path for pickle file
    pickle_file = os.path.join(load_dir, filename +'.pkl')

    # Load the file
    with open(pickle_file, 'rb') as file:
        results = pickle.load(file)
    return results

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------
def plot_mission(results):
    """This function plots the results of the mission analysis and saves those results to 
    png files."""

    # Plot Flight Conditions 
    plot_flight_conditions(results)

    # Plot Aerodynamic Forces 
    plot_aerodynamic_forces(results)

    # Plot Aerodynamic Coefficients 
    plot_aerodynamic_coefficients(results)     

    # Drag Components
    plot_drag_components(results)

    # Plot Altitude, sfc, vehicle weight 
    plot_altitude_sfc_weight(results)

    # Plot Velocities 
    plot_aircraft_velocities(results)  

    plot_propulsor_throttles(results)

    plot_longitudinal_stability(results)
    
    #plot_emissions(results)

    plot_jet_engine_conditions(results)

    return

def plot_jet_engine_conditions(results,
                             save_figure = False,
                             show_legend = True,
                             save_filename = "Jet_Engine_Velocities" ,
                             file_type = ".png",
                             width = 11, height = 7):
 
    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)
     
    # get line colors for plots 
    line_colors   = cm.inferno(np.linspace(0,0.9,len(results.segments)))      
     
    fig   = plt.figure(save_filename)
    fig.set_size_inches(width,height)
    
    for i in range(len(results.segments)): 
        time     = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min  
        segment_tag  =  results.segments[i].tag
        segment_name = segment_tag.replace('_', ' ') 
        
        # power 
        axis_1 = plt.subplot(1,1,1)
        axis_1.set_ylabel(r'Jet Engine Velocity')
        set_axes(axis_1)               
        for network in results.segments[i].analyses.vehicle.networks: 
            for j ,  propulsor in enumerate(network.propulsors):
                core_nozzle = propulsor.core_nozzle
                core_velocity = results.segments[i].conditions.energy.converters[core_nozzle.tag].outputs.velocity[:,0]
                fan_nozzle = propulsor.fan_nozzle
                fan_velocity = results.segments[i].conditions.energy.converters[fan_nozzle.tag].outputs.velocity[:,0]
                if j == 0 and i ==0:               
                    axis_1.plot(time, core_velocity, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width, label = 'Core Nozzle')     
                    axis_1.plot(time, fan_velocity, color = line_colors[i], marker = ps.markers[1], linewidth = ps.line_width, label = 'Fan Nozzle')     
                else:
                    axis_1.plot(time, core_velocity, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width)     
                    axis_1.plot(time, fan_velocity, color = line_colors[i], marker = ps.markers[1], linewidth = ps.line_width)     
    
    if show_legend:
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4) 
        leg.set_title('Propulsor', prop={'size': ps.legend_font_size, 'weight': 'heavy'})    
    
    # Adjusting the sub-plots for legend 
    fig.tight_layout()
    fig.subplots_adjust(top=0.8)
    
    # set title of plot 
    title_text    = 'Exit Velocity'      
    fig.suptitle(title_text)
    
    if save_figure:
        plt.savefig(save_filename + file_type)   


    return fig 



if __name__ == '__main__': 
    main()    
    plt.show()
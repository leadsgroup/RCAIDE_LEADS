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
   
    # Step 1: design a vehicle
    drop_tank = True
    vehicle   = vehicle_setup(drop_tank)

    # Step 2: plot vehicle 
    plot_3d_vehicle(vehicle)  

    return

def vehicle_setup(drop_tank): 
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------      
    vehicle = RCAIDE.Vehicle()
   
    # ################################################# Vehicle-level Properties #################################################   
    vehicle.tag                                       = 'Lockheed_F22' 
    vehicle.mass_properties.max_takeoff               = 38000. * Units.kilogram 
    vehicle.mass_properties.max_zero_fuel             = 19700 * Units.kilogram  
    vehicle.mass_properties.max_fuel                  = 11900  
    vehicle.mass_properties.fuel                      =  11900 
    vehicle.mass_properties.takeoff                   = 38000. * Units.kilogram   
    vehicle.mass_properties.max_payload               = 13608 * Units.kilogram
    vehicle.mass_properties.payload                   = 10000 *Units.kilogram  
    vehicle.mass_properties.center_of_gravity         = [[8.0, 0, 0]]
    vehicle.flight_envelope.ultimate_load             = 13.5
    vehicle.flight_envelope.positive_limit_load       = 9 
    vehicle.flight_envelope.negative_limit_load       =-3
    vehicle.flight_envelope.design_mach_number        = 2
    vehicle.flight_envelope.design_cruise_altitude    = 50000.0*Units.feet 
    vehicle.flight_envelope.design_range              = 1200.0 * Units.nmi
    vehicle.reference_area                            = 85.64018 * Units['meters**2']  
    vehicle.number_of_passengers                      = 0 
    vehicle.systems.control                           = "fully powered" 
    vehicle.systems.accessories                       = "long range"
    vehicle.Drop_Tank                                 = drop_tank           

    # ------------------------------------------------------------------
    #   Main Wing 
    # ------------------------------------------------------------------

    wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing'
    wing.aspect_ratio                     = 2.2535
    wing.sweeps.quarter_chord             = 45 * Units.deg
    wing.thickness_to_chord               = 0.025
    wing.spans.projected                  = 13.89208 * Units.meter
    wing.chords.root                      = 10.34 * Units.meter
    wing.chords.tip                       = 1.222 * Units.meter
    wing.taper                            = wing.chords.tip / wing.chords.root
    wing.chords.mean_aerodynamic          = 6.907 * Units.meter 
    wing.areas.reference                  = 85.64018 * Units['meters**2']
    wing.areas.wetted                     = 137.0 * Units['meters**2'] # assumed 180% wetted area compared to reference area
    wing.twists.root                      = 0.0 * Units.degrees 
    wing.twists.tip                       = -3.0 * Units.degrees 
    wing.origin                           = [[4.631,0,0.368]]
    wing.aerodynamic_center               = [10.5,0,0.368] 
    wing.vertical                         = False
    wing.dihedral                         = -3.0 * Units.degrees 
    wing.xz_plane_symmetric               = True 
    wing.dynamic_pressure_ratio           = 1.0
    
    wing_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    wing_airfoil.coordinate_file          = 'NACA65_203.txt' 

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
    segment.percent_span_location         = 0.1239
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.leading_edge           = 40 * Units.degrees
    segment.thickness_to_chord            = .0025
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Segment_2'
    segment.percent_span_location         = 0.27689
    segment.root_chord_percent            = 0.91858 
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.leading_edge           = 84.1 * Units.degrees
    segment.thickness_to_chord            = .005
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Segment_3'
    segment.percent_span_location         = 0.31975
    segment.root_chord_percent            = 0.645
    segment.dihedral_outboard             = -3.0 * Units.degrees
    segment.sweeps.leading_edge           = 41.2 * Units.degrees
    segment.thickness_to_chord            = .005
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Segment_4'
    segment.percent_span_location         = 0.42289
    segment.root_chord_percent            = 0.6305 
    segment.dihedral_outboard             = -3.0 * Units.degrees
    segment.sweeps.leading_edge           = 41.2 * Units.degrees
    segment.thickness_to_chord            = .005
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Segment_5'
    segment.percent_span_location         = 0.93135
    segment.root_chord_percent            = 0.254
    segment.dihedral_outboard             = -3.0 * Units.degrees
    segment.sweeps.leading_edge           = 41.2 * Units.degrees
    segment.thickness_to_chord            = .005
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)
    
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Tip'
    segment.percent_span_location         = 1.0
    segment.root_chord_percent            = 0.1182
    segment.dihedral_outboard             = 0 * Units.degrees
    segment.sweeps.quarter_chord          = 0 * Units.degrees
    segment.thickness_to_chord            = .005
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)

    # control surfaces -------------------------------------------
    slat                          = RCAIDE.Library.Components.Wings.Control_Surfaces.Slat()
    slat.tag                      = 'slat'
    slat.span_fraction_start      = 0.2
    slat.span_fraction_end        = 0.963
    slat.deflection               = 0.0 * Units.degrees
    slat.chord_fraction           = 0.075
    wing.append_control_surface(slat)

    flap                          = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    flap.tag                      = 'flap'
    flap.span_fraction_start      = 0.4
    flap.span_fraction_end        = 0.7
    flap.deflection               = 0.0 * Units.degrees
    flap.configuration_type       = 'plain'
    flap.chord_fraction           = 0.14
    wing.append_control_surface(flap)

    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.7
    aileron.span_fraction_end     = 0.963
    aileron.deflection            = 0.0 * Units.degrees
    aileron.chord_fraction        = 0.25
    wing.append_control_surface(aileron)       

    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------ 
    wing                         = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag                     = 'horizontal_stabilizer' 
    wing.aspect_ratio            = 0.7332
    wing.sweeps.leading_edge     = 45.0 * Units.deg  
    wing.thickness_to_chord      = 0.05
    wing.taper                   = 0.349
    wing.spans.projected         = 6.25951 * Units.meter 
    wing.chords.root             = 2.601 * Units.meter
    wing.chords.tip              = 1.18 * Units.meter
    wing.chords.mean_aerodynamic = 2.81569 * Units.meter
    wing.areas.reference         = 16.61187 * Units['meters**2']
    wing.areas.exposed           = 15.00 * Units['meters**2']    
    wing.areas.wetted            = 35.00 * Units['meters**2']    
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees 
    wing.origin                  = [[15.00, 1.424, 0.49]]
    wing.aerodynamic_center      = [16.3,0,0.491] 
    wing.vertical                = False
    wing.xz_plane_symmetric      = True 
    wing.dynamic_pressure_ratio  = 0.9

    tail_airfoil = RCAIDE.Library.Components.Airfoils.Airfoil() 
    tail_airfoil.coordinate_file = 'supersonic_tail.txt'      

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
    segment.percent_span_location  = 0.233
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 1.223
    segment.dihedral_outboard      = 0.0 * Units.degrees
    segment.sweeps.leading_edge    = 35.0* Units.degrees 
    segment.thickness_to_chord     = .05
    segment.append_airfoil(tail_airfoil)
    wing.append_segment(segment)

    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'segment_2'
    segment.percent_span_location  = 0.4668
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 1.297
    segment.dihedral_outboard      = 0.0  * Units.degrees
    segment.sweeps.leading_edge    = 45.0 * Units.degrees 
    segment.thickness_to_chord     = .2
    segment.append_airfoil(tail_airfoil)
    wing.append_segment(segment)

    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'tip_segment'
    segment.percent_span_location  = 1.
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 0.5926              
    segment.dihedral_outboard      = 0 * Units.degrees
    segment.sweeps.leading_edge    = 45.0  * Units.degrees 
    segment.thickness_to_chord     = .05
    segment.append_airfoil(tail_airfoil)
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
    stabilizer_1 = RCAIDE.Library.Components.Wings.Vertical_Tail()
    stabilizer_1.tag                     = 'vertical_stabilizer' 
    stabilizer_1.aspect_ratio            = 1.14
    stabilizer_1.sweeps.leading_edge     = 22.92  * Units.deg   
    stabilizer_1.thickness_to_chord      = 0.05
    stabilizer_1.taper                   = 0.318 
    stabilizer_1.spans.projected         = 2.67   * Units.meter 
    stabilizer_1.total_length            = 3.09   * Units.meter  
    stabilizer_1.chords.root             = 4.1111 * Units.meter
    stabilizer_1.chords.tip              = 1.3095 * Units.meter
    stabilizer_1.chords.mean_aerodynamic = 2.7103 * Units.meter 
    stabilizer_1.areas.reference         = 8.37   * Units['meters**2']
    stabilizer_1.areas.wetted            = 18.00  * Units['meters**2']
    stabilizer_1.twists.root             = 0.0 * Units.degrees
    stabilizer_1.twists.tip              = 0.0 * Units.degrees 
    stabilizer_1.origin                  = [[13.196, 1.45, 0.543]]
    stabilizer_1.aerodynamic_center      = [0,0,0] 
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
    segment.sweeps.leading_edge           = 22.92 * Units.degrees
    segment.thickness_to_chord            = 0.05 
    segment.append_airfoil(tail_airfoil)
    stabilizer_1.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_1'
    segment.percent_span_location         = 1.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = stabilizer_1.taper
    segment.dihedral_outboard             = 0. * Units.degrees
    segment.sweeps.leading_edge           = 22.92 * Units.degrees   
    segment.thickness_to_chord            = 0.05  
    segment.append_airfoil(tail_airfoil)
    stabilizer_1.append_segment(segment)

    # control surfaces -------------------------------------------
    rudder                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    rudder.tag                   = 'rudder'
    rudder.span_fraction_start   = 0.1 
    rudder.span_fraction_end     = 0.95 
    rudder.deflection            = 0 
    rudder.chord_fraction        = 0.33  
    stabilizer_1.append_control_surface(rudder)    
    
    # Fill out more segment properties automatically
    stabilizer_1 = segment_properties(stabilizer_1)        

    # add to vehicle
    vehicle.append_component(stabilizer_1) 
    
    # ################################################# Fuselage ################################################################ 
        
    fuselage                                    = RCAIDE.Library.Components.Fuselages.Fuselage() 
    fuselage.number_coach_seats                 = vehicle.number_of_passengers 
    fuselage.seats_abreast                      = 0
    fuselage.seat_pitch                         = 0.0    * Units.meter 
    fuselage.fineness.nose                      = 2.
    fuselage.fineness.tail                      = 3.5 
    fuselage.lengths.nose                       = 3.5   * Units.meter
    fuselage.lengths.tail                       = 3.0   * Units.meter
    fuselage.lengths.total                      = 17.52 * Units.meter  
    fuselage.lengths.fore_space                 = 0.25  * Units.meter
    fuselage.lengths.aft_space                  = 2.    * Units.meter
    fuselage.width                              = 1.73  * Units.meter
    fuselage.heights.maximum                    = 2.21  * Units.meter
    fuselage.effective_diameter                 = 2.0   * Units.meter
    fuselage.areas.side_projected               = fuselage.heights.maximum * fuselage.lengths.total * Units['meters**2'] 
    fuselage.areas.wetted                       = np.pi * fuselage.width/2 * fuselage.lengths.total * Units['meters**2'] * 1
    fuselage.areas.front_projected              = np.pi * fuselage.width/2 * Units['meters**2']  
    fuselage.differential_pressure              = 5.0e4 * Units.pascal
    fuselage.heights.at_quarter_length          = fuselage.heights.maximum * Units.meter
    fuselage.heights.at_three_quarters_length   = fuselage.heights.maximum * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = fuselage.heights.maximum * Units.meter
    
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
    segment.percent_x_location                  = 0.03074
    segment.percent_z_location                  = 0.00154 
    segment.height                              = 0.57
    segment.width                               = 0.61
    segment.curvature                           = 1
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.10863
    segment.percent_z_location                  = 0.00410 
    segment.height                              = 1.18
    segment.width                               = 1.408
    segment.curvature                           = 1
    fuselage.append_segment(segment)      
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.14432
    segment.percent_z_location                  = 0.0082 
    segment.height                              = 1.491 
    segment.width                               = 1.50 
    segment.curvature                           = 1
    fuselage.append_segment(segment)   

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.2243 	
    segment.percent_z_location                  = 0.0264 
    segment.height                              = 2.216
    segment.width                               = 1.73 
    segment.curvature                           = 1
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 0.2624 
    segment.percent_z_location                  = 0.02664 
    segment.height                              = 2.21 
    segment.width                               = 1.74 
    segment.curvature                           = 1
    fuselage.append_segment(segment)     
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 0.43226 
    segment.percent_z_location                  = 0.01230 
    segment.height                              = 1.807
    segment.width                               = 1.549 
    segment.curvature                           = 1
    fuselage.append_segment(segment)             
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.5559 
    segment.percent_z_location                  = 0.00512 
    segment.height                              = 1.27786 
    segment.width                               = 1.549 
    segment.curvature                           = 3
    fuselage.append_segment(segment)  

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 0.75 
    segment.percent_z_location                  = 0.00512 
    segment.height                              = 0.7875 
    segment.width                               = 1.126 
    segment.curvature                           = 3
    fuselage.append_segment(segment)    
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_9'   
    segment.percent_x_location                  = 1.0
    segment.percent_z_location                  = 0.0 
    segment.height                              = 0
    segment.width                               = 0
    fuselage.append_segment(segment)   

    # add to vehicle
    vehicle.append_component(fuselage)

    if vehicle.Drop_Tank:
        # ################################################# Drop Tank ################################################################ 
            
        Drop_Tank_Port                                    = RCAIDE.Library.Components.Fuselages.Fuselage() 
        Drop_Tank_Port.tag                                = 'tank_port'
        Drop_Tank_Port.number_coach_seats                 = vehicle.number_of_passengers 
        Drop_Tank_Port.origin                             = [[6.8, -3.2, -0.41]]
        Drop_Tank_Port.seats_abreast                      = 0
        Drop_Tank_Port.seat_pitch                         = 0.0     * Units.meter 
        Drop_Tank_Port.fineness.nose                      = 3.
        Drop_Tank_Port.fineness.tail                      = 1.0 
        Drop_Tank_Port.lengths.nose                       = 3.5   * Units.meter
        Drop_Tank_Port.lengths.tail                       = 1.0   * Units.meter
        Drop_Tank_Port.lengths.total                      = 6.0 * Units.meter  
        Drop_Tank_Port.width                              = 1.0  * Units.meter
        Drop_Tank_Port.heights.maximum                    = 1.0  * Units.meter
        Drop_Tank_Port.effective_diameter                 = 1.0    * Units.meter
        Drop_Tank_Port.areas.side_projected               = Drop_Tank_Port.heights.maximum * Drop_Tank_Port.lengths.total * Units['meters**2'] 
        Drop_Tank_Port.areas.wetted                       = np.pi * Drop_Tank_Port.width/2 * Drop_Tank_Port.lengths.total * Units['meters**2']
        Drop_Tank_Port.areas.front_projected              = np.pi * (Drop_Tank_Port.width/2)**2      * Units['meters**2']  
        Drop_Tank_Port.differential_pressure              = 0
        Drop_Tank_Port.heights.at_quarter_length          = Drop_Tank_Port.heights.maximum * Units.meter
        Drop_Tank_Port.heights.at_three_quarters_length   = Drop_Tank_Port.heights.maximum * Units.meter
        Drop_Tank_Port.heights.at_wing_root_quarter_chord = Drop_Tank_Port.heights.maximum* Units.meter
        Drop_Tank_Port.differential_pressure              = 0
        
        # Segment  
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
        segment.tag                                 = 'segment_0'    
        segment.percent_x_location                  = 0.0000
        segment.percent_z_location                  = 0.00 
        segment.height                              = 0.000 
        segment.width                               = 0.000  
        Drop_Tank_Port.append_segment(segment)   
        
        # Segment  
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
        segment.tag                                 = 'segment_1'    
        segment.percent_x_location                  = 0.25
        segment.percent_z_location                  = 0.0 
        segment.height                              = 0.838
        segment.width                               = 0.838
        Drop_Tank_Port.append_segment(segment)   
        
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_2'   
        segment.percent_x_location                  = 0.534
        segment.percent_z_location                  = 0.0 
        segment.height                              = 1.0
        segment.width                               = 1.0
        Drop_Tank_Port.append_segment(segment)      
        
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_3'   
        segment.percent_x_location                  = 0.77
        segment.percent_z_location                  = 0.0
        segment.height                              = 1.0
        segment.width                               = 1.0
        Drop_Tank_Port.append_segment(segment)   

        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_4'   
        segment.percent_x_location                  = 0.966	
        segment.percent_z_location                  = 0.0
        segment.height                              = 0.544
        segment.width                               = 0.544
        Drop_Tank_Port.append_segment(segment)   
        
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_5'   
        segment.percent_x_location                  = 1.0 
        segment.percent_z_location                  = 0.0
        segment.height                              = 0 
        segment.width                               = 0
        Drop_Tank_Port.append_segment(segment)     
        
        # add to vehicle
        vehicle.append_component(Drop_Tank_Port)

        Drop_Tank_Starboard = deepcopy(Drop_Tank_Port)
        Drop_Tank_Starboard.tag = 'tank_starboard'
        Drop_Tank_Starboard.origin = [[6.8, 3.2, -0.41]]
        vehicle.append_component(Drop_Tank_Starboard)

    # ################################################# Landing Gear #############################################################   
    # ------------------------------------------------------------------        
    #  Landing Gear
    # ------------------------------------------------------------------  
    main_gear               = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear()
    main_gear.tire_diameter = 20  * Units.inches
    main_gear.strut_length  = 3.0 * Units.ft 
    main_gear.units         = 2    
    main_gear.wheels        = 1    
    vehicle.append_component(main_gear)  

    nose_gear               = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()       
    nose_gear.tire_diameter = 5. * Units.inches
    nose_gear.units         = 1    
    nose_gear.wheels        = 1    
    nose_gear.strut_length  = 2.0 * Units.ft 
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
    fuel_line                                    = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line() 

    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Propulsor: Starboard Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------         
    turbofan1                                    = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan()   
    turbofan1.origin                             = [[ 16.0 , 1.0 , 0.0 ]]
    turbofan1.tag                                = 'propulsor_1'    
    turbofan1.length                             = 4.978                     
    turbofan1.diameter                           = 1.3               
    turbofan1.bypass_ratio                       = 0.3                     
    turbofan1.design_altitude                    = 0*Units.ft         
    turbofan1.design_mach_number                 = 0.1                   
    turbofan1.design_thrust                      = 26000 * Units.lbf   
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
    low_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    low_pressure_compressor.tag                   = 'lpc'
    low_pressure_compressor.polytropic_efficiency = 0.91                  
    low_pressure_compressor.pressure_ratio        = 1.3                     
    turbofan1.low_pressure_compressor             = low_pressure_compressor

    # high pressure compressor  
    high_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    high_pressure_compressor.tag                   = 'hpc'
    high_pressure_compressor.polytropic_efficiency = 0.91                        
    high_pressure_compressor.pressure_ratio        = 14.286             
    turbofan1.high_pressure_compressor             = high_pressure_compressor
    
    # combustor  
    combustor                                    = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                                = 'Comb'
    combustor.efficiency                         = 0.997                    
    combustor.turbine_inlet_temperature          = 1922               
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
    afterburner                                  = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    afterburner.tag                              = 'afterburner' 
    afterburner.efficiency                       = 0.9
    afterburner.alphac                           = 1.0     
    afterburner.turbine_inlet_temperature        = 1922 
    afterburner.pressure_ratio                   = 1.0
    afterburner.fuel_data                        = RCAIDE.Library.Attributes.Propellants.Jet_A()     
    turbofan1.afterburner                        = afterburner     

    # core nozzle
    core_nozzle                                   = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                               = 'core_nozzle'
    core_nozzle.polytropic_efficiency             = 0.98                     
    core_nozzle.pressure_ratio                    = 0.995
    core_nozzle.diameter                          = 1
    core_nozzle.pressure_recovery                 = 0.99
    turbofan1.core_nozzle                         = core_nozzle
     
    # # fan nozzle             
    fan_nozzle                                    = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    fan_nozzle.tag                                = 'fan_nozzle'
    fan_nozzle.polytropic_efficiency              = 0.98                    
    fan_nozzle.pressure_ratio                     = 0.995 
    fan_nozzle.diameter                           = 1.3
    turbofan1.fan_nozzle                          = fan_nozzle
    
    # # design turbofan
    design_turbofan(turbofan1)

    # Nacelle 
    nacelle                                     = RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle()
    nacelle                                     = RCAIDE.Library.Components.Nacelles.Stack_Nacelle()
    nacelle.diameter                            = 1.3
    nacelle.tag                                 = 'nacelle_1'
    nacelle.origin                              = [[5.328,1.311,-0.164]] 
    nacelle.length                              = 11.58
    nacelle.inlet_diameter                      = 1.1  
    nacelle.areas.wetted                        = np.pi * nacelle.length * nacelle.inlet_diameter *  1
    nacelle.areas.front_projected               = 0.25 *  np.pi * (nacelle.inlet_diameter ** 2) 
    
    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_1' 
    nac_segment.orientation_euler_angles        = [-22.5*Units.degrees, -28*Units.degrees,-47.0*Units.degrees] 
    nac_segment.percent_x_location              = 0.0  
    nac_segment.percent_y_location              = -0.00921
    nac_segment.percent_z_location              = 0.0
    nac_segment.height                          = 1.0  
    nac_segment.width                           = 1.32
    nac_segment.curvature                       = 5
    nacelle.append_segment(nac_segment)         

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_2'
    nac_segment.percent_x_location              = 0.33
    nac_segment.percent_y_location              = -0.0123
    nac_segment.percent_z_location              = 0.0041
    nac_segment.height                          = 1.24
    nac_segment.width                           = 1.367
    nac_segment.curvature                       = 5
    nacelle.append_segment(nac_segment)      

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_3'
    nac_segment.percent_x_location              = 0.75
    nac_segment.percent_y_location              = -0.03996
    nac_segment.percent_z_location              = 0.01332
    nac_segment.height                          = 1.165
    nac_segment.width                           = 1.17
    nac_segment.curvature                       = 5
    nacelle.append_segment(nac_segment)   

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_4'
    nac_segment.percent_x_location              = 1.0
    nac_segment.percent_y_location              = -0.04918
    nac_segment.percent_z_location              = 0.02971
    nac_segment.height                          = 0.80
    nac_segment.width                           = 1.13
    nac_segment.curvature                       = 5
    nacelle.append_segment(nac_segment)         

    turbofan1.nacelle                           = nacelle
    
    net.propulsors.append(turbofan1)
    
    turbofan2                                                       = deepcopy(turbofan1)
    turbofan2.tag                                                   = 'propulsor_2'
    turbofan2.origin                                                = [[ 16.0 , -1.0 , 0.0 ]]
    turbofan2.nacelle.tag                                           = 'nacelle_2'
    turbofan2.nacelle.origin                                        = [[5.328,-1.311,-0.164]]
    turbofan2.nacelle.segments.segment_1.orientation_euler_angles   = [22.5*Units.degrees, -28*Units.degrees,47.0*Units.degrees]  
    turbofan2.nacelle.segments.segment_1.percent_y_location        *= -1
    turbofan2.nacelle.segments.segment_2.percent_y_location        *= -1
    turbofan2.nacelle.segments.segment_3.percent_y_location        *= -1
    turbofan2.nacelle.segments.segment_4.percent_y_location        *= -1
    net.propulsors.append(turbofan2)

    #------------------------------------------------------------------------------------------------------------------------- 
    #  Energy Source: Fuel Tank
    #------------------------------------------------------------------------------------------------------------------------- 
    # fuel tank
    fuel_tank                                        = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Wing_Fuel_Tank()
    fuel_tank.origin                                 = vehicle.wings.main_wing.origin  
    fuel_tank.fuel                                   = RCAIDE.Library.Attributes.Propellants.Jet_A1()     
    fuel_line.fuel_tanks.append(fuel_tank)

    if vehicle.Drop_Tank:
        Drop_tank_fuel                                        = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Wing_Fuel_Tank()
        Drop_tank_fuel.tag                                    = 'tank_starboard'
        Drop_tank_fuel.origin                                 = vehicle.fuselages.tank_starboard.origin
        Drop_tank_fuel.fuel                                   = RCAIDE.Library.Attributes.Propellants.Jet_A1()     
        fuel_line.fuel_tanks.append(Drop_tank_fuel)

        Drop_tank_fuel_Port                                   = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Wing_Fuel_Tank()
        Drop_tank_fuel_Port.tag                               = 'tank_port'
        Drop_tank_fuel_Port.origin                            = vehicle.fuselages.tank_port.origin
        Drop_tank_fuel_Port.fuel                              = RCAIDE.Library.Attributes.Propellants.Jet_A1()    
        fuel_line.fuel_tanks.append(Drop_tank_fuel_Port)

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to fuel line to network   
    #------------------------------------------------------------------------------------------------------------------------------------   
    fuel_line.assigned_propulsors =  [['propulsor_1', 'propulsor_2']]

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Append fuel line to fuel line to network  
    #------------------------------------------------------------------------------------------------------------------------------------   
    net.fuel_lines.append(fuel_line)        

    #------------------------------------------------------------------------------------------------------------------------- 
    # Done ! 
    #-------------------------------------------------------------------------------------------------------------------------      

    # Append energy network to aircraft 
    vehicle.append_energy_network(net)
    
    return vehicle 

if __name__ == '__main__': 
    main()    

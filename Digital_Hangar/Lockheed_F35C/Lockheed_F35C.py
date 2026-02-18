# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units   
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan    import design_turbofan 
from RCAIDE.Library.Plots                 import *       

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
    vehicle  = vehicle_setup()  

    try:
        import vsp as vsp
        from RCAIDE.Framework.External_Interfaces.OpenVSP import export_vsp_vehicle 
        export_vsp_vehicle(vehicle, 'Lockheed_F35C')
    except ImportError:
        pass
    
    # Step 2: plot vehicle 
    plot_3d_vehicle(vehicle,export_gltf=True)  
    
    return  

def vehicle_setup(): 
    airfoil_file_path =  os.path.join(os.path.split(sys.path[0])[0], 'Airfoils_and_Polars') + os.sep 
    polar_file_path   =  os.path.join(os.path.join(os.path.split(sys.path[0])[0], 'Airfoils_and_Polars') , 'Polars') + os.sep
    
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
    vehicle.flight_envelope.design_range              = 1200.0 * Units.nmi 
    vehicle.reference_area                            = 66.04 * Units['meters**2']
    vehicle.number_of_passengers                      = 0
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
    wing.twists.tip                       =-3.0 * Units.degrees 
    wing.origin                           = [[5.046,0,0.426]]
    wing.aerodynamic_center               = [10.0,0,0.426] 
    wing.vertical                         = False
    wing.dihedral                         = 0.0 * Units.degrees 
    wing.xz_plane_symmetric               = True 
    wing.dynamic_pressure_ratio           = 1.0
    
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Segment_0'
    segment.percent_span_location         = 0.0
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.leading_edge           = 0.0 * Units.degrees
    segment.thickness_to_chord            =.005
    wing_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    wing_airfoil.coordinate_file          = airfoil_file_path +  'NACA65_203.txt' 
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Segment_1'
    segment.percent_span_location         = 0.1292
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.leading_edge           = -47.04 * Units.degrees
    segment.thickness_to_chord            = .0025
    wing_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    wing_airfoil.coordinate_file          = airfoil_file_path +  'NACA65_203.txt' 
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Segment_2'
    segment.percent_span_location         = 0.16150
    segment.root_chord_percent            = 1.414
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.leading_edge           = -47.04 * Units.degrees
    segment.thickness_to_chord            = .0025
    wing_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    wing_airfoil.coordinate_file          = airfoil_file_path +  'NACA65_203.txt' 
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Segment_3'
    segment.percent_span_location         = 0.1938
    segment.root_chord_percent            = 1.4462
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.leading_edge           = -47.04 * Units.degrees
    segment.thickness_to_chord            = .0025
    wing_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    wing_airfoil.coordinate_file          = airfoil_file_path +  'NACA65_203.txt' 
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Segment_4'
    segment.percent_span_location         = 0.2216
    segment.root_chord_percent            = 1.1529 
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.leading_edge           =-47.04 * Units.degrees
    segment.thickness_to_chord            =.0025
    wing_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    wing_airfoil.coordinate_file          = airfoil_file_path +  'NACA65_203.txt' 
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Segment_5'
    segment.percent_span_location         = 0.258
    segment.root_chord_percent            = 1.1689
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.leading_edge           = 86.6 * Units.degrees
    segment.thickness_to_chord            =.0025
    wing_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    wing_airfoil.coordinate_file          = airfoil_file_path +  'NACA65_203.txt' 
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Segment_6'
    segment.percent_span_location         = 0.2836
    segment.root_chord_percent            = 0.78137
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.leading_edge           = 35.0 * Units.degrees
    segment.thickness_to_chord            = .0025
    wing_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    wing_airfoil.coordinate_file          = airfoil_file_path +  'NACA65_203.txt' 
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)
    
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Tip'
    segment.percent_span_location         = 1.0
    segment.root_chord_percent            = 0.1983
    segment.dihedral_outboard             = 0 * Units.degrees
    segment.sweeps.quarter_chord          = 0 * Units.degrees
    segment.thickness_to_chord            = .0025
    wing_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    wing_airfoil.coordinate_file          = airfoil_file_path +  'NACA65_203.txt' 
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)

    # control surfaces -------------------------------------------

    flap                          = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    flap.tag                      = 'flap'
    flap.span_fraction_start      = 0.3
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
    wing.aspect_ratio            = 1.6018
    wing.sweeps.leading_edge     = 32.79 * Units.deg  
    wing.thickness_to_chord      = 0.05
    wing.taper                   = 0.2245
    wing.spans.projected         = 5.85863 * Units.meter 
    wing.chords.root             = 2.722 * Units.meter
    wing.chords.tip              = 0.6111 * Units.meter
    wing.chords.mean_aerodynamic = 2.05 * Units.meter
    wing.areas.reference         = 10.71349 * Units['meters**2']
    wing.areas.exposed           = 10 * Units['meters**2']   
    wing.areas.wetted            = 21.00 * Units['meters**2'] 
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees 
    wing.origin                  = [[12.75, 1.311, 0.426]]
    wing.aerodynamic_center      = [14.0, 1.311, 0.426] 
    wing.vertical                = False
    wing.xz_plane_symmetric      = True 
    wing.dynamic_pressure_ratio  = 0.9

    

    # Wing Segments
    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'root_segment'
    segment.percent_span_location  = 0
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 1.0
    segment.dihedral_outboard      = 0.0 * Units.degrees
    segment.sweeps.leading_edge    = 0.0  * Units.degrees 
    segment.thickness_to_chord     = .05
    tail_airfoil                   = RCAIDE.Library.Components.Airfoils.Airfoil() 
    tail_airfoil.coordinate_file   = airfoil_file_path + 'supersonic_tail.txt'  
    segment.append_airfoil(tail_airfoil)
    wing.append_segment(segment)

    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'segment_1'
    segment.percent_span_location  = 0.29
    segment.twist                  = 0.* Units.deg
    segment.root_chord_percent     = 0.918
    segment.dihedral_outboard      = 0.0 * Units.degrees
    segment.sweeps.leading_edge    = 32.75 * Units.degrees 
    segment.thickness_to_chord     = .05
    tail_airfoil                   = RCAIDE.Library.Components.Airfoils.Airfoil() 
    tail_airfoil.coordinate_file   = airfoil_file_path + 'supersonic_tail.txt'  
    segment.append_airfoil(tail_airfoil)
    wing.append_segment(segment)

    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'tip_segment'
    segment.percent_span_location  = 1.
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 0.2245              
    segment.dihedral_outboard      = 0 * Units.degrees
    segment.sweeps.leading_edge    = 32.75  * Units.degrees 
    segment.thickness_to_chord     = .05
    tail_airfoil                   = RCAIDE.Library.Components.Airfoils.Airfoil() 
    tail_airfoil.coordinate_file   = airfoil_file_path + 'supersonic_tail.txt'  
    segment.append_airfoil(tail_airfoil)
    wing.append_segment(segment) 

    # control surfaces -------------------------------------------
    elevator                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator() # F-35 appears to have a stabilator instead of an elevator
    elevator.tag                   = 'elevator'
    elevator.span_fraction_start   = 0.09
    elevator.span_fraction_end     = 0.92
    elevator.deflection            = 0.0  * Units.deg
    elevator.chord_fraction        = 0.7
    wing.append_control_surface(elevator)     

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
    stabilizer_1.spans.projected         = 2.2 * Units.meter 
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
    stabilizer_1.xz_plane_symmetric      = True
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

    # control surfaces -------------------------------------------
    rudder                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    rudder.tag                   = 'rudder'
    rudder.span_fraction_start   = 0.1 
    rudder.span_fraction_end     = 0.95 
    rudder.deflection            = 0 
    rudder.chord_fraction        = 0.33  
    stabilizer_1.append_control_surface(rudder)    
 
    # add to vehicle
    vehicle.append_component(stabilizer_1) 
    
    # ################################################# Fuselage ################################################################ 
        
    fuselage                                    = RCAIDE.Library.Components.Fuselages.Fuselage() 
    fuselage.number_coach_seats                 = vehicle.number_of_passengers 
    fuselage.seats_abreast                      = 0
    fuselage.seat_pitch                         = 0.0   * Units.meter 
    fuselage.fineness.nose                      = 2.
    fuselage.fineness.tail                      = 3.5 
    fuselage.lengths.nose                       = 3.3   * Units.meter
    fuselage.lengths.tail                       = 3.0   * Units.meter
    fuselage.lengths.total                      = 13.6  * Units.meter  
    fuselage.lengths.fore_space                 = 0.25  * Units.meter
    fuselage.lengths.aft_space                  = 1.0   * Units.meter
    fuselage.width                              = 1.75  * Units.meter
    fuselage.heights.maximum                    = 2.08  * Units.meter
    fuselage.effective_diameter                 = 1.467 * Units.meter
    fuselage.areas.side_projected               = fuselage.heights.maximum * fuselage.lengths.total * Units['meters**2'] 
    fuselage.areas.wetted                       = np.pi * fuselage.width/2 * fuselage.lengths.total * Units['meters**2'] 
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
    main_gear.units         = 2    
    main_gear.wheels        = 1    
    vehicle.append_component(main_gear)  

    nose_gear               = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()       
    nose_gear.tire_diameter = 18. * Units.inches
    nose_gear.units         = 1   
    nose_gear.wheels        = 1    
    nose_gear.strut_length  = 3.0 * Units.ft 
    vehicle.append_component(nose_gear)

    #------------------------------------------------------------------------------------------------------------------------------------
    # ########################################################## Energy Network ######################################################### 
    #------------------------------------------------------------------------------------------------------------------------------------ 
    #initialize the fuel network
    net                                            = RCAIDE.Framework.Networks.Fuel() 
    net.identical_propulsors                       = True 
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Fuel Distribution Line 
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
    low_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    low_pressure_compressor.tag                   = 'lpc'
    low_pressure_compressor.polytropic_efficiency = 0.91                  
    low_pressure_compressor.pressure_ratio        = 1.3                     
    turbofan1.low_pressure_compressor             = low_pressure_compressor

    # high pressure compressor  
    high_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    high_pressure_compressor.tag                   = 'hpc'
    high_pressure_compressor.polytropic_efficiency = 0.91                        
    high_pressure_compressor.pressure_ratio        = 15.38             
    turbofan1.high_pressure_compressor             = high_pressure_compressor
    
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
    core_nozzle                                   = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                               = 'core_nozzle'
    core_nozzle.polytropic_efficiency             = 0.98                     
    core_nozzle.pressure_ratio                    = 0.995
    core_nozzle.diameter                          = 1.1
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
    nacelle                                     = RCAIDE.Library.Components.Nacelles.Stack_Nacelle()
    nacelle.diameter                            = 1.05
    nacelle.tag                                 = 'nacelle_1'
    nacelle.origin                              = [[4.590,1.148,0.164]] 
    nacelle.length                              = 9.415
    nacelle.inlet_diameter                      = 2.0 
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

    turbofan1.nacelle                           = nacelle
    
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
    nacelle_mirror.segments['segment_0'].orientation_euler_angles = [7.71*Units.degrees, 0.0*Units.degrees, -45.0*Units.degrees] 
    nacelle_mirror.segments['segment_1'].percent_y_location = -0.0082
    nacelle_mirror.segments['segment_2'].percent_y_location = -0.00713
    nacelle_mirror.segments['segment_3'].percent_y_location = -0.00217
    nacelle_mirror.segments['segment_4'].percent_y_location = -0.00806
    ghost_propulsor.nacelle = nacelle_mirror

    net.propulsors.append(ghost_propulsor) 

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


if __name__ == '__main__': 
    main()    

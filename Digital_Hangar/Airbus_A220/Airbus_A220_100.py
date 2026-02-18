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
        export_vsp_vehicle(vehicle, 'Airbus_A220_100')
    except ImportError:
        pass
    
    # Step 2: plot vehicle 
    plot_3d_vehicle(vehicle,export_gltf=True)  
    
    return  

def vehicle_setup(): 
    airfoil_file_path =  os.path.join(os.path.split(sys.path[0])[0], '_Airfoils_and_Polars') + os.sep 
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    

    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'Airbus_A220_100'   

    # ################################################# Vehicle-level Properties ########################################################  
    vehicle.mass_properties.max_takeoff             = 63100  # kg 
    vehicle.mass_properties.takeoff                 = 63100  # kg 
    vehicle.mass_properties.max_zero_fuel           = 52200  # kg 
    vehicle.mass_properties.max_payload             = 17230  # kg
    vehicle.mass_properties.max_fuel                = 17000  # kg 17600
    vehicle.mass_properties.min_payload             = 0  # kg
    vehicle.flight_envelope.ultimate_load           = 3.75
    vehicle.flight_envelope.positive_limit_load     = 1.5
    vehicle.flight_envelope.design_mach_number      = 0.78
    vehicle.flight_envelope.design_cruise_altitude  = 35000 * Units.feet
    vehicle.flight_envelope.design_range            = 3600 * Units.nmi
    vehicle.reference_area                          = 112.3* Units['meters**2']
    vehicle.number_of_passengers                    = 135
    vehicle.systems.control                         = "fully powered"
    vehicle.systems.accessories                     = "medium range"              
    cruise_speed                                    = 470 * Units.kts
    altitude                                        = 30000 * Units.feet
    atmo                                            = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    freestream                                      = atmo.compute_values (0.)
    freestream0                                     = atmo.compute_values (altitude)
    mach_number                                     = (cruise_speed/freestream.speed_of_sound)[0][0] 
    vehicle.design_dynamic_pressure                 = ( .5 *freestream0.density*(cruise_speed*cruise_speed))[0][0]
    vehicle.design_mach_number                      =  mach_number

   
    # ################################################# Wings #############################################################
    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------

    wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing'
    wing.aspect_ratio                     = 9.24167
    wing.sweeps.quarter_chord             = 23 * Units.deg
    wing.thickness_to_chord               = 0.12
    wing.spans.projected                  = 35.1
    wing.chords.root                      = 7.0 * Units.meter
    wing.chords.tip                       = 0.4 * Units.meter
    wing.taper                            = wing.chords.tip / wing.chords.root
    wing.chords.mean_aerodynamic          = 4.88* Units.meter 
    wing.areas.reference                  = 122.915
    wing.areas.wetted                     = 390 
    wing.twists.root                      = 4.0 * Units.degrees 
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[ 10.543,0,  -0.652]]
    wing.aerodynamic_center               = [0,0,0] 
    wing.vertical                         = False
    wing.dihedral                         = 7.0 * Units.degrees 
    wing.xz_plane_symmetric               = True 
    wing.high_lift                        = True 
    wing.dynamic_pressure_ratio           = 1.0
    wing.twists.outwash                   = -6 * Units.degree
    wing.twists.root_twist                = 3 * Units.degree  
        
    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root'
    segment.percent_span_location         = 0.0
    segment.twist                         = wing.twists.root_twist  +  segment.percent_span_location * wing.twists.outwash 
    segment.root_chord_percent            = 1.
    segment.dihedral_outboard             = 7.82609 * Units.degrees
    segment.sweeps.quarter_chord          = 22.77 * Units.degrees 
    root_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil() 
    root_airfoil.coordinate_file          = airfoil_file_path + 'transonic_wing_root_section_airfoil.txt' 
    segment.append_airfoil(root_airfoil)
    wing.append_segment(segment) 

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'inboard'
    segment.percent_span_location         = 0.368 
    segment.twist                         = wing.twists.root_twist  +  segment.percent_span_location * wing.twists.outwash 
    segment.root_chord_percent            = 0.5057
    segment.dihedral_outboard             = 6.41304 * Units.degrees
    segment.sweeps.quarter_chord          = 26.54545 * Units.degrees
    mid_airfoil                           = RCAIDE.Library.Components.Airfoils.Airfoil()
    mid_airfoil.coordinate_file           = airfoil_file_path +'transonic_wing_inboard_section_airfoil.txt'
    segment.append_airfoil(mid_airfoil) 
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'outboard'
    segment.percent_span_location         = 0.96
    segment.twist                         = wing.twists.root_twist  +  segment.percent_span_location * wing.twists.outwash 
    segment.root_chord_percent            = 0.1986
    segment.dihedral_outboard             = 53.55* Units.degrees
    segment.sweeps.quarter_chord          = 48.0* Units.degrees
    tip_airfoil                           =  RCAIDE.Library.Components.Airfoils.Airfoil()
    tip_airfoil.coordinate_file           = airfoil_file_path + 'transonic_wing_outboard_section_airfoil.txt'
    segment.append_airfoil(tip_airfoil)
    wing.append_segment(segment)
    
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.0
    segment.twist                         = wing.twists.root_twist  +  segment.percent_span_location * wing.twists.outwash 
    segment.root_chord_percent            = 0.1066
    segment.thickness_to_chord            = 0.1
    segment.dihedral_outboard             = 53.55* Units.degrees
    segment.sweeps.quarter_chord          = 0.
    tip_airfoil                           =  RCAIDE.Library.Components.Airfoils.Airfoil()
    tip_airfoil.coordinate_file           =  airfoil_file_path +'transonic_wing_tip_section_airfoil.txt'
    segment.append_airfoil(tip_airfoil)
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
    flap.span_fraction_start      = 0.2
    flap.span_fraction_end        = 0.7
    flap.deflection               = 0.0 * Units.degrees
    flap.configuration_type       = 'double_slotted'
    flap.chord_fraction           = 0.30
    wing.append_control_surface(flap)

    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.7
    aileron.span_fraction_end     = 0.963
    aileron.deflection            = 0.0 * Units.degrees
    aileron.chord_fraction        = 0.16
    wing.append_control_surface(aileron)

    spoiler                                        = RCAIDE.Library.Components.Wings.Control_Surfaces.Spoiler()
    spoiler.tag                                    = 'spoiler'
    spoiler.span_fraction_start                    = 0.3
    spoiler.span_fraction_end                      = 0.7
    spoiler.deflection                             = 0.0 * Units.degrees
    spoiler.chord_fraction                         = 0.05
    wing.append_control_surface(spoiler)     
   
    
    # add to vehicle
    vehicle.append_component(wing) 


    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing     = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag = 'horizontal_stabilizer'

    wing.aspect_ratio              = 2.55111
    wing.sweeps.quarter_chord      = 30
    wing.thickness_to_chord        = 0.08
    wing.taper                     = 0.355 
    wing.spans.projected           = 12.30000
    wing.chords.root               = 3.55556
    wing.chords.tip                = 1.26587
    wing.chords.mean_aerodynamic   = 2.27
    wing.areas.reference           = 16.47
    wing.areas.exposed             = 25.8 
    wing.areas.wetted              = 29.65179
    wing.twists.root               = 3.0 * Units.degrees
    wing.twists.tip                = 3.0 * Units.degrees
    wing.origin                    = [[ 28.852,0,  1.230]]
    wing.aerodynamic_center        = [0,0,0]
    wing.vertical                  = False
    wing.xz_plane_symmetric        = True
    wing.dynamic_pressure_ratio    = 0.9



    # Wing Segments
    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'root_segment'
    segment.percent_span_location  = 0.0
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 1.0
    segment.dihedral_outboard      = 6.35714 * Units.degrees
    segment.sweeps.quarter_chord   = 30.00000 * Units.degrees 
    segment.thickness_to_chord     = .07
    wing.append_segment(segment)

    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'tip_segment'
    segment.percent_span_location  = 1.
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 0.355             
    segment.dihedral_outboard      = 0 * Units.degrees
    segment.sweeps.quarter_chord   = 0 * Units.degrees  
    segment.thickness_to_chord     = .07
    wing.append_segment(segment)
    
        

    # Control Surfaces -------------------------------------------
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

    wing.aspect_ratio            = 3.60335
    wing.sweeps.quarter_chord    = 38.92857
    wing.thickness_to_chord      = 0.08
    wing.taper                   = 0.25

    wing.spans.projected         = 7.0709
    wing.total_length            = 7.0787

    wing.chords.root             = 6.27778
    wing.chords.tip              = 1.5714
    wing.chords.mean_aerodynamic = 4.0

    wing.areas.reference         = 30.83
    wing.areas.wetted            = 55.5

    wing.twists.root             = 3.0 * Units.degrees
    wing.twists.tip              = 3.0 * Units.degrees

    wing.origin                  = [[25.328,0, 1.311]]
    wing.aerodynamic_center      = [0,0,0]

    wing.vertical                = True
    wing.xz_plane_symmetric      = False
    wing.t_tail                  = False

    wing.dynamic_pressure_ratio  = 1.0


    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root'
    segment.percent_span_location         = 0.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 1.
    segment.dihedral_outboard             = 0 * Units.degrees
    segment.sweeps.quarter_chord          = 38.92857 * Units.degrees  
    segment.thickness_to_chord            = .07
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.25
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.quarter_chord          = 0.0    
    segment.thickness_to_chord            = .07  
    wing.append_segment(segment)
    
 
    # control surfaces -------------------------------------------
    rudder                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    rudder.tag                   = 'rudder'
    rudder.span_fraction_start   = 0.1 
    rudder.span_fraction_end     = 0.95 
    rudder.deflection            = 0 
    rudder.chord_fraction        = 0.33  
    wing.append_control_surface(rudder)   


    # add to vehicle
    vehicle.append_component(wing)

    # ################################################# Landing Gear #############################################################    
    # ------------------------------------------------------------------        
    #  Landing Gear
    # ------------------------------------------------------------------  
    main_gear                                = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear() 
    main_gear.tire_diameter                  = 44.5 *  Units.inches 
    main_gear.rim_diameter                   = 21   *  Units.inches 
    main_gear.tire_width                     = 16.5  *  Units.inches 
    main_gear.strut_length                   = 1.8  * Units.m  
    main_gear.wheels                         = 4   
    main_gear.number_of_gear_types_in_tandem = 1
    main_gear.number_of_wheels_in_gear_type  = 2  
    main_gear.xz_plane_symmetric             = True
    vehicle.append_component(main_gear)  

    nose_gear                                = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()   
    nose_gear.tire_diameter                  = 27    *  Units.inches   
    nose_gear.rim_diameter                   = 15    *  Units.inches 
    nose_gear.tire_width                     = 7.75  *  Units.inches 
    nose_gear.strut_length                   = 1.8   * Units.m  
    nose_gear.wheels                         = 2   
    nose_gear.number_of_gear_types_in_tandem = 1
    nose_gear.number_of_wheels_in_gear_type  = 2    
    vehicle.append_component(nose_gear)
    


    # ##########################################################   Fuselage  ############################################################    
    fuselage = RCAIDE.Library.Components.Fuselages.Fuselage() 

    cabin                                             = RCAIDE.Library.Components.Fuselages.Cabins.Cabin() 
    cabin.origin                                      = [[3.5, 0, 0]]
    cabin.offset_x = 3.5
    economy_class                                     = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest              = 5
    economy_class.seat_pitch                          = 29 *  Units.inches
    economy_class.number_of_rows                      = 27
    economy_class.galley_lavatory_percent_x_locations = [0, 1]      
    economy_class.emergency_exit_percent_x_locations  = [0.5, 0.5]      
    economy_class.type_A_exit_percent_x_locations     = [0, 1]     
    cabin.append_cabin_class(economy_class)
    fuselage.append_cabin(cabin) 

    fuselage.seats_abreast                      = 5
    fuselage.fineness.nose                      = 0.58
    fuselage.fineness.tail                      = 1.75
    fuselage.lengths.nose                       = 0.921 
    fuselage.lengths.tail                       = 4.181
    fuselage.lengths.cabin                      = 29.798 #m
    fuselage.lengths.total                      = 34.9
    fuselage.width                              = 3.95 
    fuselage.heights.maximum                    = 4.19   
    fuselage.heights.at_quarter_length          = 3.71  
    fuselage.heights.at_three_quarters_length   = 3.83
    fuselage.heights.at_wing_root_quarter_chord = 4.19 
    fuselage.areas.side_projected               = fuselage.lengths.total *fuselage.heights.maximum  # estimate    
    fuselage.areas.wetted                       = 2 * np.pi * fuselage.width *  fuselage.lengths.total +  2 * np.pi * fuselage.width ** 2
    fuselage.areas.front_projected              =  np.pi * fuselage.width ** 2 
    fuselage.effective_diameter                 = 3.6 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_0'
    segment.percent_x_location                  = 0
    segment.percent_z_location                  = 0
    segment.height                              = 0
    segment.width                               = 0
    fuselage.append_segment(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_1'
    segment.percent_x_location                  = 0.01
    segment.percent_z_location                  = 0
    segment.height                              = 0.90000
    segment.width                               = 0.68182
    fuselage.append_segment(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'
    segment.percent_x_location                  = 0.02000
    segment.percent_z_location                  = 0.00100	 
    segment.height                              = 1.30000
    segment.width                               = 1.25000
    fuselage.append_segment(segment) 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'
    segment.percent_x_location                  = 0.03000
    segment.percent_z_location                  = 0.00300 
    segment.height                              = 1.65000
    segment.width                               = 1.64773
    fuselage.append_segment(segment)  

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'
    segment.percent_x_location                  = 0.04000 
    segment.percent_z_location                  = 0.00500	 
    segment.height                              = 1.96000
    segment.width                               = 1.93182
    fuselage.append_segment(segment) 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'
    segment.percent_x_location                  = 0.06000
    segment.percent_z_location                  = 0.01080 
    segment.height                              = 2.60000
    segment.width                               = 2.50000
    fuselage.append_segment(segment) 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'
    segment.percent_x_location                  = 0.07500 
    segment.percent_z_location                  = 0.01350	 
    segment.height                              = 2.92000
    segment.width                               = 2.67045 
    fuselage.append_segment(segment) 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'
    segment.percent_x_location                  = 0.10000
    segment.percent_z_location                  = 0.01750
    segment.height                              = 3.30000  
    segment.width                               = 3.01136
    fuselage.append_segment(segment)
  

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_8'
    segment.percent_x_location                  = 0.16000
    segment.percent_z_location                  = 0.02174	 	 
    segment.height                              = 3.70000	 
    segment.width                               = 3.50000
    fuselage.append_segment(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_9'
    segment.percent_x_location                  = 0.67000
    segment.percent_z_location                  = 0.02170
    segment.height                              = 3.70000
    segment.width                               = 3.50000
    fuselage.append_segment(segment)
  

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_10'
    segment.percent_x_location                  = 0.73000
    segment.percent_z_location                  = 0.02600	 	 
    segment.height                              = 3.30000
    segment.width                               = 3.50000
    fuselage.append_segment(segment)
    

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_11'
    segment.percent_x_location                  = 0.89582
    segment.percent_z_location                  = 0.04200	 	 
    segment.height                              = 1.95000 
    segment.width                               = 2.10000
    fuselage.append_segment(segment)
    

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_12'
    segment.percent_x_location                  = 0.93715
    segment.percent_z_location                  = 0.04348 
    segment.height                              = 1.54000	 
    segment.width                               = 1.80000 
    fuselage.append_segment(segment)
    
    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_13'
    segment.percent_x_location                  = 0.98463
    segment.percent_z_location                  = 0.04800
    segment.height                              = 0.85000	 
    segment.width                               = 0.80000 
    fuselage.append_segment(segment)
    
    
    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_14'
    segment.percent_x_location                  = 1.0
    segment.percent_z_location                  = 0.04800
    segment.height                              = 0.0	 
    segment.width                               = 0.0000 
    fuselage.append_segment(segment)    

 

    # add to vehicle
    vehicle.append_component(fuselage) 


    # ################################################# Energy Network #######################################################         
    # Step 1: Define network
    # Step 2: Define Distribution Type
    # Step 3: Define Propulsors 
    # Step 4: Define Enegy Source 

    #------------------------------------------------------------------------------------------------------------------------- 
    #  Turbofan Network
    #-------------------------------------------------------------------------------------------------------------------------   
    net                                         = RCAIDE.Framework.Networks.Fuel() 
    
    #------------------------------------------------------------------------------------------------------------------------- 
    # Fuel Distrubition Line 
    #------------------------------------------------------------------------------------------------------------------------- 
    fuel_line                                   = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()  
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Starboard Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------         
    turbofan                                    = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan() 
    turbofan.tag                                = 'starboard_propulsor'
    turbofan.active_fuel_tanks                  = ['fuel_tank'] 
    turbofan.origin                             = [[ 10.150,  5.435, -1.087]] 
    turbofan.engine_length                      = 3.175    
    turbofan.eninge_diameter                    = 2.086
    turbofan.bypass_ratio                       = 12.5   
    turbofan.design_altitude                    = 40000.0*Units.ft
    turbofan.design_mach_number                 = 0.78   
    turbofan.design_thrust                      = 15500.0* Units.N 

    # fan                
    fan                                         = RCAIDE.Library.Components.Powertrain.Converters.Fan()   
    fan.tag                                     = 'fan'
    fan.polytropic_efficiency                   = 0.93
    fan.pressure_ratio                          = 1.7   
    turbofan.fan                                = fan        

    # working fluid                   
    turbofan.working_fluid                      = RCAIDE.Library.Attributes.Gases.Air() 
    ram                                         = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                     = 'ram' 
    turbofan.ram                                = ram 

    # inlet nozzle          
    inlet_nozzle                                = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                            = 'inlet nozzle'
    inlet_nozzle.polytropic_efficiency          = 0.98
    inlet_nozzle.pressure_ratio                 = 0.98 
    turbofan.inlet_nozzle                       = inlet_nozzle 

    # low pressure compressor    
    low_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    low_pressure_compressor.tag                   = 'lpc'
    low_pressure_compressor.polytropic_efficiency = 0.91
    low_pressure_compressor.pressure_ratio        = 1.9   
    turbofan.low_pressure_compressor              = low_pressure_compressor

    # high pressure compressor  
    high_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    high_pressure_compressor.tag                   = 'hpc'
    high_pressure_compressor.polytropic_efficiency = 0.91
    high_pressure_compressor.pressure_ratio        = 10.0    
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
    combustor.turbine_inlet_temperature            = 1600
    combustor.pressure_ratio                       = 0.95
    combustor.fuel_data                            = RCAIDE.Library.Attributes.Propellants.Jet_A1()  
    turbofan.combustor                             = combustor

    # core nozzle
    core_nozzle                                    = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                = 'core nozzle'
    core_nozzle.polytropic_efficiency              = 0.95
    core_nozzle.pressure_ratio                     = 0.99  
    turbofan.core_nozzle                           = core_nozzle

    # fan nozzle             
    fan_nozzle                                     = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    fan_nozzle.tag                                 = 'fan nozzle'
    fan_nozzle.polytropic_efficiency               = 0.95
    fan_nozzle.pressure_ratio                      = 0.99 
    turbofan.fan_nozzle                            = fan_nozzle 

    # design turbofan
    design_turbofan(turbofan)    

    # Nacelle 
    nacelle                                     = RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle()
    nacelle.diameter                            = 2.1
    nacelle.length                              = 3.258
    nacelle.tag                                 = 'nacelle_1'
    nacelle.inlet_diameter                      = 2.08
    nacelle.origin                              = [[ 10.150, 5.435, -1.087]] 
    nacelle.areas.wetted                        = 1.1*np.pi*nacelle.diameter*nacelle.length
    nacelle_airfoil                             = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    nacelle_airfoil.NACA_4_Series_code          = '2410'
    nacelle.append_airfoil(nacelle_airfoil) 
    turbofan.nacelle                            = nacelle
    net.propulsors.append(turbofan)

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Port Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------      
    # copy turbofan
    turbofan_2                                  = deepcopy(turbofan)
    turbofan_2.active_fuel_tanks                = ['fuel_tank'] 
    turbofan_2.tag                              = 'port_propulsor' 
    turbofan_2.origin                           = [[10.150, -5.435, -1.087]]  
    turbofan_2.nacelle.origin                   = [[10.150, -5.435, -1.087]]
         
    # append propulsors to distribution line 
    fuel_line.assigned_propulsors = [['starboard_propulsor', 'port_propulsor']]
    net.propulsors.append(turbofan_2)
  
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Fuel Tank & Fuel
    #------------------------------------------------------------------------------------------------------------------------------------   
    inboard_tank                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    inboard_tank.fuel                         = RCAIDE.Library.Attributes.Propellants.Jet_A1()  
    inboard_tank.segments_bounding_tank       = ['root','inboard']  
    inboard_tank.segments_percent_chord_start = [0.1 ,0.1 ]
    inboard_tank.segments_percent_chord_end   = [0.8   ,0.7]  
    fuel_line.fuel_tanks.append(inboard_tank)
    
    outboard_tank                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    outboard_tank.fuel                         = RCAIDE.Library.Attributes.Propellants.Jet_A1()  
    outboard_tank.segments_bounding_tank       = ['inboard', 'outboard']  
    outboard_tank.segments_percent_chord_start = [0.1 ,0.1 ]
    outboard_tank.segments_percent_chord_end   = [0.7,0.7]  
    fuel_line.fuel_tanks.append(outboard_tank)

    center_tank                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank()  
    center_tank.fuel                         = RCAIDE.Library.Attributes.Propellants.Jet_A1()
    center_tank.origin                       = [[27, 0, -0.5]]
    center_tank.outer_length                 = 2.5
    center_tank.outer_width                  = 2
    center_tank.outer_height                 = 0.5
    center_tank.geometry_type                = 'prismatic'
    fuel_line.fuel_tanks.append(center_tank) 

    # Append fuel line to Network      
    net.fuel_lines.append(fuel_line)   

    # Append energy network to aircraft 
    vehicle.append_energy_network(net)     
     
    return vehicle
 

if __name__ == '__main__': 
    main() 
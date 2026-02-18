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
        export_vsp_vehicle(vehicle, 'Bombardier_CRJ_700')
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
    vehicle.tag = 'Bombardier_CRJ_700'    
    
    # ################################################# Vehicle-level Properties #################################################   
    vehicle.mass_properties.max_takeoff               = 32999 * Units.kilogram  
    vehicle.mass_properties.takeoff                   = 30000 * Units.kilogram    
    vehicle.mass_properties.operating_empty           = 19051 * Units.kilogram  
    vehicle.mass_properties.max_zero_fuel             = 28259 * Units.kilogram 
    vehicle.mass_properties.cargo                     = 7000  * Units.kilogram 
    vehicle.flight_envelope.ultimate_load             = 3.75
    vehicle.flight_envelope.positive_limit_load       = 2.5 
    vehicle.flight_envelope.design_mach_number        = 0.78 
    vehicle.flight_envelope.design_cruise_altitude    = 35000*Units.feet
    vehicle.flight_envelope.design_range              = 3500 * Units.nmi
    vehicle.reference_area                            = 70.61 * Units['meters**2']   
    vehicle.number_of_passengers                      = 70
    vehicle.systems.control                           = "fully powered" 
    vehicle.systems.accessories                       = "medium range"
    

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
    

    # ################################################# Wings ##################################################################### 
    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------
    
    wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing' 
    wing.aspect_ratio                     = 7.656
    wing.sweeps.quarter_chord             = 26 * Units.deg
    wing.thickness_to_chord               = 0.11 # Update with airfoil type
    wing.taper                            = 0.309 
    wing.spans.projected                  = 23.25
    wing.chords.root                      = 4.85 * Units.meter
    wing.chords.tip                       = 1.5 * Units.meter
    wing.chords.mean_aerodynamic          = 3.036 * Units.meter 
    wing.areas.reference                  = 70.61
    wing.areas.wetted                     = 148.281 
    wing.twists.root                      = 1.5 * Units.degrees # guess based on autocad
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[13.38,0,-0.5]] 
    wing.aerodynamic_center               = [0,0,0] 
    wing.vertical                         = False
    wing.xz_plane_symmetric               = True
    wing.high_lift                        = True 
    wing.dynamic_pressure_ratio           = 1.0


    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Root'
    segment.percent_span_location         = 0.0
    segment.twist                         = 1.5 * Units.deg 
    segment.root_chord_percent            = 1.
    segment.thickness_to_chord            = 0.11 # adjust
    segment.dihedral_outboard             = 2 * Units.degrees
    segment.sweeps.quarter_chord          = 25 * Units.degree
    root_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil() 
    root_airfoil.coordinate_file          = airfoil_file_path +  'transonic_wing_root_section_airfoil.txt'
    segment.append_airfoil(root_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'inboard'
    segment.percent_span_location         = 0.4
    segment.twist                         = wing.twists.root * (1 - segment.percent_span_location) * Units.deg
    segment.root_chord_percent            = 0.5
    segment.thickness_to_chord            = 0.11
    segment.dihedral_outboard             = 2 * Units.degrees
    segment.sweeps.quarter_chord          = 27. * Units.degrees
    yehudi_airfoil                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    yehudi_airfoil.coordinate_file        =airfoil_file_path +'transonic_wing_inboard_section_airfoil.txt'
    segment.append_airfoil(yehudi_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'outboard'
    segment.percent_span_location         = 0.98
    segment.twist                         = wing.twists.root *  (1 - segment.percent_span_location) * Units.deg
    segment.root_chord_percent            = 0.304
    segment.thickness_to_chord            = 0.11
    segment.dihedral_outboard             = 80 * Units.degrees
    segment.sweeps.quarter_chord          = 20 * Units.degrees  
    mid_airfoil                           = RCAIDE.Library.Components.Airfoils.Airfoil()
    mid_airfoil.coordinate_file           = airfoil_file_path +'transonic_wing_outboard_section_airfoil.txt'
    segment.append_airfoil(mid_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.
    segment.twist                         = 0. * Units.degrees 
    segment.root_chord_percent            = 0.08
    segment.thickness_to_chord            = 0.11
    segment.dihedral_outboard             = 0. * Units.degrees
    segment.sweeps.quarter_chord          = 0. * Units.degrees
    tip_airfoil                           =  RCAIDE.Library.Components.Airfoils.Airfoil()
    tip_airfoil.coordinate_file           = airfoil_file_path + 'transonic_wing_outboard_section_airfoil.txt'
    segment.append_airfoil(tip_airfoil)
    wing.append_segment(segment)
    
    

    # control surfaces -------------------------------------------
    slat                          = RCAIDE.Library.Components.Wings.Control_Surfaces.Slat()
    slat.tag                      = 'slat'
    slat.span_fraction_start      = 0.21
    slat.span_fraction_end        = 0.94
    slat.deflection               = 0.0 * Units.degrees
    slat.chord_fraction           = 0.14
    wing.append_control_surface(slat)

    flap                          = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    flap.tag                      = 'flap'
    flap.span_fraction_start      = 0.12
    flap.span_fraction_end        = 0.7 
    flap.deflection               = 0.0 * Units.degrees
    flap.configuration_type       = 'double_slotted'
    flap.chord_fraction           = 0.16 
    wing.append_control_surface(flap)

    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.7 
    aileron.span_fraction_end     = 0.85 
    aileron.deflection            = 0.0 * Units.degrees
    aileron.chord_fraction        = 0.30 
    wing.append_control_surface(aileron)

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing     = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag = 'horizontal_stabilizer'

    wing.aspect_ratio            = 4.538
    wing.sweeps.quarter_chord    = 30 * Units.deg  
    wing.thickness_to_chord      = 0.11
    wing.taper                   = 0.5  
    wing.spans.projected         = 8.54 
    wing.chords.root             = 2.5 
    wing.chords.tip              = 1.25 
    wing.chords.mean_aerodynamic = 1.875 
    wing.areas.reference         = 16.01
    wing.areas.exposed           = 15.91    # Exposed area of the horizontal tail
    wing.areas.wetted            = 33.2     # Wetted area of the horizontal tail
    wing.twists.root             = -1.0 * Units.degrees # check
    wing.twists.tip              = -1.0 * Units.degrees # check 
    wing.origin                  = [[28.5,0,4.37]]
    wing.aerodynamic_center      = [0,0,0] 
    wing.vertical                = False
    wing.xz_plane_symmetric      = True 
    wing.dynamic_pressure_ratio  = 0.95


    # Wing Segments
    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'root_segment'
    segment.percent_span_location  = 0.0
    segment.twist                  = 1.0 * Units.deg
    segment.root_chord_percent     = 1.0
    segment.dihedral_outboard      = 3.25 * Units.degrees
    segment.sweeps.quarter_chord   = 30 * Units.degrees 
    segment.thickness_to_chord     = .11
    wing.append_segment(segment)

    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'tip_segment'
    segment.percent_span_location  = 1.
    segment.twist                  = 1. * Units.deg
    segment.root_chord_percent     = 0.5               
    segment.dihedral_outboard      = 0 * Units.degrees
    segment.sweeps.quarter_chord   = 0 * Units.degrees  
    segment.thickness_to_chord     = .11
    wing.append_segment(segment)
    
        

    # control surfaces -------------------------------------------
    elevator                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
    elevator.tag                   = 'elevator'
    elevator.span_fraction_start   = 0.04
    elevator.span_fraction_end     = 0.94
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

    wing.aspect_ratio            = 1.224
    wing.sweeps.quarter_chord    = 38.0  * Units.deg   
    wing.thickness_to_chord      = 0.08
    wing.taper                   = 0.5

    wing.spans.projected         = 3.9
    wing.total_length            = wing.spans.projected 
    
    wing.chords.root             = 4.5 
    wing.chords.tip              = 2.25 
    wing.chords.mean_aerodynamic = 3.185

    wing.areas.reference         = 12.425
    wing.areas.wetted            = 26.0925 
    
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.origin                  = [[25,0,1.25]]
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
    segment.root_chord_percent            = 0.889
    segment.dihedral_outboard             = 0 * Units.degrees
    segment.sweeps.quarter_chord          = 5.0 * Units.degrees  
    segment.thickness_to_chord            = .1
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_1'
    segment.percent_span_location         = 0.256
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 1
    segment.dihedral_outboard             = 0. * Units.degrees
    segment.sweeps.quarter_chord          = 70.0 * Units.degrees   
    segment.thickness_to_chord            = .1
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_2'
    segment.percent_span_location         = 0.385
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.667 
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.quarter_chord          = 38.0 * Units.degrees
    segment.thickness_to_chord            = 0.1  
    wing.append_segment(segment)
    
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_3'
    segment.percent_span_location         = 1.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.5 
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.quarter_chord          = 0.0 * Units.degrees    
    segment.thickness_to_chord            = 0.1  
    wing.append_segment(segment)
    
        

    # add to vehicle
    vehicle.append_component(wing)

    # ################################################# Fuselage ################################################################ 
    
    fuselage                                    = RCAIDE.Library.Components.Fuselages.Fuselage() 
    fuselage.number_coach_seats                 = vehicle.number_of_passengers 
    fuselage.seats_abreast                      = 4
    fuselage.seat_pitch                         = 0.85    * Units.meter 
    fuselage.fineness.nose                      = 1.6
    fuselage.fineness.tail                      = 2. 
    fuselage.lengths.nose                       = 4.23    * Units.meter
    fuselage.lengths.tail                       = 7.62    * Units.meter
    fuselage.lengths.total                      = 29.68   * Units.meter # here  
    fuselage.lengths.fore_space                 = 2.37    * Units.meter
    fuselage.lengths.aft_space                  = 7.62    * Units.meter
    fuselage.width                              = 2.69    * Units.meter
    fuselage.heights.maximum                    = 2.69    * Units.meter
    fuselage.effective_diameter                 = 2.69    * Units.meter
    
    fuselage.areas.side_projected               = 67.43   * Units['meters**2'] 
    fuselage.areas.wetted                       = 216     * Units['meters**2'] 
    fuselage.areas.front_projected              = 22.73   * Units['meters**2']  
    fuselage.differential_pressure              = 5.0e4   * Units.pascal 
    fuselage.heights.at_quarter_length          = 2.69    * Units.meter
    fuselage.heights.at_three_quarters_length   = 2.69    * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = 2.69    * Units.meter

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_0'    
    segment.percent_x_location                  = 0.0000
    segment.percent_z_location                  = -0.002 
    segment.height                              = 0.0 
    segment.width                               = 0.0  
    fuselage.append_segment(segment)   
    
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_1'    
    segment.percent_x_location                  = 0.01 
    segment.percent_z_location                  = -0.00067 
    segment.height                              = 0.52 
    segment.width                               = 0.64  
    fuselage.append_segment(segment)       
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_2'    
    segment.percent_x_location                  = 0.02421 
    segment.percent_z_location                  = 0.001 
    segment.height                              = 0.9
    segment.width                               = 1.100
    fuselage.append_segment(segment)   
    
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_3'    
    segment.percent_x_location                  = 0.039610
    segment.percent_z_location                  = 0.00387
    segment.height                              = 1.3 
    segment.width                               = 1.6  
    fuselage.append_segment(segment)   
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.055 
    segment.percent_z_location                  = 0.007410 
    segment.height                              = 1.6 
    segment.width                               = 1.95 
    fuselage.append_segment(segment)      
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 0.08
    segment.percent_z_location                  = 0.01470000 
    segment.height                              = 2.15 
    segment.width                               = 2.35 
    fuselage.append_segment(segment)   

    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_6'    
    segment.percent_x_location                  = 0.10110
    segment.percent_z_location                  = 0.01949 
    segment.height                              = 2.46 
    segment.width                               = 2.56  
    fuselage.append_segment(segment)   

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.12223 	
    segment.percent_z_location                  = 0.022 
    segment.height                              = 2.60 
    segment.width                               = 2.66 
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 0.14266 
    segment.percent_z_location                  = 0.023 
    segment.height                              = 2.69 
    segment.width                               = 2.69 
    fuselage.append_segment(segment)     
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_9'   
    segment.percent_x_location                  = 0.75
    segment.percent_z_location                  = 0.023 
    segment.height                              = 2.69 
    segment.width                               = 2.69 
    fuselage.append_segment(segment)             
     
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_10'   
    segment.percent_x_location                  = 0.83 
    segment.percent_z_location                  = 0.027 
    segment.height                              = 2.30 
    segment.width                               = 2.35 
    fuselage.append_segment(segment)    
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_11'   
    segment.percent_x_location                  = 0.96674
    segment.percent_z_location                  = 0.0330 
    segment.height                              = 0.95 
    segment.width                               = 0.60 
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_12'     
    segment.percent_x_location                  = 1.0 
    segment.percent_z_location                  = 0.033
    segment.height                              = 0
    segment.width                               = 0
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
    net                                          = RCAIDE.Framework.Networks.Fuel() 
    
    #------------------------------------------------------------------------------------------------------------------------- 
    # Fuel Distrubition Line 
    #------------------------------------------------------------------------------------------------------------------------- 
    fuel_line                                    = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()  
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Starboard Propulsor CF34-8C
    #------------------------------------------------------------------------------------------------------------------------------------         
    turbofan                                       = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan() 
    turbofan.tag                                   = 'starboard_propulsor'
    turbofan.active_fuel_tanks                     = ['fuel_tank']   
    turbofan.origin                                = [[21.5, -2.2,1.45]]  
    turbofan.engine_length                         = 3.3     
    turbofan.bypass_ratio                          = 5    
    turbofan.design_altitude                       = 0.0*Units.ft
    turbofan.design_mach_number                    = 0.1   
    turbofan.design_thrust                         = 60000.0* Units.N 
             
    # fan                
    fan                                            = RCAIDE.Library.Components.Powertrain.Converters.Fan()   
    fan.tag                                        = 'fan'
    fan.polytropic_efficiency                      = 0.93
    fan.pressure_ratio                             = 1.7   
    turbofan.fan                                   = fan        
                   
    # working fluid                   
    turbofan.working_fluid                         = RCAIDE.Library.Attributes.Gases.Air() 
    ram                                            = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                        = 'ram' 
    turbofan.ram                                   = ram 
          
    # inlet nozzle          
    inlet_nozzle                                   = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                               = 'inlet nozzle'
    inlet_nozzle.polytropic_efficiency             = 0.98
    inlet_nozzle.pressure_ratio                    = 0.98 
    turbofan.inlet_nozzle                          = inlet_nozzle 

    # low pressure compressor    
    low_pressure_compressor                        = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    low_pressure_compressor.tag                    = 'lpc'
    low_pressure_compressor.polytropic_efficiency  = 0.91
    low_pressure_compressor.pressure_ratio         = 1.65   
    turbofan.low_pressure_compressor               = low_pressure_compressor

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
    combustor.turbine_inlet_temperature            = 1760
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
   
 
    # Nacelle updated for CRJ 
    nacelle                                       = RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle()
    nacelle.diameter                              = 1.55
    nacelle.length                                = 3.90
    nacelle.tag                                   = 'nacelle_1'
    nacelle.inlet_diameter                        = 1.30
    nacelle.origin                                = [[21.5, -2.2,1.45]] 
    nacelle.areas.wetted                          = 1.1*np.pi*nacelle.diameter*nacelle.length 
    nacelle_airfoil                               = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    nacelle_airfoil.NACA_4_Series_code            = '2410'
    nacelle.append_airfoil(nacelle_airfoil)  
    turbofan.nacelle                              = nacelle
    
    net.propulsors.append(turbofan)  

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Port Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------      
    # copy turbofan
    turbofan_2                                  = deepcopy(turbofan)
    turbofan_2.active_fuel_tanks                = ['fuel_tank'] 
    turbofan_2.tag                              = 'port_propulsor' 
    turbofan_2.origin                           = [[21.5, 2.2,1.45]]   
    turbofan_2.nacelle.origin                   = [[21.5,2.2,1.45]]
         
    # append propulsor to distribution line 
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
 
    # Append fuel line to Network      
    net.fuel_lines.append(fuel_line)   

    # Append energy network to aircraft 
    vehicle.append_energy_network(net)    
          
    return vehicle 

if __name__ == '__main__': 
    main() 

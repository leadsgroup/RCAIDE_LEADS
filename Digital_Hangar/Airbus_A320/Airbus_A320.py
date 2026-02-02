# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units   
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan             import design_turbofan      
from RCAIDE.Library.Plots                 import *      
import matplotlib.ticker as ticker
from RCAIDE.Library.Methods.Performance import *  
import matplotlib.cm as cm
from RCAIDE.Framework.External_Interfaces.OpenVSP import export_vsp_vehicle
from RCAIDE.Library.Methods.Performance.compute_load_and_trim_diagram        import compute_load_and_trim_diagram

# python imports 
import numpy as np  
from copy import deepcopy
import matplotlib.pyplot as plt  
from matplotlib.gridspec import GridSpec
import pickle
import os   

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main(plot_results=True, plot_vehicle=True):
    
    # Step 1 design a vehicle
    vehicle  = vehicle_setup()
    
    # Step 2 create aircraft configuration based on vehicle 
    configs  = configs_setup(vehicle)
    
    # Step 3 set up analysis
    analyses = analyses_setup(configs)
    
    # plot vehicle
    if plot_vehicle:
        plot_3d_vehicle(vehicle,  fuselage_opacity = 0.5,)
        
    return 
    

def vehicle_setup(): 
    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'Airbus_A320'

    # ################################################# Vehicle-level Properties #################################################   
    vehicle.mass_properties.max_takeoff               = 73500 * Units.kilogram    
    vehicle.mass_properties.takeoff                   = 70000 * Units.kilogram   
    vehicle.mass_properties.max_fuel                  = 18729 * Units.kilogram 
    vehicle.mass_properties.operating_empty           = 43144 * Units.kilogram
    vehicle.mass_properties.max_payload               = 18200  * Units.kilogram   
    vehicle.mass_properties.payload                   = 18000 * Units.kilogram    
    vehicle.mass_properties.fuel                      = 15000 * Units.kilogram
    vehicle.mass_properties.max_zero_fuel             = 60500 * Units.kilogram
    vehicle.mass_properties.center_of_gravity         = [[15.969, 0, 0]] 
    vehicle.flight_envelope.ultimate_load             = 3.5
    vehicle.flight_envelope.positive_limit_load       = 2.5
    vehicle.flight_envelope.design_mach_number        = 0.78 
    vehicle.flight_envelope.design_cruise_altitude    = 38000*Units.feet
    vehicle.flight_envelope.design_range              = 2500 * Units.nmi
    vehicle.reference_area                            = 126.44 * Units['meters**2']   
    vehicle.number_of_passengers                      = 150
    vehicle.systems.control                           = "fully powered" 
    vehicle.systems.accessories                       = "medium range"

    # ################################################# Landing Gear #############################################################   
    # ------------------------------------------------------------------        
    #  Landing Gear
    # ------------------------------------------------------------------  
    main_gear                   = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear() 
    main_gear.origin            = [[17.50,-3.8,-1.]]
    main_gear.tire_diameter     =  1.0 *  Units.inches 
    main_gear.tire_width        =  0.35 *  Units.inches 
    main_gear.strut_length      =  2.36 * Units.m  
    main_gear.wheels                         = 4  
    main_gear.number_of_gear_types_in_tandem  = 1
    main_gear.number_of_wheels_in_gear_type  = 2  
    main_gear.xz_plane_symmetric= True
    vehicle.append_component(main_gear)  

    nose_gear                   = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()   
    nose_gear.origin            = [[5.2,0,-1.4]]
    nose_gear.tire_diameter     = 0.8 * Units.inches  
    nose_gear.tire_width        = 0.25  *  Units.inches 
    nose_gear.strut_length      = 2.20  * Units.m  
    nose_gear.number_of_gear_types_in_tandem  = 1
    nose_gear.number_of_wheels_in_gear_type  = 2  
    main_gear.wheels                         = 2    
    vehicle.append_component(nose_gear)
     

    # ################################################# Wings ##################################################################### 
    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------
 
    wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing' 
    wing.aspect_ratio                     = 9.07
    wing.sweeps.leading_edge              = 26 * Units.deg
    wing.thickness_to_chord               = 0.1
    wing.taper                            = 0.222 
    wing.spans.projected                  = 33.87
    wing.chords.root                      = 7.12 * Units.meter
    wing.chords.tip                       = 1.58 * Units.meter
    wing.chords.mean_aerodynamic          = 4.05 * Units.meter 
    wing.areas.reference                  = 126.44
    wing.areas.wetted                     = 214.0  
    wing.origin                           = [[11.725,0,-0.459]]
    wing.aerodynamic_center               = [0,0,0] 
    wing.vertical                         = False
    wing.xz_plane_symmetric               = True
    wing.high_lift                        = True
    wing.twists.root                      = 2.5 * Units.degrees 
    wing.twists.tip                       = -4.0  * Units.degrees 

    # Wing Segments
    root_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    ospath                                = os.path.abspath(__file__)
    separator                             = os.path.sep
    rel_path                              = os.path.dirname(ospath) + separator + '..' + separator   
    root_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator + 'transonic_wing_root_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Root'
    segment.percent_span_location         = 0.0 
    segment.root_chord_percent            = 1.
    segment.dihedral_outboard             = 5.36 * Units.degrees
    segment.twist                         = 2.5   * Units.degrees
    segment.sweeps.leading_edge           = 27.796 * Units.degrees 
    segment.append_airfoil(root_airfoil)
    wing.append_segment(segment)

    yehudi_airfoil                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    yehudi_airfoil.coordinate_file        = rel_path+ 'Airfoils' + separator + 'transonic_wing_inboard_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Yehudi'
    segment.percent_span_location         = 0.382 
    segment.root_chord_percent            = 0.526
    segment.twist                         = 0.0  * Units.degrees
    segment.dihedral_outboard             = 5.5 * Units.degrees
    segment.sweeps.leading_edge           = 24.857 * Units.degrees  
    segment.append_airfoil(yehudi_airfoil)
    wing.append_segment(segment)

    mid_airfoil                           = RCAIDE.Library.Components.Airfoils.Airfoil()
    mid_airfoil.coordinate_file           = rel_path + 'Airfoils' + separator + 'transonic_wing_outboard_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Tip'
    segment.percent_span_location         = 1.0
    segment.root_chord_percent            = 0.222  
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.quarter_chord          = 0.0 * Units.degrees 
    segment.twist                         = -2 * Units.degrees
    segment.append_airfoil(mid_airfoil)
    wing.append_segment(segment)
    

    # control surfaces -------------------------------------------
    slat                          = RCAIDE.Library.Components.Wings.Control_Surfaces.Slat()
    slat.tag                      = 'slat'
    slat.span_fraction_start      = 0.14
    slat.span_fraction_end        = 0.96
    slat.deflection               = 0.0 * Units.degrees
    slat.chord_fraction           = 0.12
    wing.append_control_surface(slat)

    flap                          = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    flap.tag                      = 'flap'
    flap.span_fraction_start      = 0.11
    flap.span_fraction_end        = 0.77
    flap.deflection               = 0.0 * Units.degrees
    flap.configuration_type       = 'double_slotted'
    flap.chord_fraction           = 0.28
    wing.append_control_surface(flap)

    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.78
    aileron.span_fraction_end     = 0.97
    aileron.deflection            = 0.0 * Units.degrees
    aileron.chord_fraction        = 0.32
    wing.append_control_surface(aileron)

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing     = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag = 'horizontal_stabilizer'
    wing.aspect_ratio            = 4.83
    wing.sweeps.quarter_chord    = 28.2250 * Units.deg  
    wing.thickness_to_chord      = 0.08
    wing.taper                   = 0.30 
    wing.spans.projected         = 12.52 
    wing.chords.root             = 4.0
    wing.chords.tip              = 1.18 
    wing.chords.mean_aerodynamic = 2.572 
    wing.areas.reference         = 32.45
    wing.areas.exposed           = 30    
    wing.areas.wetted            = 62.0    
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees 
    wing.origin                  = [[31.381,0,1.302]]
    wing.aerodynamic_center      = [0,0,0]  
    wing.vertical                = False
    wing.xz_plane_symmetric      = True     


    # Wing Segments
    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'root_segment'
    segment.percent_span_location  = 0.0
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 1.0
    segment.dihedral_outboard      = 7.13 * Units.degrees
    segment.sweeps.quarter_chord   = 28.2250  * Units.degrees 
    segment.thickness_to_chord     = .14
    wing.append_segment(segment)

    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'tip_segment'
    segment.percent_span_location  = 1.
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 0.30              
    segment.dihedral_outboard      = 0 * Units.degrees
    segment.sweeps.quarter_chord   = 0 * Units.degrees  
    segment.thickness_to_chord     = .10
    wing.append_segment(segment)
        

    # control surfaces -------------------------------------------
    elevator                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
    elevator.tag                   = 'elevator'
    elevator.span_fraction_start   = 0.18
    elevator.span_fraction_end     = 1.0
    elevator.deflection            = 0.0  * Units.deg
    elevator.chord_fraction        = 0.32
    wing.append_control_surface(elevator)

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------
    wing = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag                              = 'vertical_stabilizer'
    wing.aspect_ratio                     = 1.54
    wing.sweeps.quarter_chord             = 35.0  * Units.deg   
    wing.thickness_to_chord               = 0.08
    wing.taper                            = 0.272 
    wing.spans.projected                  = 6.22
    wing.total_length                     = wing.spans.projected  
    wing.chords.root                      = 6.85
    wing.chords.tip                       = 1.87 
    wing.chords.mean_aerodynamic          = 4.80 
    wing.areas.reference                  = 23.38
    wing.areas.wetted                     = 46.0  
    wing.twists.root                      = 0.0 * Units.degrees
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[28.665,0,2.324]]
    wing.aerodynamic_center               = [0,0,0] 
    wing.vertical                         = True
    wing.xz_plane_symmetric               = False
    wing.t_tail                           = False 
    wing.dynamic_pressure_ratio           = 1.0
    wing.transition_x_upper               = 0.5
    wing.transition_x_lower               = 0.5

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tail_root'
    segment.percent_span_location         = 0.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 1.
    segment.dihedral_outboard             = 0 * Units.degrees
    segment.sweeps.quarter_chord          = 57.56  * Units.degrees  
    segment.thickness_to_chord            =.15
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tail_segment_1'
    segment.percent_span_location         = 0.131
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.7669
    segment.dihedral_outboard             = 0. * Units.degrees
    segment.sweeps.quarter_chord          = 35.271* Units.degrees   
    segment.thickness_to_chord            = .13
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tail_segment_2'
    segment.percent_span_location         = 1.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.273
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.quarter_chord          = 0.0    
    segment.thickness_to_chord            = .12 
    wing.append_segment(segment) 

    # control surfaces -------------------------------------------
    rudder                                = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    rudder.tag                            = 'rudder'
    rudder.span_fraction_start            = 0.08
    rudder.span_fraction_end              = 1.0
    rudder.deflection                     = 0.0  * Units.deg
    rudder.chord_fraction                 = 0.3
    wing.append_control_surface(rudder)

    # add to vehicle
    vehicle.append_component(wing)

    # ################################################# Fuselage ################################################################ 
    
    fuselage                                              = RCAIDE.Library.Components.Fuselages.Fuselage()  
    fuselage.seats_abreast                                = 6
    fuselage.seat_pitch                                   = 1     * Units.meter 
    fuselage.fineness.nose                                = 1.327
    fuselage.fineness.tail                                = 3.45 
    fuselage.lengths.nose                                 = 5.75   * Units.meter
    fuselage.lengths.tail                                 = 13.8   * Units.meter
    fuselage.lengths.total                                = 37.57 * Units.meter  
    fuselage.lengths.fore_space                           = 1.25    * Units.meter
    fuselage.lengths.aft_space                            = 6.    * Units.meter
    fuselage.width                                        = 4.12  * Units.meter
    fuselage.heights.maximum                              = 3.95  * Units.meter
    fuselage.effective_diameter                           = 4.0     * Units.meter
    fuselage.areas.side_projected                         = 132.132 * Units['meters**2'] 
    fuselage.areas.wetted                                 = 409.04  * Units['meters**2'] 
    fuselage.areas.front_projected                        = np.pi *(fuselage.effective_diameter **2) /4 
    fuselage.differential_pressure                        = 5.0e4 * Units.pascal 
    fuselage.heights.at_quarter_length                    = 3.95 * Units.meter
    fuselage.heights.at_three_quarters_length             = 3.95 * Units.meter
    fuselage.heights.at_wing_root_quarter_chord           = 3.95 * Units.meter
    
    cabin                                                 = RCAIDE.Library.Components.Fuselages.Cabins.Cabin()
    cabin.origin                                          = [[3.75, 0, 0]]
    cabin.offset_x = 3
    business_class                                        = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Business() 
    business_class.number_of_seats_abrest                 = 4
    business_class.number_of_rows                         = 4
    business_class.seat_pitch                             = 36 * Units.inches
    business_class.galley_lavatory_percent_x_locations    = [0]       
    business_class.type_A_exit_percent_x_locations        = [0.2]
    cabin.append_cabin_class(business_class) 
    
    economy_class = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest                  = 6
    economy_class.number_of_rows                          = 23
    economy_class.seat_pitch                              = 32 * Units.inches
    economy_class.galley_lavatory_percent_x_locations     = [1]      
    economy_class.emergency_exit_percent_x_locations      = [0.2,0.25] 
    economy_class.type_A_exit_percent_x_locations         = [0.99]
    cabin.append_cabin_class(economy_class)
    
    fuselage.append_cabin(cabin)          
    

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_0'    
    segment.percent_x_location                  = 0.0000
    segment.percent_z_location                  = -0.00144
    fuselage.append_segment(segment)   
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_1'    
    segment.percent_x_location                  = 0.00576 
    segment.percent_z_location                  = -0.00144 
    segment.height                              = 0.97
    segment.width                               = 0.87
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.02017 
    segment.percent_z_location                  = -0.00064
    segment.height                              = 1.694
    segment.width                               = 1.691 
    fuselage.append_segment(segment)      
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.03529 
    segment.percent_z_location                  = 0.00000 
    segment.height                              = 2.149 
    segment.width                               = 2.299 
    fuselage.append_segment(segment)   

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.04899 	
    segment.percent_z_location                  = 0.00533
    segment.height                              = 2.790 
    segment.width                               = 2.6633 
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 0.07143 
    segment.percent_z_location                  = 0.01066 
    segment.height                              = 3.466 
    segment.width                               = 3.21224 
    fuselage.append_segment(segment)     
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 0.10375 
    segment.percent_z_location                  = 0.01495 
    segment.height                              = 3.912 
    segment.width                               = 3.84 
    fuselage.append_segment(segment)             
     
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.14839 
    segment.percent_z_location                  = 0.01712 
    segment.height                              = 4.14 
    segment.width                               = 3.95 
    fuselage.append_segment(segment)    
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 0.6359 
    segment.percent_z_location                  = 0.01753 
    segment.height                              = 4.14 
    segment.width                               = 3.95 
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_9'     
    segment.percent_x_location                  = 0.743 
    segment.percent_z_location                  = 0.02337
    segment.height                              = 3.721
    segment.width                               = 3.95
    fuselage.append_segment(segment)     
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_10'     
    segment.percent_x_location                  = 0.853
    segment.percent_z_location                  = 0.03449
    segment.height                              = 2.729
    segment.width                               = 2.59776
    fuselage.append_segment(segment)   
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_11'     
    segment.percent_x_location                  = 0.99 
    segment.percent_z_location                  = 0.04747
    segment.height                              = 0.927
    segment.width                               = 0.661
    fuselage.append_segment(segment)    
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_12'     
    segment.percent_x_location                  = 1.0
    segment.percent_z_location                  = 0.0459
    segment.height                              = 0.0
    segment.width                               = 0.0
    fuselage.append_segment(segment)             
    
    # add to vehicle
    vehicle.append_component(fuselage)
     

    # ################################################# Energy Network #######################################################          
    #------------------------------------------------------------------------------------------------------------------------- 
    #  Turbofan Network
    #-------------------------------------------------------------------------------------------------------------------------   
    net                                         = RCAIDE.Framework.Networks.Fuel() 
    
    #------------------------------------------------------------------------------------------------------------------------- 
    # Fuel Distrubition Line 
    #------------------------------------------------------------------------------------------------------------------------- 
    fuel_line                                   = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()  
    #------------------------------------------------------------------------------------------------------------------------------------         
    turbofan                                       = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan() 
    turbofan.tag                                   = 'starboard_propulsor' 
    turbofan.origin                                = [[11.5, 5.75,-1.332]] 
    turbofan.length                                = 3.36     
    turbofan.bypass_ratio                          = 4.8          
    turbofan.diameter                              = 1.7
    turbofan.design_altitude                       = 40000.0*Units.ft
    turbofan.design_mach_number                    = 0.78   
    turbofan.design_thrust                         = 30000.0* Units.N 
                
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
    low_pressure_compressor.pressure_ratio         = 1.9   
    turbofan.low_pressure_compressor               = low_pressure_compressor

    # high pressure compressor  
    high_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    high_pressure_compressor.tag                   = 'hpc'
    high_pressure_compressor.polytropic_efficiency = 0.91
    high_pressure_compressor.pressure_ratio        = 10.34    
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
    combustor.turbine_inlet_temperature            = 1400
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
    nacelle                                        = RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle()
    nacelle.diameter                               = 1.7
    nacelle.length                                 = 3.36
    nacelle.tag                                    = 'nacelle_1'
    nacelle.inlet_diameter                         = 1.6
    nacelle.origin                                 = [[11.5, 5.75,-1.332]] 
    nacelle.areas.wetted                           = 1.1*np.pi*nacelle.diameter*nacelle.length 
    nacelle_airfoil                                = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    nacelle_airfoil.NACA_4_Series_code             = '2410'
    nacelle.append_airfoil(nacelle_airfoil)  
    turbofan.nacelle                               = nacelle

    # append propulsor to network    
    net.propulsors.append(turbofan)  

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Port Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    turbofan_2                                  = deepcopy(turbofan) 
    turbofan_2.tag                              = 'port_propulsor' 
    turbofan_2.origin                           = [[11.5, -5.75,-1.332]]  
    turbofan_2.nacelle.origin                   = [[11.5, -5.75,-1.332]] 
         
    # append propulsor to network
    net.propulsors.append(turbofan_2)
  
    #------------------------------------------------------------------------------------------------------------------------- 
    #  Energy Source: Fuel Tank
    #-------------------------------------------------------------------------------------------------------------------------  
    inboard_tank                                = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    inboard_tank.fuel                           = RCAIDE.Library.Attributes.Propellants.Jet_A()
    inboard_tank.segments_bounding_tank         = ['root','yehudi']  
    inboard_tank.segments_percent_chord_start   = [0.1 ,0.1 ]
    inboard_tank.segments_percent_chord_end     = [0.8   ,0.7]   
    fuel_line.fuel_tanks.append(inboard_tank)
    
    outboard_tank                               = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    outboard_tank.fuel                          = RCAIDE.Library.Attributes.Propellants.Jet_A()
    outboard_tank.segments_bounding_tank        = ['yehudi', 'tip'] 
    outboard_tank.segments_percent_chord_start  = [0.1 ,0.1 ]
    outboard_tank.segments_percent_chord_end    = [0.7   ,0.6]   
    fuel_line.fuel_tanks.append(outboard_tank)    
    
    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to fuel line to network      
    fuel_line.assigned_propulsors =  [[turbofan.tag, turbofan_2.tag]]

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Append fuel line to fuel line to network      
    net.fuel_lines.append(fuel_line)        
    
    # Append energy network to aircraft 
    vehicle.append_energy_network(net)
    
    #------------------------------------------------------------------------------------------------------------------------- 
    # Done ! 
    #-------------------------------------------------------------------------------------------------------------------------  
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

    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------ 
    config = RCAIDE.Library.Components.Configs.Config(vehicle)
    config.tag = 'idle' 
    for landing_gear in  config.landing_gears:
        landing_gear.gear_extended = True     
    configs.append(config) 
    
    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'cruise'
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
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 30. * Units.deg 
    config.networks.fuel.propulsors['starboard_propulsor'].fan.angular_velocity      =  3470. * Units.rpm
    config.networks.fuel.propulsors['port_propulsor'].fan.angular_velocity      =  3470. * Units.rpm       
    config.networks.fuel.propulsors['starboard_propulsor'].fan.rotation              = 3470. 
    config.networks.fuel.propulsors['port_propulsor'].fan.rotation              = 3470. 
    config.networks.fuel.propulsors['starboard_propulsor'].fan_nozzle.noise_speed    = 315.
    config.networks.fuel.propulsors['port_propulsor'].fan_nozzle.noise_speed    = 315.
    config.networks.fuel.propulsors['starboard_propulsor'].core_nozzle.noise_speed   = 415.
    config.networks.fuel.propulsors['port_propulsor'].core_nozzle.noise_speed   = 415. 
    for landing_gear in  config.landing_gears:
        landing_gear.gear_extended = True
    
    config.V2_VS_ratio = 1.21
    configs.append(config)


    # ------------------------------------------------------------------
    #   Cutback Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'cutback'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 20. * Units.deg
    config.networks.fuel.propulsors['starboard_propulsor'].fan.angular_velocity    =  2780. * Units.rpm
    config.networks.fuel.propulsors['port_propulsor'].fan.angular_velocity    =  2780. * Units.rpm        
    config.networks.fuel.propulsors['starboard_propulsor'].fan.rotation            = 2780.  
    config.networks.fuel.propulsors['port_propulsor'].fan.rotation            = 2780. 
    config.networks.fuel.propulsors['starboard_propulsor'].fan_nozzle.noise_speed  = 210.
    config.networks.fuel.propulsors['port_propulsor'].fan_nozzle.noise_speed  = 210.
    config.networks.fuel.propulsors['starboard_propulsor'].core_nozzle.noise_speed = 360.
    config.networks.fuel.propulsors['port_propulsor'].core_nozzle.noise_speed = 360.
    for landing_gear in  config.landing_gears:
        landing_gear.gear_extended = True
    
    configs.append(config)   



    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'landing'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 30. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg
    config.networks.fuel.propulsors['starboard_propulsor'].fan.angular_velocity =  2780. * Units.rpm
    config.networks.fuel.propulsors['port_propulsor'].fan.angular_velocity      =  2780. * Units.rpm     
    config.networks.fuel.propulsors['starboard_propulsor'].fan.rotation            = 2780.  
    config.networks.fuel.propulsors['port_propulsor'].fan.rotation            = 2780. 
    config.networks.fuel.propulsors['starboard_propulsor'].core_nozzle.noise_speed = 92.
    config.networks.fuel.propulsors['port_propulsor'].core_nozzle.noise_speed = 92.
    config.networks.fuel.propulsors['starboard_propulsor'].fan_nozzle.noise_speed  = 109.3
    config.networks.fuel.propulsors['port_propulsor'].fan_nozzle.noise_speed  = 109.3 
    for landing_gear in  config.landing_gears:
        landing_gear.gear_extended = True 
    config.Vref_VS_ratio = 1.23
    configs.append(config)   

    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------ 

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'short_field_takeoff'    
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg
    config.networks.fuel.propulsors['starboard_propulsor'].fan.angular_velocity =  3470. * Units.rpm
    config.networks.fuel.propulsors['port_propulsor'].fan.angular_velocity      =  3470. * Units.rpm 
    for landing_gear in  config.landing_gears:
        landing_gear.gear_extended = True
    config.V2_VS_ratio = 1.21 
    configs.append(config)

    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------  

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'reverse_thrust'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 30. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg 
    for landing_gear in  config.landing_gears:
        landing_gear.gear_extended = True
    configs.append(config)
    
    return configs
# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def analyses_setup(configs):
    """Set up analyses for each of the different configurations."""

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

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
    
    #  Geometry
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry() 
    analyses.append(geometry)
    
    # ------------------------------------------------------------------
    #  Weights 
    weights         = RCAIDE.Framework.Analyses.Weights.Conventional_Transport()
    weights.settings.update_center_of_gravity                      = True
    weights.settings.update_moment_of_inertia                      = True 
    weights.settings.weight_correction_additions.empty.structural.paint      = 250
    weights.settings.weight_correction_additions.operational_items.ETOPS     = 7.7 * vehicle.number_of_passengers
    weights.settings.update_mass_properties                = True
    weights.settings.FLOPS.fidelity        = 'Complex' 
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis 
    aerodynamics          = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.settings.number_of_spanwise_vortices   = 40
    aerodynamics.settings.number_of_chordwise_vortices  = 2 
    aerodynamics.training.Mach                          = np.array([0.1  ,0.3,  0.5,  0.65 , 0.85 , 0.9])    
    analyses.append(aerodynamics)   

    # ------------------------------------------------------------------
    #  Energy
    energy          = RCAIDE.Framework.Analyses.Energy.Energy() 
    analyses.append(energy)
    
    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)   

    # done!
    return analyses  


if __name__ == '__main__': 
    main()
    plt.show()
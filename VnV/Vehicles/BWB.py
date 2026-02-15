# Boeing_BWB_450.py 
""" setup file for the BWB vehicle
"""
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units, Data       
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan      import design_turbofan    
from RCAIDE.Library.Plots                                       import *     
 
# python imports 
import numpy as np  
from copy import deepcopy 
import os

# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------


def vehicle_setup():
    ospath                                = os.path.abspath(__file__)
    separator                             = os.path.sep
    rel_path                              = os.path.dirname(ospath) + separator + 'Airfoils'+ separator
    

    ## ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------      
    vehicle                                           = RCAIDE.Vehicle()    
    vehicle.tag                                       = 'BWB' 
    vehicle.mass_properties.max_takeoff               = 280000. * Units.lbs 
    vehicle.mass_properties.payload                   = 67000. * Units.lbs 
    vehicle.mass_properties.fuel                      = 55000  
    vehicle.mass_properties.max_fuel                  = 100000  
    vehicle.mass_properties.takeoff                   = 280000 * Units.lbs
    vehicle.mass_properties.max_zero_fuel             = 206000 * Units.lbs 
    vehicle.mass_properties.max_payload               = 69600.  * Units.lb 
    vehicle.mass_properties.center_of_gravity         = [[27.0, 0, 0]]
    vehicle.flight_envelope.ultimate_load             = 3.75 
    vehicle.flight_envelope.positive_limit_load       = 2.5  
    vehicle.flight_envelope.design_mach_number        = 0.85  
    vehicle.flight_envelope.design_cruise_altitude    = 45000.0*Units.feet 
    vehicle.flight_envelope.design_range              = 5000.0 * Units.nmi
    vehicle.reference_area                            = 592.6575476422672 # 2424.9 * Units['feet**2']    
    vehicle.number_of_passengers                      = 248 # Single class. 242 in dual class (24 business, 21 economy) 
    vehicle.systems.control                           = "fully powered" 
    vehicle.systems.accessories                       = "long range"  
     
    # ################################################# Landing Gear #############################################################   
    # ------------------------------------------------------------------        
    #  Landing Gear 
    # ------------------------------------------------------------------  
    main_gear               = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear()
    main_gear.tire_diameter = 50.0 * Units.inches
    main_gear.strut_length  = 20.0 * Units.ft 
    main_gear.units         = 2    # Number of main landing gear
    main_gear.wheels        = 4    # Number of wheels on the main landing gear
    main_gear.origin        = [[20.95,0., -2 ]]  # positioned using 30 degree (15 ground strike angle + 15 degree airfoil closeout angle )
    vehicle.append_component(main_gear)  

    nose_gear               = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()       
    nose_gear.tire_diameter = 40. * Units.inches
    nose_gear.units         = 1    # Number of nose landing gear
    nose_gear.wheels        = 2    # Number of wheels on the nose landing gear
    nose_gear.strut_length  = 20.0 * Units.ft 
    nose_gear.origin        = [[3. ,0., -2 ]] 
    vehicle.append_component(nose_gear)
         

    # ------------------------------------------------------------------
    # Carbo Bays 
    # ------------------------------------------------------------------ 
    cargo_bay = RCAIDE.Library.Components.Cargo_Bays.Cargo_Bay()
    cargo_bay.origin  = [[5, 0, -1]]
    cargo_bay.length  = 10
    cargo_bay.width   = 3
    cargo_bay.height  = 1
    vehicle.append_component(cargo_bay) 
    
    # ------------------------------------------------------------------
    #  Main Wing 
    # ------------------------------------------------------------------ 
    
    wing = RCAIDE.Library.Components.Wings.Blended_Wing_Body()
    wing.tag = 'main_wing' 
    wing.aspect_ratio            = 5.32528141979797 
    wing.sweeps.quarter_chord    = 0.6859590736162121 
    wing.thickness_to_chord      = 0.1351757396035437 
    wing.taper                   = 0.05215045887445882  
    wing.spans.projected         = 64.0
    wing.areas.reference         = 592.6575476422672 
    wing.areas.wetted            = 1282.0437685524962 
    wing.chords.mean_aerodynamic = 20.059555133618403 
    wing.chords.root             = 36  
    wing.chords.tip              = 1.8359256146144747   
    wing.total_length            = 32.4
    wing.twists.root             = 0.0 
    wing.twists.tip              = 0.0  
    wing.origin                  = [[0.0,  0.0,  0.0]] 
    wing.aerodynamic_center      = [17.43511294,  0.        ,  1.08931241] 
    wing.vertical                = False
    wing.xz_plane_symmetric      = True
    wing.t_tail                  = False 
    wing.dynamic_pressure_ratio  = 1.0  
    wing.aft_center_body.origin  = [[wing.chords.root - 2/3 * wing.aft_center_body.length,0,0]]    
    wing.aft_center_body.length  = 9.021    
    wing.aft_center_body.taper   = 0.85    
    wing.center_body.origin      = [[0.5*(wing.chords.root - wing.aft_center_body.length),0,0]]
     
    cabin                                              = RCAIDE.Library.Components.Fuselages.Cabins.Cabin()
    cabin.origin                                       = [[2.5, 0, 0]]
    business_class                                     = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Business() 
    business_class.number_of_seats_abrest              = 4
    business_class.number_of_rows                      = 4
    business_class.galley_lavatory_percent_x_locations = [0] 
    business_class.seat_arm_rest_width                 = 4 *  Units.inches 
    business_class.seat_width                          = 25 *  Units.inches
    business_class.aisle_width                         = 15  *  Units.inches 
    business_class.type_A_exit_percent_x_locations     = [0,0]
    cabin.append_cabin_class(business_class)  

    economy_class                                     = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest              = 6
    economy_class.number_of_rows                      = 12
    economy_class.galley_lavatory_percent_x_locations = [0,1.0]       
    economy_class.type_A_exit_percent_x_locations     = [0, 1.0]
    cabin.append_cabin_class(economy_class)
    wing.append_cabin(cabin)  

    side_cabin                                             = RCAIDE.Library.Components.Fuselages.Cabins.Side_Cabin()
    side_cabin.nose.fineness_ratio                         = 1.75     
    business_class = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Business() 
    business_class.number_of_seats_abrest                = 4
    business_class.number_of_rows                        = 4
    business_class.galley_lavatory_percent_x_locations   = [0] 
    business_class.seat_arm_rest_width                   = 4 *  Units.inches 
    business_class.seat_width                            = 30 *  Units.inches
    business_class.aisle_width                            = 15  *  Units.inches  
    business_class.type_A_exit_percent_x_locations       = [0,0]
    side_cabin.append_cabin_class(business_class)
    
    side_economy_class                                     = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    side_economy_class.number_of_seats_abrest              = 6
    side_economy_class.number_of_rows                      = 12
    side_economy_class.galley_lavatory_percent_x_locations = [0,1.0] 
    side_economy_class.type_A_exit_percent_x_locations     = [0, 1.0]
    side_cabin.append_cabin_class(side_economy_class) 
    wing.append_cabin(side_cabin)

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment()
    segment.tag                           = 'Fuselage_Section_1'
    segment.taper                         = 0.8769841298701299
    segment.twist                         = 0.0 
    segment.percent_span_location         = 0.0  
    segment.root_chord_percent            = 1.0 
    segment.dihedral_outboard             = 2  *  Units.degrees 
    segment.sweeps.quarter_chord          = 10.037 *  Units.degrees 
    airfoil                               = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file               = rel_path + 's1016.txt' 
    segment.append_airfoil(airfoil )
    wing.append_segment(segment)


    segment                               = RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment()
    segment.tag                           = 'Fuselage_Section_2'
    segment.taper                         = 0.32260442862265637  
    segment.twist                         = 0.0   
    segment.percent_span_location         = ((0.66 * 2)/wing.spans.projected)  
    segment.root_chord_percent            = 0.9923542105
    segment.dihedral_outboard             = 5 *  Units.degrees  
    segment.thickness_to_chord            = 0.16
    segment.sweeps.quarter_chord           = 46.9023 *  Units.degrees  
    airfoil                               = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file               = rel_path +  's1016.txt'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment)
    
    
    segment                               = RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment()
    segment.tag                           = 'Fuselage_Section_3'
    segment.taper                         = 0.32260442862265637  
    segment.twist                         = 0.0   
    segment.percent_span_location         = ((1.9 * 2)/wing.spans.projected)  
    segment.root_chord_percent            = 0.9375928
    segment.dihedral_outboard             = 10.5 *  Units.degrees  
    segment.thickness_to_chord            = 0.16
    segment.sweeps.quarter_chord          = 51.027  *  Units.degrees  
    airfoil                               = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file               = rel_path +  's1016.txt'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment()
    segment.tag                           = 'Cabin_Wall'
    segment.taper                         = 0.32260442862265637  
    segment.twist                         = 0.0   
    segment.percent_span_location         = ((6.172 * 2)/wing.spans.projected)   
    segment.root_chord_percent            = 0.718797
    segment.dihedral_outboard             = 10 *  Units.degrees  
    segment.thickness_to_chord            = 0.11
    segment.sweeps.quarter_chord          = 45  *  Units.degrees   
    airfoil                               = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file               = rel_path +  's1016.txt'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment) 

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Fuel_Wall'
    segment.taper                         = 0.32260442862265637  
    segment.twist                         = 0.0   
    segment.percent_span_location         = 0.224625
    segment.root_chord_percent            = 0.636
    segment.dihedral_outboard             = 15 *  Units.degrees  
    segment.thickness_to_chord            = 0.1
    segment.sweeps.quarter_chord          = 25*Units.degrees  
    airfoil                               = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file               = rel_path +  's1016.txt'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment)       

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Wing_Section_1'
    segment.taper                         = 0.5151141328419794
    segment.twist                         = 0.0
    segment.percent_span_location         = 0.3346858066654701
    segment.root_chord_percent            =  0.2768
    segment.dihedral_outboard             = 1 *  Units.degrees
    segment.thickness_to_chord            = 0.15
    segment.sweeps.quarter_chord          = 30.*  Units.degrees  
    airfoil                               =  RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file               =  rel_path + 's1016.txt'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment)


    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Wing_Section_2'
    segment.taper                         = 0.6452627873428087
    segment.twist                         = 0.0
    segment.percent_span_location         = 0.5389354883954404
    segment.root_chord_percent            = 0.123
    segment.dihedral_outboard             = 1 *  Units.degrees
    segment.thickness_to_chord            = 0.1154 
    segment.sweeps.quarter_chord          = 30.*  Units.degrees 
    segment.chords.reference_area_root    = True
    airfoil                               =  RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file               =  rel_path + 's1016.txt'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment)  

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Wing_Section_3'
    segment.taper                         = 0.8999999162104734 
    segment.twist                         = 0.0 
    segment.percent_span_location         = 0.98
    segment.root_chord_percent            = 0.052
    segment.dihedral_outboard             = 65 *  Units.degrees 
    segment.thickness_to_chord            = 0.0972 
    segment.sweeps.quarter_chord          = 55 *  Units.degrees 
    airfoil                               =  RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file               =  rel_path + 's1016.txt'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment)  

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Wing_Section_4'
    segment.taper                         = 0.0 
    segment.twist                         = 0.0 
    segment.percent_span_location         = 1.0 
    segment.root_chord_percent            = 0.035 
    segment.dihedral_outboard             = 0 
    segment.thickness_to_chord            = 0.098 
    segment.sweeps.quarter_chord          = 0.0 
    airfoil                               =  RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file               =  rel_path + 's1016.txt'
    segment.append_airfoil(airfoil )
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
    flap.chord_fraction           = 0.14
    wing.append_control_surface(flap)

    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.7
    aileron.span_fraction_end     = 0.963
    aileron.deflection            = 0.0 * Units.degrees
    aileron.chord_fraction        = 0.25
    wing.append_control_surface(aileron)
        
    spoiler                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Spoiler()
    spoiler.tag                   = 'spoiler'
    spoiler.span_fraction_start   = 0.3
    spoiler.span_fraction_end     = 0.7
    spoiler.deflection            = 0.0 * Units.degrees
    spoiler.chord_fraction        = 0.05
    wing.append_control_surface(spoiler)    
    # add to vehicle
    vehicle.append_component(wing)    
    
    
    # ------------------------------------------------------------------
    # Vertical Stabilizer 
    # ------------------------------------------------------------------ 
    wing = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag = 'vertical_stabilizer' 
    wing.aspect_ratio            = 1.1  
    wing.thickness_to_chord      = 0.08 
    wing.taper                   = 0.35 
    wing.areas.reference         = 20
    wing.sweeps.quarter_chord    = 45 * Units.degrees  
    wing.spans.projected         = np.sqrt(wing.areas.reference  * wing.aspect_ratio)     
    wing.areas.wetted            = wing.spans.projected * 2.1
    wing.chords.root             = (2 *wing.areas.reference  / wing.spans.projected ) /(wing.taper +1)    
    wing.chords.tip              = wing.taper *   wing.chords.root                           
    wing.chords.mean_aerodynamic = wing.chords.root * 2/3 * (( 1 + wing.taper + wing.taper**2 ) / ( 1 + wing.taper )) 
    wing.total_length            = wing.chords.root 
    wing.twists.root             = 0.0 
    wing.twists.tip              = 0.0
    wing.dihedral                = 70 * Units.degrees 
    wing.origin                  = [[27 ,  6.4008    ,  1.39714742]]
    wing.aerodynamic_center      = [3.489773550255852, 0, 2.1541732160090175] 
    wing.vertical                = False
    wing.xz_plane_symmetric      = True
    wing.t_tail                  = False 
    wing.dynamic_pressure_ratio  = 1.0 
    

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Section_6'
    segment.taper                         = 1
    segment.twist                         = 0.0 
    segment.percent_span_location         = 0
    segment.root_chord_percent            = 1.0
    segment.dihedral_outboard             = 70 * Units.degrees 
    segment.thickness_to_chord            = 0.1
    segment.sweeps.quarter_chord           = 45 * Units.degrees  
    wing.append_segment(segment)  

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Section_9'
    segment.taper                         = 0.0 
    segment.twist                         = 0.0 
    segment.percent_span_location         = 1.0 
    segment.root_chord_percent            = 0.35 
    segment.sweeps.quarter_chord           = 0.0 
    segment.dihedral_outboard             = 0 
    segment.thickness_to_chord            = 0.1 
    wing.append_segment(segment)
    
    # add to vehicle
    vehicle.append_component(wing)      

    # ################################################# Landing Gear #############################################################   

    main_gear                                = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear() 
    main_gear.tire_diameter                  = 50.0 *  Units.inches 
    main_gear.rim_diameter                   = 22   *  Units.inches 
    main_gear.tire_width                     = 20.0 *  Units.inches 
    main_gear.strut_length                   = 5.5  * Units.ft 
    main_gear.wheels                         = 8   
    main_gear.number_of_gear_types_in_tandem = 2
    main_gear.number_of_wheels_in_gear_type  = 2  
    main_gear.xz_plane_symmetric             = True
    vehicle.append_component(main_gear)  

    nose_gear                                = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()   
    nose_gear.tire_diameter                  = 40. *  Units.inches   
    nose_gear.rim_diameter                   = 16  *  Units.inches 
    nose_gear.tire_width                     = 16  *  Units.inches 
    nose_gear.strut_length                   = 9.0 * Units.ft 
    nose_gear.wheels                         = 2   
    nose_gear.number_of_gear_types_in_tandem = 1
    nose_gear.number_of_wheels_in_gear_type  = 2    
    vehicle.append_component(nose_gear)
    
    
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
    # Propulsor: Center Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------         
    turbofan1                                    = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan()  
    turbofan1.origin                             = [[30, 3, 3]] 
    turbofan1.tag                                = 'propulsor_1'    
    turbofan1.length                             = 3.5                   
    turbofan1.diameter                           = 2.6  # 3.556                     
    turbofan1.bypass_ratio                       = 9.1                        
    turbofan1.design_altitude                    = 40000*Units.ft             
    turbofan1.design_mach_number                 = 0.7                       
    turbofan1.design_thrust                      = 55000* Units.N 
    turbofan1.specific_fuel_consumption_reduction_factor  = 0.1 
    turbofan1.wing_mounted                        = False

    # working fluid                   
    turbofan1.working_fluid                      = RCAIDE.Library.Attributes.Gases.Air() 
    
    # Ram inlet 
    ram                                          = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                      = 'ram' 
    turbofan1.ram                                = ram 
            
    # inlet nozzle          
    inlet_nozzle                                = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                            = 'inlet nozzle'
    inlet_nozzle.polytropic_efficiency          = 0.97                                       
    inlet_nozzle.pressure_ratio                 = 1
    inlet_nozzle.compressibility_effects        = False
    turbofan1.inlet_nozzle                       = inlet_nozzle
    
    # fan                
    fan                                         = RCAIDE.Library.Components.Powertrain.Converters.Fan()   
    fan.tag                                     = 'fan'
    fan.polytropic_efficiency                   = 0.91                   
    fan.pressure_ratio                          = 1.4                    
    turbofan1.fan                                = fan        

    # low pressure compressor    
    low_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    low_pressure_compressor.tag                   = 'lpc'
    low_pressure_compressor.polytropic_efficiency = 0.91                  
    low_pressure_compressor.pressure_ratio        = 1.3                     
    turbofan1.low_pressure_compressor              = low_pressure_compressor

    # high pressure compressor  
    high_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    high_pressure_compressor.tag                   = 'hpc'
    high_pressure_compressor.polytropic_efficiency = 0.91                        
    high_pressure_compressor.pressure_ratio        = 23.9                    
    turbofan1.high_pressure_compressor              = high_pressure_compressor

    # low pressure turbine  
    low_pressure_turbine                           = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    low_pressure_turbine.tag                       ='lpt'
    low_pressure_turbine.mechanical_efficiency     = 0.99                     
    low_pressure_turbine.polytropic_efficiency     = 0.93                     
    turbofan1.low_pressure_turbine                  = low_pressure_turbine
    
    # high pressure turbine     
    high_pressure_turbine                          = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    high_pressure_turbine.tag                      ='hpt'
    high_pressure_turbine.mechanical_efficiency    = 0.99                     
    high_pressure_turbine.polytropic_efficiency    = 0.93                     
    turbofan1.high_pressure_turbine                 = high_pressure_turbine 

    # combustor  
    combustor                                      = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                                  = 'Comb'
    combustor.efficiency                           = 0.997                    
    combustor.turbine_inlet_temperature            = 1400                  
    combustor.pressure_ratio                       = 0.94                     
    combustor.fuel_data                            = RCAIDE.Library.Attributes.Propellants.Jet_A()  
    turbofan1.combustor                             = combustor

    # core nozzle
    core_nozzle                                    = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                = 'core nozzle'
    core_nozzle.polytropic_efficiency              = 0.98                     
    core_nozzle.pressure_ratio                     = 0.995
    core_nozzle.diameter                           = 1.5  # may be incorrect 
    turbofan1.core_nozzle                          = core_nozzle
        
    # fan nozzle             
    fan_nozzle                                     = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    fan_nozzle.tag                                 = 'fan nozzle'
    fan_nozzle.polytropic_efficiency               = 0.98                    
    fan_nozzle.pressure_ratio                      = 0.995 
    fan_nozzle.diameter                            = 2.822 # may be incorrect         
    turbofan1.fan_nozzle                           = fan_nozzle
    
    # design turbofan
    design_turbofan(turbofan1)

    # Nacelle 
    nacelle                                     = RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle()
    nacelle.diameter                            = 2.6
    nacelle.length                              = 3.5
    nacelle.tag                                 = 'nacelle_1'
    nacelle.inlet_diameter                      = 2.5
    nacelle.origin                              = [[30, 3, 3]] 
    nacelle.areas.wetted                        = np.pi*nacelle.diameter*nacelle.length
    nacelle_airfoil                             = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    nacelle_airfoil.NACA_4_Series_code          = '3409'
    nacelle.append_airfoil(nacelle_airfoil) 
    turbofan1.nacelle                            = nacelle
    
    net.propulsors.append(turbofan1)

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Propulsor 2 (Inner Port Side)
    #------------------------------------------------------------------------------------------------------------------------------------       
    turbofan2                                  = deepcopy(turbofan1)
    turbofan2.active_fuel_tanks                = ['fuel_tank'] 
    turbofan2.tag                              = 'propulsor_2' 
    turbofan2.origin                           = [[30, -3, 3]] 
    turbofan2.nacelle.tag                      = 'propulsor_2_nacelle'
    turbofan2.nacelle.origin                   = [[30, -3, 3]] 
        
    # append propulsor to distribution line 
    net.propulsors.append(turbofan2) 

    #------------------------------------------------------------------------------------------------------------------------- 
    #  Energy Source: Fuel Tank
    #-------------------------------------------------------------------------------------------------------------------------  
    # fuel tank
    fuel_tank_1                                                = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank(vehicle.wings.main_wing)
    fuel_tank_1.tag                                            = 'H2_Fuel_Tank_1' 
    fuel_tank_1.fuel                                           = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()   
    fuel_tank_1.material                                       = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_1.insulation_material                            = RCAIDE.Library.Attributes.Materials.Vacuum_Jacketed_Multilayer_Insulation()
    fuel_tank_1.fuel.gravimetric_efficiency                    = 0.5
    fuel_tank_1.segments_bounding_tank                         = ['fuel_wall', 'wing_section_1']  
    fuel_tank_1.segments_percent_chord_start                   = [0.2,0.2] 
    fuel_tank_1.segments_percent_chord_end                     = [0.6,0.6]  
    fuel_line.fuel_tanks.append(fuel_tank_1)
    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to fuel line to network      
    fuel_line.assigned_propulsors =  [['propulsor_1', 'propulsor_2']]

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Append fuel line to fuel line to network      
    net.fuel_lines.append(fuel_line)         

    # Append energy network to aircraft 
    vehicle.append_energy_network(net)   
 
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
    base_config.tag = 'cruise'  
    configs.append(base_config)


    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------  

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'reverse_thrust' 
    config.networks.fuel.reverse_thrust             = True    
    for landing_gear in  config.landing_gears:
        landing_gear.gear_extended = True 
    configs.append(config)    
  
    return configs

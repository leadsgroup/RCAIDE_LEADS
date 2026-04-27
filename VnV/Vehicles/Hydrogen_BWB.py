""" setup file for the BWB vehicle
"""
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units, Data       
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan      import design_turbofan    
from RCAIDE.Library.Methods.Powertrain.Converters.Pump          import design_pump
from RCAIDE.Library.Plots                                       import *     
 
# python imports 
import numpy as np  
from copy import deepcopy 
import os

# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------


def vehicle_setup(MTOW = 125225.92487939,
                  span_input         = 64, 
                  root_chord         = 34.590950446109, 
                  vertical_chord     = 3.33984778055753, 
                  vertical_height    = 7.90551686864603, 
                  fuel_wall_sweep    = 0.746391195462114, 
                  fuel_wall_chord    = 0.570169900478341, 
                  fuel_wall_dihedral = 0.0238583066378036, 
                  section1_sweep     = 0.622074639446857, 
                  section1_dihedral  = 0.0388862791060358, 
                  section1_chord     = 0.320234286874057, 
                  section_1_span_percent = 0.3347,
                  root_twist         = 0.05, 
                  tip_twist          = -0.0174894184557103) : 
    
    ospath                                = os.path.abspath(__file__)
    rel_path                              = os.path.dirname(ospath) + os.path.sep  + 'Airfoils' + os.path.sep
     
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------      
    vehicle                                           = RCAIDE.Vehicle()    
    vehicle.tag                                       = 'BWB_2050_LH2' 
    vehicle.staub_factor = 0.955
    vehicle.mass_properties.max_takeoff               = MTOW  
    vehicle.mass_properties.takeoff                   = MTOW  
    vehicle.mass_properties.payload                   = 55800* Units.lbs 
    vehicle.mass_properties.max_payload               = 66960.  * Units.lb    
    vehicle.mass_properties.min_payload               = 43200.  * Units.lb   
    vehicle.flight_envelope.ultimate_load             = 3.75 
    vehicle.flight_envelope.positive_limit_load       = 2.5  
    vehicle.flight_envelope.design_mach_number        = 0.85  
    vehicle.flight_envelope.design_cruise_altitude    = 40000.0*Units.feet 
    vehicle.flight_envelope.design_range              = 5000.0 * Units.nmi
    vehicle.reference_area                            = 298.94 * Units.m**2      
    vehicle.number_of_passengers                      = 248 
    vehicle.systems.control                           = "fully powered" 
    vehicle.systems.accessories                       = "long range"
    vehicle.neutral_point                             = 400. # PLACEHOLDER
    
    # ------------------------------------------------------------------
    # Carbo Bays 
    # ------------------------------------------------------------------ 
    cargo_bay1 = RCAIDE.Library.Components.Cargo_Bays.Cargo_Bay()
    cargo_bay1.origin = [[10,0,-1]]
    cargo_bay1.length = 12
    cargo_bay1.width = 3.25
    cargo_bay1.height = 4.75 * Units.feet 
    vehicle.append_component(cargo_bay1)
    
    cargo_bay2 = RCAIDE.Library.Components.Cargo_Bays.Cargo_Bay()
    #cargo_bay2.mass_properties.mass =  1100  
    cargo_bay2.origin = [[13,-2.5,-1]]
    cargo_bay2.length = 9
    cargo_bay2.width = 1.75
    cargo_bay2.height = 4.75 * Units.feet 
    vehicle.append_component(cargo_bay2)
    
    cargo_bay3 = RCAIDE.Library.Components.Cargo_Bays.Cargo_Bay()
    cargo_bay3.origin = [[13,2.5,-1]]
    cargo_bay3.length = 9
    cargo_bay3.width = 1.75
    cargo_bay3.height = 4.75 * Units.feet

    vehicle.append_component(cargo_bay3)

    
    # ------------------------------------------------------------------
    #  Main Wing 
    # ------------------------------------------------------------------ 
    
    wing = RCAIDE.Library.Components.Wings.Blended_Wing_Body()
    wing.tag = 'main_wing' 
    wing.aspect_ratio            = 5.7178477994757815 
    wing.sweeps.quarter_chord    = 0.6562637381290588 
    wing.thickness_to_chord      = 0.12823921650789133
    wing.taper                   = 2.1 / root_chord   
    wing.spans.projected         = span_input
    wing.areas.reference         = 298.9408702885464
    wing.areas.wetted            = 1464.0003661503454
    wing.chords.mean_aerodynamic = 5.263452352647592
    wing.chords.root             = root_chord
    wing.chords.tip              = 1.8359256146144747  
    wing.aft_center_body.length  = 9.021        
    wing.aft_center_body.taper   = 0.85
    wing.total_length            = root_chord
    wing.twists.root             = root_twist
    wing.twists.tip              = tip_twist
    wing.origin                  = [[0.0,  0.0,  0.0]] 
    wing.aerodynamic_center      = [17.43511294,  0.        ,  1.08931241] 
    wing.vertical                = False
    wing.xz_plane_symmetric      = True
    wing.t_tail                  = False 
    wing.dynamic_pressure_ratio  = 1.0
     
    cabin                                                  = RCAIDE.Library.Components.Fuselages.Cabins.Cabin()
    cabin.offset_x                                         = 2.54
    cabin.origin                                           = [[2.54, 0, 0]]
    
    business_class                                         = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Business() 
    business_class.number_of_seats_abrest                  = 4
    business_class.number_of_rows                          = 4
    business_class.galley_lavatory_percent_x_locations     = [0] 
    business_class.seat_arm_rest_width                     = 4 *  Units.inches 
    business_class.seat_width                              = 25 *  Units.inches
    business_class.aisle_width                              = 15  *  Units.inches 
    business_class.type_A_exit_percent_x_locations         = [0,0]
    cabin.append_cabin_class(business_class)  

    economy_class                                          = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest                   = 6
    economy_class.number_of_rows                           = 12
    economy_class.galley_lavatory_percent_x_locations      = [0,1.0]       
    economy_class.type_A_exit_percent_x_locations          = [0, 1.0]
    cabin.append_cabin_class(economy_class)
    wing.append_cabin(cabin)  

    side_cabin                                             = RCAIDE.Library.Components.Fuselages.Cabins.Side_Cabin()
    side_cabin.nose.fineness_ratio                         = 1.75
    side_cabin.offset_x                                    = 2.54
    side_cabin.origin                                      = [[2.54, 0, 0]]
   
    business_class                                         = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Business() 
    business_class.number_of_seats_abrest                  = 4
    business_class.number_of_rows                          = 4
    business_class.galley_lavatory_percent_x_locations     = [0] 
    business_class.seat_arm_rest_width                     = 4 *  Units.inches 
    business_class.seat_width                              = 30 *  Units.inches
    business_class.aisle_width                              = 15  *  Units.inches  
    business_class.type_A_exit_percent_x_locations         = [0,0]
    side_cabin.append_cabin_class(business_class)
    
    side_economy_class                                     = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    side_economy_class.number_of_seats_abrest              = 6
    side_economy_class.number_of_rows                      = 12
    side_economy_class.galley_lavatory_percent_x_locations = [0,1.0] 
    side_economy_class.type_A_exit_percent_x_locations     = [0, 1.0]
    side_cabin.append_cabin_class(side_economy_class) 
    wing.append_cabin(side_cabin)


    # Wing Segments
    segment                                        = RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment()
    segment.tag                                    = 'Fuselage_Section_1'
    segment.taper                                  = 0.8769841298701299
    segment.twist                                  = (wing.twists.root +  segment.percent_span_location * wing.twists.tip)
    segment.percent_span_location                  = 0.0  
    segment.root_chord_percent                     = 1.0 
    segment.dihedral_outboard                      = 0  *  Units.degrees 
    segment.sweeps.quarter_chord                   = 10.037 *  Units.degrees
    airfoil                                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        = rel_path + 's1016.txt'
    segment.append_airfoil(airfoil )         
    wing.append_segment(segment)         
         
         
    segment                                        = RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment()
    segment.tag                                    = 'Fuselage_Section_2'
    segment.taper                                  = 0.32260442862265637  
    segment.percent_span_location                  = 0.020625/  wing.spans.projected*64
    segment.twist                                  = (wing.twists.root + segment.percent_span_location * wing.twists.tip)
    segment.root_chord_percent                     = 0.9923542105
    segment.dihedral_outboard                      = 17 *  Units.degrees   
    segment.sweeps.quarter_chord                   = 46.9023 *  Units.degrees  
    airfoil                                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        = rel_path +  's1016.txt'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment)
    
    
    segment                                        = RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment()
    segment.tag                                    = 'Fuselage_Section_3'
    segment.taper                                  = 0.32260442862265637   
    segment.percent_span_location                  = 0.059375/  wing.spans.projected*64
    segment.twist                                  = (wing.twists.root +  segment.percent_span_location * wing.twists.tip)
    segment.root_chord_percent                     = 0.9375928
    segment.dihedral_outboard                      = 2 *  Units.degrees  
    segment.sweeps.quarter_chord                   = 51.027  *  Units.degrees   
    airfoil                                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        = rel_path +  's1016.txt'
    segment.append_airfoil(airfoil )         
    wing.append_segment(segment)         
         
    segment                                        = RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment()
    segment.tag                                    = 'Cabin_Wall'
    segment.taper                                  = 0.32260442862265637   
    segment.percent_span_location                  = 0.1913125/  wing.spans.projected*64
    segment.twist                                  = (wing.twists.root + segment.percent_span_location * wing.twists.tip)
    segment.root_chord_percent                     = 0.68285715
    segment.dihedral_outboard                      = 12 *  Units.degrees   
    segment.sweeps.quarter_chord                   = 42.5  *  Units.degrees   
    airfoil                                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        = rel_path + 's1016.txt'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment) 


    segment                                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                                    = 'Fuel_Wall'
    segment.taper                                  = 0.32260442862265637   
    segment.percent_span_location                  = 0.224625/  wing.spans.projected*64
    segment.twist                                  = (wing.twists.root +  segment.percent_span_location * wing.twists.tip)
    segment.root_chord_percent                     = fuel_wall_chord
    segment.dihedral_outboard                      = fuel_wall_dihedral    
    segment.sweeps.quarter_chord                   = fuel_wall_sweep
    airfoil                                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        = rel_path + 's1016.txt'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment)     
    
    segment                                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                                    = 'Wing_Section_1' 
    segment.percent_span_location                  = section_1_span_percent
    segment.twist                                  = (wing.twists.root + segment.percent_span_location * wing.twists.tip)
    segment.root_chord_percent                     = section1_chord
    segment.dihedral_outboard                      = section1_dihedral 
    segment.sweeps.quarter_chord                   = section1_sweep 
    airfoil                                        =  RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        =  rel_path +'transonic_wing_outboard_section_airfoil.txt'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment)

    segment                                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                                    = 'Wing_Section_2' 
    segment.percent_span_location                  = 0.5389354883954404
    segment.twist                                  = (wing.twists.root +  segment.percent_span_location * wing.twists.tip)
    segment.root_chord_percent                     = 4.5/35
    segment.dihedral_outboard                      = 1 *  Units.degrees 
    segment.sweeps.quarter_chord                   = (30)*  Units.degrees 
    segment.chords.reference_area_root             = True
    airfoil                                        =  RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        =  rel_path + 'transonic_wing_outboard_section_airfoil.txt'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment)  

    segment                                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                                    = 'Wing_Section_3' 
    segment.percent_span_location                  = 0.98
    segment.root_chord_percent                     = 2.1/wing.chords.root
    segment.twist                                  = (wing.twists.root +  segment.percent_span_location * wing.twists.tip)
    segment.dihedral_outboard                      = 65 *  Units.degrees  
    segment.sweeps.quarter_chord                   = 55 *  Units.degrees 
    airfoil                                        =  RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        =  rel_path + 'transonic_wing_outboard_section_airfoil.txt'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment)  

    segment                                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                                    = 'Wing_Section_4' 
    segment.twist                                  = 0.0 
    segment.percent_span_location                  = 1.0 
    segment.twist                                  = 0
    segment.root_chord_percent                     = 0.7/wing.chords.root
    segment.dihedral_outboard                      = 0  
    airfoil                                        =  RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        =  rel_path + 'transonic_wing_outboard_section_airfoil.txt'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment)
    
    # control surfaces -------------------------------------------
    slat                                           = RCAIDE.Library.Components.Wings.Control_Surfaces.Slat()
    slat.tag                                       = 'slat'
    slat.span_fraction_start                       = 0.33
    slat.span_fraction_end                         = 0.95
    slat.deflection                                = 0.0 * Units.degrees
    slat.chord_fraction                            = 0.075
    wing.append_control_surface(slat) 

    flap                                           = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    flap.tag                                       = 'flap'
    flap.span_fraction_start                       = 0.2
    flap.span_fraction_end                         = 0.7
    flap.deflection                                = 0.0 * Units.degrees
    flap.configuration_type                        = 'double_slotted'
    flap.chord_fraction                            = 0.14
    wing.append_control_surface(flap)

    aileron                                        = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                                    = 'aileron'
    aileron.span_fraction_start                    = 0.7
    aileron.span_fraction_end                      = 0.95
    aileron.deflection                             = 0.0 * Units.degrees
    aileron.chord_fraction                         = 0.25
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
    # Vertical Stabilizer 
    # ------------------------------------------------------------------ 
    wing = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag = 'vertical_stabilizer'
    wing.aspect_ratio                             = 1.73
    wing.thickness_to_chord                       = .08
    wing.spans.projected                          = vertical_height
    wing.sweeps.quarter_chord                     = 35 * Units.degrees   
    wing.chords.root                              = vertical_chord
    taper = 1/3
    wing.chords.tip                               = wing.chords.root * taper                   
    wing.chords.mean_aerodynamic                  = wing.chords.root * 2/3 * (( 1 + wing.taper + wing.taper**2 ) / ( 1 + wing.taper )) 
    wing.areas.reference                          = vertical_height * (1+taper)/2*vertical_chord
    wing.total_length                             = wing.chords.root 
    wing.taper                                    = wing.chords.tip /  wing.chords.root 
    wing.twists.root                              = 0.0 
    wing.twists.tip                               = 0.0 
    wing.origin                                   = [[vehicle.wings.main_wing.chords.root * 0.90 - wing.chords.root,  6.4 , 0.0]] 
    wing.xz_plane_symmetric                       = True
    wing.dynamic_pressure_ratio                   = 1.0  

    segment                                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                                   = 'Root_Section'
    segment.twist                                 = 0.0 
    segment.percent_span_location                 = 0
    segment.root_chord_percent                    = 1.0 
    segment.dihedral_outboard                     = 25 * Units.degrees 
    segment.thickness_to_chord                    = 0.08
    segment.sweeps.quarter_chord                  = 40 * Units.degrees 
    wing.append_segment(segment) 
        
    segment                                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                                   = 'Tip_Section'
    segment.twist                                 = 0.0 
    segment.percent_span_location                 = 1.0 
    segment.root_chord_percent                    = wing.taper
    segment.sweeps.quarter_chord                  = 0.0 
    segment.dihedral_outboard                     = 0 
    segment.thickness_to_chord                    = 0.08
    wing.append_segment(segment) 

    # control surfaces -------------------------------------------
    rudder                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    rudder.tag                   = 'rudder'
    rudder.span_fraction_start   = 0.09
    rudder.span_fraction_end     = 1
    rudder.deflection            = 0.0  * Units.deg
    rudder.chord_fraction        = 0.3
    wing.append_control_surface(rudder)

    # add to vehicle
    vehicle.append_component(wing)     


    # ################################################# Landing Gear #############################################################   

    main_gear                                     = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear() 
    main_gear.tire_diameter                       = 50.0 *  Units.inches 
    main_gear.rim_diameter                        = 22   *  Units.inches 
    main_gear.tire_width                          = 20.0 *  Units.inches 
    main_gear.strut_length                        = 5.5  * Units.ft 
    main_gear.wheels                              = 8   
    main_gear.number_of_gear_types_in_tandem      = 2
    main_gear.number_of_wheels_in_gear_type       = 2  
    main_gear.xz_plane_symmetric                  = True 
    main_gear.origin                              = [[20.9,0,0]]
    vehicle.append_component(main_gear)         
       
    nose_gear                                     = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()   
    nose_gear.tire_diameter                       = 40. *  Units.inches   
    nose_gear.rim_diameter                        = 16  *  Units.inches 
    nose_gear.tire_width                          = 16  *  Units.inches 
    nose_gear.strut_length                        = 9.0 * Units.ft 
    nose_gear.wheels                              = 2   
    nose_gear.number_of_gear_types_in_tandem      = 1
    nose_gear.number_of_wheels_in_gear_type       = 2  
    nose_gear.origin                              = [[5.8,0,0]]    
    vehicle.append_component(nose_gear)
    
    
    
 # ################################################# Energy Network #######################################################          
    #------------------------------------------------------------------------------------------------------------------------- 
    #  Turbofan Network
    #-------------------------------------------------------------------------------------------------------------------------   
    net                                         = RCAIDE.Framework.Networks.Fuel() 

    #------------------------------------------------------------------------------------------------------------------------- 
    # Fuel Distrubition Line 
    #------------------------------------------------------------------------------------------------------------------------- 
    fuel_line                                      = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
    fuel_line.pipe.rigid_material                  = RCAIDE.Library.Attributes.Materials.Stainless_Steel_304()
    fuel_line.pipe.flexible_material               = RCAIDE.Library.Attributes.Materials.Stainless_Steel_304() 
    fuel_line.venting_system_length                = vehicle.wings.main_wing.chords.root/2 # Length of venting system
    fuel_line.pipe.flexible_material_ratio         = 0.25
    fuel_line.pipe.diameters                       = Data()
    fuel_line.pipe.diameters.external              = 3.5 *  Units.inches 
    fuel_line.pipe.diameters.internal              = 3.5 *  Units.inches -  (2 * 0.083)*  Units.inches
    fuel_line.insulation                           = Data()
    fuel_line.insulation.rigid_material            = RCAIDE.Library.Attributes.Materials.Stainless_Steel_304() 
    fuel_line.insulation.flexible_material         = RCAIDE.Library.Attributes.Materials.Stainless_Steel_304() 
    fuel_line.insulation.flexible_material_ratio   = 0.25
    fuel_line.insulation.diameters                 = Data()
    fuel_line.insulation.diameters.external        = 5.563 *  Units.inches 
    fuel_line.insulation.diameters.internal        = 5.563 *  Units.inches -  (2 * 0.109)*  Units.inches
    
    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------         
    turbofan1                                      = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan()  
    turbofan1.tag                                  = 'propulsor_1' # 'Pratt_and_Whitney_2043'  https://prd-sc102-cdn.rtx.com/-/media/pw/products/commercial-jet-engines/pw2000/files/ce_pw2000_fact.pdf?rev=7377aaa21c9b415bad4b295dd5fc4c9b&hash=8B755FC15B1A44AB0817E984A8B3E2CB   
    turbofan1.length                               = 141.4 * Units.inches                 
    turbofan1.diameter                             = 78.5 * Units.inches                     
    turbofan1.bypass_ratio                         = 8.         
    turbofan1.design_altitude                      = 40000*Units.ft             
    turbofan1.design_mach_number                   = 0.78                      
    turbofan1.design_thrust                        = 34000.0
    turbofan1.wing_mounted                         = False
    # working fluid                   
    turbofan1.working_fluid                        = RCAIDE.Library.Attributes.Gases.Air() 
        
    # Ram inlet   
    ram                                            = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                        = 'ram' 
    turbofan1.ram                                  = ram 
            
    # inlet nozzle          
    inlet_nozzle                                   = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                               = 'inlet nozzle'
    inlet_nozzle.polytropic_efficiency             = 0.97                                       
    inlet_nozzle.pressure_ratio                    = 1
    inlet_nozzle.compressibility_effects           = False
    turbofan1.inlet_nozzle                         = inlet_nozzle
        
    # fan                   
    fan                                            = RCAIDE.Library.Components.Powertrain.Converters.Fan()   
    fan.tag                                        = 'fan'
    fan.polytropic_efficiency                      = 0.95                   
    fan.pressure_ratio                             = 1.4     
    turbofan1.fan                                  = fan        
    
    # low pressure compressor     
    low_pressure_compressor                        = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    low_pressure_compressor.tag                    = 'lpc'
    low_pressure_compressor.polytropic_efficiency  = 0.95                
    low_pressure_compressor.pressure_ratio         = 2.5                  
    turbofan1.low_pressure_compressor              = low_pressure_compressor

    # high pressure compressor  
    high_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    high_pressure_compressor.tag                   = 'hpc'
    high_pressure_compressor.polytropic_efficiency = 0.98                       
    high_pressure_compressor.pressure_ratio        = 16
    turbofan1.high_pressure_compressor             = high_pressure_compressor

    # low pressure turbine  
    low_pressure_turbine                           = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    low_pressure_turbine.tag                       ='lpt'
    low_pressure_turbine.mechanical_efficiency     = 0.99                     
    low_pressure_turbine.polytropic_efficiency     = 0.94                     
    turbofan1.low_pressure_turbine                 = low_pressure_turbine
    
    # high pressure turbine     
    high_pressure_turbine                          = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    high_pressure_turbine.tag                      ='hpt'
    high_pressure_turbine.mechanical_efficiency    = 0.99                     
    high_pressure_turbine.polytropic_efficiency    = 0.94                         
    high_pressure_turbine.external_shaft_power     = 0                                 
    turbofan1.high_pressure_turbine                = high_pressure_turbine 

    # combustor  
    combustor                                      = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                                  = 'Comb'
    combustor.efficiency                           = 0.997                    
    combustor.turbine_inlet_temperature            = 1450               
    combustor.pressure_ratio                       = 0.94                     
    combustor.fuel_data                            = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()  
    turbofan1.combustor                            = combustor

    # core nozzle
    core_nozzle                                    = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                = 'core nozzle'
    core_nozzle.polytropic_efficiency              = 0.98                     
    core_nozzle.pressure_ratio                     = 0.995 
    turbofan1.core_nozzle                          = core_nozzle
        
    # fan nozzle             
    fan_nozzle                                     = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    fan_nozzle.tag                                 = 'fan nozzle'
    fan_nozzle.polytropic_efficiency               = 0.98                   
    fan_nozzle.pressure_ratio                      = 0.995  
    turbofan1.fan_nozzle                           = fan_nozzle     

    # design turbofan
    design_turbofan(turbofan1)

    # Nacelle 
    nacelle                                     = RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle()
    nacelle.diameter                            = 85 * Units.inches    
    nacelle.length                              = 160 * Units.inches  
    nacelle.tag                                 = 'nacelle_1'
    nacelle.inlet_diameter                      = 80 * Units.inches    
    nacelle.origin                              = [[0.8*vehicle.wings.main_wing.chords.root, 4.2, 2.25]] 
    nacelle.areas.wetted                        = np.pi*nacelle.diameter*nacelle.length
    nacelle_airfoil                             = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    nacelle_airfoil.NACA_4_Series_code          = '4305'
    nacelle.append_airfoil(nacelle_airfoil) 
    turbofan1.nacelle                            = nacelle  
    turbofan1.origin                             = [[0.8*vehicle.wings.main_wing.chords.root, 4.2, 2.25]] 
    
    
    net.propulsors.append(turbofan1)
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Propulsor 2 (Inner Port Side)
    #------------------------------------------------------------------------------------------------------------------------------------       
    turbofan2                                  = deepcopy(turbofan1) 
    turbofan2.tag                              = 'propulsor_2' 
    turbofan2.origin                           = [[0.8*vehicle.wings.main_wing.chords.root, -4.2, 2.25]] 
    turbofan2.nacelle.tag                      =  'propulsor_2_nacelle'
    turbofan2.nacelle.origin                   = [[0.8*vehicle.wings.main_wing.chords.root, -4.2, 2.25]] 
        
    # append propulsor to distribution line 
    net.propulsors.append(turbofan2) 

  
    #------------------------------------------------------------------------------------------------------------------------- 
    #  Energy Source: Fuel Tank
    #------------------------------------------------------------------------------------------------------------------------- 
    fuel_tank_1                                        = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank(vehicle.wings.main_wing)
    fuel_tank_1.tag                                    = 'tank_1l_1r' 
    fuel_tank_1.material                               = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_1.insulation_material                    = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
    fuel_tank_1.segments_bounding_tank                 = ['fuel_wall', 'wing_section_1']                
    fuel_tank_1.segments_percent_chord_bounds          = [0.1 ,0.1] 
    fuel_tank_1.segments_percent_chord_end             = [0.55,0.55]
    fuel_tank_1.fuel.tag                               = '_lh2' 
    fuel_line.fuel_tanks.append(fuel_tank_1)

    fuel_tank_2                                        = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank(vehicle.wings.main_wing)
    fuel_tank_2.tag                                    = 'tank_2l_2r' 
    fuel_tank_2.material                               = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_2.insulation_material                    = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation() 
    fuel_tank_2.segments_bounding_tank                 = ['fuel_wall', 'wing_section_1']                
    fuel_tank_2.segments_percent_chord_bounds          = [0.1 ,0.1] 
    fuel_tank_2.segments_percent_chord_end             = [0.55,0.55]  
    fuel_tank_2.fuel.tag                               = '_lh2' 
    
    fuel_line.fuel_tanks.append(fuel_tank_2)
    fuel_tank_3                                        = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank(vehicle.wings.main_wing)
    fuel_tank_3.tag                                    = 'tank_3l_3r' 
    fuel_tank_3.material                               = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_3.insulation_material                    = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
    fuel_tank_3.segments_bounding_tank                 = ['fuel_wall', 'wing_section_1']                     
    fuel_tank_3.segments_percent_chord_bounds          = [0.1 ,0.1] 
    fuel_tank_3.segments_percent_chord_end             = [0.55,0.55]  
    fuel_tank_3.fuel.tag                               =  '_lh2' 
    fuel_line.fuel_tanks.append(fuel_tank_3)    

    # ------------------------------------------------
    # Bounds for aft tank
    cabin_length = 23.4
    cabin_bound = cabin_length / vehicle.wings.main_wing.chords.root

    aft_segment_bound = 'cabin_wall'
    engine2wall = abs(turbofan2.nacelle.origin[0][1]) + vehicle.wings.main_wing.segments[aft_segment_bound].percent_span_location * vehicle.wings.main_wing.spans.projected / 2 # Distance from engine to the far wall
    forward_location = np.tan(15*Units.degrees) * engine2wall
    rotor_burst_bound = (turbofan2.nacelle.origin[0][0] - forward_location) / vehicle.wings.main_wing.chords.root

    if rotor_burst_bound < cabin_bound:
        print("Error: rotor burst criteria and cabin bounds intersect. Tank 4 and 5 are impossible to place")
    else:

        fuel_tank_4                                        = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank(vehicle.wings.main_wing)
        fuel_tank_4.tag                                    = 'aft_tank' 
        fuel_tank_4.geometry_type                          = 'conformal'
        fuel_tank_4.material                               = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
        fuel_tank_4.insulation_material                    = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
        fuel_tank_4.xz_plane_symmetric                     = False
        fuel_tank_4.orientation_euler_angles               = [0,0,np.pi/2]
        fuel_tank_4.bwb_aft_tank                           = True
        fuel_tank_4.aft_tank_root_chord_bounds             = [cabin_bound,rotor_burst_bound]
        fuel_tank_4.aft_tank_segment_bound                 = aft_segment_bound
        fuel_tank_4.radial_offset                          = 0.1
        fuel_tank_4.fuel.tag                               = '_lh2' 
        fuel_line.fuel_tanks.append(fuel_tank_4)


    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to fuel line to network      
    fuel_line.assigned_propulsors =  [['propulsor_1', 'propulsor_2']] 

    ##------------------------------------------------------------------------------------------------------------------------- 
    ##  Systems
    ##-------------------------------------------------------------------------------------------------------------------------   
    avionics =  RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.origin                   = [[1,0,0]]   
    vehicle.append_component(avionics)

    flight_controls =  RCAIDE.Library.Components.Powertrain.Systems.Flight_Controls()    
    flight_controls.origin            = [[0.5 * vehicle.wings.main_wing.chords.root,0,0]]  
    vehicle.append_component(flight_controls)
    
    auxillary_power_unit =  RCAIDE.Library.Components.Powertrain.Systems.Auxillary_Power_Unit()  
    auxillary_power_unit.tag= 'fuel_cell_apu_0'
    auxillary_power_unit.mass_properties.mass = 235.8
    auxillary_power_unit.origin       = [[0.76 * vehicle.wings.main_wing.chords.root,0,0]] 
    vehicle.append_component(auxillary_power_unit)
    
    auxillary_power_unit =  RCAIDE.Library.Components.Powertrain.Systems.Auxillary_Power_Unit()  
    auxillary_power_unit.tag= 'fuel_cell_apu_1'
    auxillary_power_unit.mass_properties.mass = 235.8
    auxillary_power_unit.origin       = [[0.76 * vehicle.wings.main_wing.chords.root,-2,0]]  
    vehicle.append_component(auxillary_power_unit)
    
    auxillary_power_unit =  RCAIDE.Library.Components.Powertrain.Systems.Auxillary_Power_Unit()  
    auxillary_power_unit.origin       = [[0.76 * vehicle.wings.main_wing.chords.root,2,0]] 
    auxillary_power_unit.mass_properties.mass = 235.8
    auxillary_power_unit.tag= 'fuel_cell_apu_2'
    vehicle.append_component(auxillary_power_unit)



    electrical =  RCAIDE.Library.Components.Powertrain.Systems.Electrical()      
    electrical.origin                 = [[0.2 * vehicle.wings.main_wing.chords.root,0,0]]  
    vehicle.append_component(electrical)
    
    hydraulics =  RCAIDE.Library.Components.Powertrain.Systems.Hydraulics()  
    hydraulics.origin                 = [[0.70 * vehicle.wings.main_wing.chords.root,0,0]]  
    vehicle.append_component(hydraulics)
    
    environmental_controls =  RCAIDE.Library.Components.Powertrain.Systems.Environmental_Controls()  
    environmental_controls.origin     = [[0.2 * vehicle.wings.main_wing.chords.root,0,0]]   
    vehicle.append_component(environmental_controls)
    
    instruments =  RCAIDE.Library.Components.Powertrain.Systems.Instruments()  
    instruments.origin                = [[1,0,0]]  
    vehicle.append_component(instruments)    

    #------------------------------------------------------------------------------------------------------------------------- 
    #  PUMPS      
    #------------------------------------------------------------------------------------------------------------------------- 
    # Starboard Pump 
    starboard_pump                                 = RCAIDE.Library.Components.Powertrain.Converters.Liquid_Hydrogen_Fuel_Cell_Pump()
    starboard_pump.working_fluid                   = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()  
    starboard_pump.power_density                   = 15000 # W/kg
    starboard_pump.pump_efficiency                 = 0.8
    starboard_pump.turbine_efficiency              = 0.92
    starboard_pump.design_mass_flow_rate           = 1      # kg/s
    starboard_pump.distributor_split               = 0.5
    starboard_pump.design_power_rating             = 3E5    
    starboard_pump.design_inlet_pressure           = 200000 # Pascals (2 bar)
    starboard_pump.tag                             = 'starboard_engine_pump' 
    starboard_pump.design_outlet_pressure          = 35000000 # Pascals (350 bar)    
    design_pump(starboard_pump)
    starboard_pump.origin                          = [[27 * 36  /35,1,0]] # Location checked
    net.converters.append(starboard_pump)  
   
    # Port Pump 
    port_pump                                      = deepcopy(starboard_pump) 
    port_pump.tag                                  = 'port_engine_pump' 
    port_pump.origin                               = [[27 * 36  /35,-1,0]] # Location checked
    net.converters.append(port_pump)  

    # Reserve Pump 
    reserve_pump                                  = deepcopy(starboard_pump)
    reserve_pump.active                           = False
    reserve_pump.tag                              = 'reserve_pump' 
    reserve_pump.origin                           = [[27 * 36  /35,0,0]] # Location checked
    reserve_pump.distributor_split                = 0
    net.converters.append(reserve_pump)  

    starboard_pump                                 = RCAIDE.Library.Components.Powertrain.Converters.Pump()
    starboard_pump.working_fluid                   = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()  
    starboard_pump.power_density                   = 15000 # W/kg
    starboard_pump.pump_efficiency                 = 0.8
    starboard_pump.turbine_efficiency              = 0.92
    starboard_pump.design_mass_flow_rate           = 1      # kg/s
    starboard_pump.distributor_split               = 0.5
    starboard_pump.design_power_rating             = 3E5    
    starboard_pump.design_inlet_pressure           = 200000 # Pascals (2 bar)
    starboard_pump.tag                             = 'starboard_engine_pump' 
    starboard_pump.design_outlet_pressure          = 35000000 # Pascals (350 bar)    
    design_pump(starboard_pump)
    starboard_pump.origin                          = [[27 * 36  /35,1,0]] # Location checked
    net.converters.append(starboard_pump)  
          
    # Assign propulsors to fuel line to network      
    fuel_line.assigned_converters =  [[starboard_pump.tag, port_pump.tag,reserve_pump.tag]]        

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
    #   Cruise Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'cruise'
    configs.append(config) 


    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------ 
    config = RCAIDE.Library.Components.Configs.Config(vehicle)
    config.tag = 'idle' 
    config.networks.fuel.propulsors['propulsor_1'].emission_indices.NOx      = 4.85 /1000      
    config.networks.fuel.propulsors['propulsor_2'].emission_indices.NOx      = 4.85 /1000      
    configs.append(config) 

    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'takeoff'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 30. * Units.deg   
    config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity =  3470. * Units.rpm 
    config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity      =  3470. * Units.rpm 
    config.landing_gears.main_gear.gear_extended    = True
    config.landing_gears.nose_gear.gear_extended    = True  
    config.V2_VS_ratio = 1.21
    configs.append(config)

    
    # ------------------------------------------------------------------
    #   Cutback Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'cutback'
    config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity =  2780. * Units.rpm 
    config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity      =  2780. * Units.rpm     
    configs.append(config)   
    
        
    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'landing'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 30. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg 
    config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity =  2030. * Units.rpm 
    config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity      =  2030. * Units.rpm 
    config.landing_gears.main_gear.gear_extended    = True
    config.landing_gears.nose_gear.gear_extended    = True  
    config.Vref_VS_ratio = 1.23
    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------  

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'reverse_thrust'
    config.landing_gears.main_gear.gear_extended    = True
    config.landing_gears.nose_gear.gear_extended    = True  
    configs.append(config)    
    
    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'descent' 
    config.wings['main_wing'].control_surfaces.spoiler.deflection  = 45. * Units.deg    
    configs.append(config) 

    # ------------------------------------------------------------------
    #   OEI Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'oei'  
    config.networks.fuel.fuel_lines.fuel_line.assigned_propulsors = [['propulsor_1']]
    config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity =  2030. * Units.rpm 
    configs.append(config)


    return configs
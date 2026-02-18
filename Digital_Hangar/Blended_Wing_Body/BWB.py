# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units   
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan  import design_turbofan   
from RCAIDE.Library.Plots                 import *    

# python imports 
import numpy as np   
from copy import deepcopy
import os
import sys

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    
    # Step 1: design a vehicle
    vehicle  = vehicle_setup()  

    try:
        import vsp as vsp
        from RCAIDE.Framework.External_Interfaces.OpenVSP import export_vsp_vehicle 
        export_vsp_vehicle(vehicle, 'BWB')
    except ImportError:
        pass 
    
    # Step 2: plot vehicle 
    plot_3d_vehicle(vehicle,export_gltf=True) 
    
    return 


def vehicle_setup():  
    airfoil_file_path =  os.path.join(os.path.split(sys.path[0])[0], '_Airfoils_and_Polars') + os.sep 
    polar_file_path   =  os.path.join(os.path.join(os.path.split(sys.path[0])[0], '_Airfoils_and_Polars') , '_Polars') + os.sep 
     
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------      
    vehicle                                           = RCAIDE.Vehicle()    
    vehicle.tag                                       = 'BWB' 
    vehicle.mass_properties.max_takeoff               = 90535.0202
    vehicle.mass_properties.takeoff                   = 90535.0202
    vehicle.mass_properties.moments_of_inertia.tensor = np.array([[1e6, 100, 100], [100, 1e6, 100], [100, 100, 1e7]]) 
    vehicle.mass_properties.max_payload               = 52920.  * Units.lb    
    vehicle.mass_properties.min_payload               = 33880.  * Units.lb     
    vehicle.mass_properties.center_of_gravity         = [[27.0, 0, 0]]
    vehicle.flight_envelope.ultimate_load             = 3.75 
    vehicle.flight_envelope.positive_limit_load       = 2.5  
    vehicle.flight_envelope.design_mach_number        = 0.85  
    vehicle.flight_envelope.design_cruise_altitude    = 40000.0*Units.feet 
    vehicle.flight_envelope.design_range              = 2500.0 * Units.nmi
    vehicle.reference_area                            = 296  
    vehicle.number_of_passengers                      = 150 
    vehicle.systems.control                           = "fully powered" 
    vehicle.systems.accessories                       = "long range"    
    
    # ------------------------------------------------------------------
    # Carbo Bays 
    # ------------------------------------------------------------------ 
    center_cargo_bay = RCAIDE.Library.Components.Cargo_Bays.Cargo_Bay()
    center_cargo_bay.cargo.mass_properties.mass  = 0 
    center_cargo_bay.origin                      = [[6, 0, -22.5  * Units.inches]]  
    center_cargo_bay.length                      = 60.4 *  Units.inches *  9
    center_cargo_bay.width                       = 96   *  Units.inches
    center_cargo_bay.height                      = 45   *  Units.inches 
    vehicle.cargo_bays.append(center_cargo_bay) 
 
    left_cargo_bay  = RCAIDE.Library.Components.Cargo_Bays.Cargo_Bay()
    left_cargo_bay.cargo.mass_properties.mass  = 0  
    left_cargo_bay.origin                      = [[10, 100   *  Units.inches, -22.5* Units.inches]] 
    left_cargo_bay.length                      =  60.4 *  Units.inches *  5
    left_cargo_bay.width                       =  96   *  Units.inches
    left_cargo_bay.height                      =  45   *  Units.inches
    vehicle.cargo_bays.append(left_cargo_bay)  
 
    right_cargo_bay  = RCAIDE.Library.Components.Cargo_Bays.Cargo_Bay()
    right_cargo_bay.cargo.mass_properties.mass  = 0  
    right_cargo_bay.origin                      = [[10 ,  -100   *  Units.inches, -22.5* Units.inches]]  
    right_cargo_bay.length                      = 60.4 *  Units.inches  *  5
    right_cargo_bay.width                       = 96   *  Units.inches
    right_cargo_bay.height                      = 45   *  Units.inches
    vehicle.cargo_bays.append(right_cargo_bay)        
    
    
    # ------------------------------------------------------------------
    #  Main Wing 
    # ------------------------------------------------------------------ 
    
    wing = RCAIDE.Library.Components.Wings.Blended_Wing_Body()
    wing.tag = 'main_wing' 
    wing.aspect_ratio            = 5.32528141979797 
    wing.sweeps.quarter_chord    = 0.6859590736162121 
    wing.thickness_to_chord      = 0.1351757396035437 
    wing.taper                   = 0.05215045887445882  
    wing.spans.projected         = 48.37 
    wing.areas.reference         = 592.6575476422672 
    wing.areas.wetted            = 1282.0437685524962 
    wing.chords.mean_aerodynamic = 20.059555133618403 
    wing.chords.root             = 30
    wing.chords.tip              = 1.8359256146144747 
    wing.aft_center_body.length  = 9.021    
    wing.aft_center_body.taper   = 0.85
    wing.total_length            = 32.4
    wing.twists.outwash          = 0.055547748
    wing.twists.root_twist       = 0.014967432
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
    business_class.number_of_seats_abrest                  = 2
    business_class.number_of_rows                          = 3
    business_class.galley_lavatory_percent_x_locations     = [0] 
    business_class.seat_arm_rest_width                     = 4 *  Units.inches 
    business_class.seat_width                              = 25 *  Units.inches
    business_class.aisle_width                              = 15  *  Units.inches 
    business_class.type_A_exit_percent_x_locations         = [0,0]
    cabin.append_cabin_class(business_class)  

    economy_class                                          = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest                   = 6
    economy_class.number_of_rows                           = 11
    economy_class.galley_lavatory_percent_x_locations      = [0,1.0]       
    economy_class.type_A_exit_percent_x_locations          = [0, 1.0]
    cabin.append_cabin_class(economy_class)
    wing.append_cabin(cabin)  

    side_cabin                                             = RCAIDE.Library.Components.Fuselages.Cabins.Side_Cabin()
    side_cabin.nose.fineness_ratio                         = 1.75
   
    business_class                                         = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Business() 
    business_class.number_of_seats_abrest                  = 2
    business_class.number_of_rows                          = 3
    business_class.galley_lavatory_percent_x_locations     = [0] 
    business_class.seat_arm_rest_width                     = 4 *  Units.inches 
    business_class.seat_width                              = 30 *  Units.inches
    business_class.aisle_width                              = 15  *  Units.inches  
    business_class.type_A_exit_percent_x_locations         = [0,0]
    side_cabin.append_cabin_class(business_class)
    
    side_economy_class                                     = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    side_economy_class.number_of_seats_abrest              = 4
    side_economy_class.number_of_rows                      = 11
    side_economy_class.galley_lavatory_percent_x_locations = [0,1.0] 
    side_economy_class.type_A_exit_percent_x_locations     = [0, 1.0]
    side_economy_class.offset_y                            = 1
    side_cabin.append_cabin_class(side_economy_class) 
    wing.append_cabin(side_cabin) 

    # Wing Segments
    segment                                        = RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment()
    segment.tag                                    = 'Fuselage_Section_1'
    segment.taper                                  = 0.8769841298701299
    segment.twist                                  = wing.twists.root_twist  +  segment.percent_span_location * wing.twists.outwash 
    segment.percent_span_location                  = 0.0  
    segment.root_chord_percent                     = 1.0 
    segment.dihedral_outboard                      = 0  *  Units.degrees 
    segment.sweeps.quarter_chord                   = 10.037 *  Units.degrees
    airfoil                                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        = 's1014.txt'
    segment.append_airfoil(airfoil )         
    wing.append_segment(segment)         
         
         
    segment                                        = RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment()
    segment.tag                                    = 'Fuselage_Section_2'
    segment.taper                                  = 0.32260442862265637  
    segment.percent_span_location                  = 0.020625/  wing.spans.projected*64
    segment.twist                                  = wing.twists.root_twist  +  segment.percent_span_location * wing.twists.outwash 
    segment.root_chord_percent                     = 0.9923542105
    segment.dihedral_outboard                      = 0 *  Units.degrees   
    segment.sweeps.quarter_chord                   = 46.9023 *  Units.degrees  
    airfoil                                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        = 's1014.txt'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment)
    
    
    segment                                        = RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment()
    segment.tag                                    = 'Fuselage_Section_3'
    segment.taper                                  = 0.32260442862265637   
    segment.percent_span_location                  = 0.059375/  wing.spans.projected*64
    segment.twist                                  = wing.twists.root_twist  +  segment.percent_span_location * wing.twists.outwash 
    segment.root_chord_percent                     = 0.9375928
    segment.dihedral_outboard                      = 2 *  Units.degrees  
    segment.sweeps.quarter_chord                   = 51.027  *  Units.degrees   
    airfoil                                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        = 's1014.txt'
    segment.append_airfoil(airfoil )         
    wing.append_segment(segment)         
         
    segment                                        = RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment()
    segment.tag                                    = 'Cabin_Wall'
    segment.taper                                  = 0.32260442862265637   
    segment.percent_span_location                  = 0.17 /  wing.spans.projected*64
    segment.twist                                  = wing.twists.root_twist  +  segment.percent_span_location * wing.twists.outwash 
    segment.root_chord_percent                     = 0.68285715
    segment.dihedral_outboard                      = 12 *  Units.degrees   
    segment.sweeps.quarter_chord                   = 42.5  *  Units.degrees   
    airfoil                                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        = 's1014.txt'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment) 


    segment                                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                                    = 'Fuel_Wall'
    segment.taper                                  = 0.32260442862265637   
    segment.percent_span_location                  = 0.19 /  wing.spans.projected*64
    segment.twist                                  = wing.twists.root_twist  +  segment.percent_span_location * wing.twists.outwash 
    segment.root_chord_percent                     = 0.58
    segment.dihedral_outboard                      = 5 *  Units.degrees    
    segment.sweeps.quarter_chord                   = 0.775369385#769415328
    airfoil                                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        = airfoil_file_path + 's1014.txt'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment)     
    
    segment                                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                                    = 'Wing_Section_1' 
    segment.percent_span_location                  = 0.3346858066654701/  wing.spans.projected*64
    segment.twist                                  = wing.twists.root_twist  +  segment.percent_span_location * wing.twists.outwash 
    segment.root_chord_percent                     = 0.27 
    segment.dihedral_outboard                      = 1 *  Units.degrees 
    segment.sweeps.quarter_chord                   = 0.610554743 #0.557707107
    airfoil                                        =  RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        = airfoil_file_path +  'transonic_wing_inboard_section_airfoil.txt'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment)

    segment                                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                                    = 'Wing_Section_2' 
    segment.percent_span_location                  = 0.5389354883954404
    segment.twist                                  = wing.twists.root_twist  +  segment.percent_span_location * wing.twists.outwash 
    segment.root_chord_percent                     = 0.13 
    segment.dihedral_outboard                      = 1 *  Units.degrees 
    segment.sweeps.quarter_chord                   = 0.557683513
    segment.chords.reference_area_root             = True
    airfoil                                        =  RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        = airfoil_file_path +  'transonic_wing_inboard_section_airfoil.txt'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment)  

    segment                                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                                    = 'Wing_Section_3' 
    segment.percent_span_location                  = 0.98
    segment.root_chord_percent                     = 0.052
    segment.twist                                  = wing.twists.root_twist  +  segment.percent_span_location * wing.twists.outwash 
    segment.dihedral_outboard                      = 65 *  Units.degrees  
    segment.sweeps.quarter_chord                   = 55 *  Units.degrees 
    airfoil                                        =  RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        = airfoil_file_path +  'transonic_wing_outboard_section_airfoil.txt'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment)  

    segment                                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                                    = 'Wing_Section_4' 
    segment.twist                                  = 0.0 
    segment.percent_span_location                  = 1.0 
    segment.twist                                  = 0.0 * Units.deg
    segment.root_chord_percent                     = 0.02 
    segment.dihedral_outboard                      = 0  
    airfoil                                        =  RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        = airfoil_file_path + 'transonic_wing_tip_section_airfoil.txt'
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

    elevator                                        = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
    elevator.tag                                    = 'elevator'
    elevator.span_fraction_start                    = 0.01
    elevator.span_fraction_end                      = 0.2
    elevator.deflection                             = 0.0 * Units.degrees
    elevator.chord_fraction                         = 0.1
    wing.append_control_surface(elevator)  
    # add to vehicle
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    # Vertical Stabilizer 
    # ------------------------------------------------------------------ 
    wing = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag = 'vertical_stabilizer'
    wing.aspect_ratio                             = 1.5
    wing.thickness_to_chord                       = .08
    wing.areas.reference                          = 48.79*2
    wing.spans.projected                          = 7.620591106 #8.49945964 # Must be doubled for a symmetric tail
    wing.sweeps.quarter_chord                     = 35 * Units.degrees   
    wing.areas.wetted                             = 48.79*4.2
    wing.taper                                    = 0.375
    wing.chords.root                              = 3.218430874 #2501937
    wing.chords.tip                               = wing.chords.root * wing.taper                  
    wing.chords.mean_aerodynamic                  = wing.chords.root * 2/3 * (( 1 + wing.taper + wing.taper**2 ) / ( 1 + wing.taper )) 
    wing.total_length                             = wing.chords.root 
    wing.twists.root                              = 0.0 
    wing.twists.tip                               = 0.0 
    wing.origin                                   = [[vehicle.wings.main_wing.chords.root - 1.4*wing.chords.root,  5.5 , -0.5]]  
    wing.xz_plane_symmetric                       = True
    wing.dynamic_pressure_ratio                   = 1.0  

    segment                                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                                   = 'Root_Section'
    segment.twist                                 = 0.0 * Units.degrees 
    segment.percent_span_location                 = 0
    segment.root_chord_percent                    = 1.0 
    segment.dihedral_outboard                     = 25 * Units.degrees 
    segment.thickness_to_chord                    = 0.08
    segment.sweeps.quarter_chord                  = 40 * Units.degrees 
    wing.append_segment(segment) 
        
    segment                                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                                   = 'Tip_Section'
    segment.twist                                 = 0.0 * Units.degrees 
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
    main_gear.origin = [[17,0,-2]]
    vehicle.append_component(main_gear)         
       
    nose_gear                                     = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()   
    nose_gear.tire_diameter                       = 40. *  Units.inches   
    nose_gear.rim_diameter                        = 16  *  Units.inches 
    nose_gear.tire_width                          = 16  *  Units.inches 
    nose_gear.strut_length                        = 9.0 * Units.ft 
    nose_gear.wheels                              = 2   
    nose_gear.number_of_gear_types_in_tandem      = 1
    nose_gear.number_of_wheels_in_gear_type       = 2   
    nose_gear.origin = [[2,0,-1.5]] 
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
    # Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------         
    turbofan1                                      = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan()  
    turbofan1.tag                                  = 'port_propulsor' 
    turbofan1.length                               = 141.4 * Units.inches                 
    turbofan1.diameter                             = 78.5 * Units.inches                     
    turbofan1.bypass_ratio                         = 8.         
    turbofan1.design_altitude                      = 40000*Units.ft             
    turbofan1.design_mach_number                   = 0.78                      
    turbofan1.design_thrust                        = 40000.0
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
    turbofan1.high_pressure_turbine                = high_pressure_turbine 

    # combustor  
    combustor                                      = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                                  = 'Comb'
    combustor.efficiency                           = 0.997                    
    combustor.turbine_inlet_temperature            = 1450               
    combustor.pressure_ratio                       = 0.94                     
    combustor.fuel_data                            = RCAIDE.Library.Attributes.Propellants.Jet_A()  
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
    nacelle.origin                              = [[23, 4.2, 1.75]]
    nacelle.orientation_euler_angles            = [0, -7 * Units.degree, 0]
    nacelle.areas.wetted                        = np.pi*nacelle.diameter*nacelle.length
    nacelle_airfoil                             = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    nacelle_airfoil.NACA_4_Series_code          = '4305'
    nacelle.append_airfoil(nacelle_airfoil) 
    turbofan1.nacelle                            = nacelle 
    turbofan1.origin                             = [[23, 4.2, 1.75]]  
    
    net.propulsors.append(turbofan1)
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Propulsor 2 (Inner Port Side)
    #------------------------------------------------------------------------------------------------------------------------------------       
    turbofan2                                  = deepcopy(turbofan1)
    turbofan2.active_fuel_tanks                = ['fuel_tank'] 
    turbofan2.tag                              = 'starboard_propulsor' 
    turbofan2.origin                           = [[23, -4.2, 1.75]] 
    turbofan2.nacelle.tag                      =  'starboard_propulsor_nacelle'
    turbofan2.nacelle.origin                   = [[23, -4.2, 1.75]] 
        
    # append propulsor to distribution line 
    net.propulsors.append(turbofan2) 


    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Propulsor 3 (Center Engine)
    #------------------------------------------------------------------------------------------------------------------------------------       
    turbofan3                                  = deepcopy(turbofan1)
    turbofan3.active_fuel_tanks                = ['fuel_tank'] 
    turbofan3.tag                              = 'center_propulsor' 
    turbofan3.origin                           = [[24, 0, 1.5]] 
    turbofan3.nacelle.tag                      =  'center_engine_nacelle'
    turbofan3.nacelle.origin                   = [[24, 0, 1.5]] 
        
    # append propulsor to distribution line 
    net.propulsors.append(turbofan3)  

    #------------------------------------------------------------------------------------------------------------------------- 
    #  Energy Source: Fuel Tank
    #------------------------------------------------------------------------------------------------------------------------- 
    fuel_tank                                        = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)
    fuel_tank.fuel                                   = RCAIDE.Library.Attributes.Propellants.Jet_A1()   
    fuel_tank.segments_bounding_tank                 = ['fuel_wall','wing_section_1'] 
    fuel_tank.segments_percent_chord_start           = [0.2,0.2]
    fuel_tank.segments_percent_chord_end             = [0.425,0.425]
    fuel_line.fuel_tanks.append(fuel_tank) 

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to fuel line to network      
    fuel_line.assigned_propulsors =  [['starboard_propulsor', 'port_propulsor', 'center_propulsor']]

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Append fuel line to fuel line to network      
    net.fuel_lines.append(fuel_line)         

    # Append energy network to aircraft 
    vehicle.append_energy_network(net)  

    return vehicle
      
if __name__ == '__main__': 
    main()     
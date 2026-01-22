# RESEARCH/Aircraft/AACES_BWB_2050.py
# 
#
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units           
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan  import design_turbofan  
from RCAIDE.Framework.External_Interfaces.OpenVSP.export_vsp_vehicle import export_vsp_vehicle     
from RCAIDE.Library.Plots                                   import *
from RCAIDE.Library.Methods.Performance import *  

# python imports 
import numpy as np  
from copy import deepcopy
import matplotlib.pyplot as plt  
import os 
import pickle 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main(plot_results=True, plot_vehicle=True) :
    
    # Step 1 design a vehicle
    vehicle  = vehicle_setup()  
    # Export VSP Model
    # export_vsp_vehicle(vehicle, 'AACES_BWB_2050')
     
    # Step 2 create aircraft configuration based on vehicle 
    configs  = configs_setup(vehicle)
    
    # Step 3 set up analysis 
    analyses = analyses_setup(configs)
    
    # Step 4 set up a flight mission
    mission  = mission_setup(analyses)
    missions = missions_setup(mission) 
    
    # Step 5 execute flight profile
    results = missions.base_mission.evaluate()  
    
    # Step 6 plot results
    if plot_results:
        plot_mission(results) 

    # plot vehicle
    if plot_vehicle:
        plot_3d_vehicle(vehicle)
    
    return 

def vehicle_setup(MTOW=100000, span_input=64) : 

    ospath                                = os.path.abspath(__file__)
    separator                             = os.path.sep
    rel_path                              = os.path.dirname(ospath) + separator + '..' + os.sep + 'Airfoils'+ os.sep
     
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------      
    vehicle                                           = RCAIDE.Vehicle()    
    vehicle.tag                                       = 'BOB' 
    vehicle.mass_properties.max_takeoff               = MTOW  
    # vehicle.mass_properties.takeoff                   = MTOW  
    vehicle.mass_properties.payload                   = 42000* Units.lbs 
    vehicle.mass_properties.max_payload               = 52920.  * Units.lb    
    vehicle.mass_properties.min_payload               = 33880.  * Units.lb     
    vehicle.mass_properties.center_of_gravity         = [[27.0, 0, 0]]
    vehicle.flight_envelope.ultimate_load             = 3.75 
    vehicle.flight_envelope.positive_limit_load       = 2.5  
    vehicle.flight_envelope.design_mach_number        = 0.85  
    vehicle.flight_envelope.design_cruise_altitude    = 40000.0*Units.feet 
    vehicle.flight_envelope.design_range              = 2500.0 * Units.nmi
    vehicle.reference_area                            = 592.6575476422672 # 2424.9 * Units['feet**2']    
    vehicle.number_of_passengers                      = 150 # Single class. 242 in dual class (24 business, 21 economy) 
    vehicle.systems.control                           = "fully powered" 
    vehicle.systems.accessories                       = "long range"  

    # ------------------------------------------------------------------
    # Carbo Bays 
    # ------------------------------------------------------------------ 
    cargo_bay = RCAIDE.Library.Components.Cargo_Bays.Cargo_Bay()
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
    wing.spans.projected         = span_input
    wing.areas.reference         = 592.6575476422672 
    wing.areas.wetted            = 1282.0437685524962 
    wing.chords.mean_aerodynamic = 20.059555133618403 
    wing.chords.root             = 30
    wing.chords.tip              = 1.8359256146144747 
    wing.aft_center_body.length  = 9.021    
    wing.aft_center_body.taper   = 0.85
    wing.total_length            = 32.4
    wing.twists.outwash                   = -2.5*Units.degree
    wing.twists.root_twist               = 2.15*Units.degree    
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
    airfoil.coordinate_file                        = rel_path + 's1014.dat' 
    segment.append_airfoil(airfoil )         
    wing.append_segment(segment)         
         
         
    segment                                        = RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment()
    segment.tag                                    = 'Fuselage_Section_2'
    segment.taper                                  = 0.32260442862265637  
    segment.percent_span_location                  = 0.020625/ 64 * wing.spans.projected
    segment.twist                                  = wing.twists.root_twist  +  segment.percent_span_location * wing.twists.outwash 
    segment.root_chord_percent                     = 0.9923542105
    segment.dihedral_outboard                      = 0 *  Units.degrees   
    segment.sweeps.quarter_chord                   = 46.9023 *  Units.degrees  
    airfoil                                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        = rel_path +  's1014.dat'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment)
    
    
    segment                                        = RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment()
    segment.tag                                    = 'Fuselage_Section_3'
    segment.taper                                  = 0.32260442862265637   
    segment.percent_span_location                  = 0.059375/ 64 * wing.spans.projected
    segment.twist                                  = wing.twists.root_twist  +  segment.percent_span_location * wing.twists.outwash 
    segment.root_chord_percent                     = 0.9375928
    segment.dihedral_outboard                      = 2 *  Units.degrees  
    segment.sweeps.quarter_chord                   = 51.027  *  Units.degrees   
    airfoil                                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        = rel_path +  's1014.dat'
    segment.append_airfoil(airfoil )         
    wing.append_segment(segment)         
         
    segment                                        = RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment()
    segment.tag                                    = 'Cabin_Wall'
    segment.taper                                  = 0.32260442862265637   
    segment.percent_span_location                  = 0.17 / 64 * wing.spans.projected
    segment.twist                                  = wing.twists.root_twist  +  segment.percent_span_location * wing.twists.outwash 
    segment.root_chord_percent                     = 0.68285715
    segment.dihedral_outboard                      = 12 *  Units.degrees   
    segment.sweeps.quarter_chord                   = 42.5  *  Units.degrees   
    airfoil                                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                           = rel_path +  's1014.dat'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment) 


    segment                                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                                    = 'Fuel_Wall'
    segment.taper                                  = 0.32260442862265637   
    segment.percent_span_location                  = 0.2033125 / 64 * wing.spans.projected
    segment.twist                                  = wing.twists.root_twist  +  segment.percent_span_location * wing.twists.outwash 
    segment.root_chord_percent                     = 0.58
    segment.dihedral_outboard                      = 5 *  Units.degrees    
    segment.sweeps.quarter_chord                   = 25*Units.degrees
    airfoil                                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        = rel_path +  's1014.dat'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment)     
    
    segment                                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                                    = 'Wing_Section_1' 
    segment.percent_span_location                  = 0.3346858066654701
    segment.twist                                  = wing.twists.root_twist  +  segment.percent_span_location * wing.twists.outwash 
    segment.root_chord_percent                     = 0.27 
    segment.dihedral_outboard                      = 1 *  Units.degrees 
    segment.sweeps.quarter_chord                   = (30)*  Units.degrees 
    airfoil                                        =  RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        =  rel_path + 'transonic_wing_inboard_section_airfoil.txt'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment)

    segment                                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                                    = 'Wing_Section_2' 
    segment.percent_span_location                  = 0.5389354883954404
    segment.twist                                  = wing.twists.root_twist  +  segment.percent_span_location * wing.twists.outwash 
    segment.root_chord_percent                     = 0.13 
    segment.dihedral_outboard                      = 1 *  Units.degrees 
    segment.sweeps.quarter_chord                   = (30)*  Units.degrees 
    segment.chords.reference_area_root             = True
    airfoil                                        =  RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        =  rel_path + 'transonic_wing_inboard_section_airfoil.txt'
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
    airfoil.coordinate_file                        =  rel_path + 'transonic_wing_outboard_section_airfoil.txt'
    segment.append_airfoil(airfoil )
    wing.append_segment(segment)  

    segment                                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                                    = 'Wing_Section_4' 
    segment.twist                                  = 0.0 
    segment.percent_span_location                  = 1.0 
    segment.twist                                  = 0
    segment.root_chord_percent                     = 0.02 
    segment.dihedral_outboard                      = 0  
    airfoil                                        =  RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                        =  rel_path + 'transonic_wing_tip_section_airfoil.txt'
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
    wing.areas.reference                          = 26.87
    wing.spans.projected                          = 10
    wing.sweeps.quarter_chord                     = 35 * Units.degrees   
    wing.areas.wetted                             = 21
    wing.chords.root                              = 4
    wing.chords.tip                               = 1.5                      
    wing.chords.mean_aerodynamic                  = wing.chords.root * 2/3 * (( 1 + wing.taper + wing.taper**2 ) / ( 1 + wing.taper )) 
    wing.total_length                             = wing.chords.root 
    wing.taper                                    = wing.chords.tip /  wing.chords.root 
    wing.twists.root                              = 0.0 
    wing.twists.tip                               = 0.0 
    wing.origin                                   = [[21.5 ,  6.4 , 0.5]] 
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
    vehicle.append_component(main_gear)         
       
    nose_gear                                     = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()   
    nose_gear.tire_diameter                       = 40. *  Units.inches   
    nose_gear.rim_diameter                        = 16  *  Units.inches 
    nose_gear.tire_width                          = 16  *  Units.inches 
    nose_gear.strut_length                        = 9.0 * Units.ft 
    nose_gear.wheels                              = 2   
    nose_gear.number_of_gear_types_in_tandem      = 1
    nose_gear.number_of_wheels_in_gear_type       = 2    
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
    turbofan1.tag                                  = 'port_propulsor' # 'Pratt_and_Whitney_2043'  https://prd-sc102-cdn.rtx.com/-/media/pw/products/commercial-jet-engines/pw2000/files/ce_pw2000_fact.pdf?rev=7377aaa21c9b415bad4b295dd5fc4c9b&hash=8B755FC15B1A44AB0817E984A8B3E2CB   
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
    nacelle.origin                              = [[23, 4.2, 1.75]] 
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
    fuel_tank_1                                        = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank(vehicle.wings.main_wing)
    fuel_tank_1.tag                                    = 'H2_Fuel_Tank_1' 
    fuel_tank_1.fuel                                   = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()   
    # fuel_tank_1.material                               = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    # fuel_tank_1.insulation_material                    = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
    fuel_tank_1.fuel.gravimetric_efficiency            = 0.5
    # fuel_tank_1.wall_thickness                         = 2*Units.inches
    fuel_tank_1.segment.start_tag                      = 'fuel_wall'
    fuel_tank_1.segment.end_tag                        = 'wing_section_1'
    fuel_tank_1.segment.percent_chord_start_location  = 0.1  
    fuel_tank_1.segment.percent_chord_end_location    = 0.55
    fuel_line.fuel_tanks.append(fuel_tank_1)

    fuel_tank_2                                        = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank(vehicle.wings.main_wing)
    fuel_tank_2.tag                                    = 'H2_Fuel_Tank_2' 
    fuel_tank_2.fuel                                   = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()  
    # fuel_tank_2.material                               = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    # fuel_tank_2.insulation_material                    = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation() 
    fuel_tank_2.fuel.gravimetric_efficiency            = 0.5
    # fuel_tank_2.wall_thickness                         = 2*Units.inches
    fuel_tank_2.segment.start_tag                      = 'fuel_wall'
    fuel_tank_2.segment.end_tag                        = 'wing_section_1'
    fuel_tank_2.segment.percent_chord_start_location  = 0.1  
    fuel_tank_2.segment.percent_chord_end_location    = 0.55
    fuel_line.fuel_tanks.append(fuel_tank_2)

    fuel_tank_3                                        = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank(vehicle.wings.main_wing)
    fuel_tank_3.tag                                    = 'H2_Fuel_Tank_3' 
    fuel_tank_3.fuel                                   = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()   
    # fuel_tank_3.material                               = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    # fuel_tank_3.insulation_material                    = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
    fuel_tank_3.fuel.gravimetric_efficiency            = 0.5
    fuel_tank_3.segment.start_tag                      = 'fuel_wall'
    fuel_tank_3.segment.end_tag                        = 'wing_section_1'
    fuel_tank_3.segment.percent_chord_start_location  = 0.1  
    fuel_tank_3.segment.percent_chord_end_location    = 0.55
    # fuel_tank_3.wall_thickness                         = 2*Units.inches
    fuel_line.fuel_tanks.append(fuel_tank_3)    

    fuel_tank_4                                        = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank(vehicle.wings.main_wing)
    fuel_tank_4.tag                                    = 'H2_Fuel_Tank_4' 
    fuel_tank_4.fuel                                   = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()   
    # fuel_tank_4.material                               = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    # fuel_tank_4.insulation_material                    = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
    fuel_tank_4.fuel.gravimetric_efficiency            = 0.5
    fuel_tank_4.xz_plane_symmetric                     = False
    fuel_tank_4.orientation_euler_angles               = [0,0,np.pi/2]
    fuel_tank_4.bwb_aft_tank                           = True
    fuel_tank_4.aft_tank_root_chord_bounds   = [0.7,0.9]
    fuel_tank_4.segment.end_tag             = 'fuel_wall' 
    fuel_tank_4.radial_offset                          = 0.1
    fuel_line.fuel_tanks.append(fuel_tank_4)


    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to fuel line to network      
    fuel_line.assigned_propulsors =  [['starboard_propulsor', 'port_propulsor', 'center_propulsor']]

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Append fuel line to fuel line to network      
    net.fuel_lines.append(fuel_line)         

    # Append energy network to aircraft 
    vehicle.append_energy_network(net)  

    return vehicle
     

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
    config.networks.fuel.propulsors['starboard_propulsor'].emission_indices.NOx   = 4.85 /1000      
    config.networks.fuel.propulsors['port_propulsor'].emission_indices.NOx        = 4.85 /1000         
    config.networks.fuel.propulsors['center_propulsor'].emission_indices.NOx      = 4.85 /1000    
    configs.append(config) 

    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'takeoff'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 30. * Units.deg   
    config.networks.fuel.propulsors['starboard_propulsor'].fan.angular_velocity =  3470. * Units.rpm 
    config.networks.fuel.propulsors['port_propulsor'].fan.angular_velocity      =  3470. * Units.rpm 
    config.networks.fuel.propulsors['center_propulsor'].fan.angular_velocity    =  3470. * Units.rpm 
    config.V2_VS_ratio = 1.21
    configs.append(config)

    
    # ------------------------------------------------------------------
    #   Cutback Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'cutback'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 20. * Units.deg 
    config.networks.fuel.propulsors['starboard_propulsor'].fan.angular_velocity =  2780. * Units.rpm 
    config.networks.fuel.propulsors['port_propulsor'].fan.angular_velocity      =  2780. * Units.rpm
    config.networks.fuel.propulsors['center_propulsor'].fan.angular_velocity    =  2780. * Units.rpm      
    configs.append(config)   
    
        
    
    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'landing'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 30. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg 
    config.networks.fuel.propulsors['starboard_propulsor'].fan.angular_velocity =  2030. * Units.rpm 
    config.networks.fuel.propulsors['port_propulsor'].fan.angular_velocity      =  2030. * Units.rpm 
    config.networks.fuel.propulsors['center_propulsor'].fan.angular_velocity    =  2030. * Units.rpm  
    config.landing_gears.main_gear.gear_extended    = True
    config.landing_gears.nose_gear.gear_extended    = True  
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
    config.networks.fuel.propulsors['center_propulsor'].fan.angular_velocity    =  3470. * Units.rpm  
    config.landing_gears.main_gear.gear_extended    = True
    config.landing_gears.nose_gear.gear_extended    = True  
    config.V2_VS_ratio = 1.21 
    configs.append(config)

    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------  

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'reverse_thrust'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 30. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg 
    config.landing_gears.main_gear.gear_extended    = True
    config.landing_gears.nose_gear.gear_extended    = True  
    configs.append(config)    
    
    

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'descent' 
    config.wings['main_wing'].control_surfaces.spoiler.deflection  = 45. * Units.deg    
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
        if config.wings['main_wing'].control_surfaces.flap.deflection != 0: 
            analysis.aerodynamics.settings.drag_coefficient_increment =  0.05        
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
    #  Geometry
    # ------------------------------------------------------------------
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.settings.update_fuselage_properties = False
    geometry.settings.overwrite_reference        = True
    geometry.settings.update_wing_properties     = True
    geometry.settings.update_fuel_volume         = True
    geometry.settings.unique_geometry            = False
    analyses.append(geometry)
    

    # ------------------------------------------------------------------
    # Emissions 
    emissions = RCAIDE.Framework.Analyses.Emissions.Emission_Index_Correlation_Method() 
    emissions.settings.use_surrogate     = False                          
    analyses.append(emissions) 


   # ------------------------------------------------------------------
    #  Weights
    weights = RCAIDE.Framework.Analyses.Weights.Conventional_BWB()
    weights.aircraft_type                                                    = 'BWB'
    weights.settings.FLOPS.fidelity                                          = 'Complex' 
    weights.settings.weight_correction_additions.empty.structural.paint      = 464.6384576160517  
    weights.settings.weight_correction_factors.empty.systems.electrical      = 2.67
    weights.settings.weight_correction_factors.empty.systems.hydraulics      = 1.5 
    weights.settings.weight_correction_factors.empty.structural.landing_gear = 1.1 
    weights.settings.weight_correction_factors.empty.systems.control_systems = 1.9   # scaled based on wetted area when compared to 787 
    weights.settings.weight_correction_factors.empty.structurals.nacelle                        = 0.06   
    weights.settings.weight_correction_factors.empty.structurals.empennage                      = 0.08    
    analyses.append(weights)
    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()  
    aerodynamics.settings.number_of_spanwise_vortices   = 20
    aerodynamics.settings.number_of_chordwise_vortices  = 2  
    aerodynamics.settings.store_training_data           = False
    aerodynamics.training.Mach                          = np.array([0.1  ,0.3,  0.5,  0.65 , 0.85 , 0.9])
    analyses.append(aerodynamics)
 
    # ------------------------------------------------------------------
    #  Energy
    energy = RCAIDE.Framework.Analyses.Energy.Energy() 
    analyses.append(energy)

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
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses):
    """This function defines the baseline mission that will be flown by the aircraft in order
    to compute performance."""

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'
  
    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()
    base_segment.state.numerics.solver.type = 'root_finder'
    # base_segment.state.numerics.solver.max_evaluations = 500
    # base_segment.state.network_numerics.solver.type = None
 
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Takeoff Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Takeoff(base_segment)
    segment.tag = "Takeoff_Ground_Run" 
    segment.analyses.extend( analyses.takeoff )
    segment.velocity_start                                           = 0.* Units.knots
    segment.velocity_end                                             = 167.0 * Units['knots']
    segment.friction_coefficient                                     = 0.03
    segment.altitude                                                 = 0.0   
    segment.throttle                                                 = 1.0
    mission.append_segment(segment)
     
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Takeoff_Climb" 
    segment.analyses.extend( analyses.takeoff ) 
    segment.altitude_end                                             = 35 * Units['ft']
    segment.air_speed_end                                            = 175.0 * Units['knots']
    segment.climb_rate                                               = 250 * Units['fpm']  
             
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor', 'center_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment) 

    #------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Inital_Climb" 
    segment.analyses.extend( analyses.cutback )  
    segment.altitude_end                                             = 1400  * Units['feet']
    segment.air_speed                                                = 200.0 * Units['knots']
    segment.climb_rate                                               = 1800   * Units['fpm']  
              
    # define flight dynamics to model               
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor', 'center_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_1" 
    segment.analyses.extend( analyses.cutback ) 
    segment.altitude_end                                             = 8000   * Units['ft']
    segment.air_speed_end                                            = 260 * Units['knots']
    segment.climb_rate                                               = 1700   * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor', 'center_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_2" 
    segment.analyses.extend( analyses.cruise )  
    segment.altitude_end                                             = 16000   * Units['ft']
    segment.air_speed                                                = 420 * Units['knots'] 
    segment.climb_rate                                               = 1400   * Units['fpm']  
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor', 'center_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)


    segment = Segments.Climb.Constant_Mach_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_3" 
    segment.analyses.extend( analyses.cruise ) 

    segment.altitude_end                                             = 40000   * Units['ft']
    segment.mach_number                                              = 0.75
    segment.climb_rate                                               = 1100   * Units['fpm']  
              
    # define flight dynamics to model               
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor', 'center_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)
    
    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Mach_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude                                                 = 40000 * Units['ft']  
    segment.mach_number                                              = 0.78
    segment.distance                                                 = 7370/2 * Units.km  + 626 *Units.nmi  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor', 'center_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_1" 
    segment.analyses.extend( analyses.descent ) 
    segment.altitude_end                                             = 10000   * Units.ft
    segment.air_speed                                                = 400 * Units['knots']
    segment.descent_rate                                             = 1850   * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor', 'center_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag  = "approach" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                             = 2000 * Units.ft
    segment.air_speed                                                = 225.0 * Units['knots']
    segment.descent_rate                                             = 650  * Units['fpm']  
             
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor', 'center_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "final_approach"  
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                             = 50.0   * Units.ft
    segment.air_speed                                                = 175.0 * Units['knots']
    segment.descent_rate                                             = 600.0   * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor', 'center_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Fourth Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "level_off_touchdown" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                             = 0  * Units.ft
    segment.air_speed                                                = 160 * Units['knots']
    segment.descent_rate                                             = 300   * Units['fpm']  
           
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor', 'center_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Landing Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Landing(base_segment)
    segment.tag = "Landing"

    segment.analyses.extend( analyses.reverse_thrust ) 
    segment.velocity_start                                                = 160.0 * Units['knots']
    segment.velocity_end                                                  = 10 * Units.knots 
    segment.friction_coefficient                                          = 0.4
    segment.altitude                                                      = 0.0   
    segment.assigned_control_variables.elapsed_time.active                = True  
    segment.assigned_control_variables.elapsed_time.initial_guess_values  = [[30.]]  
    mission.append_segment(segment)     

    return mission 
 
def missions_setup(mission):
    """This allows multiple missions to be incorporated if desired, but only one is used here."""

    missions     = RCAIDE.Framework.Mission.Missions() 
    mission.tag  = 'base_mission'
    missions.append(mission)

    return missions

def save_aircraft_geometry(geometry,filename,save_dir): 
       
    # Create full path for pickle file
    pickle_file = os.path.join(save_dir, filename + '.pkl')
    
    os.makedirs(save_dir, exist_ok=True)
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
    # plot_liquid_hydrogen_tank_properties(results)
        
    return

if __name__ == '__main__': 
    main()    
    plt.show()
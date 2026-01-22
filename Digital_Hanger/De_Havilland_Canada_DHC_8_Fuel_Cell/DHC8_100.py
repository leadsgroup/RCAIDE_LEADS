# RESEARCH/Aircraft/DHC8-100/DHC8-100.py
# 
# 
# Created:  Mar. 2025, A. Molloy 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units           
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan   import design_turbofan    
from RCAIDE.Library.Plots                                   import *     
import RCAIDE.Framework.External_Interfaces.OpenVSP as openvsp
from   RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop  import design_turboprop
from RCAIDE.Framework.External_Interfaces.OpenVSP.export_vsp_vehicle import export_vsp_vehicle

# python imports 
import numpy as np  
from copy import deepcopy
import matplotlib.pyplot as plt  
import os
import sys
import pickle



# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():

    # Step 1 design a vehicle
    save_file_path = os.path.dirname(os.path.abspath(__file__))
    
    vehicle  = vehicle_setup(save_file_path,resize_aircraft=True) 
    # plot vehicle 
    # export_vsp_vehicle(vehicle, 'DHC8-100_Nov30')
    # plot_3d_vehicle(vehicle, fuselage_opacity            = 1.0, top_view = True) 

    # Step 2 create aircraft configuration based on vehicle 
    configs  = configs_setup(vehicle)

    # Step 3 set up analysis
    analyses = analyses_setup(configs)

    # Step 4 set up a flight mission
    mission = mission_setup(analyses)
    missions = missions_setup(mission) 

    # Step 5 execute flight profile
    results = missions.base_mission.evaluate()  

    # Step 6 plot results 
    plot_mission(results)

    # plot vehicle 
    # plot_3d_vehicle(vehicle)          


    return

def vehicle_setup(save_file_path,resize_aircraft=False, vehicle_name = 'De_Havilland_DHC8-100') :
    if resize_aircraft:
        # ------------------------------------------------------------------
        #   Initialize the Vehicle
        # ------------------------------------------------------------------      

        # Sources: https://contentzone.eurocontrol.int/aircraftperformance/details.aspx?ICAO=DH8A&NameFilter=dash
        #          https://customer.aero.bombardier.com/webd/BAG/CustSite/BRAD/RACSDocument.nsf/51aae8b2b3bfdf6685256c300045ff31/ec63f8639ff3ab9d85257c1500635bd8/$FILE/ATT19ELL.pdf/D8100-APM.pdf
    
        vehicle                                        = RCAIDE.Vehicle()       
        vehicle.tag                                    = 'De_Havilland_DHC8-100'
        vehicle.mass_properties.max_takeoff            = 15650. * Units.kilogram 
        vehicle.mass_properties.takeoff                = 15650 * Units.kilogram 
        vehicle.mass_properties.operating_empty        = 10245 * Units.kilogram  
        vehicle.mass_properties.max_zero_fuel          = 14061.0 * Units.kilogram  
        vehicle.mass_properties.max_fuel               = 2576 * Units.kilogram  
        vehicle.mass_properties.max_payload            = 3814 *Units.kilogram  
        vehicle.mass_properties.payload                = 3814 *Units.kilogram  
        vehicle.mass_properties.center_of_gravity      = [[9.94, 0, 0]]
        vehicle.flight_envelope.ultimate_load          = 3.75 
        vehicle.flight_envelope.positive_limit_load    = 2.5  
        vehicle.flight_envelope.negative_limit_load    = 0.75
        vehicle.flight_envelope.design_mach_number     = 0.38  
        vehicle.flight_envelope.design_cruise_altitude = 25000.0*Units.feet 
        vehicle.flight_envelope.design_range           = 1350.0 * Units.nmi
        vehicle.reference_area                         = 395.0 * Units['meters**2']    
        vehicle.number_of_passengers                             = 37 
        vehicle.systems.control                        = "fully powered" 
        vehicle.systems.accessories                    = "short range"

        #------------------------------------------------------------------------------------------------------------------------------------  
        #  Main Wing
        #------------------------------------------------------------------------------------------------------------------------------------
        
        wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
        wing.tag                              = 'main_wing'
        wing.aspect_ratio                     = 11.07
        wing.sweeps.quarter_chord             = 1 * Units.deg
        wing.thickness_to_chord               = 0.1
        wing.spans.projected                  = 25.57
        wing.chords.root                      = 2.75 * Units.meter
        wing.chords.tip                       = 1.39 * Units.meter
        wing.taper                            = wing.chords.tip / wing.chords.root
        wing.chords.mean_aerodynamic          = 2.41171 * Units.meter 
        wing.areas.reference                  = 59.06
        wing.areas.wetted                     = 124.026
        wing.twists.root                      = 0.0 * Units.degrees 
        wing.twists.tip                       = 0.0 * Units.degrees 
        wing.origin                           = [[8.197,0,2.131]]
        wing.aerodynamic_center               = [0,0,0] 
        wing.vertical                         = False
        wing.dihedral                         = 0.5 * Units.degrees 
        wing.xz_plane_symmetric               = True 
        wing.high_lift                        = True 
        wing.dynamic_pressure_ratio           = 1.0
            
        # Wing Segments

        root_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
        ospath                                = os.path.abspath(__file__)
        separator                             = os.path.sep
        rel_path                              = os.path.dirname(ospath) + separator  + '..'  + separator 
        root_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator + 'transonic_wing_root_section_airfoil.txt'
        segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                           = 'Root'
        segment.percent_span_location         = 0.0
        segment.twist                         = 0. * Units.deg
        segment.root_chord_percent            = 1.
        segment.thickness_to_chord            = 0.15
        segment.dihedral_outboard             = 0 * Units.degrees
        segment.sweeps.quarter_chord          = 0 * Units.degrees
        segment.append_airfoil(root_airfoil)
        wing.append_segment(segment)

        mid_airfoil                           = RCAIDE.Library.Components.Airfoils.Airfoil()
        mid_airfoil.coordinate_file           = rel_path+ 'Airfoils' + separator + 'transonic_wing_inboard_section_airfoil.txt'
        segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                           = 'Yehudi'
        segment.percent_span_location         = 0.35
        segment.twist                         = 0.0 * Units.deg
        segment.root_chord_percent            = 1
        segment.thickness_to_chord            = 0.15
        segment.dihedral_outboard             = 1.5 * Units.degrees
        segment.sweeps.quarter_chord          = 3.357 * Units.degrees
        segment.append_airfoil(mid_airfoil)
        wing.append_segment(segment)

        tip_airfoil                           =  RCAIDE.Library.Components.Airfoils.Airfoil()
        tip_airfoil.coordinate_file           = rel_path + 'Airfoils' + separator + 'transonic_wing_tip_section_airfoil.txt'
        segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                           = 'Tip'
        segment.percent_span_location         = 1.
        segment.twist                         = 0 * Units.degrees
        segment.root_chord_percent            = 0.5
        segment.thickness_to_chord            = 0.1
        segment.dihedral_outboard             = 0 * Units.degrees
        segment.sweeps.quarter_chord          = 0 * Units.degrees
        segment.append_airfoil(tip_airfoil)
        wing.append_segment(segment)


        # control surfaces
        
        flap                                  = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
        flap.tag                              = 'flap'
        flap.span_fraction_start              = 0.15
        flap.span_fraction_end                = 0.8
        flap.deflection                       = 0.0 * Units.degrees
        flap.configuration_type               = 'double_slotted'
        flap.chord_fraction                   = 0.3
        wing.append_control_surface(flap)

        aileron                               = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
        aileron.tag                           = 'aileron'
        aileron.span_fraction_start           = 0.8
        aileron.span_fraction_end             = 1.0
        aileron.deflection                    = 0.0 * Units.degrees
        aileron.chord_fraction                = 0.33
        wing.append_control_surface(aileron)

        vehicle.append_component(wing)

        # ------------------------------------------------------------------
        #  Horizontal Stabilizer
        # ------------------------------------------------------------------

        wing     = RCAIDE.Library.Components.Wings.Horizontal_Tail()
        wing.tag = 'horizontal_stabilizer'

        wing.aspect_ratio                     = 4.33
        wing.sweeps.quarter_chord             = 5.89 * Units.deg  
        wing.thickness_to_chord               = 0.1
        wing.taper                            = 0.746  
        wing.spans.projected                  = 8.14
        wing.chords.root                      = 2.15833
        wing.chords.tip                       = 1.6
        wing.chords.mean_aerodynamic          = 1.879
        wing.areas.reference                  = 15.3079
        wing.areas.exposed                    = 32.133    # Exposed area of the horizontal tail
        wing.areas.wetted                     = 32.133     # Wetted area of the horizontal tail
        wing.twists.root                      = 0.0 * Units.degrees
        wing.twists.tip                       = 0.0 * Units.degrees 
        wing.origin                           = [[19.877, 0, 6.242]]
        wing.aerodynamic_center               = [0,0,0] 
        wing.vertical                         = False
        wing.xz_plane_symmetric               = True 
        wing.dynamic_pressure_ratio           = 0.9

        # Segments

        segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                    = 'root_segment'
        segment.percent_span_location  = 0.0
        segment.twist                  = 0. * Units.deg
        segment.root_chord_percent     = 1.0
        segment.dihedral_outboard      = 0 * Units.degrees
        segment.sweeps.quarter_chord   = 5.8988 * Units.degrees 
        segment.thickness_to_chord     = .1
        wing.append_segment(segment)

        segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                    = 'tip_segment'
        segment.percent_span_location  = 1.
        segment.twist                  = 0. * Units.deg
        segment.root_chord_percent     = 0.741              
        segment.dihedral_outboard      = 0 * Units.degrees
        segment.sweeps.quarter_chord   = 0 * Units.degrees  
        segment.thickness_to_chord     = .1
        wing.append_segment(segment)
                

        # control surfaces 

        elevator                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
        elevator.tag                   = 'elevator'
        elevator.span_fraction_start   = 0.05
        elevator.span_fraction_end     = 1.0
        elevator.deflection            = 0.0  * Units.deg
        elevator.chord_fraction        = 0.37
        wing.append_control_surface(elevator)

        vehicle.append_component(wing)

        # ------------------------------------------------------------------
        #   Vertical Stabilizer
        # ------------------------------------------------------------------

        wing = RCAIDE.Library.Components.Wings.Vertical_Tail()
        wing.tag = 'vertical_stabilizer'

        wing.aspect_ratio            = 0.98439
        wing.sweeps.quarter_chord    = 32.5  * Units.deg   
        wing.thickness_to_chord      = 0.08
        wing.taper                   = 0.30

        wing.spans.projected         = 4.48
        wing.total_length            = wing.spans.projected 
        
        wing.chords.root             = 10.15
        wing.chords.tip              = 3.0 
        wing.chords.mean_aerodynamic = 5.215

        wing.areas.reference         = 20.46834
        wing.areas.wetted            = 43.0 
        
        wing.twists.root             = 0.0 * Units.degrees
        wing.twists.tip              = 0.0 * Units.degrees

        wing.origin                  = [[10.603, 0, 1.810]]
        wing.aerodynamic_center      = [0,0,0]

        wing.vertical                = True
        wing.xz_plane_symmetric      = False
        wing.t_tail                  = True

        wing.dynamic_pressure_ratio  = 1.0

        # Segments

        segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                           = 'root'
        segment.percent_span_location         = 0.0
        segment.twist                         = 0. * Units.deg
        segment.root_chord_percent            = 1.
        segment.dihedral_outboard             = 0 * Units.degrees
        segment.sweeps.quarter_chord          = 75.0 * Units.degrees  
        segment.thickness_to_chord            = .04
        wing.append_segment(segment)

        segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                           = 'segment_1'
        segment.percent_span_location         = 0.318
        segment.twist                         = 0. * Units.deg
        segment.root_chord_percent            = 0.3793
        segment.dihedral_outboard             = 0. * Units.degrees
        segment.sweeps.quarter_chord          = 32.53 * Units.degrees   
        segment.thickness_to_chord            = .1
        wing.append_segment(segment)

        segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                           = 'segment_2'
        segment.percent_span_location         = 1.0
        segment.twist                         = 0. * Units.deg
        segment.root_chord_percent            = 0.2955
        segment.dihedral_outboard             = 0.0 * Units.degrees
        segment.sweeps.quarter_chord          = 0.0    
        segment.thickness_to_chord            = .1  
        wing.append_segment(segment)
        
        # control surfaces

        rudder                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
        rudder.tag                   = 'rudder'
        rudder.span_fraction_start   = 0.1 
        rudder.span_fraction_end     = 1.0  
        rudder.deflection            = 0 
        rudder.chord_fraction        = 0.30  
        wing.append_control_surface(rudder)      

        # add to vehicle
        vehicle.append_component(wing)

        # ------------------------------------------------------------------
        #   Fuselage
        # ------------------------------------------------------------------

        fuselage                                    = RCAIDE.Library.Components.Fuselages.Fuselage() 
        fuselage.number_coach_seats                 = vehicle.number_of_passengers 
        fuselage.seats_abreast                      = 4
        fuselage.seat_pitch                         = 0.9 * Units.meter 
        fuselage.fineness.nose                      = 1.5
        fuselage.fineness.tail                      = 2.5 
        fuselage.lengths.nose                       = 2.9412 * Units.meter
        fuselage.lengths.tail                       = 8.24 * Units.meter
        fuselage.lengths.total                      = 20.78 * Units.meter  
        fuselage.lengths.fore_space                 = 0.75 * Units.meter
        fuselage.lengths.aft_space                  = 3 * Units.meter
        fuselage.width                              = 2.75 * Units.meter
        fuselage.heights.maximum                    = 2.65 * Units.meter
        fuselage.effective_diameter                 = 2.7 * Units.meter
        fuselage.areas.side_projected               = fuselage.heights.maximum * fuselage.lengths.total * Units['meters**2'] 
        fuselage.areas.wetted                       = np.pi * fuselage.width/2 * fuselage.lengths.total * Units['meters**2'] 
        fuselage.areas.front_projected              = np.pi * fuselage.width/2 * Units['meters**2']  
        fuselage.differential_pressure              = 5.0e4 * Units.pascal
        fuselage.heights.at_quarter_length          = fuselage.heights.maximum * Units.meter
        fuselage.heights.at_three_quarters_length   = fuselage.heights.maximum * Units.meter
        fuselage.heights.at_wing_root_quarter_chord = fuselage.heights.maximum * Units.meter
        
        # define cabin    
        cabin                                             = RCAIDE.Library.Components.Fuselages.Cabins.Cabin()
        cabin.offset_x                                    = 3.5  #origin                                      = [[2, 0, 0]]
        economy_class                                     = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
        economy_class.number_of_seats_abrest              = 4
        economy_class.seat_pitch                          = 31 * Units.inches
        economy_class.aisle_width                         = 0.40 * Units.meters
        economy_class.seat_width                          = 17 *  Units.inches
        economy_class.number_of_rows                      = 9
        economy_class.galley_lavatory_percent_x_locations = [0 ]  
        economy_class.emergency_exit_percent_x_locations  = []      
        economy_class.type_A_exit_percent_x_locations     = [0.01,1] 
        economy_class.number_of_seats                     = economy_class.number_of_rows  * economy_class.number_of_seats_abrest 
        # cabin.append_cabin_class(economy_class)
        # fuselage.append_cabin(cabin)
    

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
        segment.percent_x_location                  = 0.01147
        segment.percent_z_location                  = 0.00254
        segment.height                              = 0.53670
        segment.width                               = 0.63073
        fuselage.append_segment(segment)   
        
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_2'   
        segment.percent_x_location                  = 0.04508
        segment.percent_z_location                  = 0.01171
        segment.height                              = 1.22
        segment.width                               = 1.31881
        fuselage.append_segment(segment)      
        
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_3'   
        segment.percent_x_location                  = 0.08302
        segment.percent_z_location                  = 0.02101
        segment.height                              = 1.75
        segment.width                               = 1.9454
        fuselage.append_segment(segment)   

        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_4'   
        segment.percent_x_location                  = 0.10807
        segment.percent_z_location                  = 0.03048
        segment.height                              = 2.20183
        segment.width                               = 2.275
        fuselage.append_segment(segment)   
        
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_5'   
        segment.percent_x_location                  = 0.12066
        segment.percent_z_location                  = 0.03479
        segment.height                              = 2.4
        segment.width                               = 2.4
        fuselage.append_segment(segment)     
        
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_6'   
        segment.percent_x_location                  = 0.14154
        segment.percent_z_location                  = 0.03901
        segment.height                              = 2.58
        segment.width                               = 2.6
        fuselage.append_segment(segment)             
        
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_7'   
        segment.percent_x_location                  = 0.17278
        segment.percent_z_location                  = 0.04064
        segment.height                              = 2.6789
        segment.width                               = 2.75
        fuselage.append_segment(segment)    
        
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_8'   
        segment.percent_x_location                  = 0.55671
        segment.percent_z_location                  = 0.04268
        segment.height                              = 2.65
        segment.width                               = 2.75
        fuselage.append_segment(segment)   
        
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_9'     
        segment.percent_x_location                  = 0.60348
        segment.percent_z_location                  = 0.04123
        segment.height                              = 2.58
        segment.width                               = 2.75
        fuselage.append_segment(segment)     
            
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_10'     
        segment.percent_x_location                  = 0.80902
        segment.percent_z_location                  = 0.05797
        segment.height                              = 1.72477
        segment.width                               = 1.94954
        fuselage.append_segment(segment)   
            
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_11'     
        segment.percent_x_location                  = 0.95882
        segment.percent_z_location                  = 0.07021
        segment.height                              = 0.95413
        segment.width                               = 0.97477
        fuselage.append_segment(segment)    
            
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_12'     
        segment.percent_x_location                  = 0.99509
        segment.percent_z_location                  = 0.07511
        segment.height                              = 0.48695
        segment.width                               = 0.34404
        fuselage.append_segment(segment)             
            
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_13'     
        segment.percent_x_location                  = 1
        segment.percent_z_location                  = 0.08327
        segment.height                              = 0
        segment.width                               = 0
        fuselage.append_segment(segment)               
            
        # add to vehicle
        vehicle.append_component(fuselage)

        # ------------------------------------------------------------------
        #   Landing Gear
        # ------------------------------------------------------------------   

        main_gear               = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear()
        main_gear.tire_diameter = 0.5 * Units.meter
        main_gear.strut_length  = 1.9 * Units.meter 
        main_gear.units         = 1    # Number of main landing gear
        main_gear.wheels        = 2    # Number of wheels on the main landing gear
        vehicle.append_component(main_gear)  

        nose_gear               = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()       
        nose_gear.tire_diameter = 0.4 * Units.meter
        nose_gear.units         = 1    # Number of nose landing gear
        nose_gear.wheels        = 2    # Number of wheels on the nose landing gear
        nose_gear.strut_length  = 0.5 * Units.meter 
        vehicle.append_component(nose_gear)
        
        
        # ########################################################  Energy Network  #########################################################  
        net                                              = RCAIDE.Framework.Networks.Fuel()    

        #------------------------------------------------------------------------------------------------------------------------- 
        # Fuel Distrubition Line 
        #------------------------------------------------------------------------------------------------------------------------- 
        fuel_line                                       = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()  

        #------------------------------------------------------------------------------------------------------------------------------------  
        # Propulsor
        #------------------------------------------------------------------------------------------------------------------------------------    
        starboard_propulsor                              = RCAIDE.Library.Components.Powertrain.Propulsors.Turboprop()        
        starboard_propulsor.tag                          = 'starboard_propulsor'  
        starboard_propulsor.origin                       = [[5.780,3.945, 1.86 ]]
        starboard_propulsor.working_fluid                = RCAIDE.Library.Attributes.Gases.Air()               
        starboard_propulsor.gearbox.efficiency           = 0.99                                             
        starboard_propulsor.design_thrust                = 8200.0 * Units.N 
        starboard_propulsor.design_altitude              = 25000*Units.ft                                
        starboard_propulsor.design_freestream_velocity   = 270 * Units.kts  

        #Propeller Design              
        propeller                                        = RCAIDE.Library.Components.Powertrain.Converters.Propeller()   
        propeller.tag                                    = 'starboard_propulsor_propeller' 
        propeller.origin                                 = [[6.4, 3.945, 1.86 ]]
        propeller.active                                 = True          
        propeller.tip_radius                             = 3.96/2
        propeller.hub_radius                             = 0.1 
        propeller.number_of_blades                       = 4   
        propeller.design_efficiency                      = 0.83      
        propeller.design_angular_velocity                = 2000.0 * Units.rpm        
        propeller.design_thrust                          = starboard_propulsor.design_thrust              
        propeller.design_altitude                        = starboard_propulsor.design_altitude                                        
        propeller.design_freestream_velocity             = starboard_propulsor.design_freestream_velocity                                                               
        starboard_propulsor.propeller                    = propeller     
        
        # Ram inlet 
        ram                                              = RCAIDE.Library.Components.Powertrain.Converters.Ram()
        ram.tag                                          = 'ram' 
        starboard_propulsor.ram                          = ram 
            
        # inlet nozzle          
        inlet_nozzle                                     = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
        inlet_nozzle.tag                                 = 'inlet nozzle'                                       
        inlet_nozzle.pressure_ratio                      = 0.98
        inlet_nozzle.compressibility_effects             = False
        starboard_propulsor.inlet_nozzle                 = inlet_nozzle
                                                        
        # compressor                        
        compressor                                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
        compressor.tag                                   = 'lpc'                   
        compressor.pressure_ratio                        = 10                   
        starboard_propulsor.compressor                   = compressor
        
        # combustor      
        combustor                                        = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
        combustor.tag                                    = 'Comb'
        combustor.efficiency                             = 0.99                   
        combustor.turbine_inlet_temperature              = 1370                    
        combustor.pressure_ratio                         = 0.96                    
        combustor.fuel_data                              = RCAIDE.Library.Attributes.Propellants.Jet_A()  
        starboard_propulsor.combustor                    = combustor
            
        # high pressure turbine         
        high_pressure_turbine                            = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
        high_pressure_turbine.tag                        ='hpt'
        high_pressure_turbine.mechanical_efficiency      = 0.99                       
        starboard_propulsor.high_pressure_turbine        = high_pressure_turbine 
            
        # low pressure turbine      
        low_pressure_turbine                             = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
        low_pressure_turbine.tag                         ='lpt'
        low_pressure_turbine.mechanical_efficiency       = 0.99                      
        starboard_propulsor.low_pressure_turbine         = low_pressure_turbine
        
        # core nozzle    
        core_nozzle                                      = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
        core_nozzle.tag                                  = 'core nozzle'          
        core_nozzle.pressure_ratio                       = 0.99
        starboard_propulsor.core_nozzle                  = core_nozzle
        
        # design starboard_propulsor
        design_turboprop(starboard_propulsor)


        # ------------------------------------------------------------------
        #   Nacelles
        # ------------------------------------------------------------------

        nacelle                    = RCAIDE.Library.Components.Nacelles.Stack_Nacelle()
        nacelle.tag                = 'nacelle_1'
        nacelle.length             = 7
        nacelle.diameter           = 1.178 
        nacelle.areas.wetted       = 0.01*(2*np.pi*0.01/2)
        nacelle.origin             = [[5.780,3.945 , 1.86]]
        nacelle.flow_through       = False  
        
        nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
        nac_segment.tag                = 'segment_0'
        nac_segment.percent_x_location = 0.0  
        nac_segment.height             = 0.0
        nac_segment.width              = 0.0
        nacelle.append_segment(nac_segment)   

        nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
        nac_segment.tag                = 'segment_1a'
        nac_segment.percent_x_location = 0.10413
        nac_segment.percent_z_location = 0.0
        nac_segment.height             = 0.73394
        nac_segment.width              = 0.73394
        nacelle.append_segment(nac_segment)   

        nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
        nac_segment.tag                = 'segment_1'
        nac_segment.percent_x_location = 0.12047
        nac_segment.percent_z_location = -0.03225
        nac_segment.height             = 1.2844
        nac_segment.width              = 0.8
        nacelle.append_segment(nac_segment)   
            
        nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
        nac_segment.tag                = 'segment_2'
        nac_segment.percent_x_location = 0.27193
        nac_segment.percent_z_location = -0.03184
        nac_segment.height             = 1.52
        nac_segment.width              = 0.92 
        nacelle.append_segment(nac_segment)   

        nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
        nac_segment.tag                = 'segment_2b'
        nac_segment.percent_x_location = 0.75
        nac_segment.percent_z_location = -0.04327
        nac_segment.height             = 1.35780	 
        nac_segment.width              = 0.92 
        nacelle.append_segment(nac_segment)  
            
        nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
        nac_segment.tag                = 'segment_3'
        nac_segment.percent_x_location = 0.88188
        nac_segment.percent_z_location = -0.04449
        nac_segment.height             = 1.06422	 
        nac_segment.width              = 0.57339 
        nacelle.append_segment(nac_segment)  
            
        nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
        nac_segment.tag                = 'segment_4'
        nac_segment.percent_x_location = 0.94094  
        nac_segment.percent_z_location = -0.04368
        nac_segment.height             = 0.80734	 
        nac_segment.width              = 0.34404 
        nacelle.append_segment(nac_segment)  
            
        nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
        nac_segment.tag                = 'segment_5'
        nac_segment.percent_x_location = 1.0  
        nac_segment.percent_z_location = -0.0449
        nac_segment.height             = 0.0 
        nac_segment.width              = 0.0
        nacelle.append_segment(nac_segment)   
        
        starboard_propulsor.nacelle =  nacelle

        net.propulsors.append(starboard_propulsor)    

        #------------------------------------------------------------------------------------------------------------------------------------  
        # Propulsor: Port Propulsor
        #------------------------------------------------------------------------------------------------------------------------------------      
        port_propulsor                                  = deepcopy(starboard_propulsor) 
        port_propulsor.tag                              = 'port_propulsor' 
        port_propulsor.origin                           = [[5.780,-3.945 , 1.86]]   # change origin 
        port_propulsor.nacelle.tag                      = 'port_propulsor_nacelle' 
        port_propulsor.nacelle.origin                   = [[5.780,-3.945 , 1.86]] 
        port_propulsor.propeller.tag                    = 'port_propulsor_propeller' 
        port_propulsor.propeller.origin                 = [[6.40,-3.945 , 1.86]] 
            
        # append propulsor to distribution line 
        net.propulsors.append(port_propulsor) 

        #------------------------------------------------------------------------------------------------------------------------------------   
        # Assign propulsors to fuel line    
        fuel_line.assigned_propulsors =  [[starboard_propulsor.tag, port_propulsor.tag]]
        
        # Append fuel line to Network      
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
    #   Takeoff Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'takeoff'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    #config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity =  3470. * Units.rpm
    #config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity      =  3470. * Units.rpm 
    config.V2_VS_ratio = 1.21
    configs.append(config)


    # ------------------------------------------------------------------
    #   Cutback Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'cutback'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    #config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity =  2780. * Units.rpm
    #config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity      =  2780. * Units.rpm 
    configs.append(config)   



    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'landing'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 30. * Units.deg
    #config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity =  2030. * Units.rpm
    #config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity      =  2030. * Units.rpm
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
    #config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity =  3470. * Units.rpm
    #config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity      =  3470. * Units.rpm 
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
    config.landing_gears.main_gear.gear_extended    = True
    config.landing_gears.nose_gear.gear_extended    = True  
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
    analyses.vehicle.mass_properties.takeoff = None

    #  Geometry
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.settings.overwrite_reference        = True
    geometry.settings.update_wing_properties     = True
    geometry.settings.print_weight_analysis_report = True
    analyses.append(geometry)
 
    # ------------------------------------------------------------------
    #  Weights
    weights = RCAIDE.Framework.Analyses.Weights.Conventional()
    weights.method = 'Raymer'
    weights.aircraft_type                                 = 'Transport'
    # weights.settings.FLOPS.fidelity                        = 'Complex' 
    weights.settings.weight_correction_factors.empty.propulsion.engines = 1.4
    weights.settings.update_center_of_gravity                            = False
    weights.settings.update_moment_of_inertia = True
    
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Stability Analysis
    # ------------------------------------------------------------------     
    stability                                           = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method() 
    stability.settings.compute_neutral_point = True
    stability.settings.model_fuselage   = True
    analyses.append(stability)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
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
    atmosphere.features.planet = planet.features
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


    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Takeoff Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Takeoff(base_segment)
    segment.tag = "Takeoff" 
    segment.analyses.extend( analyses.takeoff )
    segment.velocity_start           = 5.* Units.knots
    segment.velocity_end             = 80.0 * Units['knots']
    segment.friction_coefficient     = 0.02
    segment.altitude                 = 0.0   
    segment.throttle                 = 0.8
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1" 
    segment.analyses.extend( analyses.takeoff ) 
    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 5000   * Units.feet
    segment.air_speed      = 130.0 * Units['knots']
    segment.climb_rate     = 1800   * Units['ft/min']  

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_2" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end   = 15000.0   * Units.feet
    segment.air_speed      = 200.0 * Units['knots']
    segment.climb_rate     = 1500   * Units['ft/min']  

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Third Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_3" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end = 24000.0   * Units.feet
    segment.air_speed    = 200.0  * Units['knots']
    segment.climb_rate   = 1000   * Units['ft/min']  

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)

    
    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude                                      = 24000.0 * Units.feet
    segment.air_speed                                     = 250 * Units['knots']
    segment.distance                                      = 500 * Units.nmi   

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)

    
    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_1" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_start                                = 24000.0 * Units.feet 
    segment.altitude_end                                  = 10000.0   * Units.feet
    segment.air_speed                                     = 250.0 * Units['knots']
    segment.descent_rate                                  = 1250   * Units['ft/min']  

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_3"  
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                  = 2000.0   * Units.feet
    segment.air_speed                                     = 180.0 * Units['knots']
    segment.descent_rate                                  = 1000   * Units['ft/min']  

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Fifth Descent Segment:Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_5" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                  = 0.0   * Units.km
    segment.air_speed                                     = 120.0 * Units['knots']
    segment.descent_rate                                  = 3.5   * Units['knots']  

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Landing Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Landing(base_segment)
    segment.tag = "Landing"

    segment.analyses.extend( analyses.reverse_thrust ) 
    segment.velocity_start                                = 100.0 * Units['m/s']
    segment.velocity_end                                  = 5.0 * Units.knots 
    segment.friction_coefficient                          = 0.4
    segment.altitude                                      = 0.0   
    segment.assigned_control_variables.elapsed_time.active           = True  
    segment.assigned_control_variables.elapsed_time.initial_guess_values  = [[30.]]  
    mission.append_segment(segment)     


    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------
    
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
  
    # Plot throttles
    plot_propulsor_throttles(results)

  

    return

if __name__ == '__main__': 
    main()    
    plt.show()
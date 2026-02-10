# Airbus_AS350.py
# 
# Created: 2025, A. Molloy
# 
# Note! Model is not complete as of 10/18/2025

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports 
import RCAIDE
from   RCAIDE.Framework.Core import Units
from   RCAIDE.Library.Plots  import *  
from   RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop  import design_turboprop
from RCAIDE.Framework.External_Interfaces.OpenVSP.export_vsp_vehicle import export_vsp_vehicle     
from RCAIDE.Library.Methods.Performance import *  

# python imports 
import numpy as np  
from   copy import deepcopy
import matplotlib.pyplot as plt 
import  os
import  sys 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main(plot_results=True, plot_vehicle=True): 

    vehicle  = vehicle_setup()

    if plot_vehicle:
        plot_3d_vehicle(vehicle, 
                    fuselage_opacity            = 0.25, 
                    nacelle_opacity             = 0.5, 
                    boom_opacity                = 1.0,
                    fuel_tank_opacity=1.0, 
                    fuel_tank_color= 'lightblue', 
                    boom_color                  = 'lightgreen')
    
    configs  = configs_setup(vehicle)
     
    analyses = analyses_setup(configs)

    mission  = mission_setup(analyses) 

    missions = missions_setup(mission) 

    results  = missions.base_mission.evaluate()   
   
    if plot_results:
        plot_mission(results)   
        
    return results
 
def vehicle_setup():

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------

    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'AS350'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------

    # mass properties
    vehicle.mass_properties.max_takeoff               =  
    vehicle.mass_properties.takeoff                   =   
    vehicle.mass_properties.operating_empty           = 
    vehicle.mass_properties.max_zero_fuel             =  
    vehicle.mass_properties.max_payload               = 
    vehicle.mass_properties.cargo                     = 
    vehicle.mass_properties.center_of_gravity         = [[ , , ]] # Unknown 
    vehicle.mass_properties.moments_of_inertia.tensor = [[ , , ]] # Unknown 
    vehicle.mass_properties.max_fuel                  = 
    vehicle.mass_properties.fuel                      = 

    # envelope properties
    vehicle.flight_envelope.design_mach_number        = 0.2 
    vehicle.flight_envelope.design_range              = 400 * Units.nmi
    vehicle.flight_envelope.design_cruise_altitude    = 10000 * Units.feet
    vehicle.flight_envelope.ultimate_load             = 3.75
    vehicle.flight_envelope.positive_limit_load       = 1.5
              
    # basic parameters              
    vehicle.reference_area                            = 61.0  
    vehicle.number_of_passengers                      = 4
    vehicle.systems.control                           = "fully powered"
    vehicle.systems.accessories                       = "short range"  


    # ################################################# Landing Gear #############################################################    
    
    main_gear                                = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear() 
    main_gear.tire_diameter                  = 34  *  Units.inches 
    main_gear.rim_diameter                   = 16  *  Units.inches 
    main_gear.tire_width                     = 10  *  Units.inches 
    main_gear.strut_length                   = 1 *  Units.meter 
    main_gear.wheels                         = 4   
    main_gear.number_of_gear_types_in_tandem = 1
    main_gear.number_of_wheels_in_gear_type  = 2  
    main_gear.xz_plane_symmetric             = True
    vehicle.append_component(main_gear)  

    nose_gear                                 = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()   
    nose_gear.tire_diameter                   = 17   *  Units.inches   
    nose_gear.rim_diameter                    = 7    *  Units.inches 
    nose_gear.tire_width                      = 17   *  Units.inches 
    nose_gear.strut_length                    = 1 *  Units.meter 
    nose_gear.wheels                          = 2   
    nose_gear.number_of_gear_types_in_tandem  = 1
    nose_gear.number_of_wheels_in_gear_type   = 2    
    vehicle.append_component(nose_gear)

    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------ 
    wing                         = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag                     = 'horizontal_stabilizer'  
    wing.spans.projected         = 3.61*2 
    wing.areas.reference         = 15.2 
    wing.aspect_ratio            = (wing.spans.projected**2) /  wing.areas.reference
    wing.sweeps.leading_edge     = 11.56*Units.degrees  
    wing.thickness_to_chord      = 0.12  
    wing.chords.root             = 2.078645129 
    wing.chords.tip              = 0.953457347 
    wing.total_length            = wing.chords.root  
    wing.taper                   = wing.chords.tip/wing.chords.root  
    wing.chords.mean_aerodynamic = wing.chords.root * 2/3 * (( 1 + wing.taper + wing.taper**2 )/( 1 + wing.taper ))
    wing.twists.root             = 0 * Units.degrees  
    wing.twists.tip              = 0 * Units.degrees   
    wing.origin                  = [[25.505088,0,5.510942426]]  
    wing.aerodynamic_center      = [25.505088+ 0.25*wing.chords.root,0,2.009316366] 
    wing.vertical                = False  
    wing.xz_plane_symmetric      = True  
    wing.dynamic_pressure_ratio  = 0.95 

    # Wing Segments
    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'root_segment'
    segment.percent_span_location  = 0.0
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 1.0
    segment.sweeps.leading_edge    = wing.sweeps.leading_edge 
    segment.thickness_to_chord     = .12
    wing.append_segment(segment)

    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'tip_segment'
    segment.percent_span_location  = 1.
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = wing.taper          
    segment.thickness_to_chord     = .12
    wing.append_segment(segment)      

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------ 
    wing                                   = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag                               = 'vertical_stabilizer'   
    wing.spans.projected                   = 4.5  
    wing.areas.reference                   = 12.7
    wing.sweeps.quarter_chord              = 54 * Units.degrees  
    wing.thickness_to_chord                = 0.1  
    wing.aspect_ratio                      = (wing.spans.projected**2) /  wing.areas.reference    
    wing.chords.root                       = 8.75
    wing.chords.tip                        = 1.738510759 
    wing.total_length                      = wing.chords.root  
    wing.taper                             = wing.chords.tip/wing.chords.root  
    wing.chords.mean_aerodynamic           = wing.chords.root * 2/3 * (( 1 + wing.taper + wing.taper**2 )/( 1 + wing.taper )) 
    wing.areas.exposed                     = 2 * wing.areas.reference
    wing.areas.wetted                      = 2 * wing.areas.reference 
    wing.twists.root                       = 0 * Units.degrees  
    wing.twists.tip                        = 0 * Units.degrees   
    wing.origin                            = [[17.34807199,0,1.3]]  
    wing.aerodynamic_center                = [17.34807199,0,1.3+ 0.25*wing.chords.root]   
    wing.vertical                          = True  
    wing.xz_plane_symmetric                = False
    wing.dynamic_pressure_ratio            = 1.0  
 
    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_1'
    segment.percent_span_location         = 0.0
    segment.twist                         = 0.0
    segment.root_chord_percent            = 1.0
    segment.dihedral_outboard             = 0.0
    segment.sweeps.leading_edge           = 75 * Units.degrees  
    segment.thickness_to_chord            = 0.15
    wing.append_segment(segment)
    
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_5'
    segment.percent_span_location         = 1.0
    segment.twist                         = 0.0
    segment.root_chord_percent            = 1.3/wing.chords.root  
    segment.dihedral_outboard             = 0  
    segment.sweeps.leading_edge           = 0 * Units.degrees   
    segment.thickness_to_chord            = 0.13
    wing.append_segment(segment)   
    
    # add to vehicle
    vehicle.append_component(wing) 

    # ########################################################## Fuselage  ################################################################   
    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------ 
    fuselage = RCAIDE.Library.Components.Fuselages.Fuselage()
    fuselage.tag = 'fuselage' 
    fuselage.seats_abreast                      = 4 
    fuselage.seat_pitch                         = 1  
    fuselage.fineness.nose                      = 1.6
    fuselage.fineness.tail                      = 2. 
    fuselage.lengths.total                      = 27.12   
    fuselage.lengths.nose                       = 3.375147531 
    fuselage.lengths.tail                       = 9.2 
    fuselage.effective_diameter                 = 2.985093814  
    fuselage.lengths.cabin                      = fuselage.lengths.total- (fuselage.lengths.nose + fuselage.lengths.tail  )
    fuselage.width                              = 2.985093814  
    fuselage.heights.maximum                    = 2.755708426  
    fuselage.areas.side_projected               = fuselage.heights.maximum * fuselage.lengths.total * Units['meters**2'] 
    fuselage.areas.wetted                       = np.pi * fuselage.width   * fuselage.lengths.total * Units['meters**2'] 
    fuselage.areas.front_projected              = np.pi * (fuselage.width/2) **2   
    fuselage.differential_pressure              = 5.0e4 * Units.pascal
    fuselage.heights.at_quarter_length          = fuselage.heights.maximum * Units.meter
    fuselage.heights.at_three_quarters_length   = fuselage.heights.maximum * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = fuselage.heights.maximum* Units.meter

    # define cabin    
    cabin                                             = RCAIDE.Library.Components.Fuselages.Cabins.Cabin()
    economy_class                                     = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest              = 4
    economy_class.number_of_rows                      = 1
    economy_class.number_of_seats                     = economy_class.number_of_rows  * economy_class.number_of_seats_abrest 
    cabin.append_cabin_class(economy_class)
    fuselage.append_cabin(cabin)
    
     # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_1'    
    segment.percent_x_location                  = 0.0000
    segment.percent_z_location                  = 0.0000
    segment.height                              = 0.0
    segment.width                               = 0.0  
    fuselage.append_segment(segment)   
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_2'    
    segment.percent_x_location                  = 0.08732056/fuselage.lengths.total  
    segment.percent_z_location                  = 0.0000
    segment.height                              = 0.459245202 
    segment.width                               = 0.401839552 
    fuselage.append_segment(segment)   
  
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_3'    
    segment.percent_x_location                  = 0.197094977/fuselage.lengths.total  
    segment.percent_z_location                  = 0.001
    segment.height                              = 0.688749197
    segment.width                               = 0.918490404  
    fuselage.append_segment(segment)   

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_4'    
    segment.percent_x_location                  = 0.41997031/fuselage.lengths.total 
    segment.percent_z_location                  = 0.0000 
    segment.height                              = 0.975896055   
    segment.width                               = 1.320329956 
    fuselage.append_segment(segment)   

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_5'    
    segment.percent_x_location                  = 0.753451685/fuselage.lengths.total
    segment.percent_z_location                  = 0.0014551442477876075 # this is given as a percentage of the fuselage length i.e. location of the center of the cross section/fuselage length
    segment.height                              = 1.320329956 
    segment.width                               = 1.664763858 
    fuselage.append_segment(segment)   

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_6'    
    segment.percent_x_location                  = 1.14389933/fuselage.lengths.total
    segment.percent_z_location                  = 0.0036330994100294946
    segment.height                              = 1.607358208   
    segment.width                               = 2.009316366 
    fuselage.append_segment(segment)   

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_7'    
    segment.percent_x_location                  = 1.585491874/fuselage.lengths.total
    segment.percent_z_location                  = 0.008262262758112099
    segment.height                              = 2.18141471 
    segment.width                               = 2.411155918 
    fuselage.append_segment(segment)   

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_8'    
    segment.percent_x_location                  = 2.031242539/fuselage.lengths.total
    segment.percent_z_location                  = 0.013612882669616513
    segment.height                              = 2.468442962  
    segment.width                               = 2.698065563  
    fuselage.append_segment(segment)   

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_9'    
    segment.percent_x_location                  = 2.59009412/fuselage.lengths.total
    segment.percent_z_location                  = 0.01636321766224188
    segment.height                              = 2.640659912   
    segment.width                               = 2.812876863 
    fuselage.append_segment(segment)   

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_10'    
    segment.percent_x_location                  = 3.375147531/fuselage.lengths.total
    segment.percent_z_location                  = 0.01860240047935103
    segment.height                              = 2.755708426
    segment.width                               = 2.985093814 
    fuselage.append_segment(segment)   

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_11'    
    segment.percent_x_location                  = 17.01420312/fuselage.lengths.total 
    segment.percent_z_location                  = 0.01860240047935103
    segment.height                              = 2.755708426
    segment.width                               = 2.985093814 
    fuselage.append_segment(segment)   
 
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_12'    
    segment.percent_x_location                  = 18.64210783/fuselage.lengths.total
    segment.percent_z_location                  = 0.01860240047935103
    segment.height                              = 2.698302776 
    segment.width                               = 2.927925377  
    fuselage.append_segment(segment)    
     
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_13'    
    segment.percent_x_location                  = 22.7416002/fuselage.lengths.total 
    segment.percent_z_location                  = 0.043363795685840714
    segment.height                              = 1.779575158  
    segment.width                               = 1.722050901 
    fuselage.append_segment(segment)     
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_14'    
    segment.percent_x_location                  = 1.0
    segment.percent_z_location                  = 
    segment.height                              = 
    segment.width                               =  
    fuselage.append_segment(segment) 
    
    # add to vehicle
    vehicle.append_component(fuselage)

    #------------------------------------------------------------------------------------------------------------------------------------
    # ##########################################################  Booms  ################################################################  
    #------------------------------------------------------------------------------------------------------------------------------------          
    boom                                    = RCAIDE.Library.Components.Booms.Boom()
    boom.tag                                = 'tail_boom'
    boom.origin                             = [[   0.036, 1.950,  1]]  
    boom.fineness.nose                      = 0.950   
    boom.fineness.tail                      = 1.029   
    boom.lengths.nose                       = 0.2 
    boom.lengths.tail                       = 0.2
    boom.lengths.cabin                      = 4.15
    boom.lengths.total                      = 4.2
    boom.width                              = 0.15 
    boom.heights.maximum                    = 0.15  
    boom.heights.at_quarter_length          = 0.15  
    boom.heights.at_three_quarters_length   = 0.15 
    boom.heights.at_wing_root_quarter_chord = 0.15 
    boom.areas.wetted                       = 0.018
    boom.areas.front_projected              = 0.018 
    boom.effective_diameter                 = 0.15  
    boom.differential_pressure              = 0.  
    boom.xz_plane_symmetric                 = True 
    boom.index                              = 1
    
    # Segment  
    segment                           = RCAIDE.Library.Components.Booms.Segments.Segment() 
    segment.tag                       = 'segment_1'   
    segment.percent_x_location        = 0.
    segment.percent_z_location        = 0.0 
    segment.height                    = 0.05  
    segment.width                     = 0.05   
    boom.append_segment(segment)           
    
    # Segment                                   
    segment                           = RCAIDE.Library.Components.Booms.Segments.Segment()
    segment.tag                       = 'segment_2'   
    segment.percent_x_location        = 0.03
    segment.percent_z_location        = 0. 
    segment.height                    = 0.15 
    segment.width                     = 0.15 
    boom.append_segment(segment) 
    
    # Segment                                   
    segment                           = RCAIDE.Library.Components.Booms.Segments.Segment()
    segment.tag                       = 'segment_3'    
    segment.percent_x_location        = 0.97
    segment.percent_z_location        = 0. 
    segment.height                    = 0.15
    segment.width                     = 0.15
    boom.append_segment(segment)           
    
    # Segment                                  
    segment                           = RCAIDE.Library.Components.Booms.Segments.Segment()
    segment.tag                       = 'segment_4'   
    segment.percent_x_location        = 1.   
    segment.percent_z_location        = 0.   
    segment.height                    = 0.05   
    segment.width                     = 0.05   
    boom.append_segment(segment)           
    
    # add to vehicle
    vehicle.append_component(boom)   


    # ########################################################  Energy Network  #########################################################  
    net                                              = RCAIDE.Framework.Networks.Fuel()    

    #------------------------------------------------------------------------------------------------------------------------- 
    # Fuel Distrubition Line 
    #------------------------------------------------------------------------------------------------------------------------- 
    fuel_line                                       = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()  
 
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------    
    main_rotor_propulsor                              = RCAIDE.Library.Components.Powertrain.Propulsors.Turboprop()        
    main_rotor_propulsor.tag                          = 'main_rotor_propulsor'  
    main_rotor_propulsor.origin                       = [[ , , ]]
    main_rotor_propulsor.working_fluid                = RCAIDE.Library.Attributes.Gases.Air()               
    main_rotor_propulsor.gearbox.efficiency           = 0.99                                             
    main_rotor_propulsor.design_thrust                = vehicle.mass_properties.max_takeoff * 1.05 
    main_rotor_propulsor.design_altitude              = 10000*Units.ft  # have no clue about this                           
    main_rotor_propulsor.design_freestream_velocity   = 80 * Units.kts  # Have no clue about this

     #Propeller Design              
    main_rotor                                        = RCAIDE.Library.Components.Powertrain.Converters.Propeller()   
    main_rotor.tag                                    = 'starboard_propulsor_propeller' 
    main_rotor.origin                                 = [[ , , ]]
    main_rotor.active                                 = True          
    main_rotor.tip_radius                             = 
    main_rotor.hub_radius                             = 0.1 
    main_rotor.number_of_blades                       = 3   
    main_rotor.design_efficiency                      = 0.83 # Have no clue about this      
    main_rotor.design_angular_velocity                =  * Units.rpm  #  Change this     
    main_rotor.design_thrust                          = main_rotor_propulsor.design_thrust              
    main_rotor.design_altitude                        = main_rotor_propulsor.design_altitude                                        
    main_rotor.design_freestream_velocity             = main_rotor_propulsor.design_freestream_velocity                                                               
    starboard_propulsor.propeller                     = main_rotor     
    
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

    main_rotor.nacelle =  nacelle
    
    net.propulsors.append(starboard_propulsor)    

    #------------------------------------------------------------------------------------------------------------------------- 
    #  Energy Source: Fuel Tank
    #------------------------------------------------------------------------------------------------------------------------- 
    # fuel tank
    fuel_tank                                        = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    fuel_tank.fuel                                   = RCAIDE.Library.Attributes.Propellants.Jet_A1()    
    
    # apend fuel tank to dataclass of fuel tanks on fuel line 
    fuel_line.fuel_tanks.append(fuel_tank) 

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

    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs         = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config     = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base'  
    configs.append(base_config)    
    return configs

# ----------------------------------------------------------------------
#   Define the Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

# ----------------------------------------------------------------------
#   Define the Base Analysis
# ----------------------------------------------------------------------

def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle
    analyses.vehicle

    #  Geometry
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry() 
    geometry.settings.overwrite_reference        = True
    geometry.settings.update_wing_properties     = True
    geometry.settings.print_weight_analysis_report = True
    analyses.append(geometry)


    # ------------------------------------------------------------------
    #  Weights
    # ------------------------------------------------------------------     
    # weights = RCAIDE.Framework.Analyses.Weights.Conventional() 
    # weights.settings.update_center_of_gravity                            = False
    # weights.settings.print_weight_analysis_report                        = True
    # weights.method                                                       = "FLOPS"
    # weights.settings.FLOPS.fidelity                                      = "Complex"
    # weights.settings.weight_correction_factors.empty.systems.apu         = 0.0
    # weights.settings.weight_correction_factors.empty.systems.furnishings = 0.75
    # analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    # ------------------------------------------------------------------     
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()  
    analyses.append(aerodynamics) 

    # ------------------------------------------------------------------
    #  Energy
    # ------------------------------------------------------------------     
    energy= RCAIDE.Framework.Analyses.Energy.Energy() 
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    # ------------------------------------------------------------------     
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    # ------------------------------------------------------------------     
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)   

    # done!
    return analyses 

# ----------------------------------------------------------------------
#   Define the Missions
# ----------------------------------------------------------------------

def missions_setup(mission): 
 
    missions         = RCAIDE.Framework.Mission.Missions()
    
    # base mission 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions 

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
def simplified_mission_setup(analyses):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission' 

    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments  
    base_segment = Segments.Segment() 
    base_segment.state.numerics.solver.type                      = "root_finder"

    # ------------------------------------------------------------------
    #   Cruise Segment: constant Speed, constant altitude
    # ------------------------------------------------------------------ 
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "Cruise" 
    segment.analyses.extend(analyses.base) 
    segment.altitude                                      = 25000  * Units.feet 
    segment.air_speed                                     = 270    * Units.kts
    segment.distance                                      = 862.   * Units.nautical_mile  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls  
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.throttle.initial_guess_values = [[0.74]]
    segment.assigned_control_variables.body_angle.active             = True     
    segment.assigned_control_variables.body_angle.initial_guess_values = [[4.2 * Units.degree]]                
          
    mission.append_segment(segment)    
      
    return mission


# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------
def plot_mission(results):

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
    
    # Plot Trajectory
    plot_flight_trajectory(results)
    
    # Plot throttles
    plot_propulsor_throttles(results)

    return 

if __name__ == '__main__': 
    main()    
    plt.show()

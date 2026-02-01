 

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units   
from RCAIDE.Framework.Core import Units   
from RCAIDE.Library.Plots  import *     
from RCAIDE.Library.Methods.Powertrain.Propulsors.Internal_Combustion_Engine import design_internal_combustion_engine
from RCAIDE.Framework.External_Interfaces.OpenVSP import export_vsp_vehicle

# python imports 
import numpy as np 
from copy import deepcopy
import os 
import matplotlib.pyplot        as plt 
 
def main():     
      
    # vehicle data
    vehicle  = vehicle_setup() 

    # set up vehicle configs
    configs  = configs_setup(vehicle)

    # create analyses
    analyses = analyses_setup(configs)

    # mission analyses 
    mission  = mission_setup(analyses)

    # create mission instances (for multiple types of missions)
    missions = missions_setup(mission) 

    # mission analysis 
    results  = missions.base_mission.evaluate()  

    # plot the results
    plot_mission(results) 

    # plot vehicle 
    plot_3d_vehicle(vehicle)         
           
    return 

# ----------------------------------------------------------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------------------------------------------------------
def vehicle_setup():

    #------------------------------------------------------------------------------------------------------------------------------------
    #   Initialize the Vehicle
    #------------------------------------------------------------------------------------------------------------------------------------

    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'Tecnam_P2012'

 
    # ################################################# Vehicle-level Properties ########################################################  

    # mass properties
    vehicle.mass_properties.max_takeoff             = 3680 
    # vehicle.mass_properties.takeoff               = 3680 
    vehicle.mass_properties.max_zero_fuel           = 3680 
    vehicle.mass_properties.max_fuel                = 1190 * Units.lbs
    vehicle.mass_properties.max_payload             = 1394
    vehicle.flight_envelope.ultimate_load           = 5.7
    vehicle.flight_envelope.positive_limit_load     = 3.8 
    vehicle.reference_area                          = 25.76
    vehicle.number_of_passengers                    = 9
    vehicle.systems.control                         = "fully powered"
    vehicle.systems.accessories                     = "commuter"    
    
    cruise_speed                                    = 173.*Units['kts']    # Cruise at 75% throttle.
    altitude                                        = 10000. * Units.ft
    atmo                                            = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    freestream                                      = atmo.compute_values (0.)
    freestream0                                     = atmo.compute_values (altitude)
    mach_number                                     = (cruise_speed/freestream.speed_of_sound)[0][0] 
    vehicle.flight_envelope.design_dynamic_pressure = ( .5 *freestream0.density*(cruise_speed*cruise_speed))[0][0]
    vehicle.flight_envelope.design_mach_number      = mach_number
    vehicle.flight_envelope.design_range            = 950 * Units.nmi
    vehicle.mass_properties.center_of_gravity       = [[5.1772, 0, 0.46]]

         
    # ##########################################################  Wings ################################################################    
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Main Wing
    #------------------------------------------------------------------------------------------------------------------------------------
    wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing' 
    wing.sweeps.quarter_chord             = 0.0 * Units.deg
    wing.thickness_to_chord               = 0.12
    wing.areas.reference                  = 25.3
    wing.spans.projected                  = 14.0 
    wing.chords.root                      = 1.91
    wing.chords.tip                       = 1.51
    wing.chords.mean_aerodynamic          = 1.81
    wing.taper                            = wing.chords.root/wing.chords.tip 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 3.0 * Units.degrees
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[4.447, 0., 1.216]]
    wing.aerodynamic_center               = [3., 0., 1.01] 
    wing.vertical                         = False
    wing.xz_plane_symmetric               = True
    wing.high_lift                        = True 
    wing.winglet_fraction                 = 0.0  
    wing.dynamic_pressure_ratio           = 1.0  
    ospath                                = os.path.abspath(__file__)
    separator                             = os.path.sep
    rel_path                              = os.path.dirname(ospath) + separator + '..' + separator 
    airfoil                               = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                           = 'NACA_63_412.txt' 
    airfoil.coordinate_file               = rel_path + 'Airfoils' + separator + 'NACA_63_412.txt'   # absolute path     
    cg_x                                  = wing.origin[0][0] + 0.25*wing.chords.mean_aerodynamic
    cg_z                                  = wing.origin[0][2] - 0.2*wing.chords.mean_aerodynamic
    # vehicle.mass_properties.center_of_gravity = [[cg_x,   0.  ,  cg_z ]]  # SOURCE: Design and aerodynamic analysis of a twin-engine commuter aircraft

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'inboard'
    segment.percent_span_location         = 0.0 
    segment.twist                         = 3. * Units.degrees   
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'outboard'
    segment.percent_span_location         = 0.5438
    segment.twist                         = 2.* Units.degrees 
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0. 
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12 
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)
    
    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'winglet'
    segment.percent_span_location         = 1.0
    segment.twist                         = 1.  * Units.degrees 
    segment.root_chord_percent            = 0.630
    segment.dihedral_outboard             = 0. * Units.degrees 
    segment.sweeps.quarter_chord          = 0. * Units.degrees 
    segment.thickness_to_chord            = 0.12 
    segment.append_airfoil(airfoil)
    wing.append_segment(segment) 

    # control surfaces -------------------------------------------
    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.68
    aileron.span_fraction_end     = 0.96
    aileron.deflection            = 0.0  * Units.deg
    aileron.chord_fraction        = 0.30
    wing.append_control_surface(aileron)   

     
    # add to vehicle
    vehicle.append_component(wing)


    #------------------------------------------------------------------------------------------------------------------------------------  
    #   Horizontal Tail
    #------------------------------------------------------------------------------------------------------------------------------------    
    wing                                  = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag                              = 'horizontal_stabilizer' 
    wing.sweeps.quarter_chord             = 0.0 * Units.deg
    wing.thickness_to_chord               = 0.12
    wing.areas.reference                  = 7.23 
    wing.spans.projected                  = 5.64  * Units.meter 
    wing.sweeps.leading_edge              = 7.5 * Units.deg 
    wing.chords.root                      = 1.35 * Units.meter 
    wing.chords.tip                       = 0.84 * Units.meter 
    wing.chords.mean_aerodynamic          = 1.10 * Units.meter  
    wing.taper                            = wing.chords.tip/wing.chords.root 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 0.0 * Units.degrees
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[10.366, 0., 0.414]]
    wing.aerodynamic_center               = [10.366, 0., 0.414] 
    wing.vertical                         = False
    wing.winglet_fraction                 = 0.0  
    wing.xz_plane_symmetric               = True
    wing.high_lift                        = False 
    wing.dynamic_pressure_ratio           = 0.9

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root'
    segment.percent_span_location         = 0.0   
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = wing.sweeps.leading_edge
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root_2'
    segment.percent_span_location         = 1.0  
    segment.root_chord_percent            = wing.taper 
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = 0 * Units.deg
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)

    # control surfaces -------------------------------------------
    elevator                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
    elevator.tag                   = 'elevator'
    elevator.span_fraction_start   = 0.01
    elevator.span_fraction_end     = 1.00
    elevator.deflection            = 0.0  * Units.deg
    elevator.chord_fraction        = 0.37
    wing.append_control_surface(elevator)

    # add to vehicle
    vehicle.append_component(wing)

    #------------------------------------------------------------------------------------------------------------------------------------  
    #   Vertical Stabilizer
    #------------------------------------------------------------------------------------------------------------------------------------ 
    wing                                  = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag                              = 'vertical_stabilizer'     
    wing.sweeps.quarter_chord             = 25. * Units.deg
    wing.thickness_to_chord               = 0.12
    wing.areas.reference                  = 4.26 * Units['meters**2']  
    wing.spans.projected                  = 2.50 * Units.meter  
    wing.chords.root                      = 2.891 * Units.meter 
    wing.chords.tip                       = 0.8 * Units.meter 
    wing.chords.mean_aerodynamic          = 2.20   * Units.meter 
    wing.taper                            = wing.chords.tip/wing.chords.root 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 0.0 * Units.degrees
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[8.8 ,0, 0.623]]
    wing.aerodynamic_center               = [8.8 ,0,0]  
    wing.vertical                         = True 
    wing.xz_plane_symmetric               = False
    wing.t_tail                           = False
    wing.winglet_fraction                 = 0.0  
    wing.dynamic_pressure_ratio           = 1.0

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root'
    segment.percent_span_location         = 0.0   
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = 0. * Units.deg
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root_2'
    segment.percent_span_location         = 0.18   
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = 75.667 * Units.deg
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Canted'
    segment.percent_span_location         = 0.26   
    segment.root_chord_percent            = 0.6485 
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = 28.0 * Units.deg
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.0   
    segment.root_chord_percent            = 0.346
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)

    # control surfaces -------------------------------------------
    elevator                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    elevator.tag                   = 'rudder'
    elevator.span_fraction_start   = 0.2
    elevator.span_fraction_end     = 1.00
    elevator.deflection            = 0.0  * Units.deg
    elevator.chord_fraction        = 0.44
    wing.append_control_surface(elevator)

    # add to vehicle
    vehicle.append_component(wing)

 
    # ##########################################################   Fuselage  ############################################################    
    fuselage = RCAIDE.Library.Components.Fuselages.Fuselage() 
    fuselage.seats_abreast                      = 2.
    fuselage.fineness.nose                      = 1.8
    fuselage.fineness.tail                      = 2.
    fuselage.lengths.nose                       = 2.8  * Units.meter
    fuselage.lengths.tail                       = 3.5  * Units.meter
    fuselage.lengths.total                      = 11.8 * Units.meter
    fuselage.lengths.cabin                      = fuselage.lengths.total - (fuselage.lengths.nose  + fuselage.lengths.tail ) 
    fuselage.lengths.fore_space                 = 0.
    fuselage.lengths.aft_space                  = 0.
    fuselage.width                              = 1.57
    fuselage.heights.maximum                    = 2
    fuselage.heights.at_quarter_length          = 2. * Units.meter
    fuselage.heights.at_three_quarters_length   = 2. * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = 2. * Units.meter
    fuselage.areas.side_projected               = 16.9613 * Units.meter**2.
    fuselage.areas.wetted                       = 52.94 * Units.meter**2.
    fuselage.areas.front_projected              = 2.72 * Units.meter**2.
    fuselage.effective_diameter                 = 1.760 * Units.meter 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_0'
    segment.percent_x_location                  = 0
    segment.percent_z_location                  = -0.01025
    segment.height                              = 0.0
    segment.width                               = 0.0
    fuselage.append_segment(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_1'
    segment.percent_x_location                  = 0.01941
    segment.percent_z_location                  = -0.00698
    segment.height                              = 0.50026
    segment.width                               = 0.43658
    fuselage.append_segment(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'
    segment.percent_x_location                  = 0.06309
    segment.percent_z_location                  = -0.00197
    segment.height                              = 0.81075
    segment.width                               = 0.90653
    fuselage.append_segment(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'
    segment.percent_x_location                  = 0.13101
    segment.percent_z_location                  = 0.00931
    segment.height                              = 1.12583
    segment.width                               = 1.37708
    fuselage.append_segment(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'
    segment.percent_x_location                  = 0.23545
    segment.percent_z_location                  = 0.03003
    segment.height                              = 1.65695
    segment.width                               = 1.5748
    segment.curvature                           = 3
    fuselage.append_segment(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'
    segment.percent_x_location                  = 0.296002
    segment.percent_z_location                  = 0.03026
    segment.height                              = 1.68589
    segment.width                               = 1.5748
    segment.curvature                           = 3
    fuselage.append_segment(segment)

    # Segment Left off here
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'
    segment.percent_x_location                  = 0.581
    segment.percent_z_location                  = 0.03264
    segment.height                              = 1.68589
    segment.width                               = 1.5748
    segment.curvature                           = 3
    fuselage.append_segment(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'
    segment.percent_x_location                  = 0.70469
    segment.percent_z_location                  = 0.03426
    segment.height                              = 1.5662
    segment.width                               = 1.33392
    segment.curvature                           = 3
    fuselage.append_segment(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_8'
    segment.percent_x_location                  = 0.98511 
    segment.percent_z_location                  = 0.045
    segment.height                              = 0.4667
    segment.width                               = 0.2
    fuselage.append_segment(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_9'
    segment.percent_x_location                  = 1.0
    segment.percent_z_location                  = 0.04133
    segment.height                              = 0.0
    segment.width                               = 0.0
    fuselage.append_segment(segment)

    # define cabin    
    cabin                                             = RCAIDE.Library.Components.Fuselages.Cabins.Cabin()
    cabin.offset_x                                    = 2.5  #origin                                      = [[2, 0, 0]]
    economy_class                                     = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest              = 2
    economy_class.seat_pitch                          = 31 * Units.inches
    economy_class.aisle_width                         = 0.28 * Units.meters
    economy_class.seat_width                          = 16 *  Units.inches
    economy_class.number_of_rows                      = 5
    economy_class.galley_lavatory_percent_x_locations = []  
    economy_class.emergency_exit_percent_x_locations  = []      
    economy_class.type_A_exit_percent_x_locations     = [0.01,1] 
    economy_class.number_of_seats                     = economy_class.number_of_rows  * economy_class.number_of_seats_abrest 
    # cabin.append_cabin_class(economy_class)
    # fuselage.append_cabin(cabin)
    

    # add to vehicle
    vehicle.append_component(fuselage)
 
    # ##########################################################   Nacelles  ############################################################    
    nacelle                    = RCAIDE.Library.Components.Nacelles.Stack_Nacelle()
    nacelle.tag                = 'nacelle_1'
    nacelle.length             = 3
    nacelle.diameter           = 42 * Units.inches
    nacelle.areas.wetted       = 0.01*(2*np.pi*0.01/2)
    nacelle.origin             = [[3.0,2.25,1.232]]
    nacelle.flow_through       = False  
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_0'
    nac_segment.percent_x_location = 0.0 
    nac_segment.height             = 0.0
    nac_segment.width              = 0.0
    nacelle.append_segment(nac_segment)   
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_1'
    nac_segment.percent_x_location = 0.16316  
    nac_segment.height             = 0.4
    nac_segment.width              = 0.4
    nacelle.append_segment(nac_segment)   

    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_2'
    nac_segment.percent_x_location = 0.1933799  
    nac_segment.percent_z_location = -0.027046 
    nac_segment.height             = 0.587
    nac_segment.width              = 0.90
    nacelle.append_segment(nac_segment)  
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_3'
    nac_segment.percent_x_location = 0.469773  
    nac_segment.percent_z_location = 0.014215
    nac_segment.height             = 0.62
    nac_segment.width              = 0.9
    nacelle.append_segment(nac_segment)  
     
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_4'
    nac_segment.percent_x_location = 0.9
    nac_segment.percent_z_location = 0.02025
    nac_segment.height             = 0.2
    nac_segment.width              = 0.9
    nacelle.append_segment(nac_segment)  

    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_5'
    nac_segment.percent_x_location = 1.0
    nac_segment.percent_z_location = 0.02025
    nac_segment.height             = 0.0
    nac_segment.width              = 0.0
    nacelle.append_segment(nac_segment)  
    
    vehicle.append_component(nacelle)  

    nacelle_2          = deepcopy(nacelle)
    nacelle_2.tag      = 'nacelle_2'
    nacelle_2.origin   = [[3,-2.25,1.232]]
    vehicle.append_component(nacelle_2)    
 
    # ########################################################  Energy Network  #########################################################  
    net                                         = RCAIDE.Framework.Networks.Fuel()   

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Fuel Line
    #------------------------------------------------------------------------------------------------------------------------------------  
    fuel_line                                   = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()   
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Fuel Tank & Fuel. Update Fuel tank location and size
    #------------------------------------------------------------------------------------------------------------------------------------       
    # fuel_tank                                             = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank() 
    # fuel_tank.origin                                      = vehicle.wings.main_wing.origin  
    # fuel_tank.fuel                                        = RCAIDE.Library.Attributes.Propellants.Aviation_Gasoline() 
    # fuel_tank.fuel.mass_properties.mass                   = 1190 *Units.lbs 
    # fuel_tank.fuel.mass_properties.center_of_gravity      = wing.mass_properties.center_of_gravity
    # fuel_tank.internal_volume                             = fuel_tank.fuel.mass_properties.mass/fuel_tank.fuel.density   
    # fuel_line.fuel_tanks.append(fuel_tank)  

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Starboard Propulsor. Continental GTSIO-520-S.
    #------------------------------------------------------------------------------------------------------------------------------------   
    starboard_propulsor                        = RCAIDE.Library.Components.Powertrain.Propulsors.Internal_Combustion_Engine()      
    starboard_propulsor.origin                 = [[3.36,2.25,1.15]]
    starboard_propulsor.tag                    = 'starboard_propulsor'        

    # Engine                     
    engine                                     = RCAIDE.Library.Components.Powertrain.Converters.Engine()
    engine.sea_level_power                     = 375. * Units.horsepower 
    engine.rated_speed                         = 3350. * Units.rpm 
    engine.power_specific_fuel_consumption     = 0.40  * Units['lb/hp/hr'] # This is a very rough estimate
    engine.origin                              = [[3.75,2.25,1.15]]
    starboard_propulsor.engine                 = engine 
    # ice_prop.sealevel_static_thrust          = 2500 # N
     
    # Propeller. propeller dimensions: TCDS_EASA_A.637_TECNAM_P2012_issue_13.pdf         
    propeller                                        = RCAIDE.Library.Components.Powertrain.Converters.Propeller() 
    propeller.tag                                    = 'propeller_1'  
    propeller.tip_radius                             = 2.26/2   
    propeller.number_of_blades                       = 3
    propeller.hub_radius                             = 10.     * Units.inches 
    propeller.cruise.design_freestream_velocity      = 175.*Units['mph']   
    propeller.cruise.design_angular_velocity         = 2700. * Units.rpm 
    propeller.cruise.design_Cl                       = 0.7 
    propeller.cruise.design_altitude                 = 2500. * Units.feet 
    propeller.cruise.design_thrust                   = 2000   
    propeller.clockwise_rotation                     = False
    propeller.variable_pitch                         = True  
    propeller.origin                                 = [[3.36,2.25,1.15]]   
    airfoil                                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                                      = 'NACA_4412' 
    airfoil.coordinate_file                          =  rel_path + 'Airfoils' + separator + 'NACA_4412.txt'   # absolute path   
    airfoil.polar_files                              =[ rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_50000.txt',
                                                        rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_100000.txt',
                                                        rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_200000.txt',
                                                        rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_500000.txt',
                                                        rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_1000000.txt']   
    propeller.append_airfoil(airfoil)                       
    propeller.airfoil_polar_stations                 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]   
    starboard_propulsor.propeller                    = propeller   
    starboard_propulsor.nacelle = nacelle  
              
    # design propeller ICE  
    design_internal_combustion_engine(starboard_propulsor)
    net.propulsors.append(starboard_propulsor) 

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Port Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    port_propulsor                                  = deepcopy(starboard_propulsor)
    port_propulsor.active_fuel_tanks                = ['fuel_tank'] 
    port_propulsor.tag                              = 'port_propulsor' 
    port_propulsor.origin                           = [[3.36,-2.25,1.15]]
    port_propulsor.nacelle.tag                      = 'port_propulsor_nacelle' 
    port_propulsor.nacelle.origin                   = [[3.0,-2.25,1.232]]
    port_propulsor.propeller.origin                 = [[3.36,-2.25,1.15]]
    port_propulsor.engine.origin                    = [[3.75,-2.25,1.15]]
    port_propulsor.propeller.tag                    = 'propeller_2'
    
    # append propulsor to distribution line 
    net.propulsors.append(port_propulsor) 
    fuel_line.assigned_propulsors =  [[starboard_propulsor.tag, port_propulsor.tag]]

    # append bus   
    net.fuel_lines.append(fuel_line) 
    vehicle.append_energy_network(net)

    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Avionics
    #------------------------------------------------------------------------------------------------------------------------------------ 
    Wuav                                        = 2. * Units.lbs
    avionics                                    = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.mass_properties.uninstalled        = Wuav
    vehicle.avionics                            = avionics    

    return vehicle

# ---------------------------------------------------------------------
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

    # done!
    return configs
 
 
# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis      = base_analysis(config)
        analyses[tag] = analysis

    return analyses

def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle

    #  Geometry
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.settings.overwrite_reference          = True
    geometry.settings.update_wing_properties       = True
    geometry.settings.print_weight_analysis_report = True
    analyses.append(geometry)
 
    # ------------------------------------------------------------------
    #  Weights
    weights = RCAIDE.Framework.Analyses.Weights.Conventional_General_Aviation()
    weights.aircraft_type                                               = 'General_Aviation'
    weights.settings.FLOPS.fidelity                                     = 'Complex' 
    weights.settings.weight_correction_factors.empty.systems.electrical = 0 
    # weights.settings.update_center_of_gravity = True
    weights.settings.update_moment_of_inertia = True


    
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    
    # Calculate extra drag from landing gear: 
    main_wheel_width        = 4. * Units.inches
    main_wheel_height       = 12. * Units.inches
    nose_gear_height        = 10. * Units.inches
    nose_gear_width         = 4. * Units.inches 
    total_wheel             = 2*main_wheel_width*main_wheel_height + nose_gear_width*nose_gear_height 
    main_gear_strut_height  = 2. * Units.inches
    main_gear_strut_length  = 24. * Units.inches
    nose_gear_strut_height  = 12. * Units.inches
    nose_gear_strut_width   = 2. * Units.inches 
    total_strut             = 2*main_gear_strut_height*main_gear_strut_length + nose_gear_strut_height*nose_gear_strut_width 
    drag_area               = 1.4*( total_wheel + total_strut)
    
    
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.settings.drag_coefficient_increment = drag_area/vehicle.reference_area
    analyses.append(aerodynamics)
 
    # ------------------------------------------------------------------
    #  Stability Analysis
    # ------------------------------------------------------------------     
    stability                                = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method() 
    stability.settings.compute_neutral_point = True
    analyses.append(stability)

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

    # done!
    return analyses    

# ----------------------------------------------------------------------
#   Define the Mission
# ---------------------------------------------------------------------- 

def mission_setup(analyses):   
    
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission     = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission' 

    # unpack Segments module
    Segments     = RCAIDE.Framework.Mission.Segments  
    base_segment = Segments.Segment() 
    base_segment.state.numerics.solver.type = 'root_finder'
  
    # # ------------------------------------------------------------------
    # #   Departure End of Runway Segment Flight 1 : 
    # # ------------------------------------------------------------------ 
    # segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    # segment.tag = 'DER'       
    # segment.analyses.extend( analyses.base )  
    # segment.altitude_start                                = 0.0 * Units.feet
    # segment.altitude_end                                  = 50.0 * Units.feet
    # segment.air_speed_start                               = 45  * Units['m/s'] 
    # segment.air_speed_end                                 = 45      
    # segment.initial_battery_state_of_charge               = 0.89  
            
    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    # segment.assigned_control_variables.body_angle.active             = True                  
       
    # mission.append_segment(segment)
    
    # # ------------------------------------------------------------------
    # #   Initial Climb Area Segment Flight 1  
    # # ------------------------------------------------------------------ 
    # segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    # segment.tag = 'ICA' 
    # segment.analyses.extend( analyses.base )   
    # segment.altitude_start                                = 50.0 * Units.feet
    # segment.altitude_end                                  = 500.0 * Units.feet
    # segment.air_speed_start                               = 45  * Units['m/s']   
    # segment.air_speed_end                                 = 50 * Units['m/s']   
    # segment.climb_rate                                    = 600 * Units['ft/min']   
    
    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    # segment.assigned_control_variables.body_angle.active             = True                  
          
    # mission.append_segment(segment)  
   
             
    # # ------------------------------------------------------------------
    # #   Climb Segment Flight 1 
    # # ------------------------------------------------------------------ 
    # segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment) 
    # segment.tag = 'climb_1'        
    # segment.analyses.extend( analyses.base )      
    # segment.altitude_start                                = 500.0 * Units.feet
    # segment.altitude_end                                  = 2500 * Units.feet
    # segment.air_speed                                     = 120 * Units['mph']
    # segment.climb_rate                                    = 500* Units['ft/min']  
    
    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    # segment.assigned_control_variables.body_angle.active             = True                 
           
    # mission.append_segment(segment)
    
        
    # # ------------------------------------------------------------------
    # #   Climb 1 : constant Speed, constant rate segment 
    # # ------------------------------------------------------------------ 
    # segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    # segment.tag = "climb_2"
    # segment.analyses.extend( analyses.base )
    # segment.altitude_start                                = 2500.0  * Units.feet
    # segment.altitude_end                                  = 8012    * Units.feet 
    # segment.air_speed                                     = 96.4260 * Units['mph'] 
    # segment.climb_rate                                    = 700.034 * Units['ft/min']   
    
    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    # segment.assigned_control_variables.body_angle.active             = True                 
            
    # mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Cruise Segment: constant Speed, constant altitude
    # ------------------------------------------------------------------ 
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend(analyses.base) 
    segment.altitude                                      = 8012   * Units.feet
    segment.air_speed                                     = 175. * Units['mph'] 
    segment.distance                                      = 950.   * Units.nautical_mile  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                  
          
    mission.append_segment(segment)    


    # # ------------------------------------------------------------------
    # #   Descent Segment Flight 1   
    # # ------------------------------------------------------------------ 
    # segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    # segment.tag = "Descent"  
    # segment.analyses.extend( analyses.base )       
    # segment.altitude_start                                = 8012 * Units.feet  
    # segment.altitude_end                                  = 1000 * Units.feet  
    # segment.air_speed_end                                 = 110 * Units['mph']   
    # segment.climb_rate                                    = -200 * Units['ft/min']  
    
    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    # segment.assigned_control_variables.body_angle.active             = True                 
          
    # mission.append_segment(segment)   
               
    # # ------------------------------------------------------------------
    # #  Downleg_Altitude Segment Flight 1 
    # # ------------------------------------------------------------------ 
    # segment = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment) 
    # segment.tag = 'Downleg'
    # segment.analyses.extend(analyses.base)   
    # segment.air_speed_end                                 = 45.0 * Units['m/s']            
    # segment.distance                                      = 6000 * Units.feet
    # segment.acceleration                                  = -0.025  * Units['m/s/s']   
    # segment.descent_rate                                  = 300 * Units['ft/min']   
    
    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    # segment.assigned_control_variables.body_angle.active             = True                   
            
    # mission.append_segment(segment)     
    
    # # ------------------------------------------------------------------
    # #  Reserve Climb 
    # # ------------------------------------------------------------------ 
    # segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment) 
    # segment.tag = 'Reserve_Climb'        
    # segment.analyses.extend( analyses.base )      
    # segment.altitude_end                                  = 1500 * Units.feet
    # segment.air_speed                                     = 120 * Units['mph']
    # segment.climb_rate                                    = 500* Units['ft/min']  
    
    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    # segment.assigned_control_variables.body_angle.active             = True                
        
    # mission.append_segment(segment)
    
    # # ------------------------------------------------------------------
    # #  Researve Cruise Segment 
    # # ------------------------------------------------------------------ 
    # segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment) 
    # segment.tag = 'Reserve_Cruise'  
    # segment.analyses.extend(analyses.base) 
    # segment.air_speed                                     = 145* Units['mph']
    # segment.distance                                      = 60 * Units.miles * 0.1  
    
    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    # segment.assigned_control_variables.body_angle.active             = True                  
       
    # mission.append_segment(segment)     
    
    # # ------------------------------------------------------------------
    # #  Researve Descent
    # # ------------------------------------------------------------------ 
    # segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment) 
    # segment.tag = 'Reserve_Descent'
    # segment.analyses.extend( analyses.base )    
    # segment.altitude_end                                  = 1000 * Units.feet 
    # segment.air_speed                                     = 110 * Units['mph']
    # segment.descent_rate                                  = 300 * Units['ft/min']   
    
    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    # segment.assigned_control_variables.body_angle.active             = True                
    # mission.append_segment(segment)  

    
    # # ------------------------------------------------------------------
    # #  Baseleg Segment Flight 1  
    # # ------------------------------------------------------------------ 
    # segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    # segment.tag = 'Baseleg'
    # segment.analyses.extend( analyses.base)   
    # segment.altitude_start                                = 1000 * Units.feet
    # segment.altitude_end                                  = 500.0 * Units.feet
    # segment.air_speed_start                               = 45 
    # segment.air_speed_end                                 = 40    
    # segment.climb_rate                                    = -350 * Units['ft/min'] 
    
    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    # segment.assigned_control_variables.body_angle.active             = True                
    # mission.append_segment(segment) 

    # # ------------------------------------------------------------------
    # #  Final Approach Segment Flight 1  
    # # ------------------------------------------------------------------ 
    # segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    # segment_name = 'Final_Approach'
    # segment.tag = segment_name          
    # segment.analyses.extend( analyses.base)      
    # segment.altitude_start                                = 500.0 * Units.feet
    # segment.altitude_end                                  = 00.0 * Units.feet
    # segment.air_speed_start                               = 40 
    # segment.air_speed_end                                 = 35   
    # segment.climb_rate                                    = -300 * Units['ft/min']   
    
    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    # segment.assigned_control_variables.body_angle.active             = True                      
    # mission.append_segment(segment)  
 
    # # ------------------------------------------------------------------
    # #   Mission definition complete    
    # # ------------------------------------------------------------------ 
    
    return mission 
 

def missions_setup(mission): 
 
    missions     = RCAIDE.Framework.Mission.Missions()
    
    # base mission 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions 


def plot_mission(results):  
    
    plot_flight_conditions(results) 
    
    plot_aerodynamic_forces(results)

    plot_aerodynamic_coefficients(results)  

    plot_drag_components(results)
    
    plot_aircraft_velocities(results)

    plot_rotor_conditions(results) 

    plot_altitude_sfc_weight(results) 
     
    return
 
if __name__ == '__main__': 
    main()    
    plt.show()



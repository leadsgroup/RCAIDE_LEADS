# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Core import Units  
from RCAIDE.Visualization  import *      
from RCAIDE.Methods.Energy.Propulsors.Converters.Rotor import design_propeller
from RCAIDE.Methods.Geometry.Two_Dimensional.Planform      import segment_properties   

# python imports     
import numpy as np  
import sys
import matplotlib.pyplot as plt  
from copy import deepcopy
import pickle
import os 


# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main(): 

    vehicle    = vehicle_setup()   
    
    plot_3d_vehicle (vehicle,
                    show_wing_control_points    = False,
                    show_rotor_wake_vortex_core = False,
                    min_x_axis_limit            = 0,
                    max_x_axis_limit            = 40,
                    min_y_axis_limit            = -20,
                    max_y_axis_limit            = 20,
                    min_z_axis_limit            = -20,
                    max_z_axis_limit            = 20)         
    
    configs    = configs_setup(vehicle)
     
    # create analyses
    analyses = analyses_setup(configs)

    # mission analyses
    mission  = mission_setup(analyses) 

    # create mission instances (for multiple types of missions)
    missions = missions_setup(mission) 

    # mission analysis 
    results = missions.base_mission.evaluate()   

    # plot the results 
    plot_mission(results)    
        
    return 

# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def vehicle_setup():

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------

    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'MD_88'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------

    ## mass properties
    #vehicle.mass_properties.max_takeoff               = 23000 
    #vehicle.mass_properties.takeoff                   = 23000  
    #vehicle.mass_properties.operating_empty           = 13600  
    #vehicle.mass_properties.max_zero_fuel             = 21000 
    #vehicle.mass_properties.cargo                     = 7400
    #vehicle.mass_properties.center_of_gravity         = [[0,0,0]] # Unknown 
    #vehicle.mass_properties.moments_of_inertia.tensor = [[0,0,0]] # Unknown 
    #vehicle.mass_properties.max_fuel                  = 5000
    #vehicle.design_mach_number                        = 0.41 
    #vehicle.design_range                              = 1528000  
    #vehicle.design_cruise_alt                         = 25000 *Units.feet

    ## envelope properties
    #vehicle.flight_envelope.ultimate_load        = 3.75
    #vehicle.flight_envelope.positive_limit_load           = 1.5
       
    ## basic parameters       
    #vehicle.reference_area                = 61.0  
    #vehicle.number_of_passengers                    = 72
    #vehicle.systems.control               = "fully powered"
    #vehicle.systems.accessories           = "short range"

 
    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------

    wing                                  = RCAIDE.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing'
    wing.areas.reference                  = 61.0  
    wing.spans.projected                  = 27.05 
    wing.aspect_ratio                     = (wing.spans.projected**2) /  wing.areas.reference
    wing.sweeps.quarter_chord             = 0.0 
    wing.thickness_to_chord               = 0.1 
    wing.chords.root                      = 2.7 
    wing.chords.tip                       = 1.35 
    wing.total_length                     = wing.chords.root  
    wing.taper                            = wing.chords.tip/wing.chords.root 
    wing.chords.mean_aerodynamic          = wing.chords.root * 2/3 * (( 1 + wing.taper + wing.taper**2 )/( 1 + wing.taper )) 
    wing.areas.exposed                    = 2 * wing.areas.reference
    wing.areas.wetted                     = 2 * wing.areas.reference 
    wing.twists.root                      = 0 * Units.degrees  
    wing.twists.tip                       = 0 * Units.degrees   
    wing.origin                           = [[11.52756129,0,2.009316366]]  
    wing.aerodynamic_center               = [[11.52756129 + 0.25*wing.chords.root ,0,2.009316366]]   
    wing.vertical                         = False   
    wing.xz_plane_symmetric               = True  
    wing.dynamic_pressure_ratio           = 1.0 
 

    # Wing Segments 
    segment                               = RCAIDE.Components.Wings.Segment()
    segment.tag                           = 'inboard'
    segment.percent_span_location         = 0.0
    segment.twist                         = 0.0 * Units.deg
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.0  * Units.degrees
    segment.sweeps.quarter_chord          = 0.0 * Units.degrees
    segment.thickness_to_chord            = .1 
    wing.append_segment(segment)
 
    segment                               = RCAIDE.Components.Wings.Segment()
    segment.tag                           = 'outboard'
    segment.percent_span_location         = 0.324
    segment.twist                         = 0.0 * Units.deg
    segment.root_chord_percent            = 1.0
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.leading_edge           = 4.7 * Units.degrees
    segment.thickness_to_chord            = .1 
    wing.append_segment(segment) 
 
    segment                               = RCAIDE.Components.Wings.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.
    segment.twist                         = 0. * Units.degrees
    segment.root_chord_percent            = wing.taper 
    segment.thickness_to_chord            = 0.1
    segment.dihedral_outboard             = 0.
    segment.sweeps.quarter_chord          = 0.  
    wing.append_segment(segment)     
    
    
    
    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------ 
    wing                         = RCAIDE.Components.Wings.Horizontal_Tail()
    wing.tag                     = 'horizontal_stabilizer'  
    wing.spans.projected         = 3.61*2 
    wing.areas.reference         = 15.2 
    wing.aspect_ratio            = (wing.spans.projected**2) /  wing.areas.reference
    wing.sweeps.leading_edge     = 11.56*Units.degrees  
    wing.thickness_to_chord      = 0.1  
    wing.chords.root             = 2.078645129 
    wing.chords.tip              = 0.953457347 
    wing.total_length            = wing.chords.root  
    wing.taper                   = wing.chords.tip/wing.chords.root  
    wing.chords.mean_aerodynamic = wing.chords.root * 2/3 * (( 1 + wing.taper + wing.taper**2 )/( 1 + wing.taper )) 
    wing.areas.exposed           = 2 * wing.areas.reference
    wing.areas.wetted            = 2 * wing.areas.reference 
    wing.twists.root             = 0 * Units.degrees  
    wing.twists.tip              = 0 * Units.degrees   
    wing.origin                  = [[25.505088,0,5.510942426]]   
    wing.aerodynamic_center      = [[25.505088+ 0.25*wing.chords.root,0,2.009316366]]   
    wing.vertical                = False  
    wing.xz_plane_symmetric      = True  
    wing.dynamic_pressure_ratio  = 1.0 

    # update properties of the wing using segments     
    wing = segment_properties(wing,update_wet_areas=True,update_ref_areas=True)

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------ 
    wing                                   = RCAIDE.Components.Wings.Vertical_Tail()
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
    wing.aerodynamic_center                = [[17.34807199,0,1.3]]   
    wing.vertical                          = True  
    wing.xz_plane_symmetric                = False  
    wing.t_tail                            = True  
    wing.dynamic_pressure_ratio            = 1.0  
 

    # Wing Segments
    segment                               = RCAIDE.Components.Wings.Segment()
    segment.tag                           = 'segment_1'
    segment.percent_span_location         = 0.0
    segment.twist                         = 0.0
    segment.root_chord_percent            = 1.0
    segment.dihedral_outboard             = 0.0
    segment.sweeps.leading_edge           = 75 * Units.degrees  
    segment.thickness_to_chord            = 1.0
    wing.append_segment(segment)

    segment                               = RCAIDE.Components.Wings.Segment()
    segment.tag                           = 'segment_2'
    segment.percent_span_location         = 1.331360381/wing.spans.projected
    segment.twist                         = 0.0
    segment.root_chord_percent            = 4.25/wing.chords.root  
    segment.dihedral_outboard             = 0   
    segment.sweeps.leading_edge           = 54 * Units.degrees   
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)

    segment                               = RCAIDE.Components.Wings.Segment()
    segment.tag                           = 'segment_3'
    segment.percent_span_location         = 3.058629978/wing.spans.projected
    segment.twist                         = 0.0
    segment.root_chord_percent            = 2.35/wing.chords.root    
    segment.dihedral_outboard             = 0 
    segment.sweeps.leading_edge           = 31 * Units.degrees   
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)
    

    segment                               = RCAIDE.Components.Wings.Segment()
    segment.tag                           = 'segment_4'
    segment.percent_span_location         = 4.380739035/wing.spans.projected
    segment.twist                         = 0.0
    segment.root_chord_percent            = 2.190082795/wing.chords.root  
    segment.dihedral_outboard             = 0
    segment.sweeps.leading_edge           = 52 * Units.degrees   
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)    
    

    segment                               = RCAIDE.Components.Wings.Segment()
    segment.tag                           = 'segment_5'
    segment.percent_span_location         = 1.0
    segment.twist                         = 0.0
    segment.root_chord_percent            = 1.3/wing.chords.root  
    segment.dihedral_outboard             = 0  
    segment.sweeps.leading_edge           = 0 * Units.degrees   
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)    

    
    
    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

  
    fuselage = RCAIDE.Components.Fuselages.Fuselage()
    fuselage.tag = 'fuselage' 
    fuselage.seats_abreast                      = 5 
    fuselage.seat_pitch                         = 31   
    fuselage.fineness.nose                      = 0.54
    fuselage.fineness.tail                      = 1.18 
    fuselage.lengths.total                      = 45.0596 * Units.meters   
    fuselage.lengths.nose                       = 3.62 * Units.meters 
    fuselage.lengths.tail                       = 7.5 * Units.meters 
    fuselage.lengths.cabin                      = fuselage.lengths.total- (fuselage.lengths.nose + fuselage.lengths.tail  )
    fuselage.width                              = 3.18  
    fuselage.heights.maximum                    = 3.565  
    fuselage.areas.side_projected               = 1.0 # incorrect 
    fuselage.areas.wetted                       = 1.0 # incorrect 
    fuselage.areas.front_projected              = 1.0 # incorrect 
    fuselage.effective_diameter                 = 3.5  
    fuselage.differential_pressure              = 8.5 * Units.psi    
    
     # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_1'    
    segment.percent_x_location                  = 0.0000
    segment.percent_z_location                  = 0.0000
    segment.height                              = 1E-3
    segment.width                               = 1E-3  
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
    segment.tag                                 = 'segment_16'    
    segment.percent_x_location                  = 22.7416002/fuselage.lengths.total 
    segment.percent_z_location                  = 0.043363795685840714
    segment.height                              = 1.779575158  
    segment.width                               = 1.722050901 
    fuselage.append_segment(segment)     
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_20'    
    segment.percent_x_location                  = 1.
    segment.percent_z_location                  = 0.06630560070058995
    segment.height                              = 0.401839552 
    segment.width                               = 0.401839552  
    fuselage.append_segment(segment) 
    
    # add to vehicle
    vehicle.append_component(fuselage)  
    
    
    # ------------------------------------------------------------------
    #   Landing gear
    # ------------------------------------------------------------------  
    landing_gear                                = RCAIDE.Library.Components.Landing_Gear.Landing_Gear()
    main_gear                                   = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear()
    nose_gear                                   = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()
    main_gear.strut_length                      = 12. * Units.inches  
    nose_gear.strut_length                      = 6. * Units.inches 
                                                
    landing_gear.main                           = main_gear
    landing_gear.nose                           = nose_gear
                                                
    #add to vehicle                             
    vehicle.landing_gear                        = landing_gear

    # ########################################################  Energy Network  #########################################################  
    net                                         = RCAIDE.Framework.Networks.Fuel()   

    # add the network to the vehicle
    vehicle.append_energy_network(net) 

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus
    #------------------------------------------------------------------------------------------------------------------------------------  
    fuel_line                                            = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()   

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Fuel Tank & Fuel
    #------------------------------------------------------------------------------------------------------------------------------------       
    fuel_tank                                            = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing) 
    fuel_tank.fuel                                       = RCAIDE.Library.Attributes.Propellants.Aviation_Gasoline()
    fuel_line.fuel_tanks.append(fuel_tank)  

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    starboard_propulsor                                  = RCAIDE.Library.Components.Powertrain.Propulsors.Internal_Combustion_Engine()     
    starboard_propulsor.active_fuel_tanks                = ['fuel_tank']   
                                                     
    # Engine                     
    starboard_engine                                     = RCAIDE.Library.Components.Powertrain.Converters.Engine()
    starboard_engine.sea_level_power                     = 2475 * Units.horsepower # 1,846 KW
    starboard_engine.flat_rate_altitude                  = 0.0
    starboard_engine.rated_speed                         = 1200* Units.rpm
    starboard_engine.power_specific_fuel_consumption     = 0.459 * Units['lb/hp/hr']
    starboard_engine.mass_properties.mass                = 480
    starboard_engine.lenght                              = 2.10
    starboard_engine.origin                              = [[ 9.559106394 ,-4.219315295, 1.616135105]]  
    starboard_propulsor.engine                           = starboard_engine 
     
    # Propeller 
    propeller = RCAIDE.Library.Components.Powertrain.Converters.Propeller()
    propeller.tag                                        = 'starboard_propeller'
    propeller.origin                                     = [[ 9.559106394 ,4.219315295, 1.616135105]]
    propeller.number_of_blades                           = 6.0
    propeller.tip_radius                                 = 3.93/2
    propeller.hub_radius                                 = 0.4
    propeller.cruise.design_freestream_velocity          = 280 * Units.knots
    propeller.cruise.design_angular_velocity             = 1200 * Units.rpm
    propeller.cruise.design_Cl                           = 0.7
    propeller.cruise.design_altitude                     = 25000  * Units.feet
    propeller.cruise.design_thrust                       = 10000 # incorrect 
    propeller.variable_pitch                             = True  
    ospath                                               = os.path.abspath(__file__)
    separator                                            = os.path.sep
    rel_path                                             = os.path.dirname(ospath) + separator + '..' + separator  
    airfoil                                              = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                                          = 'NACA_4412' 
    airfoil.coordinate_file                              =  rel_path + 'Airfoils' + separator + 'NACA_4412.txt'   # absolute path   
    airfoil.polar_files                                  = [ rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_50000.txt',
                                                            rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_100000.txt',
                                                            rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_200000.txt',
                                                            rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_500000.txt',
                                                            rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_1000000.txt']  
    propeller.append_airfoil(airfoil)                   
    propeller.airfoil_polar_stations                     = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]  
    propeller                                            = design_propeller(propeller)    
    starboard_propulsor.propeller                        = propeller

    # ------------------------------------------------------------------
    #   Nacelles
    # ------------------------------------------------------------------ 
    nacelle                                     = RCAIDE.Library.Components.Nacelles.Nacelle()
    nacelle.tag                                 = 'nacelle_1'
    nacelle.length                              = 4.214256527
    nacelle.diameter                            = 0.465412755  
    nacelle.areas.wetted                        = 1.0 # incorrect  
    nacelle.origin                              = [[8.941625295,4.219315295, 1.616135105 ]]
    nacelle.flow_through                        = False    
         
    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_1'
    nac_segment.percent_x_location              = 0.0  
    nac_segment.height                          = 1E-3
    nac_segment.width                           = 1E-3 
    nacelle.append_segment(nac_segment)         
           
    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_2'
    nac_segment.percent_x_location              = 0.4/nacelle.length
    nac_segment.height                          = 0.403618657  
    nac_segment.width                           = 0.403618657 
    nacelle.append_segment(nac_segment)         
           
    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_3'
    nac_segment.percent_x_location              = 1/nacelle.length
    nac_segment.height                          = 0.65
    nac_segment.width                           = 0.65
    nacelle.append_segment(nac_segment)         
            
    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_4'
    nac_segment.percent_x_location              = 2/ nacelle.length
    nac_segment.height                          = 0.7 
    nac_segment.width                           = 0.7 
    nacelle.append_segment(nac_segment)         
    
    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_5'
    nac_segment.percent_x_location              = 4.20344541/ nacelle.length
    nac_segment.height                          = 0.7
    nac_segment.width                           = 0.7
    nacelle.append_segment(nac_segment)             
    
    
    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_6'
    nac_segment.percent_x_location              = 4.214256527/ nacelle.length
    nac_segment.height                          = 3.9167E-09 
    nac_segment.width                           = 3.9167E-09 
    nacelle.append_segment(nac_segment)             
        
    starboard_propulsor.nacelle =  nacelle 
    
    fuel_line.propulsors.append(starboard_propulsor)
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Port Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    port_propulsor                             = RCAIDE.Library.Components.Powertrain.Propulsors.Internal_Combustion_Engine()   
    port_propulsor.tag                         = "port_propulsor"
    port_propulsor.active_fuel_tanks           = ['fuel_tank']   

    port_propeller                             = deepcopy(propeller)
    port_propeller.tag                         = 'port_propeller' 
    port_propeller.origin                      = [[ 9.559106394 ,-4.219315295, 1.616135105]] 
    port_propeller.clockwise_rotation          = False        
    port_propulsor.propeller                   = port_propeller  
              
    port_engine                                = deepcopy(starboard_engine)
    port_engine.origin                         = [[ 9.559106394 ,-4.219315295, 1.616135105]]  
    port_propulsor.engine                      = port_engine

    nacelle_2                                    = deepcopy(nacelle)
    nacelle_2.tag                                = 'nacelle_2'
    nacelle_2.origin                             = [[8.941625295,-4.219315295, 1.616135105 ]]
    port_propulsor.nacelle =  nacelle_2  
    
    # append propulsor to distribution line 
    fuel_line.propulsors.append(port_propulsor) 

    #net.fuel_lines.append(fuel_line)        

    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Avionics
    #------------------------------------------------------------------------------------------------------------------------------------ 
    Wuav                                        = 2. * Units.lbs
    avionics                                    = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.mass_properties.uninstalled        = Wuav
    vehicle.avionics                            = avionics     

    #------------------------------------------------------------------------------------------------------------------------------------ 
    #   Vehicle Definition Complete
    #------------------------------------------------------------------------------------------------------------------------------------ 


    return vehicle


# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle): 
    return configs

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------
def plot_mission(results,line_style='bo-'):

    
    # Plot Flight Conditions 
    plot_flight_conditions(results, line_style)
    
    # Plot Aerodynamic Forces 
    plot_aerodynamic_forces(results, line_style)
    
    # Plot Aerodynamic Coefficients 
    plot_aerodynamic_coefficients(results, line_style)
    
    # Plot Static Stability Coefficients 
    plot_stability_coefficients(results, line_style)    
    
    # Drag Components
    plot_drag_components(results, line_style)
    
    # Plot Altitude, sfc, vehicle weight 
    plot_altitude_sfc_weight(results, line_style)
    
    # Plot Velocities 
    plot_aircraft_velocities(results, line_style)  
    
    # Plot Trajectory
    plot_flight_trajectory(results)

    return 
 

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'
 

    # unpack Segments module
    Segments = RCAIDE.Framework.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()
    


    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------    

    segment     = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.base ) 
    segment.altitude                                = 12000. * Units.feet
    segment.air_speed                               = 119.   * Units.knots
    segment.distance                                = 10 * Units.nautical_mile
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.RPM.active                    = True           
    segment.assigned_control_variables.RPM.assigned_propulsors       = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.RPM.initial_guess             = True 
    segment.assigned_control_variables.RPM.initial_guess_values      = [[2500]] 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                  
    
    mission.append_segment(segment)


    return mission


def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle 

    # ------------------------------------------------------------------
    #  Weights
    weights = RCAIDE.Framework.Analyses.Weights.Weights_Transport()
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()  
    aerodynamics.settings.drag_coefficient_increment = 0.0000
    analyses.append(aerodynamics) 

    # ------------------------------------------------------------------
    #  Energy
    energy= RCAIDE.Framework.Analyses.Energy.Energy()  
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


def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

def missions_setup(mission): 
 
    missions         = RCAIDE.Framework.Analyses.Mission.Missions()
    
    # base mission 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions  


def save_results(results):
 
    # Store data (serialize)
    with open('B737_results.pkl', 'wb') as file:
        pickle.dump(results, file)
        
    return
 
if __name__ == '__main__': 
    main()    
    plt.show()
 

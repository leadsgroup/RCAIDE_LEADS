''' 
# Tiltrotor.py
# 
# Created: May 2019, M Clarke
#          Sep 2020, M. Clarke 

'''
#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units  
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Rotor  import design_electric_rotor
from RCAIDE.Library.Plots                                         import * 
from RCAIDE import  load 
from RCAIDE import  save  
from RCAIDE.Framework.External_Interfaces.OpenVSP.export_vsp_vehicle  import export_vsp_vehicle 

from RCAIDE.Library.Methods.Geometry.Airfoil.compute_airfoil_properties import compute_airfoil_properties
from RCAIDE.Library.Methods.Geometry.Airfoil.import_airfoil_geometry    import import_airfoil_geometry 

import os
import numpy as np 
from copy import deepcopy
import matplotlib.pyplot as plt 
import  pickle

# ----------------------------------------------------------------------------------------------------------------------
#  REGRESSION
# ----------------------------------------------------------------------------------------------------------------------  
def main():           
    save_figure_flag = False 
    # vehicle data
    new_geometry    = True
    redesign_rotors = True
    plot_geometry   = True
    if new_geometry :
        vehicle  = vehicle_setup(redesign_rotors)
        save_aircraft_geometry(vehicle , 'Astar_350')
    else: 
        vehicle = load_aircraft_geometry('Astar_350') 
        
    # Export VSP Model
    # export_vsp_vehicle(vehicle, 'Tiltrotor')    
    
    # # Set up vehicle configs
    # configs  = configs_setup(vehicle)

    # # # create analyses
    # analyses = analyses_setup(configs)

    # # # mission analyses 
    # mission = mission_setup(analyses)
    
    # # # create mission instances (for multiple types of missions)
    # missions = missions_setup(mission) 
     
    # # # mission analysis 
    # results = missions.base_mission.evaluate()
    
    # # # plot results
    # plot_results(results, save_figure_flag)
    
    
    if plot_geometry:
        # plot vehicle 
        plot_3d_vehicle(vehicle, fuselage_opacity = 0.5)      
     

    return
 
def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

def base_analysis(vehicle):
       # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle() 
    analyses.vehicle = vehicle
    # ------------------------------------------------------------------
    #  Geometry
    # ------------------------------------------------------------------
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    # geometry.vehicle                               = vehicle 
    geometry.settings.update_center_of_gravity     = True 
    analyses.append(geometry)

    # ------------------------------------------------------------------
    #  Weights
    weights         = RCAIDE.Framework.Analyses.Weights.Electric()
    weights.aircraft_type = "VTOL"
    # weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics         = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.settings.maximum_lift_coefficient   =  1.5 
    aerodynamics.settings.drag_coefficient_increment =  0.01 
    # aerodynamics.vehicle = vehicle 
    analyses.append(aerodynamics)
     
    ## ------------------------------------------------------------------
    ##  Stability Analysis
    #stability         = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method() 
    #stability.vehicle = vehicle 
    #analyses.append(stability)    

    # ------------------------------------------------------------------
    #  Noise Analysis
    # noise = RCAIDE.Framework.Analyses.Noise.Frequency_Domain_Buildup()
    # noise.vehicle = vehicle 
    # analyses.append(noise)    
    
    # ------------------------------------------------------------------
    #  Energy 
    energy          = RCAIDE.Framework.Analyses.Energy.Energy()
    # energy.vehicle = vehicle 
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
#   Build the Vehicle
# ----------------------------------------------------------------------
def vehicle_setup(redesign_rotors=True) : 

    ospath      = os.path.abspath(__file__)
    separator   = os.path.sep
    # airfoil_path    = os.path.dirname(ospath) + separator  + '..' + separator  
    airfoil_path  = 'RESEARCH/Aircraft/'
    local_path  = os.path.dirname(ospath) + separator          
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    vehicle                                   = RCAIDE.Vehicle()
    vehicle.tag                               = 'Astar_350'
    vehicle.configuration                     = 'eVTOL'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    # mass properties
    vehicle.mass_properties.takeoff                   = 4300 * Units['lbs']
    vehicle.mass_properties.operating_empty           = 4300 * Units['lbs']     
    vehicle.mass_properties.max_takeoff               = 4300 * Units['lbs']  
    vehicle.mass_properties.max_payload               = 0 * Units['lbs']  #maybe change
    vehicle.mass_properties.min_payload               = 0 * Units['lbs']  #maybe change
    # vehicle.mass_properties.center_of_gravity         = [[3.40,   0.  ,  0. ]] #3.4 meters from nose     
    vehicle.mass_properties.moments_of_inertia.tensor = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    vehicle.reference_area                            = 10.39 #maybe change
    vehicle.flight_envelope.ultimate_load             = 5.7   #maybe change
    vehicle.flight_envelope.positive_limit_load       = 3.  #maybe change
    vehicle.number_of_passengers                      = 4 #maybe change


    #------------------------------------------------------------------------------------------------------------------------------------
    # ##################################################### Landing Gear ################################################################    
    #------------------------------------------------------------------------------------------------------------------------------------ 
    main_gear                                = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear() 
    main_gear.tire_diameter                  = 0  *  Units.inches 
    main_gear.rim_diameter                   = 3  *  Units.inches 
    main_gear.tire_width                     = 0  *  Units.inches 
    main_gear.strut_length                   = 12  * Units.ft 
    main_gear.wheels                         = 0   
    main_gear.number_of_gear_types_in_tandem = 1
    main_gear.number_of_wheels_in_gear_type  = 1
    main_gear.fairing                        = True
    main_gear.xz_plane_symmetric             = True
    main_gear.gear_extended                  = True
    vehicle.append_component(main_gear)  

    #htail
    ospath                                = os.path.abspath(__file__)
    separator                             = os.path.sep
    airfoil                               = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    airfoil.NACA_4_Series_code            = '2312'       
                                              
                                              
    # WING PROPERTIES                         
    wing                                      = RCAIDE.Library.Components.Wings.Horizontal_Tail() 
    wing.tag                                  = 'horizontal_tail'  
    wing.aspect_ratio                         = 2.63 
    wing.sweeps.quarter_chord                 = 0  * Units.degrees 
    wing.thickness_to_chord                   = 0.1 
    wing.spans.projected                      = 2.11
    wing.chords.root                          = .4 
    wing.total_length                         = 2.11
    wing.chords.tip                           = .4 
    wing.taper                                = 1.0 
    wing.chords.mean_aerodynamic              = 0.4
    wing.dihedral                             = 0.0 * Units.degrees 
    wing.areas.reference                      = 0.844
    wing.areas.wetted                         = 0.844 * 2.1 
    wing.areas.exposed                        = 0.844 * 0.9 
    wing.twists.root                          = 0 * Units.degrees 
    wing.twists.tip                           = 0 * Units.degrees 
    wing.origin                               = [[ 6.967 , 0.0 ,0.410 ]]
    wing.aerodynamic_center                   = [  6.967,  0., 0.470  ]  
    wing.winglet_fraction                     = 0.0 
    wing.xz_plane_symmetric                   = True

    # Segment                                              
    segment                                   = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                               = 'Section_1'   
    segment.percent_span_location             = 0.0
    segment.twist                             = 0 
    segment.root_chord_percent                = 1 
    segment.dihedral_outboard                 = wing.dihedral
    segment.sweeps.quarter_chord              =  wing.sweeps.quarter_chord  
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)                           
                                              
    # Segment                                               
    segment                                   = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                               = 'Section_2'    
    segment.percent_span_location             = 1
    segment.twist                             = 0
    segment.root_chord_percent                = wing.taper
    segment.dihedral_outboard                 = 0. * Units.degrees
    segment.sweeps.quarter_chord              = 0 * Units.degrees  
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)     

    # add to vehicle
    vehicle.append_component(wing)   
    


    # WING PROPERTIES                         
    v_tail                                      = RCAIDE.Library.Components.Wings.Vertical_Tail() 
    v_tail.tag                                  = 'main_wing'  
    v_tail.aspect_ratio                         = 2.97486
    v_tail.sweeps.quarter_chord                 = 32.28571  * Units.degrees 
    v_tail.thickness_to_chord                   = 0.1 
    v_tail.spans.projected                      = 1.32511
    v_tail.chords.root                          = 0.34921
    v_tail.total_length                         = 2.11
    v_tail.chords.tip                           = 0.34921
    v_tail.taper                                = 0.34921/0.54167
    v_tail.chords.mean_aerodynamic              = 0.45237
    v_tail.dihedral                             = 0.0 * Units.degrees 
    v_tail.areas.reference                      = 0.59025
    v_tail.areas.wetted                         = 0.59025 * 2.1 
    v_tail.areas.exposed                        = 0.59025 * 0.9 
    v_tail.twists.root                          = 0 * Units.degrees 
    v_tail.twists.tip                           = 0 * Units.degrees 
    v_tail.origin                               = [[ 9.7 , 0.0 ,  0.65 ]]
    # v_tail.aerodynamic_center                   = [ 9.7 , 0.0 ,  0.65 ]
    v_tail.winglet_fraction                     = 0.0 
    # v_tail.xz_plane_symmetric                   = True
    # v_tail.xy_plane_symmetric                   = True


    # Segment                                              
    segment                                   = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                               = 'Section_1'   
    segment.percent_span_location             = 0.0
    segment.twist                             = 0 
    segment.root_chord_percent                = 1 
    segment.dihedral_outboard                 = v_tail.dihedral
    segment.sweeps.quarter_chord              =  v_tail.sweeps.quarter_chord  
    segment.append_airfoil(airfoil)
    v_tail.append_segment(segment)                           
                                              
    # Segment                                               
    segment                                   = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                               = 'Section_2'    
    segment.percent_span_location             = 1
    segment.twist                             = 0
    segment.root_chord_percent                = v_tail.taper
    segment.dihedral_outboard                 = 0. * Units.degrees
    segment.sweeps.quarter_chord              = 0 * Units.degrees  
    segment.append_airfoil(airfoil)
    v_tail.append_segment(segment)                                 

    # add to vehicle
    vehicle.append_component(v_tail)  



    # WING PROPERTIES                         
    v_tail_down                                      = RCAIDE.Library.Components.Wings.Vertical_Tail() 
    v_tail_down.tag                                  = 'v_tail_upside'  
    v_tail_down.aspect_ratio                         = 1.2
    v_tail_down.sweeps.quarter_chord                 = 32.28571  * Units.degrees 
    v_tail_down.thickness_to_chord                   = 0.1 
    v_tail_down.spans.projected                      = -1.0
    v_tail_down.chords.root                          = 0.34921
    v_tail_down.total_length                         = .5
    v_tail_down.chords.tip                           = 0.34921
    v_tail_down.taper                                = 0.34921/0.54167
    v_tail_down.chords.mean_aerodynamic              = 0.45237
    v_tail_down.dihedral                             = -180.0 * Units.degrees 
    v_tail_down.areas.reference                      = 0.59025
    v_tail_down.areas.wetted                         = 0.59025 * 2.1 
    v_tail_down.areas.exposed                        = 0.59025 * 0.9 
    v_tail_down.twists.root                          = 0 * Units.degrees 
    v_tail_down.twists.tip                           = 0 * Units.degrees 
    v_tail_down.origin                               = [[ 9.7 , 0.0 ,  0.65 ]]
    # v_tail.aerodynamic_center                   = [ 9.7 , 0.0 ,  0.65 ]
    v_tail_down.winglet_fraction                     = 0.0 
    # v_tail.xz_plane_symmetric                   = True
    # v_tail.xy_plane_symmetric                   = True


    # Segment                                              
    segment                                   = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                               = 'segment_1'   
    segment.percent_span_location             = 0.0
    segment.twist                             = 0 
    segment.root_chord_percent                = 1 
    segment.dihedral_outboard                 = v_tail_down.dihedral
    segment.sweeps.quarter_chord              =  v_tail_down.sweeps.quarter_chord  
    segment.append_airfoil(airfoil)
    v_tail_down.append_segment(segment)                           
                                              
    # Segment                                               
    segment                                   = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                               = 'segment_2'    
    segment.percent_span_location             = 1
    segment.twist                             = 0
    segment.root_chord_percent                = v_tail.taper
    segment.dihedral_outboard                 = 0. * Units.degrees
    segment.sweeps.quarter_chord              = 0 * Units.degrees  
    segment.append_airfoil(airfoil)
    v_tail_down.append_segment(segment)                                 

    # add to vehicle
    vehicle.append_component(v_tail_down)  
     
    # ---------------------------------------------------------------   
    # FUSELAGE                
    # ---------------------------------------------------------------   
    # FUSELAGE PROPERTIES
    fuselage                                    = RCAIDE.Library.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage' 
    fuselage.seats_abreast                      = 2.  
    fuselage.seat_pitch                         = 2.  
    fuselage.fineness.nose                      = 4.29/5.96   
    fuselage.fineness.tail                      = 19.43/5.96 
    fuselage.lengths.nose                       = 4.29 
    fuselage.lengths.tail                       = 1.5
    fuselage.lengths.cabin                      = 4.46 
    fuselage.lengths.total                      = 10.75
    fuselage.width                              = 6.06 * Units.feet
    fuselage.heights.maximum                    = 5.96 * Units.feet      
    fuselage.heights.at_quarter_length          = 5.96 * Units.feet
    # fuselage.heights.at_wing_root_quarter_chord = 5.31 * Units.feet      # change 
    fuselage.heights.at_three_quarters_length   = 1.24* Units.feet      # change 
    fuselage.areas.wetted                       = 17.189 * Units.feet**2   # change 
    fuselage.areas.front_projected              = 17.189 * Units.feet**2   # change 
    fuselage.effective_diameter                 = 7.15 * Units.feet     # change 
    fuselage.differential_pressure              = 0. 

    # define cabin    
    cabin                                             = RCAIDE.Library.Components.Fuselages.Cabins.Cabin()
    cabin.origin                                      = [[1, 0, 0]]
    cabin.offset_x = 1.0
    economy_class                                     = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest              = 2
    economy_class.number_of_rows                      = 2 
    economy_class.seat_arm_rest_width                 = 2 *  Units.inches 
    economy_class.seat_width                          = 15 *  Units.inches
    economy_class.aisle_width                          = 0  *  Units.inches  
    economy_class.number_of_seats                     = economy_class.number_of_rows  * economy_class.number_of_seats_abrest 
    cabin.append_cabin_class(economy_class)
    fuselage.append_cabin(cabin)



    # Segment 0
    segment = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                        = 'segment_0'
    segment.percent_x_location         = 0.0
    segment.percent_y_location         = 0.0
    segment.percent_z_location         = 0.0
    segment.width                      = 0.0
    segment.height                     = 0.0
    fuselage.append_segment(segment)

    # Segment 1
    segment = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                        = 'segment_1'
    segment.percent_x_location         = 0.06557
    segment.percent_y_location         = 0.0
    segment.percent_z_location         = 0.0052
    segment.width                      = 1.487723214
    segment.height                     = 0.8638392857
    fuselage.append_segment(segment)

    # Segment 2
    segment = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                        = 'segment_2'
    segment.percent_x_location         = 0.0824
    segment.percent_y_location         = 0.0
    segment.percent_z_location         = 0.0082
    segment.width                      = 1.586200893
    segment.height                     = 1.151785714
    fuselage.append_segment(segment)

    # Segment 3
    segment = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                        = 'segment_3'
    segment.percent_x_location         = 0.15762
    segment.percent_y_location         = 0.0
    segment.percent_z_location         = 0.02059
    segment.width                      = 1.663082589
    segment.height                     = 1.631696429
    fuselage.append_segment(segment)

    # Segment 4
    segment = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                        = 'segment_4'
    segment.percent_x_location         = 0.20422
    segment.percent_y_location         = 0.0
    segment.percent_z_location         = 0.02
    segment.width                      = 1.663082589
    segment.height                     = 1.631696429
    fuselage.append_segment(segment)

    # Segment 5
    segment = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                        = 'segment_5'
    segment.percent_x_location         = 0.22951
    segment.percent_y_location         = 0.0
    segment.percent_z_location         = 0.02
    segment.width                      = 1.823660714
    segment.height                     = 1.631696429
    fuselage.append_segment(segment)

    # Segment 6
    segment = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                        = 'segment_6'
    segment.percent_x_location         = 0.26082
    segment.percent_y_location         = 0.0
    segment.percent_z_location         = 0.03
    segment.width                      = 1.727678571
    segment.height                     = 1.919642857
    fuselage.append_segment(segment)

    # Segment 7
    segment = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                        = 'segment_7'
    segment.percent_x_location         = 0.32979
    segment.percent_y_location         = 0.0
    segment.percent_z_location         = 0.035
    segment.width                      = 1.535714286
    segment.height                     = 1.823660714
    fuselage.append_segment(segment)

    # Segment 8
    segment = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                        = 'segment_8'
    segment.percent_x_location         = 0.378
    segment.percent_y_location         = 0.0
    segment.percent_z_location         = 0.04
    segment.width                      = 1.151785714
    segment.height                     = 1.675944196
    fuselage.append_segment(segment)

    # Segment 9
    segment = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                        = 'segment_9'
    segment.percent_x_location         = 0.40592
    segment.percent_y_location         = 0.0
    segment.percent_z_location         = 0.05
    segment.width                      = 1.055803571
    segment.height                     = 1.535714286
    fuselage.append_segment(segment)

    # Segment 10
    segment = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                        = 'segment_10'
    segment.percent_x_location         = 0.42322
    segment.percent_y_location         = 0.0
    segment.percent_z_location         = 0.045
    segment.width                      = 0.9598214286
    segment.height                     = 1.34375
    fuselage.append_segment(segment)

    # Segment 11
    segment = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                        = 'segment_11'
    segment.percent_x_location         = 0.44557
    segment.percent_y_location         = 0.0
    segment.percent_z_location         = 0.02849
    segment.width                      = 0.7678571429
    segment.height                     = 0.9598214286
    fuselage.append_segment(segment)

    # Segment 12
    segment = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                        = 'segment_12'
    segment.percent_x_location         = 0.57817
    segment.percent_y_location         = 0.0
    segment.percent_z_location         = 0.04
    segment.width                      = 0.5758928571
    segment.height                     = 0.671875
    fuselage.append_segment(segment)

    # Segment 13
    segment = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                        = 'segment_13'
    segment.percent_x_location         = 0.71993
    segment.percent_y_location         = 0.0
    segment.percent_z_location         = 0.05098
    segment.width                      = 0.3839285714
    segment.height                     = 0.5279017857
    fuselage.append_segment(segment)

    # Segment 14
    segment = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                        = 'segment_14'
    segment.percent_x_location         = 0.81291
    segment.percent_y_location         = 0.0
    segment.percent_z_location         = 0.05744
    segment.width                      = 0.1919642857
    segment.height                     = 0.4607142857
    fuselage.append_segment(segment)

    # Segment 15
    segment = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                        = 'segment_15'
    segment.percent_x_location         = 1.0
    segment.percent_y_location         = 0.0
    segment.percent_z_location         = 0.0685
    segment.width                      = 0.0
    segment.height                     = 0.0
    fuselage.append_segment(segment)


    vehicle.append_component(fuselage)


    # ------------------------------------------------------------------------------------------------------------------------------------
    ########################################################  Energy Network  ######################################################### 
    # ------------------------------------------------------------------------------------------------------------------------------------
    # define network
    network                               = RCAIDE.Framework.Networks.Fuel()
    #------------------------------------------------------------------------------------------------------------------------- 
    # Fuel Distrubition Line 
    #------------------------------------------------------------------------------------------------------------------------- 
    fuel_line                             = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()  
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------      
    main_shaft_engine                     = RCAIDE.Library.Components.Powertrain.Propulsors.Internal_Combustion_Engine() 

    engine                                     = RCAIDE.Library.Components.Powertrain.Converters.Engine()
    main_shaft_engine.engine                            = engine 
    main_shaft_engine.tag                = 'astar_rotor'
    
    # define rotor 
    main_rotor                            = RCAIDE.Library.Components.Powertrain.Converters.Lift_Rotor()
    main_rotor.inputs                     = Data()
    main_rotor.origin                     = [[4.008, 0., 2]]
    main_rotor.tag                        = 'astar_rotor_prop'

    main_rotor.active                                      = True          
    # lift_rotor.tip_radius                                  = 2.8/2
    # lift_rotor.hub_radius                                  = 0.1 
    # lift_rotor.number_of_blades                            = 3     
    # lift_rotor.hover.design_altitude                       = 40 * Units.feet  
    # lift_rotor.hover.design_thrust                         = Hover_Load/8
    # lift_rotor.hover.design_freestream_velocity            = np.sqrt(lift_rotor.hover.design_thrust/(2*1.2*np.pi*(lift_rotor.tip_radius**2)))  
    # lift_rotor.oei.design_altitude                         = 40 * Units.feet  
    # lift_rotor.oei.design_thrust                           = Hover_Load/7  
    # lift_rotor.oei.design_freestream_velocity              = np.sqrt(lift_rotor.oei.design_thrust/(2*1.2*np.pi*(lift_rotor.tip_radius**2)))  

    main_rotor.tip_radius                 = 10.69/2
    main_rotor.hub_radius                 = 0.1
    main_rotor.number_of_blades           = 3  
    main_rotor.thrust_angle               = 0.
    main_rotor.airfoil_flag               = True
    num_sec                               = 20


    non_dim_r                             = np.linspace(main_rotor.hub_radius/main_rotor.tip_radius,0.99,num_sec)
    main_rotor.radius_distribution        = non_dim_r*main_rotor.tip_radius
    main_rotor.thickness_to_chord         = np.ones(num_sec)*0.12
    main_rotor.chord_distribution         = np.ones(num_sec)*0.2 
    main_rotor.max_thickness_distribution = main_rotor.thickness_to_chord* main_rotor.chord_distribution
    main_rotor.twist_distribution         =  np.ones(num_sec)* (90-12)*Units.degrees 

    g                                                      = 9.81                                   # gravitational acceleration 
    speed_of_sound                                         = 340                                    # speed of sound 
    Hover_Load                                             = vehicle.mass_properties.takeoff*g *1.1 # hover load  

    main_rotor.hover.design_altitude                       = 40 * Units.feet  
    main_rotor.hover.design_thrust                         = Hover_Load
    main_rotor.hover.design_freestream_velocity            = np.sqrt(main_rotor.hover.design_thrust/(2*1.2*np.pi*(main_rotor.tip_radius**2)))  
    main_rotor.oei.design_altitude                         = 40 * Units.feet  
    main_rotor.oei.design_thrust                           = Hover_Load  
    main_rotor.oei.design_freestream_velocity              = np.sqrt(main_rotor.oei.design_thrust/(2*1.2*np.pi*(main_rotor.tip_radius**2)))  
    # main_rotor.hover.design_blade_pitch_command =  12 * Units.degrees


    # main_rotor.design_cruise_tip_mach        = 0.6
    main_rotor.cruise.design_angular_velocity     = 396 * Units.rpm
    main_rotor.cruise.design_cruise_altitude        = 1000 * Units.ft
    main_rotor.cruise.design_cruise_thrust          = 6000 * Units.lbf
    main_rotor.cruise.design_blade_pitch_command =  12 * Units.degrees



    ospath                                = os.path.abspath(__file__)
    separator                             = os.path.sep
    airfoil_path                          = 'RESEARCH/Aircraft/'
    airfoil_1                             = RCAIDE.Library.Components.Airfoils.Airfoil()   
    airfoil_1.coordinate_file             =  airfoil_path + 'Airfoils' + separator + 'NACA_4412.txt'
    airfoil_1.polar_files                 = [airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_50000.txt' ,
                                           airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_100000.txt' ,
                                           airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_200000.txt' ,
                                           airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_500000.txt' ,
                                           airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_1000000.txt',
                                           airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_3500000.txt',
                                           airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_5000000.txt',
                                           airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_7500000.txt' ]
    
    airfoil_1.geometry                    = import_airfoil_geometry(airfoil_1.coordinate_file,airfoil_1.number_of_points)
    airfoil_1.polars                      = compute_airfoil_properties(airfoil_1.geometry,airfoil_1.polar_files)
    main_rotor.append_airfoil(airfoil_1)   
     
    main_rotor.mid_chord_alignment        = np.zeros_like(main_rotor.chord_distribution)  
    main_rotor.airfoil_polar_stations     = list(np.zeros(num_sec).astype(int))
    main_shaft_engine.propeller           = main_rotor
    
    # append propulsor to network
    network.propulsors.append(main_shaft_engine)
    
    #------------------------------------------------------------------------------------------------------------------------- 
    #  Energy Source: Fuel Tank
    #-------------------------------------------------------------------------------------------------------------------------  
    tank                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank()  
    tank.fuel                         = RCAIDE.Library.Attributes.Propellants.Jet_A() 
    fuel_line.fuel_tanks.append(tank) 

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to fuel line to network      
    fuel_line.assigned_propulsors     =  [[main_shaft_engine.tag]]

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Append fuel line to fuel line to network      
    network.fuel_lines.append(fuel_line)       
  
    # append energy network 
    vehicle.append_energy_network(network)     

    return vehicle 
             
 
def configs_setup(vehicle):
    '''
    The configration set up below the scheduling of the nacelle angle and vehicle speed.
    Since one prop_rotor operates at varying flight conditions, one must perscribe  the 
    pitch command of the prop_rotor which us used in the variable pitch model in the analyses
    Note: low pitch at take off & low speeds, high pitch at cruise
    '''
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------ 
    configs = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config                                                       = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag                                                   = 'base'     
    configs.append(base_config)  
     

    return configs


# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
def mission_setup(analyses):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments  
    base_segment = Segments.Segment() 
    base_segment.state.numerics.solver.type = 'optimize' 
    

    # beta_cruise = analyses.low_speed_transition.vehicle.networks.electric.propulsors.prop_rotor_propulsor_1.rotor.cruise.design_blade_pitch_command
    
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------ 
    segment                                            = Segments.Vertical_Flight.Climb(base_segment)
    segment.tag                                        = "Vertical_Climb"   
    segment.analyses.extend(analyses.base) 
    segment.altitude_start                             = 0.0  * Units.ft  
    segment.altitude_end                               = 15.  * Units.ft   
    segment.climb_rate                                 = 180. * Units['ft/min'] 
    segment.true_course                                = 0   * Units.degree  
    segment.state.numerics.solver.type = 'root_finder' 

    # define flight dynamics to model  
    segment.flight_dynamics.force_z                    = True 

    # define flight controls  
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['astar_rotor']]  
    segment.assigned_control_variables.throttle.initial_guess_values = [[0.5]]
    
    mission.append_segment(segment)   
     
    
    # # ------------------------------------------------------------------
    # #   First Cruise Segment: Constant Acceleration, Constant Altitude
    # # ------------------------------------------------------------------ 
    # segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    # segment.tag                      = "Accelerating_Level"  
    # segment.analyses.extend(analyses.base)   
    # # segment.climb_rate               = 0. * Units['ft/min']   #unkown climb rate in AEDT
    # segment.air_speed_start          = 0 * Units['kts']   
    # segment.air_speed_end            = 30 * Units['kts']   
    # # segment.altitude_end             = 500.0 * Units.ft
    # segment.true_course              = 0 * Units.degree 
    # segment.state.numerics.solver.type = 'root_finder' 

    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True      
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['astar_rotor']]  
    # segment.assigned_control_variables.body_angle.active             = True 
                                                                             
    # mission.append_segment(segment)   

    # # ------------------------------------------------------------------
    # #   First Cruise Segment: Constant Acceleration, Constant Altitude
    # # ------------------------------------------------------------------ 

    # segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    # segment.tag                      = "Accel_Climb"  
    # segment.analyses.extend(analyses.base)   
    # # segment.climb_rate               = 0. * Units['ft/min']   #unkown climb rate in AEDT
    # segment.air_speed_end            = 63 * Units['kts']   
    # segment.altitude_end             = 30.0 * Units.ft
    # segment.distance                 = 500.0 * Units.ft
    # segment.true_course              = 0 * Units.degree 
    # segment.state.numerics.solver.type = 'root_finder' 

    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['astar_rotor']]  
    # segment.assigned_control_variables.body_angle.active             = True 
                                                                             
    # mission.append_segment(segment)   
   
    # # ------------------------------------------------------------------
    # #   First Cruise Segment: Constant Acceleration, Constant Altitude
    # # ------------------------------------------------------------------ 
    # segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    # segment.tag                      = "Constant_Vel_Climb"  
    # segment.analyses.extend(analyses.base)   
    # # segment.climb_rate               = 30/50. * Units['ft/min']   #unkown climb rate in AEDT
    # segment.air_speed           = 63 * Units['kts']   
    # segment.altitude_end             = 1000.0 * Units.ft
    # segment.distance                 = 3500.0 * Units.ft
    # segment.true_course              = 0 * Units.degree 
    # segment.state.numerics.solver.type = 'root_finder' 

    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['astar_rotor']]
    # segment.assigned_control_variables.body_angle.active             = True 
                                                                             
    # mission.append_segment(segment)   


    # segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    # segment.tag                      = "Accel_level"  
    # segment.analyses.extend(analyses.base)   
    # # segment.climb_rate               = 30/50. * Units['ft/min']   #unkown climb rate in AEDT
    # segment.air_speed_start           = 63 * Units['kts']   
    # segment.air_speed_end            = 116 * Units['kts']
    # segment.altitude_end             = 1000.0 * Units.ft
    # segment.distance                 = 2800.0 * Units.ft
    # segment.true_course              = 0 * Units.degree 
    # segment.state.numerics.solver.type = 'root_finder' 

    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['astar_rotor']]  
    # segment.assigned_control_variables.body_angle.active             = True 
                                                                             
    # mission.append_segment(segment)   


    # segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    # segment.tag                      = "Constant_Vel_cruise"  
    # segment.analyses.extend(analyses.base)   
    # # segment.air_speed_start           = 63 * Units['kts']   
    # segment.air_speed            = 116 * Units['kts']
    # # segment.altitude_end             = 1000.0 * Units.ft
    # segment.distance                 = 93100.0 * Units.ft
    # segment.true_course              = 0 * Units.degree 
    # segment.state.numerics.solver.type = 'root_finder' 

    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['astar_rotor']]  
    # segment.assigned_control_variables.body_angle.active             = True 
                                                                             
    # mission.append_segment(segment)  

    
    return mission



def missions_setup(mission): 
 
    missions         = RCAIDE.Framework.Mission.Missions()
    
    # base mission 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions

def plot_results(results, save_figure_flag):
    # Plots fligh conditions 
    plot_flight_conditions(results, save_figure = save_figure_flag) 
    
    # Plot arcraft trajectory
    plot_flight_trajectory(results, save_figure = save_figure_flag) 
    
    # Plot Propeller Conditions 
    plot_rotor_conditions(results, save_figure = save_figure_flag)  
    plot_propulsor_throttles(results, save_figure = save_figure_flag)
    return

def save_aircraft_geometry(geometry,filename): 
    pickle_file  = filename + '.pkl'
    with open(pickle_file, 'wb') as file:
        pickle.dump(geometry, file) 
    return 


def load_aircraft_geometry(filename):  
    load_file = filename + '.pkl' 
    with open(load_file, 'rb') as file:
        results = pickle.load(file) 
    return results


def load_propulsor(filename):
    main_shaft_engine =  load(filename)
    return main_shaft_engine

def save_propulsor(main_shaft_engine, filename):
    save(main_shaft_engine, filename)
    return

def display_stability_derivatives(segment):
    """This function displays the stability derivatives of the aircraft."""

    # Get the stability derivatives
    stability_derivatives = segment.conditions.static_stability.derivatives

    # Display the stability derivatives
    print(f"CLift_alpha: {stability_derivatives.Clift_alpha[0,0]:.3f}")
    print(f"CY_beta: {stability_derivatives.CY_beta[0,0]:.3f}")
    print(f"CL_beta: {stability_derivatives.CL_beta[0,0]:.4f}")
    print(f"CM_alpha: {stability_derivatives.CM_alpha[0,0]:.3f}")
    print(f"CN_beta: {stability_derivatives.CN_beta[0,0]:.3f}")
    print(f"CL_p: {stability_derivatives.CL_p[0,0]:.5f}")
    print(f"CL_r: {stability_derivatives.CL_r[0,0]:.5f}")
    print(f"CM_q: {stability_derivatives.CM_q[0,0]:.5f}")
    print(f"CN_p: {stability_derivatives.CN_p[0,0]:.5f}")
    print(f"CN_r: {stability_derivatives.CN_r[0,0]:.5f}")
    print(f"CM_delta_e: {stability_derivatives.CM_delta_e[0,0]:.5f}")
    print(f"CL_delta_a: {stability_derivatives.CL_delta_a[0,0]:.5f}")
    print(f"CN_delta_a: {stability_derivatives.CN_delta_a[0,0]:.5f}")
    print(f"CN_delta_r: {stability_derivatives.CN_delta_r[0,0]:.5f}")
    return

if __name__ == '__main__': 
    main()    
    plt.show()
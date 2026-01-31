''' 
# Vehicle.py
# 
# Created: May 2019, M Clarke
#          Sep 2020, M. Clarke 

'''
#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import RCAIDE
from RCAIDE.Core import Units, Data   
import pickle
from RCAIDE.Visualization.Performance.Aerodynamics.Vehicle import *  
from RCAIDE.Visualization.Performance.Mission import *     
from RCAIDE.Visualization.Performance.Energy.Battery import *   
from RCAIDE.Visualization.Performance.Noise import *  
from RCAIDE.Visualization.Geometry import *
from RCAIDE.Components.Energy.Networks.Battery_Electric_Rotor                 import Battery_Electric_Rotor
from RCAIDE.Methods.Power.Battery.Sizing                                      import initialize_from_mass 
from RCAIDE.Methods.Geometry.Two_Dimensional.Planform                         import segment_properties
from RCAIDE.Methods.Power.Battery.Sizing                                      import initialize_from_circuit_configuration 
from RCAIDE.Methods.Weights.Correlations.Propulsion                           import nasa_motor
from RCAIDE.Methods.Propulsion.electric_motor_sizing                          import size_optimal_motor
from RCAIDE.Methods.Propulsion                                                import propeller_design ,lift_rotor_design 
from RCAIDE.Methods.Weights.Buildups.eVTOL.empty                              import empty
from RCAIDE.Methods.Center_of_Gravity.compute_component_centers_of_gravity    import compute_component_centers_of_gravity
from RCAIDE.Methods.Geometry.Two_Dimensional.Planform.wing_segmented_planform import wing_segmented_planform 
from RCAIDE.Methods.Weights.Buildups.eVTOL.converge_evtol_weight              import converge_evtol_weight  
from RCAIDE.Methods.Performance.estimate_cruise_drag                          import estimate_cruise_drag
 
import os
import numpy as np 
from copy import deepcopy 

# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------
def vehicle_setup(resize_aircraft,vehicle_name = 'Wisk_Cora_CRM') :
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------  
    
    if resize_aircraft:
         
        vehicle                                         = RCAIDE.Vehicle()
        vehicle.tag                                     = vehicle_name
        vehicle.configuration                           = 'eVTOL'
        
        
    
        # ------------------------------------------------------------------
        #   Initialize the Vehicle
        # ------------------------------------------------------------------    
        vehicle                                        = RCAIDE.Vehicle()
        vehicle.tag                                    = 'Wisk_Cora'
        vehicle.configuration                          = 'eVTOL'
    
        # ------------------------------------------------------------------
        #   Vehicle-level Properties
        # ------------------------------------------------------------------    
        # mass properties
        vehicle.mass_properties.takeoff                = 2450. * Units.lb 
        vehicle.mass_properties.operating_empty        = 2250. * Units.lb                
        vehicle.mass_properties.max_takeoff            = 2450. * Units.lb              
        vehicle.mass_properties.center_of_gravity      = [[2.0144,   0.  ,  0. ]]  
        vehicle.reference_area                         = 10.76  
        vehicle.flight_envelope.ultimate_load          = 5.7   
        vehicle.flight_envelope.positive_limit_load    = 3.  
        vehicle.number_of_passengers                   = 2
    
        # ------------------------------------------------------------------    
        # WINGS                                    
        # ------------------------------------------------------------------    
        # WING PROPERTIES           
        wing                                    = RCAIDE.Components.Wings.Main_Wing()
        wing.tag                                = 'main_wing'  
        wing.aspect_ratio                       = 12.000
        wing.sweeps.quarter_chord               = 0.0  * Units.degrees
        wing.thickness_to_chord                 = 0.18  
        wing.taper                              = 1.  
        wing.spans.projected                    = 36.0   * Units.feet            
        wing.chords.root                        = 6.5    * Units.feet            
        wing.total_length                       = 6.5    * Units.feet            
        wing.chords.tip                         = 3.     * Units.feet           
        wing.chords.mean_aerodynamic            = 3.     * Units.feet            
        wing.dihedral                           = 1.0    * Units.degrees         
        wing.areas.reference                    = 10.582 * Units.meter**2       
        wing.areas.wetted                       = 227.5  * Units.feet**2       
        wing.areas.exposed                      = 227.5  * Units.feet**2  
        wing.twists.root                        = 0.0    * Units.degrees  
        wing.twists.tip                         = 0.0    * Units.degrees   
        wing.origin                             = [[  1.067, 0., -0.261 ]]
        wing.aerodynamic_center                 = [1.975 , 0., -0.261]    
        wing.winglet_fraction                   = 0.0  
        wing.xz_plane_symmetric                 = True
        wing.vertical                           = False
    
        # Segment                                  
        segment                                 = RCAIDE.Components.Wings.Segment()
        segment.tag                             = 'Section_1'   
        segment.percent_span_location           = 0.  
        segment.twist                           = 0.  
        segment.root_chord_percent              = 1. 
        segment.dihedral_outboard               = 2.00 * Units.degrees
        segment.sweeps.quarter_chord            = 30.00 * Units.degrees 
        segment.thickness_to_chord              = 0.18  
        wing.append_segment(segment)               
    
        # Segment                                   
        segment                                 = RCAIDE.Components.Wings.Segment()
        segment.tag                             = 'Section_2'    
        segment.percent_span_location           = 0.141 
        segment.twist                           = 0. 
        segment.root_chord_percent              = 0.461383 
        segment.dihedral_outboard               = 2.00 * Units.degrees
        segment.sweeps.quarter_chord            = 0. 
        segment.thickness_to_chord              = 0.16 
        wing.append_segment(segment)               
    
        # Segment                                   
        segment                                 = RCAIDE.Components.Wings.Segment()
        segment.tag                             = 'Section_3'   
        segment.percent_span_location           = 0.8478 
        segment.twist                           = 0. 
        segment.root_chord_percent              = 0.461383   
        segment.dihedral_outboard               = 10.00 * Units.degrees 
        segment.sweeps.quarter_chord            = 17.00 * Units.degrees 
        segment.thickness_to_chord              = 0.16 
        wing.append_segment(segment)               
    
        # Segment                                  
        segment                                 = RCAIDE.Components.Wings.Segment()
        segment.tag                             = 'Section_4'   
        segment.percent_span_location           = 0.96726 
        segment.twist                           = 0. 
        segment.root_chord_percent              = 0.323 
        segment.dihedral_outboard               = 20.0    * Units.degrees 
        segment.sweeps.quarter_chord            = 51.000  * Units.degrees 
        segment.thickness_to_chord              = 0.16  
        wing.append_segment(segment)                
    
        # Segment                                   
        segment                                 = RCAIDE.Components.Wings.Segment()
        segment.tag                             = 'Section_5'   
        segment.percent_span_location           = 1.0 
        segment.twist                           = 0.  
        segment.root_chord_percent              = 0.0413890
        segment.dihedral_outboard               = 1.0  * Units.degrees
        segment.sweeps.quarter_chord            = 0.0 * Units.degrees
        segment.thickness_to_chord              = 0.16 
        wing.append_segment(segment)   
    
        # add to vehicle
        vehicle.append_component(wing)       
    
        # WING PROPERTIES 
        wing                                    = RCAIDE.Components.Wings.Wing()
        wing.tag                                = 'horizontal_tail'  
        wing.aspect_ratio                       = 4.78052
        wing.sweeps.quarter_chord               = 0.0  
        wing.thickness_to_chord                 = 0.12  
        wing.taper                              = 1.0  
        wing.spans.projected                    = 2.914 
        wing.chords.root                        = 0.609
        wing.total_length                       = 0.609
        wing.chords.tip                         = 0.609
        wing.chords.mean_aerodynamic            = 0.609
        wing.dihedral                           = 0.  * Units.degrees  
        wing.areas.reference                    = 5.82204
        wing.areas.wetted                       = 5.82204*2 * Units.feet**2    
        wing.areas.exposed                      = 5.82204*2 * Units.feet**2  
        wing.twists.root                        = 0. * Units.degrees  
        wing.twists.tip                         = 0. * Units.degrees  
        wing.origin                             = [[5.440, 0.0 , 1.28]]
        wing.aerodynamic_center                 = [5.7,  0.,  0.] 
        wing.winglet_fraction                   = 0.0 
        wing.xz_plane_symmetric                 = True    
    
        # add to vehicle
        vehicle.append_component(wing)    
    
    
        # WING PROPERTIES
        wing                                    = RCAIDE.Components.Wings.Wing()
        wing.tag                                = 'vertical_tail_1'
        wing.aspect_ratio                       = 4.30556416
        wing.sweeps.quarter_chord               = 13.68 * Units.degrees 
        wing.thickness_to_chord                 = 0.12
        wing.taper                              = 0.5 
        wing.spans.projected                    = 1.6 
        wing.chords.root                        = 1.2192
        wing.total_length                       = 1.2192
        wing.chords.tip                         = 0.6096
        wing.chords.mean_aerodynamic            = 0.9144
        wing.areas.reference                    = 2.357
        wing.areas.wetted                       = 2.357*2 * Units.feet**2 
        wing.areas.exposed                      = 2.357*2 * Units.feet**2 
        wing.twists.root                        = 0. * Units.degrees 
        wing.twists.tip                         = 0. * Units.degrees  
        wing.origin                             = [[4.900 ,  -1.657 ,  -0.320 ]]
        wing.aerodynamic_center                 = 0.0   
        wing.winglet_fraction                   = 0.0  
        wing.dihedral                           = 6.  * Units.degrees  
        wing.vertical                           = True 
        wing.xz_plane_symmetric                 = False
    
        # add to vehicle
        vehicle.append_component(wing)   
    
    
        # WING PROPERTIES
        wing                                    = RCAIDE.Components.Wings.Wing()
        wing.tag                                = 'vertical_tail_2'
        wing.aspect_ratio                       = 4.30556416
        wing.sweeps.quarter_chord               = 13.68 * Units.degrees 
        wing.thickness_to_chord                 = 0.12
        wing.taper                              = 0.5 
        wing.spans.projected                    = 1.6 
        wing.chords.root                        = 1.2192
        wing.total_length                       = 1.2192
        wing.chords.tip                         = 0.6096
        wing.chords.mean_aerodynamic            = 0.8
        wing.areas.reference                    = 2.357
        wing.areas.wetted                       = 2.357*2 * Units.feet**2 
        wing.areas.exposed                      = 2.357*2 * Units.feet**2 
        wing.twists.root                        = 0. * Units.degrees 
        wing.twists.tip                         = 0. * Units.degrees  
        wing.origin                             = [[4.900 ,  1.657 ,  -0.320 ]]
        wing.aerodynamic_center                 = 0.0   
        wing.winglet_fraction                   = 0.0  
        wing.dihedral                           = -6.  * Units.degrees  
        wing.vertical                           = True   
        wing.xz_plane_symmetric                 = False
    
        # add to vehicle
        vehicle.append_component(wing)   
    
        # ---------------------------------------------------------------   
        # FUSELAGE                
        # ---------------------------------------------------------------   
        # FUSELAGE PROPERTIES
        fuselage                                    = RCAIDE.Components.Fuselages.Fuselage()
        fuselage.tag                                = 'fuselage'
        fuselage.configuration                      = 'Tube_Wing'  
        fuselage.seats_abreast                      = 2.  
        fuselage.seat_pitch                         = 3.  
        fuselage.fineness.nose                      = 0.88   
        fuselage.fineness.tail                      = 1.13   
        fuselage.lengths.nose                       = 3.2 * Units.feet 
        fuselage.lengths.tail                       = 6.4 * Units.feet
        fuselage.lengths.cabin                      = 6.4 * Units.feet 
        fuselage.lengths.total                      = 4.10534
        fuselage.width                              = 5.85 * Units.feet        
        fuselage.heights.maximum                    = 4.65 * Units.feet         
        fuselage.heights.at_quarter_length          = 3.75 * Units.feet         
        fuselage.heights.at_wing_root_quarter_chord = 4.65 * Units.feet         
        fuselage.heights.at_three_quarters_length   = 4.26 * Units.feet         
        fuselage.areas.wetted                       = 236. * Units.feet**2      
        fuselage.areas.front_projected              = 0.14 * Units.feet**2      
        fuselage.effective_diameter                 = 5.85 * Units.feet        
        fuselage.differential_pressure              = 0. 
    
        # Segment  
        segment                                      = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
        segment.tag                                  = 'segment_0'    
        segment.percent_x_location                   = 0.
        segment.percent_z_location                   = 0.
        segment.height                               = 0.1  
        segment.width                                = 0.1  
        fuselage.append_segment(segment)           
    
        # Segment                                   
        segment                                      = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                  = 'segment_1'   
        segment.percent_x_location                   = 0.04679
        segment.percent_z_location                   = 0.00485
        segment.height                               = 0.68505
        segment.width                                = 0.75  
        fuselage.append_segment(segment) 
    
        # Segment                                   
        segment                                      = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                  = 'segment_2'   
        segment.percent_x_location                   = 0.17934
        segment.percent_z_location                   = 0.01583
        segment.height                               = 1.57170
        segment.width                                = 1.19396
        fuselage.append_segment(segment)           
    
        # Segment                                  
        segment                                      = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                  = 'segment_3'   
        segment.percent_x_location                   = 0.47514
        segment.percent_z_location                   = 0.02162
        segment.height                               =  1.60249
        segment.width                                =  1.35312
        fuselage.append_segment(segment)           
    
        # Segment                                   
        segment                                      = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                  = 'segment_4'   
        segment.percent_x_location                   = 0.73757
        segment.percent_z_location                   = 0.05831
        segment.height                               = 0.8379
        segment.width                                = 0.79210 
        fuselage.append_segment(segment)    
    
        # Segment                                   
        segment                                      = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                  = 'segment_5'   
        segment.percent_x_location                   =  1.
        segment.percent_z_location                   = 0.09127
        segment.height                               = 0.1477
        segment.width                                = 0.0981
        fuselage.append_segment(segment)           
    
    
        # add to vehicle
        vehicle.append_component(fuselage) 
    
        #-------------------------------------------------------------------
        # INNER BOOMS   
        #-------------------------------------------------------------------   
        long_boom                                    = RCAIDE.Components.Fuselages.Fuselage()
        long_boom.tag                                = 'boom_1r'
        long_boom.configuration                      = 'boom'  
        long_boom.origin                             = [[0.543,1.630, -0.326]]  
        long_boom.seats_abreast                      = 0.  
        long_boom.seat_pitch                         = 0.0 
        long_boom.fineness.nose                      = 0.950   
        long_boom.fineness.tail                      = 1.029   
        long_boom.lengths.nose                       = 0.2 
        long_boom.lengths.tail                       = 0.2
        long_boom.lengths.cabin                      = 5.0 
        long_boom.lengths.total                      = 5.4
        long_boom.width                              = 0.15 
        long_boom.heights.maximum                    = 0.15  
        long_boom.heights.at_quarter_length          = 0.15  
        long_boom.heights.at_three_quarters_length   = 0.15 
        long_boom.heights.at_wing_root_quarter_chord = 0.15 
        long_boom.areas.wetted                       = 0.018
        long_boom.areas.front_projected              = 0.018 
        long_boom.effective_diameter                 = 0.15  
        long_boom.differential_pressure              = 0. 
        long_boom.y_pitch_count                      = 1
        long_boom.y_pitch                            = 1.196
        long_boom.symmetric                          = True 
        long_boom.index                              = 1
    
        # Segment  
        segment                                      = RCAIDE.Library.Components.Nacelles.Segments.Segment() 
        segment.tag                                  = 'segment_1'   
        segment.percent_x_location                   = 0.
        segment.percent_z_location                   = 0.0 
        segment.height                               = 0.05  
        segment.width                                = 0.05   
        long_boom.append_segment(segment)           
    
        # Segment                                   
        segment                                      = RCAIDE.Library.Components.Nacelles.Segments.Segment()
        segment.tag                                  = 'segment_2'   
        segment.percent_x_location                   = 0.2/ 5.6
        segment.percent_z_location                   = 0. 
        segment.height                               = 0.15 
        segment.width                                = 0.15 
        long_boom.append_segment(segment) 
    
        # Segment                                   
        segment                                      = RCAIDE.Library.Components.Nacelles.Segments.Segment()
        segment.tag                                  = 'segment_3'    
        segment.percent_x_location                   = 5.4/5.6 
        segment.percent_z_location                   = 0. 
        segment.height                               = 0.15
        segment.width                                = 0.15
        long_boom.append_segment(segment)           
    
        # Segment                                  
        segment                                      = RCAIDE.Library.Components.Nacelles.Segments.Segment()
        segment.tag                                  = 'segment_4'   
        segment.percent_x_location                   = 1.   
        segment.percent_z_location                   = 0.   
        segment.height                               = 0.05   
        segment.width                                = 0.05   
        long_boom.append_segment(segment)           
    
        # add to vehicle
        vehicle.append_component(long_boom)   
    
        # add left long boom 
        long_boom                                    = deepcopy(vehicle.fuselages.boom_1r)
        long_boom.origin[0][1]                       = -long_boom.origin[0][1]
        long_boom.tag                                = 'Boom_1L'
        long_boom.index                              = 1 
        vehicle.append_component(long_boom) 
    
    
        #-------------------------------------------------------------------
        # OUTER BOOMS   
        #-------------------------------------------------------------------   
        short_boom                                    = RCAIDE.Components.Fuselages.Fuselage()
        short_boom.tag                                = 'boom_2r'
        short_boom.configuration                      = 'boom'  
        short_boom.origin                             = [[0.543,2.826, -0.326]]    
        short_boom.seats_abreast                      = 0.   
        short_boom.seat_pitch                         = 0.0  
        short_boom.fineness.nose                      = 0.950  
        short_boom.fineness.tail                      = 1.029  
        short_boom.lengths.nose                       = 0.2  
        short_boom.lengths.tail                       = 0.2 
        short_boom.lengths.cabin                      = 2.0 
        short_boom.lengths.total                      = 3.3  
        short_boom.width                              = 0.15  
        short_boom.heights.maximum                    = 0.15   
        short_boom.heights.at_quarter_length          = 0.15   
        short_boom.heights.at_three_quarters_length   = 0.15  
        short_boom.heights.at_wing_root_quarter_chord = 0.15  
        short_boom.areas.wetted                       = 0.018 
        short_boom.areas.front_projected              = 0.018  
        short_boom.effective_diameter                 = 0.15   
        short_boom.differential_pressure              = 0. 
        short_boom.y_pitch_count                      = 2
        short_boom.y_pitch                            = 1.196 
        short_boom.symmetric                          = True 
        short_boom.index                              = 1
    
        # Segment  
        segment                           = RCAIDE.Library.Components.Nacelles.Segments.Segment() 
        segment.tag                       = 'segment_1'   
        segment.percent_x_location        = 0.
        segment.percent_z_location        = 0.0 
        segment.height                    = 0.05  
        segment.width                     = 0.05   
        short_boom.append_segment(segment)           
    
        # Segment                                   
        segment                           = RCAIDE.Library.Components.Nacelles.Segments.Segment()
        segment.tag                       = 'segment_2'   
        segment.percent_x_location        = 0.2/3.3
        segment.percent_z_location        = 0. 
        segment.height                    = 0.15 
        segment.width                     = 0.15 
        short_boom.append_segment(segment) 
    
        # Segment                                   
        segment                           = RCAIDE.Library.Components.Nacelles.Segments.Segment()
        segment.tag                       = 'segment_3'   
        segment.percent_x_location        = 3.1/3.3
        segment.percent_z_location        = 0. 
        segment.height                    = 0.15
        segment.width                     = 0.15
        short_boom.append_segment(segment)           
    
        # Segment                                  
        segment                           = RCAIDE.Library.Components.Nacelles.Segments.Segment()
        segment.tag                       = 'segment_4'     
        segment.percent_x_location        = 1.   
        segment.percent_z_location        = 0.   
        segment.height                    = 0.05   
        segment.width                     = 0.05   
        short_boom.append_segment(segment)       
    
        # add to vehicle
        vehicle.append_component(short_boom)    
    
        # add outer right boom 
        short_boom                           = deepcopy(vehicle.fuselages.boom_2r)
        short_boom.origin[0][1]              = short_boom.y_pitch + short_boom.origin[0][1]
        short_boom.tag                       = 'boom_3r'
        short_boom.index                     = 1 
        vehicle.append_component(short_boom)  
    
        # add inner left boom 
        short_boom                           = deepcopy(vehicle.fuselages.boom_2r)
        short_boom.origin[0][1]              = - (short_boom.origin[0][1])
        short_boom.tag                       = 'boom_2l'
        short_boom.index                     = 1 
        vehicle.append_component(short_boom)     
    
        short_boom                           = deepcopy(vehicle.fuselages.boom_2r)
        short_boom.origin[0][1]              = - (short_boom.origin[0][1] + short_boom.y_pitch)
        short_boom.tag                       = 'boom_3l'
        short_boom.index                     = 1 
        vehicle.append_component(short_boom) 
    
        #------------------------------------------------------------------
        # PROPULSOR
        #------------------------------------------------------------------
        #------------------------------------------------------------------
        # network
        #------------------------------------------------------------------
        net                                 = Battery_Electric_Rotor()
        net.rotor_group_indexes             = [0,1,1,1,1,1,1,1,1,1,1,1,1]
        net.motor_group_indexes             = [0,1,1,1,1,1,1,1,1,1,1,1,1]  
        net.esc_group_indexes               = [0,1,1,1,1,1,1,1,1,1,1,1,1]     
        net.active_propulsor_groups         = [True,True]  
        net.voltage                         = 500.
        
    
        #------------------------------------------------------------------
        # Design Battery
        #------------------------------------------------------------------    
        bat                                                    = RCAIDE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNiMnCoO2_18650() 
        bat.pack.electrical_configuration.series               = 120  
        bat.pack.electrical_configuration.parallel             = 48
        initialize_from_circuit_configuration(bat)  
        bat.module_config.number_of_modules                    = 12 
        bat.module.geometrtic_configuration.total              = bat.pack.electrical_configuration.total
        bat.module_config.voltage                              = bat.pack.max_voltage/bat.module_config.number_of_modules   
        bat.module.geometrtic_configuration.normal_count       = 25  
        bat.module.geometrtic_configuration.parallel_count     = 40   
        net.battery                                            = bat    
        net.voltage                                            = bat.pack.max_voltage
    
    
        # --------------------------------------------------------------
        # Forward Cruise Propulsor System 
        # --------------------------------------------------------------
        # 1. Electronic Speed Controller    
        propeller_esc                                          = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller() 
        propeller_esc.efficiency                               = 0.95  
        propeller_esc.tag                                      = 'propeller_esc'  
        net.electronic_speed_controllers.append(propeller_esc)    
    
        propeller_esc_2                                        = deepcopy(propeller_esc)
        propeller_esc_2.tag                                    = 'propeller_esc_2'
        net.electronic_speed_controllers.append(propeller_esc_2)  
        
        
        # 2. Propeller  
        g                                            = 9.81                                   
        speed_of_sound                               = 340                                       
        Hover_Load                                   = vehicle.mass_properties.takeoff*g       
        
        # Thrust Propeller         
        propeller                                   = RCAIDE.Library.Components.Powertrain.Converters.Propeller()
        

        # Thrust Propeller                          
        propeller                                   = RCAIDE.Library.Components.Powertrain.Converters.Propeller()  
        
        # Component 6: Rotors  
        Drag                                        = estimate_cruise_drag(vehicle,altitude = 1000 * Units.feet  ,speed = V_inf ,lift_coefficient = 0.5 ,profile_drag = 0.05)
        Hover_Load                                  = vehicle.mass_properties.takeoff*g         
            
        propeller.number_of_blades                  = 3
        propeller.tag                               = 'propeller_1'
        propeller.tip_radius                        = 1.0668
        propeller.hub_radius                        = 0.21336 
        propeller.cruise.design_freestream_velocity = V_inf
        propeller.cruise.design_tip_mach            = 0.65
        propeller.cruise.design_angular_velocity    = propeller.cruise.design_tip_mach *speed_of_sound/propeller.tip_radius
        propeller.cruise.design_Cl                  = 0.7
        propeller.cruise.design_altitude            = 1000 * Units.feet  
        propeller.cruise.design_thrust              = Drag*2.5
        propeller.rotation                          = 1
        propeller.variable_pitch                    = True  
        airfoil                                     = RCAIDE.Library.Components.Airfoils.Airfoil()   
        ospath                                      = os.path.abspath(__file__)
        separator                                   = os.path.sep
        rel_path                                    = ospath.split( 'Cora' + separator + 'Common')[0]          
        airfoil.coordinate_file                     = 'NACA_4412.txt'
        airfoil.polar_files                         = ['NACA_4412_polar_Re_50000.txt' ,
                                                      'NACA_4412_polar_Re_100000.txt' ,
                                                      'NACA_4412_polar_Re_200000.txt' ,
                                                      'NACA_4412_polar_Re_500000.txt' ,
                                                      'NACA_4412_polar_Re_1000000.txt',
                                                      'NACA_4412_polar_Re_3500000.txt',
                                                      'NACA_4412_polar_Re_5000000.txt',
                                                      'NACA_4412_polar_Re_7500000.txt' ]
        propeller.append_airfoil(airfoil)          
        propeller.airfoil_polar_stations            = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] 
        propeller                                   = propeller_design(propeller) 
        propeller.origin                            = [[4.10534, 0. ,2.02*0.3048 ]]
        net.rotors.append(propeller)  
     
        
        # 3. Propeller Motors
        propeller_motor                          = RCAIDE.Library.Components.Powertrain.Converters.Motor()
        propeller_motor.efficiency               = 0.95
        propeller_motor.origin                   = propeller.origin
        propeller_motor.nominal_voltage          = bat.pack.max_voltage 
        propeller_motor.origin                   = propeller.origin
        propeller_motor.propeller_radius         = propeller.tip_radius
        propeller_motor.no_load_current          = 0.001
        propeller_motor.rotor_radius             = propeller.tip_radius
        propeller_motor.design_torque            = propeller.cruise.design_torque
        propeller_motor.angular_velocity         = propeller.cruise.design_angular_velocity/propeller_motor.gear_ratio 
        propeller_motor                          = size_optimal_motor(propeller_motor)
        propeller_motor.mass_properties.mass     = nasa_motor(propeller_motor.design_torque) 
        net.motors.append(propeller_motor)  
    
    
        # --------------------------------------------------------------
        # Lift Propulsor System 
        # -------------------------------------------------------------- 
        # 1. Electronic Speed Controller  
        lift_rotor_esc                          = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
        lift_rotor_esc.efficiency               = 0.95
        for i in range(12):
            lift_rotor_ESC                      = deepcopy(lift_rotor_esc)
            lift_rotor_ESC.tag                  = 'lift_rotor_esc' + str(i + 1)  
            net.electronic_speed_controllers.append(lift_rotor_ESC) 
        
        # 2. Lift Rotors   
        rotor                                   = RCAIDE.Library.Components.Powertrain.Converters.Lift_Rotor()     
        rotor.tip_radius                        = 2.8 * Units.feet
        rotor.hub_radius                        =  0.35 * Units.feet 
        rotor.number_of_blades                  = 2     
        rotor.hover.design_altitude             = 40 * Units.feet  
        rotor.hover.design_thrust               = Hover_Load/12
        rotor.hover.design_freestream_velocity  = np.sqrt(rotor.hover.design_thrust/(2*1.2*np.pi*(rotor.tip_radius**2)))  
        rotor.oei.design_altitude               = 40 * Units.feet  
        rotor.oei.design_thrust                 = Hover_Load/10 
        rotor.oei.design_freestream_velocity    = np.sqrt(rotor.oei.design_thrust/(2*1.2*np.pi*(rotor.tip_radius**2)))  
        airfoil                                 = RCAIDE.Library.Components.Airfoils.Airfoil()   
        airfoil.coordinate_file                 = 'NACA_4412.txt'
        airfoil.polar_files                     =['NACA_4412_polar_Re_50000.txt' ,
                                                   'NACA_4412_polar_Re_100000.txt' ,
                                                   'NACA_4412_polar_Re_200000.txt' ,
                                                   'NACA_4412_polar_Re_500000.txt' ,
                                                   'NACA_4412_polar_Re_1000000.txt',
                                                   'NACA_4412_polar_Re_3500000.txt',
                                                   'NACA_4412_polar_Re_5000000.txt',
                                                   'NACA_4412_polar_Re_7500000.txt' ]
        rotor.append_airfoil(airfoil)          
        rotor.airfoil_polar_stations           = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        rotor                                  = lift_rotor_design(rotor)   
    
        lift_rotor_origins                     = [[0.543,  1.63  , 0] ,[0.543, -1.63  , 0 ] ,
                                            [3.843,  1.63  , 0] ,[3.843, -1.63  , 0 ] ,
                                            [0.543,  2.826 , 0] ,[0.543, -2.826 , 0 ] ,
                                            [3.843,  2.826 , 0] ,[3.843, -2.826 , 0 ] ,
                                            [0.543,  4.022 , 0] ,[0.543, -4.022 , 0 ] ,
                                            [3.843,  4.022 , 0] ,[3.843, -4.022 , 0 ]]
        
        # Appending rotors with different origins 
        angle_offsets                           = np.random.rand(8)*(np.pi)    
        for ii in range(12):
            lift_rotor                          = deepcopy(rotor)
            lift_rotor.tag                      = 'lift_rotor_' + str(ii+1) 
            lift_rotor.origin                   = [lift_rotor_origins[ii]]
            lift_rotor.phase_offset_angle       = angle_offsets[ii]
            net.rotors.append(lift_rotor)    
            
             
        # 3. Lift Rotor Motors   
        lift_rotor_motor                         = RCAIDE.Library.Components.Powertrain.Converters.Motor()
        lift_rotor_motor.efficiency              = 0.9
        lift_rotor_motor.nominal_voltage         = bat.pack.max_voltage*3/4  
        lift_rotor_motor.origin                  = rotor.origin 
        lift_rotor_motor.propeller_radius        = rotor.tip_radius   
        lift_rotor_motor.no_load_current         = 0.01  
        lift_rotor_motor.rotor_radius            = lift_rotor.tip_radius
        lift_rotor_motor.design_torque           = lift_rotor.hover.design_torque
        lift_rotor_motor.angular_velocity        = lift_rotor.hover.design_angular_velocity/lift_rotor_motor.gear_ratio  
        lift_rotor_motor                         = size_optimal_motor(lift_rotor_motor)
        lift_rotor_motor.mass_properties.mass    = nasa_motor(lift_rotor_motor.design_torque)    
    
        # Appending motors with different origins
        for i in range(12):
            lr_motor                              = deepcopy(lift_rotor_motor)
            lr_motor.tag                          = 'lift_rotor_motor_' + str(i+1)
            lift_rotor.origin                     = [lift_rotor_origins[ii]]
            net.motors.append(lr_motor)
            
        #------------------------------------------------------------------
        # Design Avionics
        #------------------------------------------------------------------
        avionics                                                     = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
        avionics.power_draw                                          = 20.   
        net.avionics                                                 = avionics  
    
        #------------------------------------------------------------------
        # Miscellaneous Systems 
        #------------------------------------------------------------------ 
        sys                                                          = RCAIDE.Library.Components.Powertrain.Systems.Systems()
        sys.mass_properties.mass                                     = 5       
        
           
        # append motor locations to wing 
        rotor_motor_origins                                          = np.array(lift_rotor_origins)
        propeller_motor_origins                                      = np.array(propeller_origins) 
        vehicle.wings['main_wing'].motor_spanwise_locations          = rotor_motor_origins[:,1]/vehicle.wings['main_wing'].spans.projected
        vehicle.wings['horizontal_tail'].motor_spanwise_locations    = propeller_motor_origins[:,1]/vehicle.wings['horizontal_tail'].spans.projected 
        
        # append motor origin spanwise locations onto wing data structure
        vehicle.append_component(net)
    
        settings = Data()
        converge_evtol_weight(vehicle,settings,contingency_factor = 1.0) 
        breakdown = empty(vehicle,settings,contingency_factor = 1.0 )
        print(breakdown)
        
        vehicle.weight_breakdown = breakdown
        compute_component_centers_of_gravity(vehicle)
        vehicle.center_of_gravity() 
        
        save_aircraft_geometry(vehicle,vehicle.tag)        
        
    else: 
        vehicle = load_aircraft_geometry(vehicle_name) 
    
    return vehicle 
 
# ---------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle): 

    configs                                                                        = RCAIDE.Library.Components.Configs.Config.Container()
  
    base_config                                                                    = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag                                                                = 'base'
    base_config.networks.battery_electric_rotor.blade_pitch_command                = 0
    base_config.networks.battery_electric_rotor.active_propulsor_groups            = [True,True] 
    configs.append(base_config)


    forward_config                                                                 = RCAIDE.Library.Components.Configs.Config(vehicle)
    forward_config.tag                                                             = 'forward_flight'
    forward_config.networks.battery_electric_rotor.blade_pitch_command             = 0
    forward_config.networks.battery_electric_rotor.active_propulsor_groups         = [True,False]
    configs.append(forward_config) 


    transition_config                                                              = RCAIDE.Library.Components.Configs.Config(vehicle)
    transition_config.tag                                                          = 'transition_flight'
    transition_config.networks.battery_electric_rotor.blade_pitch_command          = 0
    transition_config.networks.battery_electric_rotor.active_propulsor_groups      = [True,True]
    configs.append(transition_config)
    

    vertical_config                                                                = RCAIDE.Library.Components.Configs.Config(vehicle)
    vertical_config.tag                                                            = 'vertical_flight' 
    vertical_config.networks.battery_electric_rotor.blade_pitch_command            = 0
    vertical_config.networks.battery_electric_rotor.active_propulsor_groups        = [False,True]
    configs.append(vertical_config)  


    descent_config                                                                 = RCAIDE.Library.Components.Configs.Config(vehicle)
    descent_config.tag                                                             = 'descent'
    descent_config.networks.battery_electric_rotor.blade_pitch_command             = -5 * Units.degrees
    descent_config.networks.battery_electric_rotor.active_propulsor_groups         = [True,False]
    configs.append(descent_config)  
    
    
    # done!
    return configs

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

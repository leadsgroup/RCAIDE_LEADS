# Airbus A220-100
# 
# 
# Created:  Mar 2024, S. Shekar 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core                                                 import Units
from RCAIDE.Library.Plots                                                  import *     
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan                 import design_turbofan  


# Python imports 
import numpy                                               as np
import matplotlib.pyplot                                   as plt
from copy                                                  import deepcopy 
import os

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    
    # Step 1 design a vehicle
    vehicle  = vehicle_setup()    
    vehicle.mass_properties.takeoff = None
    
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
    # plot_3d_vehicle(vehicle, side_view = True )        
    
    return

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
    # base_config.landing_gear.gear_condition                      = 'up'
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
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg 
    # config.networks.fuel.propulsors['starboard_propulsor'].fan.angular_velocity =  3470. * Units.rpm
    # config.networks.fuel.propulsors['port_propulsor'].fan.angular_velocity      =  3470. * Units.rpm
    # config.landing_gear.gear_condition                          = 'up'     
    configs.append(config)

    
    # ------------------------------------------------------------------
    #   Cutback Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'cutback'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 20. * Units.deg
    # config.networks.fuel.propulsors['starboard_propulsor'].fan.angular_velocity =  2780. * Units.rpm
    # config.networks.fuel.propulsors['port_propulsor'].fan.angular_velocity      =  2780. * Units.rpm
    # config.landing_gear.gear_condition                          = 'up'       
    configs.append(config)   
    
        
    
    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'landing'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 30. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg
    # config.networks.fuel.propulsors['starboard_propulsor'].fan.angular_velocity =  2030. * Units.rpm
    # config.networks.fuel.propulsors['port_propulsor'].fan.angular_velocity      =  2030. * Units.rpm
    # config.landing_gear.gear_condition                          = 'down'    
    configs.append(config)   
     
    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------  

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'reverse_thrust'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 30. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg 
    # config.landing_gear.gear_condition                          = 'down'    
    configs.append(config)    


    return configs

def vehicle_setup(): 

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    

    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'Airbus_220-100'   

    # ################################################# Vehicle-level Properties ########################################################  

    # mass properties
    vehicle.mass_properties.max_takeoff   = 63100  # kg 
    vehicle.mass_properties.takeoff       = 63100  # kg 
    vehicle.mass_properties.max_zero_fuel = 52200  # kg 
    vehicle.mass_properties.max_payload   = 17230  # kg
    vehicle.mass_properties.max_fuel      = 17600  # kg
    vehicle.mass_properties.min_payload   = 0  # kg
    vehicle.flight_envelope.ultimate_load        = 3.75
    vehicle.flight_envelope.positive_limit_load           = 1.5
    vehicle.flight_envelope.design_mach_number = 0.78
    vehicle.flight_envelope.design_cruise_altitude = 35000 * Units.feet
    vehicle.flight_envelope.design_range  = 3600 * Units.nmi
    vehicle.reference_area                = 112.3* Units['meters**2']
    vehicle.number_of_passengers                    = 135
    vehicle.systems.control               = "fully powered"
    vehicle.systems.accessories           = "medium range"   
    
    cruise_speed                          = 470 * Units.kts
    altitude                              = 30000 * Units.feet
    atmo                                  = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    freestream                            = atmo.compute_values (0.)
    freestream0                           = atmo.compute_values (altitude)
    mach_number                           = (cruise_speed/freestream.speed_of_sound)[0][0] 
    vehicle.design_dynamic_pressure       = ( .5 *freestream0.density*(cruise_speed*cruise_speed))[0][0]
    vehicle.design_mach_number            =  mach_number

   

    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------

    wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing'
    wing.aspect_ratio                     = 9.24167
    wing.sweeps.quarter_chord             = 25 * Units.deg
    wing.thickness_to_chord               = 0.12
    wing.spans.projected                  = 33.70371 
    wing.chords.root                      = 7.3 * Units.meter
    wing.chords.tip                       = 0.4 * Units.meter
    wing.taper                            = wing.chords.tip / wing.chords.root
    wing.chords.mean_aerodynamic          = 4.88* Units.meter 
    wing.areas.reference                  = 122.915
    wing.areas.wetted                     = 390#? 
    wing.twists.root                      = 4.0 * Units.degrees 
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[ 10.543,0,  -0.652]]
    wing.aerodynamic_center               = [0,0,0] 
    wing.vertical                         = False
    wing.dihedral                         = 7.0 * Units.degrees 
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
    segment.twist                         = 0.0 * Units.degrees
    segment.root_chord_percent            = 1.
    segment.thickness_to_chord            = 0.1
    segment.dihedral_outboard             = 7.82609 * Units.degrees
    segment.sweeps.quarter_chord          = 24.48701 * Units.degrees
    segment.thickness_to_chord            = .1
    segment.append_airfoil(root_airfoil)
    wing.append_segment(segment) 

    mid_airfoil                           = RCAIDE.Library.Components.Airfoils.Airfoil()
    mid_airfoil.coordinate_file           = rel_path + 'Airfoils' + separator + 'transonic_wing_inboard_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Section_2'
    segment.percent_span_location         = 0.368 
    segment.twist                         = 0.00258 * Units.deg
    segment.root_chord_percent            = 0.5136
    segment.thickness_to_chord            = 0.1
    segment.dihedral_outboard             = 6.41304 * Units.degrees
    segment.sweeps.quarter_chord          = 26.54545 * Units.degrees
    segment.thickness_to_chord            = .1
    segment.append_airfoil(mid_airfoil)
    wing.append_segment(segment)

    tip_airfoil                           =  RCAIDE.Library.Components.Airfoils.Airfoil()
    tip_airfoil.coordinate_file           = rel_path + 'Airfoils' + separator + 'transonic_wing_outboard_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Tip'
    segment.percent_span_location         = 0.96
    segment.twist                         = 5 * Units.degrees
    segment.root_chord_percent            = 0.1986
    segment.thickness_to_chord            = 0.1
    segment.dihedral_outboard             = 53.55* Units.degrees
    segment.sweeps.quarter_chord          = 48.0
    segment.thickness_to_chord            = .1 
    segment.append_airfoil(tip_airfoil)
    wing.append_segment(segment)
    
    tip_airfoil                           =  RCAIDE.Library.Components.Airfoils.Airfoil()
    tip_airfoil.coordinate_file           = rel_path + 'Airfoils' + separator + 'transonic_wing_tip_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Tip_2'
    segment.percent_span_location         = 1.0
    segment.twist                         = 5 * Units.degrees
    segment.root_chord_percent            = 0.1066
    segment.thickness_to_chord            = 0.1
    segment.dihedral_outboard             = 53.55* Units.degrees
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = .1
    segment.append_airfoil(tip_airfoil)
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
    flap.chord_fraction           = 0.30
    wing.append_control_surface(flap)

    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.7
    aileron.span_fraction_end     = 0.963
    aileron.deflection            = 0.0 * Units.degrees
    aileron.chord_fraction        = 0.16
    wing.append_control_surface(aileron) 

   
    
    # add to vehicle
    vehicle.append_component(wing)





    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing     = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag = 'horizontal_stabilizer'

    wing.aspect_ratio            = 2.55111
    wing.sweeps.quarter_chord    = 30
    wing.thickness_to_chord      = 0.08
    wing.taper                   = 0.355 
    wing.spans.projected         = 12.30000
    wing.chords.root             = 3.55556
    wing.chords.tip              = 1.26587
    wing.chords.mean_aerodynamic = 2.27
    wing.areas.reference         = 16.47
    wing.areas.exposed           = 25.8 #?????
    wing.areas.wetted            = 29.65179#?
    wing.twists.root             = 3.0 * Units.degrees
    wing.twists.tip              = 3.0 * Units.degrees
    wing.origin                  = [[ 28.852,0,  1.230]]
    wing.aerodynamic_center      = [0,0,0]
    wing.vertical                = False
    wing.xz_plane_symmetric      = True
    wing.dynamic_pressure_ratio  = 0.9



    # Wing Segments
    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'root_segment'
    segment.percent_span_location  = 0.0
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 1.0
    segment.dihedral_outboard      = 6.35714 * Units.degrees
    segment.sweeps.quarter_chord   = 30.00000 * Units.degrees 
    segment.thickness_to_chord     = .1
    wing.append_segment(segment)

    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'tip_segment'
    segment.percent_span_location  = 1.
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 0.355             
    segment.dihedral_outboard      = 0 * Units.degrees
    segment.sweeps.quarter_chord   = 0 * Units.degrees  
    segment.thickness_to_chord     = .1
    wing.append_segment(segment)
    
        

    # control surfaces -------------------------------------------
    elevator                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
    elevator.tag                   = 'elevator'
    elevator.span_fraction_start   = 0.09
    elevator.span_fraction_end     = 0.92
    elevator.deflection            = 0.0  * Units.deg
    elevator.chord_fraction        = 0.3
    wing.append_control_surface(elevator)

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag = 'vertical_stabilizer'

    wing.aspect_ratio            = 3.60335
    wing.sweeps.quarter_chord    = 38.92857
    wing.thickness_to_chord      = 0.08
    wing.taper                   = 0.25

    wing.spans.projected         = 7.0709
    wing.total_length            = 7.0787

    wing.chords.root             = 6.27778
    wing.chords.tip              = 1.5714
    wing.chords.mean_aerodynamic = 4.0

    wing.areas.reference         = 30.83
    wing.areas.wetted            = 55.5

    wing.twists.root             = 3.0 * Units.degrees
    wing.twists.tip              = 3.0 * Units.degrees

    wing.origin                  = [[25.328,0, 1.311]]
    wing.aerodynamic_center      = [0,0,0]

    wing.vertical                = True
    wing.xz_plane_symmetric      = False
    wing.t_tail                  = False

    wing.dynamic_pressure_ratio  = 1.0


    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root'
    segment.percent_span_location         = 0.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 1.
    segment.dihedral_outboard             = 0 * Units.degrees
    segment.sweeps.quarter_chord          = 38.92857 * Units.degrees  
    segment.thickness_to_chord            = .1
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_1'
    segment.percent_span_location         = 1.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.25
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.quarter_chord          = 0.0    
    segment.thickness_to_chord            = .1  
    wing.append_segment(segment)
    
 
    # control surfaces -------------------------------------------
    rudder                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    rudder.tag                   = 'rudder'
    rudder.span_fraction_start   = 0.1 
    rudder.span_fraction_end     = 0.95 
    rudder.deflection            = 0 
    rudder.chord_fraction        = 0.33  
    wing.append_control_surface(rudder)    
    
    
        


    # add to vehicle
    vehicle.append_component(wing)

    # ##########################################################   Fuselage  ############################################################    
    fuselage = RCAIDE.Library.Components.Fuselages.Fuselage() 

    cabin                                             = RCAIDE.Library.Components.Fuselages.Cabins.Cabin() 
    economy_class                                     = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest              = 5
    economy_class.number_of_rows                      = 27
    economy_class.galley_lavatory_percent_x_locations = [0, 1]      
    economy_class.emergency_exit_percent_x_locations  = [0.5, 0.5]      
    economy_class.type_A_exit_percent_x_locations     = [0, 1]     
    cabin.append_cabin_class(economy_class)
    fuselage.append_cabin(cabin) 

    fuselage.seats_abreast                      = 5
    fuselage.fineness.nose                      = 0.58
    fuselage.fineness.tail                      = 1.75
    fuselage.lengths.nose                       = 0.921 
    fuselage.lengths.tail                       = 4.181
    fuselage.lengths.cabin                      = 29.798 #m
    fuselage.lengths.total                      = 34.9
    fuselage.width                              = 3.95 
    fuselage.heights.maximum                    = 4.19   
    fuselage.heights.at_quarter_length          = 3.71  
    fuselage.heights.at_three_quarters_length   = 3.83
    fuselage.heights.at_wing_root_quarter_chord = 4.19 
    fuselage.areas.side_projected               = fuselage.lengths.total *fuselage.heights.maximum  # estimate    
    fuselage.areas.wetted                       = 2 * np.pi * fuselage.width *  fuselage.lengths.total +  2 * np.pi * fuselage.width ** 2
    fuselage.areas.front_projected              =  np.pi * fuselage.width ** 2 
    fuselage.effective_diameter                 = 3.6 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_0'
    segment.percent_x_location                  = 0
    segment.percent_z_location                  = 0
    segment.height                              = 0
    segment.width                               = 0
    fuselage.append_segment(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_1'
    segment.percent_x_location                  = 0.01
    segment.percent_z_location                  = 0
    segment.height                              = 0.90000
    segment.width                               = 0.68182
    fuselage.append_segment(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'
    segment.percent_x_location                  = 0.02000
    segment.percent_z_location                  = 0.00100	 
    segment.height                              = 1.30000
    segment.width                               = 1.25000
    fuselage.append_segment(segment) 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'
    segment.percent_x_location                  = 0.03000
    segment.percent_z_location                  = 0.00300 
    segment.height                              = 1.65000
    segment.width                               = 1.64773
    fuselage.append_segment(segment)  

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'
    segment.percent_x_location                  = 0.04000 
    segment.percent_z_location                  = 0.00500	 
    segment.height                              = 1.96000
    segment.width                               = 1.93182
    fuselage.append_segment(segment) 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'
    segment.percent_x_location                  = 0.06000
    segment.percent_z_location                  = 0.01080 
    segment.height                              = 2.60000
    segment.width                               = 2.50000
    fuselage.append_segment(segment) 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'
    segment.percent_x_location                  = 0.07500 
    segment.percent_z_location                  = 0.01350	 
    segment.height                              = 2.92000
    segment.width                               = 2.67045 
    fuselage.append_segment(segment) 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'
    segment.percent_x_location                  = 0.10000
    segment.percent_z_location                  = 0.01750
    segment.height                              = 3.30000  
    segment.width                               = 3.01136
    fuselage.append_segment(segment)
  

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_8'
    segment.percent_x_location                  = 0.16000
    segment.percent_z_location                  = 0.02174	 	 
    segment.height                              = 3.70000	 
    segment.width                               = 3.50000
    fuselage.append_segment(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_9'
    segment.percent_x_location                  = 0.67000
    segment.percent_z_location                  = 0.02170
    segment.height                              = 3.70000
    segment.width                               = 3.50000
    fuselage.append_segment(segment)
  

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_10'
    segment.percent_x_location                  = 0.73000
    segment.percent_z_location                  = 0.02600	 	 
    segment.height                              = 3.30000
    segment.width                               = 3.50000
    fuselage.append_segment(segment)
    

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_11'
    segment.percent_x_location                  = 0.89582
    segment.percent_z_location                  = 0.04200	 	 
    segment.height                              = 1.95000 
    segment.width                               = 2.10000
    fuselage.append_segment(segment)
    

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_12'
    segment.percent_x_location                  = 0.93715
    segment.percent_z_location                  = 0.04348 
    segment.height                              = 1.54000	 
    segment.width                               = 1.80000 
    fuselage.append_segment(segment)
    
    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_13'
    segment.percent_x_location                  = 0.98463
    segment.percent_z_location                  = 0.04800
    segment.height                              = 0.85000	 
    segment.width                               = 0.80000 
    fuselage.append_segment(segment)
    
    
    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_14'
    segment.percent_x_location                  = 1.0
    segment.percent_z_location                  = 0.04800
    segment.height                              = 0.0	 
    segment.width                               = 0.0000 
    fuselage.append_segment(segment)    

    # add to vehicle
    vehicle.append_component(fuselage) 

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Network Initialization
    #------------------------------------------------------------------------------------------------------------------------------------  
    
    network                                 = RCAIDE.Framework.Networks.Network()   

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Systems definition
    #------------------------------------------------------------------------------------------------------------------------------------  

    # Hydraulic System
             
    hydraulics                              = RCAIDE.Library.Components.Powertrain.Systems.Hydraulic_System()
    hydraulics.tag                          = 'hydraulics'
    hydraulics.power_draw                   = 10000. # Watts
                 
    # Avionic System

    avionics                                = RCAIDE.Library.Components.Powertrain.Systems.Avionics_System()
    avionics.tag                            = 'avionics'
    avionics.power_draw                     = 10000. # Watts
    # Environmental Control System
             
    ecs                                     = RCAIDE.Library.Components.Powertrain.Systems.Environmental_Control_System()
    ecs.tag                                 = 'ecs'
    ecs.power_draw                          = 100000 # Watts

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Modulators definition
    #------------------------------------------------------------------------------------------------------------------------------------  
    
    # Transformer Rectifier Unit 1

    tru_1                                   = RCAIDE.Library.Components.Powertrain.Modulators.Transformer_Rectifier_Unit()
    tru_1.tag                               = 'tru_1'

    # Transformer Rectifier Unit 2
 
    tru_2                                   = RCAIDE.Library.Components.Powertrain.Modulators.Transformer_Rectifier_Unit()
    tru_2.tag                               = 'tru_2'

    # Transformer Rectifier Unit 3
 
    tru_3                                   = RCAIDE.Library.Components.Powertrain.Modulators.Transformer_Rectifier_Unit()
    tru_3.tag                               = 'tru_3'

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Converters definition
    #------------------------------------------------------------------------------------------------------------------------------------  
    
    # Generator 1

    generator_1                                       = RCAIDE.Library.Components.Powertrain.Converters.Generator()
    generator_1.tag                                   = 'generator_1'
    generator_1.generator_type                        = 'AC'
    generator_1.efficiency                            = 0.95
    generator_1.rated_power                           = 90. * 10**3 # Watts
    generator_1.number_of_turns                       = 80
    generator_1.stator_outer_diameter                 = 0.35  
    generator_1.stator_inner_diameter                 = 0.16  
    generator_1.mu_0                                  = 4 * np.pi * 1e-7
    generator_1.mu_r                                  = 4 * np.pi * 1e-7

    # Fan 1

    fan_1                                             = RCAIDE.Library.Components.Powertrain.Converters.Fan()   
    fan_1.tag                                         = 'starboard_fan'
    fan_1.polytropic_efficiency                       = 0.93
    fan_1.pressure_ratio                              = 1.7   

    # Ram 1
    
    ram_1                                             = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram_1.tag                                         = 'starboard_ram'  

    # Inlet Nozzle 1
                             
    inlet_nozzle_1                                    = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle_1.tag                                = 'starboard_inlet_nozzle'
    inlet_nozzle_1.polytropic_efficiency              = 0.98
    inlet_nozzle_1.pressure_ratio                     = 0.98 

    # Low Pressure Compressor 1
    
    low_pressure_compressor_1                        = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    low_pressure_compressor_1.tag                    = 'starboard_lpc'
    low_pressure_compressor_1.polytropic_efficiency  = 0.91
    low_pressure_compressor_1.pressure_ratio         = 1.9

    # High Pressure Compressor 1

    high_pressure_compressor_1                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    high_pressure_compressor_1.tag                   = 'starboard_hpc'
    high_pressure_compressor_1.polytropic_efficiency = 0.91
    high_pressure_compressor_1.pressure_ratio        = 10.0 

    # Combustor 1

    combustor_1                                      = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor_1.tag                                  = 'starboard_combustor'
    combustor_1.efficiency                           = 0.99 
    combustor_1.alphac                               = 1.0     
    combustor_1.turbine_inlet_temperature            = 1550
    combustor_1.pressure_ratio                       = 0.95
    combustor_1.fuel_data                            = RCAIDE.Library.Attributes.Propellants.Jet_A()  

    # High Pressure Turbine 2
   
    high_pressure_turbine_1                          = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    high_pressure_turbine_1.tag                      ='starboard_hpt'
    high_pressure_turbine_1.mechanical_efficiency    = 0.99
    high_pressure_turbine_1.polytropic_efficiency    = 0.93

    # Low Pressure Turbine 1

    low_pressure_turbine_1                           = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    low_pressure_turbine_1.tag                       ='starboard_lpt'
    low_pressure_turbine_1.mechanical_efficiency     = 0.99
    low_pressure_turbine_1.polytropic_efficiency     = 0.93  

    # Core Nozzle 1
           
    core_nozzle_1                                    = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle_1.tag                                = 'starboard_core_nozzle'
    core_nozzle_1.polytropic_efficiency              = 0.95
    core_nozzle_1.pressure_ratio                     = 0.99  

    # Fan Nozzle 1
              
    fan_nozzle_1                                     = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    fan_nozzle_1.tag                                 = 'starboard_fan_nozzle'
    fan_nozzle_1.polytropic_efficiency               = 0.95
    fan_nozzle_1.pressure_ratio                      = 0.99 

    # # Generator 2

    generator_2                                      = RCAIDE.Library.Components.Powertrain.Converters.Generator()
    generator_2.tag                                  = 'generator_2'
    generator_2.generator_type                       = 'AC'
    generator_2.efficiency                           = 0.95
    generator_2.rated_power                          = 90. * 10**3 # Watts
    generator_2.number_of_turns                      = 80
    generator_2.stator_outer_diameter                = 0.35  
    generator_2.stator_inner_diameter                = 0.16  
    generator_2.mu_0                                 = 4 * np.pi * 1e-7
    generator_2.mu_r                                 = 4 * np.pi * 1e-7

    # Fan 2

    fan_2                                            = RCAIDE.Library.Components.Powertrain.Converters.Fan()   
    fan_2.tag                                        = 'port_fan'
    fan_2.polytropic_efficiency                      = 0.93
    fan_2.pressure_ratio                             = 1.7   

    # Ram 2
                                       
    ram_2                                            = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram_2.tag                                        = 'port_ram'  

    # Inlet Nozzle 2
                             
    inlet_nozzle_2                                   = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle_2.tag                               = 'port_inlet_nozzle'
    inlet_nozzle_2.polytropic_efficiency             = 0.98
    inlet_nozzle_2.pressure_ratio                    = 0.98  

    # Low Pressure Compressor 2
   
    low_pressure_compressor_2                        = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    low_pressure_compressor_2.tag                    = 'port_lpc'
    low_pressure_compressor_2.polytropic_efficiency  = 0.91
    low_pressure_compressor_2.pressure_ratio         = 1.9   

    # High Pressure Compressor 2

    high_pressure_compressor_2                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    high_pressure_compressor_2.tag                   = 'port_hpc'
    high_pressure_compressor_2.polytropic_efficiency = 0.91
    high_pressure_compressor_2.pressure_ratio        = 10.0

    # Combustor 2

    combustor_2                                      = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor_2.tag                                  = 'port_combustor'
    combustor_2.efficiency                           = 0.99 
    combustor_2.alphac                               = 1.0     
    combustor_2.turbine_inlet_temperature            = 1550
    combustor_2.pressure_ratio                       = 0.95
    combustor_2.fuel_data                            = RCAIDE.Library.Attributes.Propellants.Jet_A()  

    # High Pressure Turbine 2
   
    high_pressure_turbine_2                          = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    high_pressure_turbine_2.tag                      ='hpt'
    high_pressure_turbine_2.mechanical_efficiency    = 0.99
    high_pressure_turbine_2.polytropic_efficiency    = 0.93      

    # Low Pressure Turbine 2

    low_pressure_turbine_2                           = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    low_pressure_turbine_2.tag                       ='port_lpt'
    low_pressure_turbine_2.mechanical_efficiency     = 0.99
    low_pressure_turbine_2.polytropic_efficiency     = 0.93 

    # Core Nozzle 2      
           
    core_nozzle_2                                    = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle_2.tag                                = 'port_core_nozzle'
    core_nozzle_2.polytropic_efficiency              = 0.95
    core_nozzle_2.pressure_ratio                     = 0.99     
    
    # Fan Nozzle 2
              
    fan_nozzle_2                                     = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    fan_nozzle_2.tag                                 = 'port_fan_nozzle'
    fan_nozzle_2.polytropic_efficiency               = 0.95
    fan_nozzle_2.pressure_ratio                      = 0.99 

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Propulsors definition
    #------------------------------------------------------------------------------------------------------------------------------------  
     
    # Turbofan Engine 1

    turbofan_1                                        = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan() 
    turbofan_1.tag                                    = 'starboard_propulsor'  
    turbofan_1.identical_propulsors                   = True 
    turbofan_1.bypass_ratio                           = 12.5  
    turbofan_1.length                                 = 3.175
    turbofan_1.design_altitude                        = 25000 * Units.ft
    turbofan_1.design_mach_number                     = 0.78   
    turbofan_1.design_thrust                          = 18300 *  Units.N#/2 
    turbofan_1.origin                                 = [[ 10.150,  5.435, -1.087]] 
    turbofan_1.working_fluid                          = RCAIDE.Library.Attributes.Gases.Air() 

    # Turbofan Engine 2

    turbofan_2                                        = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan() 
    turbofan_2.tag                                    = 'port_propulsor'  
    turbofan_2.identical_propulsors                   = True 
    turbofan_1.bypass_ratio                           = 12.5  
    turbofan_1.length                                 = 3.175
    turbofan_1.design_altitude                        = 25000 * Units.ft
    turbofan_1.design_mach_number                     = 0.78   
    turbofan_1.design_thrust                          = 18300 *  Units.N#/2 
    turbofan_1.origin                                 = [[ 10.150,  -5.435, -1.087]] 
    turbofan_2.working_fluid                          = RCAIDE.Library.Attributes.Gases.Air()  

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Assign components to propulsors
    #------------------------------------------------------------------------------------------------------------------------------------  
    
    turbofan_1.assigned_converters.generator_tag                     = [[generator_1.tag]]
    turbofan_1.assigned_converters.fan_tag                           = [[fan_1.tag]]
    turbofan_1.assigned_converters.ram_tag                           = [[ram_1.tag]]
    turbofan_1.assigned_converters.inlet_nozzle_tag                  = [[inlet_nozzle_1.tag]]
    turbofan_1.assigned_converters.low_pressure_compressor_tag       = [[low_pressure_compressor_1.tag]]
    turbofan_1.assigned_converters.high_pressure_compressor_tag      = [[high_pressure_compressor_1.tag]]
    turbofan_1.assigned_converters.combustor_tag                     = [[combustor_1.tag]]
    turbofan_1.assigned_converters.high_pressure_turbine_tag         = [[high_pressure_turbine_1.tag]]
    turbofan_1.assigned_converters.low_pressure_turbine_tag          = [[low_pressure_turbine_1.tag]]
    turbofan_1.assigned_converters.core_nozzle_tag                   = [[core_nozzle_1.tag]]
    turbofan_1.assigned_converters.fan_nozzle_tag                    = [[fan_nozzle_1.tag]]

    turbofan_2.assigned_converters.generator_tag                     = [[generator_2.tag]]
    turbofan_2.assigned_converters.fan_tag                           = [[fan_2.tag]]
    turbofan_2.assigned_converters.ram_tag                           = [[ram_2.tag]]
    turbofan_2.assigned_converters.inlet_nozzle_tag                  = [[inlet_nozzle_2.tag]]
    turbofan_2.assigned_converters.low_pressure_compressor_tag       = [[low_pressure_compressor_2.tag]]
    turbofan_2.assigned_converters.high_pressure_compressor_tag      = [[high_pressure_compressor_2.tag]]
    turbofan_2.assigned_converters.combustor_tag                     = [[combustor_2.tag]]
    turbofan_2.assigned_converters.high_pressure_turbine_tag         = [[high_pressure_turbine_2.tag]]
    turbofan_2.assigned_converters.low_pressure_turbine_tag          = [[low_pressure_turbine_2.tag]]
    turbofan_2.assigned_converters.core_nozzle_tag                   = [[core_nozzle_2.tag]]
    turbofan_2.assigned_converters.fan_nozzle_tag                    = [[fan_nozzle_2.tag]]
    
    # Nacelles ---------------------------------------------------

    # Nacelle 1
     
    nacelle_1                                   = RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle()
    nacelle_1.diameter                          = 1.918
    nacelle_1.length                            = 3.258
    nacelle_1.tag                               = 'starboard_propulsor_nacelle'
    nacelle_1.inlet_diameter                    = 1.50
    nacelle_1.origin                            = [[ 10.150, 5.435, -1.087]] 
    nacelle_1.areas.wetted                      = 1.1*np.pi*nacelle_1.diameter*nacelle_1.length
    nacelle_airfoil                             = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    nacelle_airfoil.NACA_4_Series_code          = '2410'
    nacelle_1.append_airfoil(nacelle_airfoil)
    turbofan_1.nacelle = nacelle_1

    # Nacelle 2
     
    nacelle_2                                   = RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle()
    nacelle_2.diameter                          = 1.918
    nacelle_2.length                            = 3.258
    nacelle_2.tag                               = 'port_propulsor_nacelle'
    nacelle_2.inlet_diameter                    = 1.50
    nacelle_2.origin                            = [[ 10.150, -5.435, -1.087]] 
    nacelle_2.areas.wetted                      = 1.1*np.pi*nacelle_2.diameter*nacelle_2.length
    nacelle_airfoil                             = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    nacelle_airfoil.NACA_4_Series_code          = '2410'
    nacelle_2.append_airfoil(nacelle_airfoil)
    turbofan_2.nacelle = nacelle_2   

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Sources definition
    #------------------------------------------------------------------------------------------------------------------------------------  
    
    # Battery Module 1

    bat_module_1                                             = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC()
    bat_module_1.electrical_configuration.series             = 20 
    bat_module_1.electrical_configuration.parallel           = 210 *  8 
    bat_module_1.cell.nominal_capacity                       = 3.8 
    bat_module_1.geometrtic_configuration.normal_count       = 42 

    # Battery Module 2

    bat_module_2                                             = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC()
    bat_module_2.electrical_configuration.series             = 20 
    bat_module_2.electrical_configuration.parallel           = 210 *  8 
    bat_module_2.cell.nominal_capacity                       = 3.8 
    bat_module_2.geometrtic_configuration.normal_count       = 42 
    bat_module_2.geometrtic_configuration.parallel_count     = 100 *  8

    # Fuel Tank
    inboard_tank                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    inboard_tank.fuel                         = RCAIDE.Library.Attributes.Propellants.Jet_A()
    inboard_tank.segments_bounding_tank       = ['root','yehudi']  
    inboard_tank.segments_percent_chord_start = [0.15  ,0.15 ]
    inboard_tank.segments_percent_chord_end   = [0.625 ,0.625]  
    fuel_line.fuel_tanks.append(inboard_tank)
    
    outboard_tank                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    outboard_tank.fuel                         = RCAIDE.Library.Attributes.Propellants.Jet_A()
    outboard_tank.segments_bounding_tank       = ['yehudi', 'section_2']  
    outboard_tank.segments_percent_chord_start = [0.15 ,0.15 ]
    outboard_tank.segments_percent_chord_end   = [0.625,0.625]
     
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Distributors definition
    #------------------------------------------------------------------------------------------------------------------------------------  
    
    # AC Bus 1

    ac_bus_1                                    = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    ac_bus_1.tag                                = 'ac_bus_1'
    ac_bus_1.type                               = 'AC'
    ac_bus_1.voltage_phase_to_neutral           = 115.0 
    ac_bus_1.voltage_phase_to_phase             = 200.0
    ac_bus_1.frequency                          = 400. 
    ac_bus_1.efficiency                         = 0.95 
     
    # AC Bus 2     
     
    ac_bus_2                                    = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    ac_bus_2.tag                                = 'ac_bus_2'
    ac_bus_2.type                               = 'AC'
    ac_bus_2.voltage_phase_to_neutral           = 115.0 
    ac_bus_2.voltage_phase_to_phase             = 200.0
    ac_bus_2.frequency                          = 400. 
    ac_bus_2.efficiency                         = 0.95   
     
    # DC Bus 1     
     
    dc_bus_1                                    = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    dc_bus_1.tag                                = 'dc_bus_1'
    dc_bus_1.type                               = 'DC'
    dc_bus_1.voltage                            = 28
    dc_bus_1.frequency                          = 0
    dc_bus_1.efficiency                         = 0.95  
     
    # DC Bus 2     
     
    dc_bus_2                                    = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    dc_bus_2.tag                                = 'dc_bus_2'
    dc_bus_2.type                               = 'DC'
    dc_bus_2.voltage                            = 28
    dc_bus_2.frequency                          = 0
    dc_bus_2.efficiency                         = 0.95 
     
    # DC Ess Bus 1     
     
    dc_ess_1                                    = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    dc_ess_1.tag                                = 'dc_ess_1'
    dc_ess_1.type                               = 'DC'
    dc_ess_1.voltage                            = 28
    dc_ess_1.frequency                          = 0
    dc_ess_1.efficiency                         = 0.95  
     
    # DC Ess Bus 2     
         
    dc_ess_2                                    = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    dc_ess_2.tag                                = 'dc_ess_2'
    dc_ess_2.type                               = 'DC'
    dc_ess_2.voltage                            = 28
    dc_ess_2.frequency                          = 0
    dc_ess_2.efficiency                         = 0.95  
     
    # DC Ess Bus 3     
         
    dc_ess_3                                    = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    dc_ess_3.tag                                = 'dc_ess_3'
    dc_ess_3.type                               = 'DC'
    dc_ess_3.voltage                            = 28
    dc_ess_3.frequency                          = 0
    dc_ess_3.efficiency                         = 0.95 

    # AC Ess Bus
    
    ac_ess_bus                                  = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    ac_ess_bus.tag                              = 'ac_ess'
    ac_ess_bus.type                             = 'AC'
    ac_ess_bus.voltage_phase_to_neutral         = 115.0 
    ac_ess_bus.voltage_phase_to_phase           = 200.0
    ac_ess_bus.frequency                        = 400. 
    ac_ess_bus.efficiency                       = 0. 

    # AC Stby Bus
    
    ac_stby_bus                                 = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    ac_stby_bus.tag                             = 'ac_stby'
    ac_stby_bus.type                            = 'AC'
    ac_stby_bus.voltage_phase_to_neutral        = 115.0 
    ac_stby_bus.voltage_phase_to_phase          = 200.0
    ac_stby_bus.frequency                       = 400. 
    ac_stby_bus.efficiency                      = 0.95

    # Fuel Line 
    
    fuel_line                                   = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line() 
    fuel_line.tag                               = 'fuel_line'

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Assign distributors to components
    #------------------------------------------------------------------------------------------------------------------------------------  
    
    # Systems

    hydraulics.assigned_distributors              = [[dc_bus_1.tag]]
    avionics.assigned_distributors                = [[dc_bus_1.tag]]
    ecs.assigned_distributors                     = [[dc_bus_1.tag]]

    # Modulators

    tru_1.assigned_distributors                   = [[ac_bus_1.tag,
                                                      dc_bus_1.tag]]
    tru_2.assigned_distributors                   = [[ac_bus_2.tag,
                                                      dc_bus_2.tag]]
    tru_3.assigned_distributors                   = [[ac_ess_bus.tag,
                                                      dc_ess_3.tag]]
    # Converters

    # Propulsors

    turbofan_1.assigned_distributors              = [[fuel_line.tag,
                                                      ac_bus_1.tag]]
    turbofan_2.assigned_distributors              = [[fuel_line.tag,
                                                      ac_bus_2.tag]]

    # Sources

    outboard_tank.assigned_distributors           = [[fuel_line.tag]]
    inboard_tank.assigned_distributors            = [[fuel_line.tag]]
    bat_module_1.assigned_distributors            = [[dc_ess_1.tag]]
    bat_module_2.assigned_distributors            = [[dc_ess_2.tag]]

    for _ in range(12):
        bat_copy_1 = deepcopy(bat_module_1)
        dc_ess_1.battery_modules.append(bat_copy_1)
    dc_ess_1.battery_module_electric_configuration = 'Series' 
    dc_ess_1.initialize_bus_properties()

    for _ in range(12):
        bat_copy_2 = deepcopy(bat_module_2)
        dc_ess_2.battery_modules.append(bat_copy_2)
    dc_ess_2.battery_module_electric_configuration = 'Series' 
    dc_ess_2.initialize_bus_properties()

    # Distributors
    
    ac_bus_2.assigned_distributors                = [[ac_ess_bus.tag]]
    dc_bus_1.assigned_distributors                = [[dc_ess_1.tag]]
    dc_bus_2.assigned_distributors                = [[dc_ess_2.tag]] 
    dc_ess_1.assigned_distributors                = [[dc_bus_1.tag]] 
    dc_ess_2.assigned_distributors                = [[dc_bus_2.tag]] 
    ac_ess_bus.assigned_distributors              = [[ac_bus_2.tag, 
                                                      ac_stby_bus.tag]]
    ac_stby_bus.assigned_distributors             = [[ac_ess_bus.tag]]

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Append systems to the network
    #------------------------------------------------------------------------------------------------------------------------------------  
     
    network.systems.append(hydraulics)
    network.systems.append(avionics)
    network.systems.append(ecs)    
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Append modulators to the network
    #------------------------------------------------------------------------------------------------------------------------------------  
    
    network.modulators.append(tru_1)
    network.modulators.append(tru_2) 
    network.modulators.append(tru_3)
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Append converters to the network
    #------------------------------------------------------------------------------------------------------------------------------------  
    
    network.converters.append(generator_1)
    network.converters.append(fan_1)
    network.converters.append(ram_1)  
    network.converters.append(inlet_nozzle_1)         
    network.converters.append(low_pressure_compressor_1)    
    network.converters.append(high_pressure_compressor_1) 
    network.converters.append(combustor_1)
    network.converters.append(high_pressure_turbine_1) 
    network.converters.append(low_pressure_turbine_1) 
    network.converters.append(core_nozzle_1)
    network.converters.append(fan_nozzle_1)
    network.converters.append(generator_2)
    network.converters.append(fan_2)
    network.converters.append(ram_2) 
    network.converters.append(inlet_nozzle_2)     
    network.converters.append(low_pressure_compressor_2)
    network.converters.append(high_pressure_compressor_2)
    network.converters.append(combustor_2)
    network.converters.append(high_pressure_turbine_2)
    network.converters.append(low_pressure_turbine_2)
    network.converters.append(core_nozzle_2)
    network.converters.append(fan_nozzle_2)

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Append propulsors to the network
    #------------------------------------------------------------------------------------------------------------------------------------  
  
    network.propulsors.append(turbofan_1)     
    network.propulsors.append(turbofan_2)

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Append sources to the network
    #------------------------------------------------------------------------------------------------------------------------------------  

    network.sources.append(bat_module_1)
    network.sources.append(bat_module_2)    
    network.sources.append(inboard_tank)  
    network.sources.append(outboard_tank)

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Append distributors to the network
    #------------------------------------------------------------------------------------------------------------------------------------  
  
    network.distributors.append(ac_bus_1)
    network.distributors.append(ac_bus_2) 
    network.distributors.append(dc_bus_1) 
    network.distributors.append(dc_bus_2) 
    network.distributors.append(dc_ess_1) 
    network.distributors.append(dc_ess_2) 
    network.distributors.append(dc_ess_3) 
    network.distributors.append(ac_ess_bus) 
    network.distributors.append(ac_stby_bus) 
    network.distributors.append(fuel_line)
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Append network to the vehicle
    #------------------------------------------------------------------------------------------------------------------------------------  
  
    vehicle.append_energy_network(network)     

    #------------------------------------------------------------------------------------------------------------------------------------  
    # vehicle_setup complete!
    #------------------------------------------------------------------------------------------------------------------------------------  
    
    return vehicle

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
   
    analyses = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle
   
    # append vehicle 
    analyses.vehicle = vehicle 

    #  Geometry
    analyses.geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()  
    
    #  Weights
    analyses.weights = RCAIDE.Framework.Analyses.Weights.Conventional() 
    analyses.weights.settings.FLOPS.fidelity                      = 'Complex'      
    analyses.weights.settings.advanced_composites                 = True 
    
    # Aerodynamics
    analyses.aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    analyses.aerodynamics.settings.store_training_data           = True
    analyses.aerodynamics.settings.number_of_spanwise_vortices   = 15
    analyses.aerodynamics.settings.number_of_chordwise_vortices  = 2  
 
    #  Energy
    analyses.energy = RCAIDE.Framework.Analyses.Energy.Energy() 

    #  Planet Analysis
    analyses.planet = RCAIDE.Framework.Analyses.Planets.Earth() 

    #  Atmosphere Analysis
    analyses.atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976() 

    return analyses

# def base_analysis(vehicle):
#     """This is the baseline set of analyses to be used with this vehicle. Of these, the most
#     commonly changed are the weights and aerodynamics methods."""

#     # ------------------------------------------------------------------
#     #   Initialize the Analyses
#     # ------------------------------------------------------------------     
#     analyses = RCAIDE.Framework.Analyses.Vehicle()

#     #  Geometry
#     geometry = RCAIDE.Framework.Analyses.Geometry.Geometry() 
#     geometry.settings.overwrite_reference        = True
#     geometry.settings.update_wing_properties     = True
#     geometry.settings.print_weight_analysis_report = True
#     analyses.append(geometry)

#     # ------------------------------------------------------------------
#     #  Weights
#     weights = RCAIDE.Framework.Analyses.Weights.Conventional_Transport() 
#     weights.settings.print_weight_analysis_report = True
#     weights.method  = "FLOPS"
#     weights.settings.FLOPS.fidelity    = "Complex"
#     analyses.append(weights)

#     # ------------------------------------------------------------------
#     #  Aerodynamics Analysis
#     aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
#     analyses.append(aerodynamics)
 
#     # ------------------------------------------------------------------
#     #  Energy
#     energy = RCAIDE.Framework.Analyses.Energy.Energy() 
#     analyses.append(energy)

#     # ------------------------------------------------------------------
#     #  Planet Analysis
#     planet = RCAIDE.Framework.Analyses.Planets.Earth()
#     analyses.append(planet)

#     # ------------------------------------------------------------------
#     #  Atmosphere Analysis
#     atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
#     atmosphere.features.planet = planet.features
#     analyses.append(atmosphere)   

#     return analyses    
    
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

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Takeoff Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Takeoff(base_segment)
    segment.tag = "Takeoff" 
    segment.analyses.extend( analyses.takeoff )
    segment.velocity_start           = 10.* Units.knots
    segment.velocity_end             = 125.0 * Units['m/s']
    segment.friction_coefficient     = 0.04
    segment.altitude                 = 0.0   
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1" 
    segment.analyses.extend( analyses.takeoff ) 
    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 3.0   * Units.km
    segment.air_speed      = 125.0 * Units['m/s']
    segment.climb_rate     = 6.0   * Units['m/s']  
     
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
    segment.altitude_end   = 8.0   * Units.km
    segment.air_speed      = 190.0 * Units['m/s']
    segment.climb_rate     = 6.0   * Units['m/s']  
    
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
    segment.altitude_end = 10.5   * Units.km
    segment.air_speed    = 226.0  * Units['m/s']
    segment.climb_rate   = 3.0    * Units['m/s']  
    
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
    segment.altitude                                      = 10.668 * Units.km  
    segment.air_speed                                     = 230.412 * Units['m/s']
    segment.distance                                      = 2600 * Units.nmi   
    
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
    segment.altitude_start                                = 10.5 * Units.km 
    segment.altitude_end                                  = 8.0   * Units.km
    segment.air_speed                                     = 220.0 * Units['m/s']
    segment.descent_rate                                  = 4.5   * Units['m/s']  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag  = "descent_2" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                  = 6.0   * Units.km
    segment.air_speed                                     = 195.0 * Units['m/s']
    segment.descent_rate                                  = 5.0   * Units['m/s']  
    
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
    segment.altitude_end                                  = 4.0   * Units.km
    segment.air_speed                                     = 170.0 * Units['m/s']
    segment.descent_rate                                  = 5.0   * Units['m/s']  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Fourth Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_4" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                  = 2.0   * Units.km
    segment.air_speed                                     = 150.0 * Units['m/s']
    segment.descent_rate                                  = 5.0   * Units['m/s']  
    
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
    segment.air_speed                                     = 145.0 * Units['m/s']
    segment.descent_rate                                  = 3.0   * Units['m/s']  
    
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
    segment.velocity_start                                = 145.0 * Units['m/s']
    segment.velocity_end                                  = 10 * Units.knots 
    segment.friction_coefficient                          = 0.4
    segment.altitude                                      = 0.0   
    segment.assigned_control_variables.elapsed_time.active           = True  
    segment.assigned_control_variables.elapsed_time.initial_guess_values  = [[30.]]  
    mission.append_segment(segment)     


    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------

    return mission

def payload_range_mission_setup(analyses):
    """This function defines the baseline mission that will be flown by the aircraft in order
    to compute performance."""

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'
  
    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()

    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1" 
    segment.analyses.extend( analyses.takeoff ) 
    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 3.0   * Units.km
    segment.air_speed      = 125.0 * Units['m/s']
    segment.climb_rate     = 6.0   * Units['m/s']  
     
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
    segment.altitude_end   = 8.0   * Units.km
    segment.air_speed      = 190.0 * Units['m/s']
    segment.climb_rate     = 6.0   * Units['m/s']  
    
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
    segment.altitude_end = 10.5   * Units.km
    segment.air_speed    = 226.0  * Units['m/s']
    segment.climb_rate   = 3.0    * Units['m/s']  
    
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
    segment.altitude                                      = 10.668 * Units.km  
    segment.air_speed                                     = 230.412 * Units['m/s']
    segment.distance                                      = 1000 * Units.nmi   
    
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
    segment.altitude_start                                = 10.5 * Units.km 
    segment.altitude_end                                  = 8.0   * Units.km
    segment.air_speed                                     = 220.0 * Units['m/s']
    segment.descent_rate                                  = 4.5   * Units['m/s']  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag  = "descent_2" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                  = 6.0   * Units.km
    segment.air_speed                                     = 195.0 * Units['m/s']
    segment.descent_rate                                  = 5.0   * Units['m/s']  
    
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
    segment.altitude_end                                  = 4.0   * Units.km
    segment.air_speed                                     = 170.0 * Units['m/s']
    segment.descent_rate                                  = 5.0   * Units['m/s']  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Fourth Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_4" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                  = 0.0   * Units.km
    segment.air_speed                                     = 140.0 * Units['m/s']
    segment.descent_rate                                  = 5.0   * Units['m/s']  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
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
        
    return
 
# This section is needed to actually run the various functions in the file
if __name__ == '__main__': 
    main()
    plt.show()
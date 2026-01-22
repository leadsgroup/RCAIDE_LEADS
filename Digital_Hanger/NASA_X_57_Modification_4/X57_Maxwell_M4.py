# X57_Maxwell.py
#
# Created: Feb 2020, M. Clarke
#          Sep 2020, M. Clarke 

""" setup file for the X57-Maxwell Electric Aircraft 
"""
 
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import RCAIDE
from RCAIDE.Core import Units 
import numpy as np   
import pickle
from RCAIDE.Core import Data 
from RCAIDE.Plots.Mission_Plots import *  
from RCAIDE.Plots.Geometry_Plots import * 
from RCAIDE.Components.Energy.Networks.Battery_Dual_Propeller import Battery_Dual_Propeller
from RCAIDE.Methods.Propulsion import propeller_design 
from RCAIDE.Methods.Power.Battery.Sizing import initialize_from_mass
from RCAIDE.Methods.Propulsion.electric_motor_sizing import  size_optimal_motor

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():   
    configs, analyses = full_setup() 

    configs.finalize()
    analyses.finalize()  

    # mission analysis
    mission = analyses.missions.base
    results = mission.evaluate() 
    
    # save results  
    #save_results(results)
    
    # plot the results
    plot_results(results) 
       
    return


# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():

    # vehicle data
    vehicle  = vehicle_setup()
    
    # Set up configs
    configs  = configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = analyses_setup(configs)

    # mission analyses
    mission  = mission_setup(configs_analyses,vehicle)
    missions_analyses = missions_setup(mission)

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses

    return configs, analyses

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

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
    #  Basic Geometry Relations
    sizing = RCAIDE.Framework.Analyses.Sizing.Sizing()
    sizing.features.vehicle = vehicle
    analyses.append(sizing)

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
    #  Noise Analysis
    noise = RCAIDE.Framework.Analyses.Noise.Fidelity_Zero()   
    noise.geometry = vehicle
    analyses.append(noise)

    # ------------------------------------------------------------------
    #  Energy
    energy= RCAIDE.Framework.Analyses.Energy.Energy()
    energy.network = vehicle.propulsors 
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


# ----------------------------------------------------------------------
#   Define the Vehicle
# ---------------------------------------------------------------------- 

def vehicle_setup():

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    

    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'X57_Maxwell_M4'    


    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    # mass properties
    vehicle.mass_properties.max_takeoff   = 2550. * Units.pounds
    vehicle.mass_properties.takeoff       = 2550. * Units.pounds
    vehicle.mass_properties.max_zero_fuel = 2550. * Units.pounds
    vehicle.mass_properties.cargo         = 0. 

    # envelope properties
    vehicle.flight_envelope.ultimate_load = 5.7
    vehicle.flight_envelope.positive_limit_load    = 3.8

    # basic parameters
    vehicle.reference_area         = 6.144 
    vehicle.number_of_passengers             = 4

    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------        

    wing = RCAIDE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'

    wing.sweeps.leading_edge     = 0.0 * Units.deg
    wing.thickness_to_chord      = 0.14 
    wing.areas.reference         = 6.144
    wing.spans.projected         = 31.6 * Units.feet
    wing.chords.root             = 2.48 * Units.feet  
    wing.chords.tip              = 1.74 * Units.feet 
    wing.chords.mean_aerodynamic = 2.13 * Units.feet 

    wing.taper                   = wing.chords.root/wing.chords.tip 
    wing.aspect_ratio            = 15
    wing.twists.root             = 3.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees 
    wing.origin                  = [[2.93, 0., 1.01]]
    wing.aerodynamic_center      = [3., 0., 1.01]

    wing.vertical                = False
    wing.xz_plane_symmetric      = True
    wing.high_lift               = True

    wing.dynamic_pressure_ratio  = 1.0 

    # Wing Segments
    segment                               = RCAIDE.Components.Wings.Segment()
    segment.tag                           = 'inboard'
    segment.percent_span_location         = 0.0 
    segment.twist                         = 3. * Units.degrees   
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.  
    segment.sweeps.leading_edge           = 0.
    segment.thickness_to_chord            = 0.14 
    wing.append_segment(segment) 

    segment                               = RCAIDE.Components.Wings.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.
    segment.twist                         = 0. * Units.degrees 
    segment.root_chord_percent            = 1.74 * Units.feet / wing.chords.root 
    segment.dihedral_outboard             = 0.
    segment.sweeps.leading_edge           = 0.
    segment.thickness_to_chord            = 0.14 
    wing.append_segment(segment)    

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------        

    wing = RCAIDE.Components.Wings.Wing()
    wing.tag = 'horizontal_stabilizer'

    wing.sweeps.quarter_chord    = 0.0 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.areas.reference         = 2.540 
    wing.spans.projected         = 3.3  * Units.meter 
    wing.sweeps.quarter_chord    = 0 * Units.deg

    wing.chords.root             = 0.769 * Units.meter 
    wing.chords.tip              = 0.769 * Units.meter 
    wing.chords.mean_aerodynamic = 0.769 * Units.meter  
    wing.taper                   = 1.

    wing.aspect_ratio            = wing.spans.projected**2. / wing.areas.reference

    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.origin                  = [[7.7, 0., 0.25]]
    wing.aerodynamic_center      = [7.8, 0., 0.25]

    wing.vertical                = False
    wing.xz_plane_symmetric      = True
    wing.high_lift               = False

    wing.dynamic_pressure_ratio  = 0.9

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = RCAIDE.Components.Wings.Wing()
    wing.tag = 'vertical_stabilizer'    

    wing.sweeps.quarter_chord    = 25. * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.areas.reference         = 2.258 * Units['meters**2']  
    wing.spans.projected         = 1.854   * Units.meter 

    wing.chords.root             = 1.6764 * Units.meter 
    wing.chords.tip              = 0.6858 * Units.meter 
    wing.chords.mean_aerodynamic = 1.21 * Units.meter 
    wing.taper                   = wing.chords.root/wing.chords.tip

    wing.aspect_ratio            = wing.spans.projected**2. / wing.areas.reference

    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.origin                  = [[6.75 ,0,  0.623]]
    wing.aerodynamic_center      = [0.508 ,0,0] 

    wing.vertical                = True 
    wing.xz_plane_symmetric      = False
    wing.t_tail                  = False

    wing.dynamic_pressure_ratio  = 1.0

    # add to vehicle
    vehicle.append_component(wing)




    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------ 
    fuselage = RCAIDE.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage' 
    fuselage.seats_abreast                      = 2. 
    fuselage.fineness.nose                      = 1.6
    fuselage.fineness.tail                      = 2. 
    fuselage.lengths.nose                       = 60.  * Units.inches
    fuselage.lengths.tail                       = 161. * Units.inches
    fuselage.lengths.cabin                      = 105. * Units.inches
    fuselage.lengths.total                      = 332.2* Units.inches
    fuselage.lengths.fore_space                 = 0.
    fuselage.lengths.aft_space                  = 0.   
    fuselage.width                              = 42. * Units.inches 
    fuselage.heights.maximum                    = 62. * Units.inches
    fuselage.heights.at_quarter_length          = 62. * Units.inches
    fuselage.heights.at_three_quarters_length   = 62. * Units.inches
    fuselage.heights.at_wing_root_quarter_chord = 23. * Units.inches 
    fuselage.areas.side_projected               = 8000.  * Units.inches**2.
    fuselage.areas.wetted                       = 30000. * Units.inches**2.
    fuselage.areas.front_projected              = 42.* 62. * Units.inches**2. 
    fuselage.effective_diameter                 = 50. * Units.inches


    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_0'  
    segment.percent_x_location                  = 0 
    segment.percent_z_location                  = 0 
    segment.height                              = 0.01 
    segment.width                               = 0.01 
    fuselage.append_segment(segment)             
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_1'   
    segment.percent_x_location                  = 0.007279116466
    segment.percent_z_location                  = 0.002502014453
    segment.height                              = 0.1669064748
    segment.width                               = 0.2780205877
    fuselage.append_segment(segment)    

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.01941097724 
    segment.percent_z_location                  = 0.001216095397 
    segment.height                              = 0.3129496403 
    segment.width                               = 0.4365777215 
    fuselage.append_segment(segment)         

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.06308567604
    segment.percent_z_location                  = 0.007395489231
    segment.height                              = 0.5841726619
    segment.width                               = 0.6735119903
    fuselage.append_segment(segment)      
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.1653761217 
    segment.percent_z_location                  = 0.02891281352 
    segment.height                              = 1.064028777 
    segment.width                               = 1.067200529 
    fuselage.append_segment(segment)  
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'    
    segment.percent_x_location                  = 0.2426372155 
    segment.percent_z_location                  = 0.04214148761 
    segment.height                              = 1.293766653 
    segment.width                               = 1.183058255 
    fuselage.append_segment(segment)  
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'    
    segment.percent_x_location                  = 0.2960174029  
    segment.percent_z_location                  = 0.04705241831  
    segment.height                              = 1.377026712  
    segment.width                               = 1.181540054  
    fuselage.append_segment(segment)  
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.3809404284 
    segment.percent_z_location                  = 0.05313580461 
    segment.height                              = 1.439568345 
    segment.width                               = 1.178218989 
    fuselage.append_segment(segment)    
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 0.5046854083 
    segment.percent_z_location                  = 0.04655492473 
    segment.height                              = 1.29352518 
    segment.width                               = 1.054390707 
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_9'   
    segment.percent_x_location                  = 0.6454149933 
    segment.percent_z_location                  = 0.03741966266 
    segment.height                              = 0.8971223022 
    segment.width                               = 0.8501926505   
    fuselage.append_segment(segment)  
      
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_10'   
    segment.percent_x_location                  = 0.985107095 
    segment.percent_z_location                  = 0.04540283436 
    segment.height                              = 0.2920863309 
    segment.width                               = 0.2012565415  
    fuselage.append_segment(segment)         
       
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_11'   
    segment.percent_x_location                  = 1 
    segment.percent_z_location                  = 0.04787575562  
    segment.height                              = 0.1251798561 
    segment.width                               = 0.1206021048 
    fuselage.append_segment(segment)             
    
    # add to vehicle
    vehicle.append_component(fuselage)   

    #---------------------------------------------------------------------------------------------
    # DEFINE NETWORK
    #--------------------------------------------------------------------------------------------- 
    # Component 1  -------------------------------------------------------- 
    net = Battery_Dual_Propeller() 
    net.number_of_rotor_engines = 2
    net.propeller_thrust_angle      = 0.   * Units.degrees
    net.propeller_nacelle_diameter  = 1.166 * Units.feet  
    net.propeller_engine_length     = 3  * Units.feet
    
    net.number_of_rotor_engines     = 12
    net.rotor_thrust_angle          = 0.   * Units.degrees
    net.rotor_nacelle_diameter      = 0.5  * Units.feet  
    net.rotor_engine_length         = 1 * Units.feet    
    
    net.areas                       = Data()
    net.areas.wetted                = np.pi*net.nacelle_diameter*net.engine_length + 0.5*np.pi*net.nacelle_diameter**2    
    net.voltage                     = 400.
    
    # Component 2 Electronic Speed Controller -------------------------------------------------------- 
    rotor_esc              = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
    rotor_esc.efficiency   = 0.95
    net.rotor_esc          = rotor_esc 

    propeller_esc            = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
    propeller_esc.efficiency = 0.95
    net.propeller_esc        = propeller_esc
    
    # Component 3 the Propeller  ------------------------------------------------------------- 
    # Cruise Propeller 
    prop_cr = RCAIDE.Library.Components.Powertrain.Converters.Propeller()  
    prop_cr.number_of_blades    = 3 
    prop_cr.design_freestream_velocity = 173.984 * Units['mph']       
    prop_cr.tip_radius          = 1.523/2
    prop_cr.hub_radius          = 0.1     
    prop_cr.design_Cl           = 0.75
    prop_cr.design_tip_mach     = 0.6   
    speed_of_sound              = 340
    prop_cr.design_angular_velocity    = 2250 * Units.rpm #  prop_cr.design_tip_mach*speed_of_sound/prop_cr.tip_radius  
    prop_cr.design_altitude     = 8000. * Units.feet 
    prop_cr.design_power        = 48100 # 115.  
    prop_cr.origin              = [[2.5, 4.97584, 1.01],[2.5, -4.97584, 1.01]]     
    prop_cr.rotation            = [-1,1] 
    prop_cr.symmetry            = True
    prop_cr                     = propeller_design(prop_cr) 
    net.propeller               = prop_cr   
    
    # Design Highlift Propeller 
    prop_hl = RCAIDE.Library.Components.Powertrain.Converters.Rotor()  
    prop_hl.number_of_blades    = 5   
    prop_hl.design_freestream_velocity = 63.379  * Units['mph']        
    prop_hl.tip_radius          = 0.58/2 
    prop_hl.hub_radius          = 0.1     
    prop_hl.design_Cl           = 0.75
    prop_hl.design_tip_mach     = 0.6   
    prop_hl.design_angular_velocity    = 2250 * Units.rpm #   prop_hl.design_tip_mach*speed_of_sound/prop_hl.tip_radius  
    prop_hl.design_altitude     = 1000. * Units.feet 
    prop_hl.design_power        = 1400 
    #prop_hl.design_thrust   -  = 300 # 115.  
    prop_pitch                  = 0.6 
    prop_hl.origin              = [[2.5, (1.05 + prop_pitch*0), 1.01], [2.5, (1.05 + prop_pitch*1), 1.01],[2.5, (1.05 + prop_pitch*2), 1.01],
                                   [2.5, (1.05 + prop_pitch*3), 1.01],[2.5,  (1.05 + prop_pitch*4), 1.01],[2.5, (1.05 + prop_pitch*5), 1.01],
                                   [2.5,-(1.05 + prop_pitch*0) ,1.01], [2.5,-(1.05 + prop_pitch*1), 1.01],[2.5,-(1.05 + prop_pitch*2), 1.01],
                                   [2.5,-(1.05 + prop_pitch*3), 1.01],[2.5 ,-(1.05 + prop_pitch*4), 1.01],[2.5,-(1.05 + prop_pitch*5), 1.01]]     
    prop_hl.rotation            = [-1,-1,-1,-1,-1,-1,1,1,1,1,1,1] 
    prop_hl.symmetry            = True
    prop_hl                     = propeller_design(prop_hl)      
    net.rotor                   = prop_hl
    
    # Component 4 the Battery --------------------------------------------------------------------
    bat = RCAIDE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion()
    bat.mass_properties.mass = 300. * Units.kg  
    bat.specific_energy      = 200. * Units.Wh/Units.kg
    bat.resistance           = 0.006
    bat.pack.max_voltage          = 400.
    
    initialize_from_mass(bat,bat.mass_properties.mass)
    net.battery              = bat 
    net.voltage              = bat.pack.max_voltage
    
    # Component 5 the Motor --------------------------------------------------------------------
    # Cruise Propeller  motor 
    motor_cr                       = RCAIDE.Library.Components.Powertrain.Converters.Motor() 
    motor_cr.efficiency           = 0.935
    motor_cr.gearbox_efficiency   = 1.  
    motor_cr.nominal_voltage      = bat.pack.max_voltage
    motor_cr.propeller_radius     = prop_cr.tip_radius    
    motor_cr.no_load_current      = 2.0 
    motor_cr.origin               = prop_cr.origin 
    motor_cr                      = size_optimal_motor(motor_cr,prop_cr) 
    net.propeller_motor           = motor_cr  
    
    # High Lift Propeller  motor 
    motor_hl                      = RCAIDE.Library.Components.Powertrain.Converters.Motor() 
    motor_hl.efficiency           = 0.935
    motor_hl.gearbox_efficiency   = 1.  
    motor_hl.nominal_voltage      = bat.pack.max_voltage
    motor_hl.propeller_radius     = prop_hl.tip_radius    
    motor_hl.no_load_current      = 2.0 
    motor_hl.origin               = prop_hl.origin 
    motor_hl                      = size_optimal_motor(motor_hl,prop_hl) 
    net.rotor_motor               = motor_hl  
     
    # Component 9 Miscellaneous Systems 
    sys = RCAIDE.Library.Components.Powertrain.Systems.Systems()
    sys.mass_properties.mass = 5 # kg  
    
    # add the solar network to the vehicle
    vehicle.append_component(net)   
    
    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------
    # plot vehicle 
    plot_3d_vehicle(vehicle)
    plt.show()

    return vehicle 

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses 

# ---------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):

    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs = RCAIDE.Library.Components.Configs.Config.Container()

    base_config = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config) 

 
    # ------------------------------------------------------------------
    #   Hover Configuration
    # ------------------------------------------------------------------
    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'landing' 
    config.propulsors.Battery_Electric_Rotor.blade_pitch_command      = 0.0 * Units.degrees 
    config.propulsors.Battery_Electric_Rotor.motor.gear_ratio   = 1.1
    configs.append(config)                  


    # done!
    return configs 
 

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses,vehicle): 
    
    Climb_Rate           = 500* Units['ft/min']
    Descent_Rate         = 300 * Units['ft/min']
    Climb_Speed          = 120 * Units['mph']  
    Descent_Speed        = 110 * Units['mph'] 
    Nominal_Range        = 51.5 * Units['miles']
    Nominal_Flight_Time  = 29.3*(Nominal_Range/ Units['miles']) + 38.1
    Cruise_Altitude      = 2500 * Units.feet
    
    # Determine Stall Speed 
    m     = vehicle.mass_properties.max_takeoff
    g     = 9.81
    S     = vehicle.reference_area
    atmo  = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    rho   = atmo.compute_values(1000.*Units.feet,0.).density
    CLmax = 1.2 
    Vstall = float(np.sqrt(2.*m*g/(rho*S*CLmax))) 
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = RCAIDE.Framework.Analyses.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # airport
    airport = RCAIDE.Attributes.Airports.Airport()
    airport.altitude   =  0. * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = RCAIDE.Attributes.Atmospheres.Earth.US_Standard_1976()

    mission.airport = airport    

    # unpack Segments module
    Segments = RCAIDE.Framework.Analyses.Mission.Segments 
    
    # base segment
    base_segment                                             = Segments.Segment()
    ones_row                                                 = base_segment.state.ones_row
    #base_segment.use_Jacobian                                = False  
    base_segment.process.initialize.initialize_battery = RCAIDE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.iterate.conditions.planet_position  = RCAIDE.Methods.skip
    base_segment.state.numerics.number_of_control_points        = 4
    base_segment.process.iterate.unknowns.network            = vehicle.propulsors.battery_electric_rotor.unpack_unknowns_high_lift
    base_segment.process.iterate.residuals.network           = vehicle.propulsors.battery_electric_rotor.residuals_high_lift
    base_segment.state.unknowns.propeller_power_coefficient  = 0.005 * ones_row(1) 
    base_segment.state.unknowns.battery_voltage_under_load   = vehicle.propulsors.Battery_Electric_Rotor.battery.max_voltage * ones_row(1)  
    base_segment.state.residuals.network                     = 0. * ones_row(2)       
 
    # Determine Stall Speed 
    m     = vehicle.mass_properties.max_takeoff
    g     = 9.81
    S     = vehicle.reference_area
    atmo  = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    rho   = atmo.compute_values(1000.*Units.feet,0.).density
    CLmax = 1.2 
    Vstall = float(np.sqrt(2.*m*g/(rho*S*CLmax))) 
    
    # Calculate Flight Parameters 
    ICA_Altitude        = 500 * Units.feet 
    Downleg_Altitude    = 1000 * Units.feet  
    Climb_Altitude      = Cruise_Altitude  - ICA_Altitude
    Descent_Altitude    = Cruise_Altitude - Downleg_Altitude  
     
    Climb_Range_Speed   = np.sqrt(Climb_Speed**2 - Climb_Rate**2)
    Climb_Time          = Climb_Altitude/Climb_Rate 
    Climb_Range         = Climb_Range_Speed * Climb_Time 
    Descent_Range_Speed = np.sqrt(Descent_Speed**2 - Descent_Rate**2)
    Descent_Time        = Descent_Altitude /(Descent_Rate )
    Descent_Range       = Descent_Range_Speed * Descent_Time 
    Cruise_Distance     = Nominal_Range - (Climb_Range + Descent_Range)
    Cruise_Time         = Nominal_Flight_Time - (Climb_Time+Descent_Time )
    Cruise_Speed        = Cruise_Distance/Cruise_Time  
    
    
    # Calculate Flight Parameters 
    Downleg_Distance        = 6000 * Units.feet # length or runway
    Downleg_Altitude        = 1000 * Units.feet 
  
    cl_throt_ukn  = 0.85
    cl_CP_ukn     = 0.3
    cr_throt_ukn  = 0.6
    cr_CP_ukn     = 0.4
    des_throt_ukn = 0.5
    des_CP_ukn    = 0.3
    
    # ------------------------------------------------------------------
    #   Departure End of Runway Segment Flight 1 : 
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    segment.tag = 'DER'       
    segment.analyses.extend( analyses.base ) 
    segment.battery_energy                                   = vehicle.propulsors.Battery_Electric_Rotor.battery.max_energy   
    segment.state.unknowns.rotor_power_coefficient          =  0.01 *  ones_row(1) 
    segment.state.unknowns.throttle_lift                    =  1.0  *  ones_row(1) 
    segment.state.unknowns.propeller_power_coefficient      =  0.01 *  ones_row(1) 
    segment.state.unknowns.throttle                         =  1.0  *  ones_row(1) 
    segment.state.residuals.network                         =  0.   *  ones_row(3)  
    segment.process.iterate.unknowns.network                = vehicle.propulsors.battery_electric_rotor.unpack_unknowns_high_lift
    segment.process.iterate.residuals.network               = vehicle.propulsors.battery_electric_rotor.residuals_high_lift
    
    segment.altitude_start                                   = 0.0 * Units.feet
    segment.altitude_end                                     = 50.0 * Units.feet
    segment.air_speed_start                                  = Vstall  
    segment.air_speed_end                                    = 45                    
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Initial Climb Area Segment Flight 1  
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    segment.tag = 'ICA' 
    segment.analyses.extend( analyses.base )  
    segment.state.unknowns.rotor_power_coefficient          =  0.01 *  ones_row(1) 
    segment.state.unknowns.throttle_lift                    =  1.0  *  ones_row(1) 
    segment.state.unknowns.propeller_power_coefficient      =  0.01 *  ones_row(1) 
    segment.state.unknowns.throttle                         =  1.0  *  ones_row(1) 
    segment.state.residuals.network                         =  0.   *  ones_row(3)  
    segment.process.iterate.unknowns.network                = vehicle.propulsors.battery_electric_rotor.unpack_unknowns_high_lift
    segment.process.iterate.residuals.network               = vehicle.propulsors.battery_electric_rotor.residuals_high_lift
    segment.altitude_start                                  = 50.0 * Units.feet
    segment.altitude_end                                    = 500.0 * Units.feet
    segment.air_speed_start                                 = 45  * Units['m/s']   
    segment.air_speed_end                                   = 50 * Units['m/s']   
    segment.climb_rate                                      = 600 * Units['ft/min']    
    mission.append_segment(segment) 
             
    # ------------------------------------------------------------------
    #   Climb Segment Flight 1 
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment) 
    segment.tag = 'Climb'        
    segment.analyses.extend( analyses.base )     
    segment.state.unknowns.rotor_power_coefficient          =  cl_CP_ukn *  ones_row(1) 
    segment.state.unknowns.throttle_lift                    =  cl_throt_ukn *  ones_row(1) 
    segment.state.unknowns.propeller_power_coefficient      =  cl_CP_ukn*  ones_row(1) 
    segment.state.unknowns.throttle                         =  cl_throt_ukn *  ones_row(1) 
    segment.state.residuals.network                         =  0.   *  ones_row(3)  
    segment.process.iterate.unknowns.network                = vehicle.propulsors.battery_electric_rotor.unpack_unknowns_high_lift
    segment.process.iterate.residuals.network               = vehicle.propulsors.battery_electric_rotor.residuals_high_lift
    segment.altitude_start                                  = 500.0 * Units.feet
    segment.altitude_end                                    = Cruise_Altitude
    segment.air_speed                                       = Climb_Speed
    segment.climb_rate                                      = Climb_Rate  
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Cruise Segment Flight 1 
    # ------------------------------------------------------------------ 
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment) 
    segment.tag = 'Cruise'  
    segment.analyses.extend(analyses.base) 
    segment.air_speed                                  = Cruise_Speed
    segment.distance                                   = Cruise_Distance 
    segment.state.unknowns.propeller_power_coefficient =  0.16   * ones_row(1)
    segment.state.unknowns.throttle                    =  0.80   * ones_row(1) 
    segment.process.iterate.unknowns.network           = vehicle.propulsors.battery_electric_rotor.unpack_unknowns_cruise
    segment.process.iterate.residuals.network          = vehicle.propulsors.battery_electric_rotor.residuals_cruise    
    mission.append_segment(segment)     
    
    # ------------------------------------------------------------------
    #   Descent Segment Flight 1   
    # ------------------------------------------------------------------ 
    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment) 
    segment.tag = 'Descent'
    segment.analyses.extend( analyses.base )    
    segment.altitude_end                                     = Downleg_Altitude
    segment.air_speed                                        = Descent_Speed
    segment.descent_rate                                     = Descent_Rate 
    segment.state.unknowns.propeller_power_coefficient       =  0.16   * ones_row(1)
    segment.state.unknowns.throttle                          =  0.80   * ones_row(1) 
    segment.process.iterate.unknowns.network                 = vehicle.propulsors.battery_electric_rotor.unpack_unknowns_cruise
    segment.process.iterate.residuals.network                = vehicle.propulsors.battery_electric_rotor.residuals_cruise    
    mission.append_segment(segment)  

    # ------------------------------------------------------------------
    #  Downleg_Altitude Segment Flight 1 
    # ------------------------------------------------------------------ 
    segment = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment) 
    segment.tag = 'Downleg'
    segment.analyses.extend(analyses.base) 
    segment.air_speed_start                                  = Descent_Speed * Units['m/s']
    segment.air_speed_end                                    = 45.0 * Units['m/s']            
    segment.distance                                         = Downleg_Distance
    segment.acceleration                                     = -0.05307 * Units['m/s/s']  
    segment.air_speed                                        = 49.174
    segment.descent_rate                                     = Descent_Rate
    segment.state.unknowns.propeller_power_coefficient       =  0.16   * ones_row(1)
    segment.state.unknowns.throttle                          =  0.80   * ones_row(1) 
    segment.process.iterate.unknowns.network                 = vehicle.propulsors.battery_electric_rotor.unpack_unknowns_cruise
    segment.process.iterate.residuals.network                = vehicle.propulsors.battery_electric_rotor.residuals_cruise       
    
    mission.append_segment(segment)       
    
    # ------------------------------------------------------------------
    #  Baseleg Segment Flight 1  
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = 'Baseleg'
    segment.analyses.extend( analyses.base)  
    segment.state.unknowns.rotor_power_coefficient          =  0.01 *  ones_row(1) 
    segment.state.unknowns.throttle_lift                    =  1.0  *  ones_row(1) 
    segment.state.unknowns.propeller_power_coefficient      =  0.01 *  ones_row(1) 
    segment.state.unknowns.throttle                         =  1.0  *  ones_row(1) 
    segment.state.residuals.network                         =  0.   *  ones_row(3)  
    segment.process.iterate.unknowns.network                = vehicle.propulsors.battery_electric_rotor.unpack_unknowns_high_lift
    segment.process.iterate.residuals.network               = vehicle.propulsors.battery_electric_rotor.residuals_high_lift   
    segment.altitude_start                                   = 1000 * Units.feet
    segment.altitude_end                                     = 500.0 * Units.feet
    segment.air_speed_start                                  = 45 
    segment.air_speed_end                                    = 40    
    segment.climb_rate                                       = -350 * Units['ft/min']
    mission.append_segment(segment) 

    # ------------------------------------------------------------------
    #  Final Approach Segment Flight 1  
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment_name = 'Final_Approach'
    segment.tag = segment_name          
    segment.state.unknowns.rotor_power_coefficient          =  0.01 *  ones_row(1) 
    segment.state.unknowns.throttle_lift                    =  1.0  *  ones_row(1) 
    segment.state.unknowns.propeller_power_coefficient      =  0.01 *  ones_row(1) 
    segment.state.unknowns.throttle                         =  1.0  *  ones_row(1) 
    segment.state.residuals.network                         =  0.   *  ones_row(3)  
    segment.process.iterate.unknowns.network                = vehicle.propulsors.battery_electric_rotor.unpack_unknowns_high_lift
    segment.process.iterate.residuals.network               = vehicle.propulsors.battery_electric_rotor.residuals_high_lift 
    segment.altitude_start                                  = 500.0 * Units.feet
    segment.altitude_end                                    = 00.0 * Units.feet
    segment.air_speed_start                                 = 40 
    segment.air_speed_end                                   = 35   
    segment.climb_rate                                      = -300 * Units['ft/min']        
    mission.append_segment(segment)  
 
    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------ 
    
    return mission

def missions_setup(base_mission):

    # the mission container
    missions = RCAIDE.Framework.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------
    missions.base = base_mission

    # done!
    return missions  


def plot_results(results,line_style = 'bo-'):  
    
    
    plot_flight_conditions(results, line_style) 
    
    # Plot Flight Conditions 
    plot_flight_conditions(results, line_style) 
    
    # Plot Aerodynamic Coefficients
    plot_aerodynamic_coefficients(results, line_style)  
    
    # Plot Aircraft Flight Speed
    plot_aircraft_velocities(results, line_style)
    
    # Plot Aircraft Electronics
    plot_electronic_conditions(results, line_style)
    
    # Plot Propeller Conditions 
    plot_propeller_conditions(results, line_style) 
    
    # Plot Electric Motor and Propeller Efficiencies 
    plot_eMotor_Prop_efficiencies(results, line_style)
    
    # Plot propeller Disc and Power Loading
    plot_disc_power_loading(results, line_style)   
     
     
    return


def save_results(results):

    # Store data (serialize)
    with open('Maxwell_M4_Results.pkl', 'wb') as file:
        pickle.dump(results, file)
        
    return   

if __name__ == '__main__': 
    main()    
    plt.show()
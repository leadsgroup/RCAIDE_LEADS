# concorde.py
# 
# Created:  Aug 2014, SUAVE Team (Stanford University)
# Modified: Nov 2016, T. MacDonald
#           Jul 2017, T. MacDonald
#           Aug 2018, T. MacDonald
#           Nov 2018, T. MacDonald
#           May 2021, E. Botero

""" setup file for a mission with Concorde
"""


# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import RCAIDE
# Units allow any units to be specificied with RCAIDE then automatically converting them the standard
from RCAIDE.Core import Units
from RCAIDE.Visualization import *     

# Numpy is use extensively throughout RCAIDE
import numpy as np

# Post processing plotting tools are imported here
import pylab as plt

# More basic RCAIDE function
from RCAIDE.Core import Data

import sys
sys.path.append('../Vehicles')
from Concorde import vehicle_setup, configs_setup

# This is a sizing function to fill turbojet parameters
from RCAIDE.Methods.Center_of_Gravity.compute_fuel_center_of_gravity_longitudinal_range \
     import compute_fuel_center_of_gravity_longitudinal_range
from RCAIDE.Methods.Center_of_Gravity.compute_fuel_center_of_gravity_longitudinal_range \
     import plot_cg_map 


# This imports lift equivalent area
from RCAIDE.Methods.Noise.Boom.lift_equivalent_area import lift_equivalent_area

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    
    # First we construct the baseline aircraft
    configs, analyses = full_setup()
    
    # Any sizing functions are included here to size components
    simple_sizing(configs)
    
    # Here we finalize the configuration and analysis settings
    configs.finalize()
    analyses.finalize()
    
    ## Use these scripts to test OpenVSP functionality if desired
    #from RCAIDE.Input_Output.OpenVSP.vsp_write import write
    #from RCAIDE.Input_Output.OpenVSP.get_vsp_measurements import get_vsp_measurements
    #write(configs.base,'Concorde')
    #get_vsp_measurements(filename='Unnamed_CompGeom.csv', measurement_type='wetted_area')
    #get_vsp_measurements(filename='Unnamed_CompGeom.csv', measurement_type='wetted_volume')

    # These functions analyze the mission
    mission = analyses.missions.base
    results = mission.evaluate()
    
    # Check the lift equivalent area
    equivalent_area(configs.base, analyses.configs.base, results.segments.cruise.state.conditions)        
    
    masses, cg_mins, cg_maxes = compute_fuel_center_of_gravity_longitudinal_range(configs.base)
    plot_cg_map(masses, cg_mins, cg_maxes, units = 'metric', fig_title = 'Metric Test')  
    plot_cg_map(masses, cg_mins, cg_maxes, units = 'imperial', fig_title = 'Foot Test')
    plot_cg_map(masses, cg_mins, cg_maxes, units = 'imperial', special_length = 'inches',
                fig_title = 'Inch Test')
    
    results.fuel_tank_test = Data()
    results.fuel_tank_test.masses   = masses
    results.fuel_tank_test.cg_mins  = cg_mins
    results.fuel_tank_test.cg_maxes = cg_maxes
    
    # leave uncommented for regression 
    save_results(results)  
    old_results = load_results()    
    
    # plt the old results
    plot_mission(results)
    plot_mission(old_results)
    plt.show()

    # check the results
    check_results(results,old_results)  
    
    return


# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():
    
    # Vehicle data
    vehicle  = vehicle_setup()
    configs  = configs_setup(vehicle)
    
    # Vehicle analyses
    configs_analyses = analyses_setup(configs)
    
    # Mission analyses
    mission  = mission_setup(configs_analyses)
    missions_analyses = missions_setup(mission)

    analyses = RCAIDE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses        
    
    return configs, analyses


# ----------------------------------------------------------------------
#   Lift Equivalent Area Regression
# ----------------------------------------------------------------------

def equivalent_area(vehicle,analyses,conditions):
    
    X_locs, AE_x, _ = lift_equivalent_area(vehicle,analyses,conditions)
    
    regression_X_locs = np.array([ 0.        , 30.0744302 , 36.06867764, 40.19118129, 42.87929299, 43.75864575,
                                   44.46769982, 44.75288937, 45.40283952, 45.75323347, 45.83551432,
                                   50.60861803, 53.90976954, 55.43400893, 56.10861803, 56.78777765,
                                   57.28419734, 57.49949357, 57.99040541, 58.61763071, 58.94738099,
                                   77.075     ])

    regression_AE_x   = np.array([ 0.        ,  8.33595464, 12.66713223, 17.73910795, 19.62471738, 23.97123771,
                                   24.43162196, 26.98062914, 34.56718328, 36.48486225, 37.03462371,
                                   37.03462016, 37.03461949, 37.0346185 , 37.03461866, 37.03461816,
                                   37.03461818, 37.03461807, 37.0346181 , 37.03461812, 37.03461812,
                                   37.03461812])

    
    assert (np.abs((X_locs[1:] - regression_X_locs[1:] )/regression_X_locs[1:] ) < 1e-6).all() 
    assert (np.abs((AE_x[1:] - regression_AE_x[1:])/regression_AE_x[1:]) < 1e-6).all()


# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):
    
    analyses = RCAIDE.Analyses.Analysis.Container()
    
    # build a base analysis for each config
    for tag,config in list(configs.items()):
        analysis = base_analysis(config)
        analyses[tag] = analysis
    
    return analyses

def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Analyses.Vehicle()
    
    # ------------------------------------------------------------------
    #  Basic Geometry Relations
    sizing = RCAIDE.Analyses.Sizing.Sizing()
    sizing.features.vehicle = vehicle
    analyses.append(sizing)
    
    # ------------------------------------------------------------------
    #  Weights
    weights = RCAIDE.Analyses.Weights.Weights_Transport()
    weights.vehicle = vehicle
    analyses.append(weights)
    
    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = RCAIDE.Analyses.Aerodynamics.Supersonic_Zero()
    aerodynamics.geometry = vehicle
    aerodynamics.settings.number_spanwise_vortices     = 5
    aerodynamics.settings.number_chordwise_vortices    = 2       
    aerodynamics.process.compute.lift.inviscid_wings.settings.model_fuselage = True
    aerodynamics.settings.drag_coefficient_increment   = 0.0000
    analyses.append(aerodynamics)
    
    
    # ------------------------------------------------------------------
    #  Energy
    energy= RCAIDE.Analyses.Energy.Energy()
    energy.networks = vehicle.networks  
    analyses.append(energy)
    
    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Analyses.Planets.Planet()
    analyses.append(planet)
    
    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   
    
    # done!
    return analyses    

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results):
    
    plot_altitude_sfc_weight(results) 
    
    plot_flight_conditions(results) 
    
    plot_aerodynamic_coefficients(results)  
    
    plot_aircraft_velocities(results)
    
    plot_drag_components(results)
    return

def simple_sizing(configs):
    
    base = configs.base
    base.pull_base()
    
    # zero fuel weight
    base.mass_properties.max_zero_fuel = 0.9 * base.mass_properties.max_takeoff 
    
    # fuselage seats
    base.fuselages['fuselage'].number_coach_seats = base.passengers
    
    # diff the new data
    base.store_diff()
    
    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------
    landing = configs.landing
    
    # make sure base data is current
    landing.pull_base()
    
    # landing weight
    landing.mass_properties.landing = 0.85 * base.mass_properties.takeoff
    
    # diff the new data
    landing.store_diff()
    
    # done!
    return

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
    
def mission_setup(analyses):
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    
    mission = RCAIDE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'
     
    # unpack Segments module
    Segments = RCAIDE.Analyses.Mission.Segments 
    base_segment = Segments.Segment()
    
    # ------------------------------------------------------------------
    #   First Climb Segment: constant Mach, constant segment angle 
    # ------------------------------------------------------------------
    
    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1" 
    segment.analyses.extend( analyses.climb ) 
    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 4000. * Units.ft
    segment.airpseed       = 250.  * Units.kts
    segment.climb_rate     = 4000. * Units['ft/min'] 
    mission.append_segment(segment)
    
    
    # ------------------------------------------------------------------
    #   Second Climb Segment: constant Speed, constant segment angle 
    # ------------------------------------------------------------------    
    
    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_2" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end = 8000. * Units.ft
    segment.airpseed     = 250.  * Units.kts
    segment.climb_rate   = 2000. * Units['ft/min'] 
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Second Climb Segment: constant Speed, constant segment angle 
    # ------------------------------------------------------------------    
    
    segment = Segments.Climb.Linear_Mach_Constant_Rate(base_segment)
    segment.tag = "climb_2" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end        = 33000. * Units.ft
    segment.mach_number_start   = .45
    segment.mach_number_end     = 0.95
    segment.climb_rate          = 3000. * Units['ft/min'] 
    mission.append_segment(segment)    

    # ------------------------------------------------------------------
    #   Third Climb Segment: linear Mach, constant segment angle 
    # ------------------------------------------------------------------    
      
    segment = Segments.Climb.Linear_Mach_Constant_Rate(base_segment)
    segment.tag = "climb_3" 
    segment.analyses.extend( analyses.climb ) 
    segment.altitude_end        = 34000. * Units.ft
    segment.mach_number_start   = 0.95
    segment.mach_number_end     = 1.1
    segment.climb_rate          = 2000.  * Units['ft/min'] 
    mission.append_segment(segment) 

    # ------------------------------------------------------------------
    #   Third Climb Segment: linear Mach, constant segment angle 
    # ------------------------------------------------------------------   
    segment     = Segments.Climb.Linear_Mach_Constant_Rate(base_segment)
    segment.tag = "climb_4" 
    segment.analyses.extend( analyses.climb ) 
    segment.altitude_end        = 40000. * Units.ft
    segment.mach_number_start   = 1.1
    segment.mach_number_end     = 1.7
    segment.climb_rate          = 1750.  * Units['ft/min'] 
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Fourth Climb Segment: linear Mach, constant segment angle 
    # ------------------------------------------------------------------  
    segment = Segments.Climb.Linear_Mach_Constant_Rate(base_segment)
    segment.tag = "climb_5" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end        = 50000. * Units.ft
    segment.mach_number_start   = 1.7
    segment.mach_number_end     = 2.02
    segment.climb_rate          = 750.  * Units['ft/min'] 
    mission.append_segment(segment)     
    

    # ------------------------------------------------------------------
    #   Fourth Climb Segment: linear Mach, constant segment angle 
    # ------------------------------------------------------------------  
    segment = Segments.Climb.Constant_Mach_Constant_Rate(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end = 56500. * Units.ft
    segment.mach_number  = 2.02
    segment.climb_rate   = 50.  * Units['ft/min'] 
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------    
    #   Cruise Segment: constant speed 
    # ------------------------------------------------------------------    
    segment     = Segments.Cruise.Constant_Mach_Constant_Altitude(base_segment)
    segment.tag = "level_cruise" 
    segment.analyses.extend( analyses.cruise ) 
    segment.mach_number                          = 2.02
    segment.distance                             = 1. * Units.nmi
    segment.state.numerics.number_control_points = 4 
    mission.append_segment(segment)    
    
    # ------------------------------------------------------------------
    #   First Descent Segment: decceleration
    # ------------------------------------------------------------------    
    segment     = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag = "decel_1" 
    segment.analyses.extend( analyses.cruise )
    segment.acceleration      = -.5  * Units['m/s/s']
    segment.air_speed_start   = 2.02*573. * Units.kts
    segment.air_speed_end     = 1.5*573.  * Units.kts 
    mission.append_segment(segment)   
    
    # ------------------------------------------------------------------
    #   First Descent Segment
    # ------------------------------------------------------------------  
    segment     = Segments.Descent.Linear_Mach_Constant_Rate(base_segment)
    segment.tag = "descent_1" 
    segment.analyses.extend( analyses.cruise )
    segment.altitude_end      = 41000. * Units.ft
    segment.mach_number_start = 1.5
    segment.mach_number_end   = 1.3
    segment.descent_rate      = 2000. * Units['ft/min'] 
    mission.append_segment(segment)     
    
    # ------------------------------------------------------------------
    #   First Descent Segment: decceleration
    # ------------------------------------------------------------------   
    segment     = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag = "decel_2" 
    segment.analyses.extend( analyses.cruise )
    segment.acceleration      = -.5  * Units['m/s/s']
    segment.air_speed_start   = 1.35*573. * Units.kts
    segment.air_speed_end     = 0.95*573.  * Units.kts 
    mission.append_segment(segment)     
    
    # ------------------------------------------------------------------
    #   First Descent Segment
    # ------------------------------------------------------------------    
      
    segment = Segments.Descent.Linear_Mach_Constant_Rate(base_segment)
    segment.tag = "descent_2" 
    segment.analyses.extend( analyses.cruise )
    segment.altitude_end      = 10000. * Units.ft
    segment.mach_number_start = 0.95
    segment.mach_number_end   = 250./638. 
    segment.descent_rate      = 2000. * Units['ft/min'] 
    mission.append_segment(segment)     
    
    # ------------------------------------------------------------------
    #   First Descent Segment
    # ------------------------------------------------------------------     
    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_3" 
    segment.analyses.extend( analyses.cruise )
    segment.altitude_end = 0. * Units.ft
    segment.air_speed    = 250. * Units.kts
    segment.descent_rate = 1000. * Units['ft/min'] 
    mission.append_segment(segment)      
    
    # ------------------------------------------------------------------    
    #   Mission definition complete    
    # ------------------------------------------------------------------
    
    return mission

def missions_setup(mission):

    missions     = RCAIDE.Analyses.Mission.Missions() 
    mission.tag  = 'base_mission'
    missions.append(mission)
    
    # done!
    return missions  
    
def check_results(new_results,old_results):

    # check segment values
    check_list = [
        'segments.cruise.conditions.aerodynamics.angle_of_attack',
        'segments.cruise.conditions.aerodynamics.drag_coefficient',
        'segments.cruise.conditions.aerodynamics.lift_coefficient',
        'segments.cruise.conditions.propulsion.throttle',
        'segments.cruise.conditions.weights.vehicle_mass_rate',
        'fuel_tank_test.masses',
        'fuel_tank_test.cg_mins',
        'fuel_tank_test.cg_maxes',
    ]

    # do the check
    for k in check_list:
        print(k)

        old_val = np.max( old_results.deep_get(k) )
        new_val = np.max( new_results.deep_get(k) )
        err = (new_val-old_val)/old_val
        print('Error at Max:' , err)
        assert np.abs(err) < 1e-6 , 'Max Check Failed : %s' % k

        old_val = np.min( old_results.deep_get(k) )
        new_val = np.min( new_results.deep_get(k) )
        err = (new_val-old_val)/old_val
        print('Error at Min:' , err)
        assert np.abs(err) < 1e-6 , 'Min Check Failed : %s' % k        

        print('') 

    return


def load_results():
    return RCAIDE.Input_Output.RCAIDE.load('results_mission_concorde.res')

def save_results(results):
    RCAIDE.Input_Output.RCAIDE.archive(results,'results_mission_concorde.res')
    return    
        
if __name__ == '__main__': 
    main()    
    plt.show()
        
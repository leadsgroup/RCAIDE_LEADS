# Regression/scripts/Tests/network_electric/battery_electric_aircraft_test.py
# 
# 
# Created:  Sep 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units  
from RCAIDE.Library.Plots  import *
from RCAIDE.Library.Methods.Performance.estimate_stall_speed        import estimate_stall_speed 

# python imports     
import sys
import matplotlib.pyplot as  plt
import os

# local imports 
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Electric_Twin_Otter    import vehicle_setup, configs_setup 


# ----------------------------------------------------------------------------------------------------------------------
#  REGRESSION
# ----------------------------------------------------------------------------------------------------------------------  
def main():           
         
    battery_types = ['lithium_ion_nmc', 'lithium_ion_lfp']
    btms_types    = ['Liquid_Cooled_Wavy_Channel', 'Air_Cooled', None] 
    CL_true       = [[ 0.8067527163616551, 0.8067527163616532 , 0.8067527163616532],
                     [ 0.8067527163616521, 0.806752716361654  , 0.806752716361654]] 
    # vehicle data
    for i , battery_type in enumerate(battery_types):
        for j , btms_type in enumerate(btms_types):
            vehicle  = vehicle_setup(battery_type, btms_type) 
        
            # Set up configs
            configs  = configs_setup(vehicle)
        
            # vehicle analyses
            analyses = analyses_setup(configs)
        
            # mission analyses
            mission  = mission_setup(analyses)
            missions = missions_setup(mission) 
             
            results = missions.base_mission.evaluate()
            
            CL    = results.segments.climb.conditions.aerodynamics.coefficients.lift.total[0, 0]
            print('****************************************')
            print('Computed value of coefficient of lift is:', CL)
            error =  abs(CL - CL_true[i][j]) /CL_true[i][j]
            assert(abs(error)<1e-6)
             
            if i ==  0 and  j == 0: 
                # plot the results 
                plot_results(results)

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
    
    #  Geometry
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.vehicle = vehicle
    geometry.settings.overwrite_reference        = False
    geometry.settings.update_wing_properties     = True
    analyses.append(geometry)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis  
    aerodynamics          = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.vehicle  = vehicle 
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    energy          = RCAIDE.Framework.Analyses.Energy.Energy()
    energy.vehicle  = vehicle 
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
    

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission' 

    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments  
    base_segment = Segments.Segment()
    base_segment.temperature_deviation  = 2.5
    base_segment.state.numerics.number_of_control_points  = 3
    
    # VSTALL Calculation  
    vehicle        = analyses.base.aerodynamics.vehicle
    vehicle_mass   = vehicle.mass_properties.max_takeoff
    reference_area = vehicle.reference_area 
    Vstall         = estimate_stall_speed(vehicle_mass,reference_area,altitude = 0.0,maximum_lift_coefficient = 1.2)
    
    # ------------------------------------------------------------------
    #   Departure End of Runway Segment  
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    segment.tag = 'climb'       
    segment.analyses.extend( analyses.base )  
    segment.altitude_start                                           = 0.0 * Units.feet
    segment.altitude_end                                             = 5 
    segment.air_speed_start                                          = Vstall *1.2  
    segment.air_speed_end                                            = Vstall *1.25
    segment.initial_battery_state_of_charge                          = 1.0
                       
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
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
 
    missions         = RCAIDE.Framework.Mission.Missions()
    
    # base mission 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions 

def plot_results(results):
    # Plots fligh conditions 
    plot_flight_conditions(results) 
    
    # Plot arcraft trajectory
    plot_flight_trajectory(results)   

    plot_propulsor_throttles(results)
    
    # Plot Aircraft Electronics
    plot_battery_module_conditions(results) 
    plot_battery_temperature(results)
    plot_battery_cell_conditions(results) 
    plot_battery_module_C_rates(results)
    plot_battery_degradation(results) 
    
    # Plot Propeller Conditions 
    plot_rotor_conditions(results) 
    plot_disc_and_power_loading(results)
    
    # Plot Electric Motor and Propeller Efficiencies 
    plot_electric_propulsor_efficiencies(results)
    
    plot_thermal_management_performance(results)
    return


if __name__ == '__main__': 
    main()    
    plt.show()

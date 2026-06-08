
""" setup file for a cruise segment of the NASA X-57 Maxwell (Twin Engine Variant) Electric Aircraft
"""
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE Imports 
import RCAIDE
from RCAIDE.Framework.Core import Units , Data  
from RCAIDE.Library.Plots import *

# Python imports
import matplotlib.pyplot as plt  
import sys 
import os
import numpy as np     
import time

base_dir = os.path.dirname(os.path.abspath(__file__))

vehicles_path = os.path.abspath(
    os.path.join(base_dir, "..", "..", "Vehicles")
)

if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)
from NASA_X57    import vehicle_setup, configs_setup     
 
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    # fidelity zero wakes
    print('Wake Fidelity Zero, Identical Props')     
    Propeller_Slipstream(wake_fidelity=0,identical_props=False) 
    
    return


def Propeller_Slipstream(wake_fidelity,identical_props): 

    rotor_type = 'Blade_Element_Momentum_Theory_Helmholtz_Wake'
    vehicle  = vehicle_setup(rotor_type)      
 
    vehicle.networks.electric.propulsors.starboard_propulsor.rotor.clockwise_rotation = True
    vehicle.networks.electric.propulsors.port_propulsor.rotor.clockwise_rotation = False
     
    configs  = configs_setup(vehicle) 
    analyses = analyses_setup(configs)  
    mission  = mission_setup(analyses)
    missions = missions_setup(mission) 
    results  = missions.base_mission.evaluate()    
       
    # Regression for Stopped Rotor Test (using Fidelity Zero wake model)
    lift_coefficient            = results.segments.cruise.conditions.aerodynamics.coefficients.lift.total[1][0]
    sectional_lift_coeff        = results.segments.cruise.conditions.aerodynamics.coefficients.lift.spanwise[0,0:40]
    
    # lift coefficient and sectional lift coefficient check
    lift_coefficient_true       = 0.7944994554508983
    sectional_lift_coeff_true   = np.array([0.7147468 , 0.67342342, 0.51334893, 0.84067887, 0.6836172 ,
                                            0.63624763, 0.58749353, 0.523184  , 0.36028242, 0.10329725,
                                            0.71474679, 0.67342348, 0.51334911, 0.84067859, 0.68361625,
                                            0.63624639, 0.5874937 , 0.52318651, 0.3602854 , 0.10329809,
                                            0.03767618, 0.03852107, 0.04029459, 0.04257655, 0.04420888,
                                            0.044047  , 0.04128749, 0.03535978, 0.0263186 , 0.01570089,
                                            0.03767618, 0.03852107, 0.04029459, 0.04257654, 0.04420887,
                                            0.044047  , 0.04128751, 0.03535972, 0.02631842, 0.01570065])

    diff_CL = np.abs(lift_coefficient  - lift_coefficient_true) / lift_coefficient_true
    print('CL difference')
    print(diff_CL)

    diff_Cl_y   = np.max(np.abs((sectional_lift_coeff - sectional_lift_coeff_true) /sectional_lift_coeff_true))
    print('Sectional Cl difference')
    print(diff_Cl_y)
    
    assert diff_CL< 1e-6
    assert diff_Cl_y < 1e-6

    # plot results, vehicle, and vortex distribution
    plot_mission(results) 
              
    return
 

def plot_mission(results):
    
    # Plot lift distribution
    plot_lift_distribution(results) 
    return 
 
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
    analyses = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle =  vehicle

    # ------------------------------------------------------------------     
    #  Weights
    weights = RCAIDE.Framework.Analyses.Weights.Electric_General_Aviation()
    analyses.append(weights)  

    # ------------------------------------------------------------------     
    #  Geometry
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    analyses.append(geometry)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics                               = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()  
    aerodynamics.settings.number_of_spanwise_vortices    = 10 # reducing the number of vortices to speed up the test 
    aerodynamics.settings.number_of_chordwise_vortices   = 5  # reducing the number of vortices to speed up the test 
    aerodynamics.settings.use_surrogate        = False 
    aerodynamics.settings.propeller_wake_model = True
    analyses.append(aerodynamics)   
  

    # ------------------------------------------------------------------
    #  Energy
    energy          = RCAIDE.Framework.Analyses.Energy.Energy() 
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
#  Set Up Mission 
# ---------------------------------------------------------------------- 
def mission_setup(analyses):
    

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------ 
 
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'
  
    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments

    #   Cruise Segment: constant Speed, constant altitude 
    segment                           = Segments.Untrimmed.Untrimmed()
    segment.analyses.extend( analyses.base ) 
    segment.tag = "cruise"    
    segment.initial_battery_state_of_charge              = 1.0       
    segment.altitude                                     = 30
    segment.air_speed                                    = 100
    segment.distance                                     = 1 * Units.miles
    segment.angle_of_attack                              = 3 *  Units.degrees
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
       
    mission.append_segment(segment)      
     
    return mission

# ----------------------------------------------------------------------
#  Set Up Missions 
# ---------------------------------------------------------------------- 
def missions_setup(mission): 
 
    missions     = RCAIDE.Framework.Mission.Missions() 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions  
 

if __name__ == '__main__':
    main()
    plt.show()

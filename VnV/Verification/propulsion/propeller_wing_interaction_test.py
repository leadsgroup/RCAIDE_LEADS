
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

sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
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
    sectional_lift_coeff        = results.segments.cruise.conditions.aerodynamics.coefficients.lift.inviscid.spanwise[0]
    
    # lift coefficient and sectional lift coefficient check
    lift_coefficient_true       = 0.7471442722450158
    sectional_lift_coeff_true   = np.array([7.11702984e-01, 6.91720619e-01, 6.48390130e-01, 5.20527453e-01,
                                            6.20553156e-01, 6.70745402e-01, 6.27849971e-01, 6.15422147e-01,
                                            5.97612552e-01, 5.72167349e-01, 5.37742801e-01, 4.92328554e-01,
                                            4.26108478e-01, 9.02284446e-02, 1.01668869e-01, 7.11702982e-01,
                                            6.91720737e-01, 6.48390436e-01, 5.20527957e-01, 6.20553804e-01,
                                            6.70746628e-01, 6.27851015e-01, 6.15423252e-01, 5.97612484e-01,
                                            5.72165425e-01, 5.37740845e-01, 4.92327347e-01, 4.26107493e-01,
                                            9.02282009e-02, 1.01668538e-01, 2.55344378e-02, 2.60801812e-02,
                                            2.71392250e-02, 2.86075232e-02, 3.03048140e-02, 3.20152237e-02,
                                            3.34897142e-02, 3.44198192e-02, 3.44663431e-02, 3.33326640e-02,
                                            3.08266897e-02, 2.69016374e-02, 2.16812251e-02, 1.54802117e-02,
                                            9.08448588e-03, 2.55344523e-02, 2.60802148e-02, 2.71392800e-02,
                                            2.86075780e-02, 3.03048365e-02, 3.20152223e-02, 3.34897236e-02,
                                            3.44198658e-02, 3.44664456e-02, 3.33328233e-02, 3.08268350e-02,
                                            2.69017405e-02, 2.16813354e-02, 1.54803762e-02, 9.08480152e-03,
                                            6.62571210e-17, 5.84147883e-16, 1.05090799e-15, 1.09055515e-15,
                                            1.68211654e-15, 2.64490362e-15, 3.27207240e-15, 3.74732084e-15,
                                            4.22264533e-15, 4.05347485e-15, 3.79628797e-15, 3.29544394e-15,
                                            2.62388100e-15, 1.85539643e-15, 1.08013074e-15])

    diff_CL = np.abs(lift_coefficient  - lift_coefficient_true)
    print('CL difference')
    print(diff_CL)

    diff_Cl_y   = max(np.abs(sectional_lift_coeff - sectional_lift_coeff_true))
    print('Cl difference')
    print(diff_Cl_y)
    
    assert diff_CL/lift_coefficient < 1e-6
    assert diff_Cl_y < 1e-6

    # plot results, vehicle, and vortex distribution
    plot_mission(results) 
              
    return
 

def plot_mission(results):

    # Plot surface pressure coefficient
    plot_surface_pressures(results)

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

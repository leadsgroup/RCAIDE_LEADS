
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
    sectional_lift_coeff        = results.segments.cruise.conditions.aerodynamics.coefficients.lift.spanwise[0]
    
    # lift coefficient and sectional lift coefficient check
    lift_coefficient_true       = 0.836636061966205
    sectional_lift_coeff_true   = np.array([ 7.69776776e-01,  7.64844290e-01,  7.55979424e-01,  7.42883640e-01,
        7.24154446e-01,  6.95287971e-01,  6.22866332e-01,  6.04731657e-01,
        6.86611378e-01,  7.25693813e-01,  8.11436885e-01,  7.88350320e-01,
        7.28903709e-01,  7.06943841e-01,  6.91012533e-01,  6.76682591e-01,
        6.62349605e-01,  6.47194837e-01,  6.30712685e-01,  6.12558483e-01,
        5.92498422e-01,  5.70386658e-01,  5.46103783e-01,  5.19322500e-01,
        4.88722067e-01,  4.48309282e-01,  1.24473483e-01,  1.01680273e-01,
        1.03015268e-01,  9.01492015e-02,  7.69776927e-01,  7.64844693e-01,
        7.55980128e-01,  7.42884158e-01,  7.24154382e-01,  6.95287818e-01,
        6.22866518e-01,  6.04732494e-01,  6.86612882e-01,  7.25694923e-01,
        8.11436509e-01,  7.88347623e-01,  7.28897687e-01,  7.06934607e-01,
        6.90999303e-01,  6.76664110e-01,  6.62327023e-01,  6.47171123e-01,
        6.30688779e-01,  6.12533220e-01,  5.92472910e-01,  5.70365460e-01,
        5.46088740e-01,  5.19311490e-01,  4.88715026e-01,  4.48304458e-01,
        1.24471885e-01,  1.01678568e-01,  1.03013343e-01,  9.01473126e-02,
        2.07879118e-02,  2.09178582e-02,  2.11741188e-02,  2.15495994e-02,
        2.20339267e-02,  2.26136836e-02,  2.32725223e-02,  2.39913210e-02,
        2.47482956e-02,  2.55187924e-02,  2.62753834e-02,  2.69879433e-02,
        2.76238744e-02,  2.81486267e-02,  2.85263898e-02,  2.87212396e-02,
        2.86985575e-02,  2.84266438e-02,  2.78786002e-02,  2.70338155e-02,
        2.58793147e-02,  2.44110154e-02,  2.26340003e-02,  2.05625045e-02,
        1.82193892e-02,  1.56354845e-02,  1.28494183e-02,  9.90901819e-03,
        6.88268921e-03,  3.99718040e-03,  2.07878637e-02,  2.09177178e-02,
        2.11738776e-02,  2.15492593e-02,  2.20335151e-02,  2.26131968e-02,
        2.32719716e-02,  2.39907537e-02,  2.47477068e-02,  2.55181529e-02,
        2.62747028e-02,  2.69872743e-02,  2.76232592e-02,  2.81480844e-02,
        2.85259177e-02,  2.87208455e-02,  2.86982626e-02,  2.84264937e-02,
        2.78785771e-02,  2.70337868e-02,  2.58792484e-02,  2.44109638e-02,
        2.26339936e-02,  2.05625684e-02,  1.82195633e-02,  1.56358526e-02,
        1.28500313e-02,  9.90968451e-03,  6.88304168e-03,  3.99696154e-03,
       -3.40173350e-15, -4.08548964e-15, -3.01452590e-15,  1.71712092e-15,
        1.28718172e-14,  1.20383104e-14,  1.19620244e-14,  1.16211266e-14,
        1.06541495e-14,  9.58824531e-15,  8.12909030e-15,  6.59432211e-15,
        5.08232530e-15,  3.71344081e-15,  2.63395896e-15,  1.45468852e-15,
        3.80731146e-16, -4.70817675e-16, -1.40998841e-15, -1.97614181e-15,
       -2.44595997e-15, -2.78960263e-15, -3.02544492e-15, -3.07070862e-15,
       -2.98050111e-15, -2.72969795e-15, -2.36301903e-15, -1.86525301e-15,
       -1.32964515e-15, -7.84984629e-16])

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

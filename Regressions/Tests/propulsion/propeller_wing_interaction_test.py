
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
    t0=time.time()
    Propeller_Slipstream(wake_fidelity=0,identical_props=True)
    print((time.time()-t0)/60) 
    
    return


def Propeller_Slipstream(wake_fidelity,identical_props): 

    rotor_type = 'Blade_Element_Momentum_Theory_Helmholtz_Wake'
    vehicle  = vehicle_setup(rotor_type)      

    for network in vehicle.networks: 
        network.identical_propulsors  = identical_props
        for propulsor in network.propulsors:
            propeller = propulsor.rotor 
            propeller.rotation = -1 
                
    
    configs  = configs_setup(vehicle) 
    analyses = analyses_setup(configs)  
    mission  = mission_setup(analyses)
    missions = missions_setup(mission) 
    results  = missions.base_mission.evaluate()    
       
    # Regression for Stopped Rotor Test (using Fidelity Zero wake model)
    lift_coefficient            = results.segments.cruise.conditions.aerodynamics.coefficients.lift.total[1][0]
    sectional_lift_coeff        = results.segments.cruise.conditions.aerodynamics.coefficients.lift.induced.spanwise[0]
    
    # lift coefficient and sectional lift coefficient check
    lift_coefficient_true       = 0.7809749834076547
    sectional_lift_coeff_true   = np.array([ 7.43027382e-01,  7.15462535e-01,  6.68448360e-01,  5.40473203e-01,
                                             6.27070678e-01,  6.41788664e-01,  6.24880912e-01,  6.16008262e-01,
                                             5.99319728e-01,  5.74243878e-01,  5.39872477e-01,  4.94350194e-01,
                                             4.27896668e-01,  9.06442717e-02,  1.02171504e-01,  7.56664506e-01,
                                             7.61899333e-01,  7.71912032e-01,  8.39930368e-01,  7.08168647e-01,
                                             6.36287433e-01,  6.47994273e-01,  6.36057724e-01,  6.15838739e-01,
                                             5.87912834e-01,  5.51303366e-01,  5.03979912e-01,  4.35875455e-01,
                                             9.24893041e-02,  1.04374261e-01,  2.43900661e-02,  2.62399895e-02,
                                             2.80462928e-02,  2.99314879e-02,  3.18465486e-02,  3.36324263e-02,
                                             3.50772347e-02,  3.59043746e-02,  3.58004488e-02,  3.44893246e-02,
                                             3.17943040e-02,  2.76781719e-02,  2.22690699e-02,  1.58837422e-02,
                                             9.31779416e-03,  2.24601734e-02,  2.06641069e-02,  1.89332581e-02,
                                             1.70430343e-02,  1.48797665e-02,  1.24464423e-02,  9.82790877e-03,
                                             7.17355307e-03,  4.67378801e-03,  2.52259204e-03,  8.75287822e-04,
                                            -1.88446122e-04, -6.83986682e-04, -7.16271522e-04, -4.69917618e-04,
                                             5.61519907e-07,  1.23558037e-06,  1.38861859e-06,  1.89159508e-06,
                                             2.53936162e-06,  3.13357041e-06,  3.58822055e-06,  3.87262133e-06,
                                             3.97712854e-06,  3.89395040e-06,  3.60457768e-06,  3.07170592e-06,
                                             2.25920396e-06,  1.26200459e-06,  4.52350290e-07])

    diff_CL = np.abs(lift_coefficient  - lift_coefficient_true)
    print('CL difference')
    print(diff_CL)

    diff_Cl   = np.abs(sectional_lift_coeff - sectional_lift_coeff_true)
    print('Cl difference')
    print(diff_Cl)
    
    assert np.abs(lift_coefficient  - lift_coefficient_true) < 1e-2
    assert  np.max(np.abs(sectional_lift_coeff - sectional_lift_coeff_true)) < 1e-2

    # plot results, vehicle, and vortex distribution
    plot_mission(results)
    plot_3d_vehicle_vlm_panelization(vehicle,
                                     show_figure= False,
                                     save_figure=False,
                                     show_wing_control_points=True)
              
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
 
    # ------------------------------------------------------------------
    #  Weights
    weights         = RCAIDE.Framework.Analyses.Weights.Electric()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics                               = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.vehicle                       = vehicle
    aerodynamics.settings.use_surrogate        = False 
    aerodynamics.settings.propeller_wake_model = True
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

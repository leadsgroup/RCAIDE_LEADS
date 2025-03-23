
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
    sectional_lift_coeff_true   = np.array([7.54785254e-01, 7.48000594e-01, 7.38124290e-01, 7.41384407e-01,
                                            6.81702447e-01, 6.42686690e-01, 6.41426118e-01, 6.30083072e-01,
                                            6.10895528e-01, 5.83840480e-01, 5.47917287e-01, 5.01141085e-01,
                                            4.33530669e-01, 9.19464595e-02, 1.03725608e-01, 7.51751645e-01,
                                            7.37645340e-01, 7.14923813e-01, 6.73337053e-01, 6.65532597e-01,
                                            6.55121051e-01, 6.39933250e-01, 6.27807066e-01, 6.08719470e-01,
                                            5.81905342e-01, 5.46232788e-01, 4.99689412e-01, 4.32313111e-01,
                                            9.16648546e-02, 1.03388119e-01, 2.38241363e-02, 2.33768195e-02,
                                            2.29066443e-02, 2.23257585e-02, 2.15508210e-02, 2.05307353e-02,
                                            1.92495232e-02, 1.77131715e-02, 1.59441592e-02, 1.39820017e-02,
                                            1.18757619e-02, 9.67156638e-03, 7.40699400e-03, 5.12230343e-03,
                                            2.96994804e-03, 2.42759895e-02, 2.46835954e-02, 2.50421710e-02,
                                            2.53414511e-02, 2.55123770e-02, 2.54668782e-02, 2.51210461e-02,
                                            2.43832907e-02, 2.31607568e-02, 2.13854041e-02, 1.90304420e-02,
                                            1.61157160e-02, 1.27124196e-02, 8.95820553e-03, 5.23155776e-03,
                                            2.78949028e-08, 6.14589037e-08, 6.79571663e-08, 9.23039951e-08,
                                            1.23618416e-07, 1.52146032e-07, 1.73729803e-07, 1.86958417e-07,
                                            1.91469021e-07, 1.86985395e-07, 1.72704181e-07, 1.46901017e-07,
                                            1.07881108e-07, 6.01765460e-08, 2.15224565e-08])

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

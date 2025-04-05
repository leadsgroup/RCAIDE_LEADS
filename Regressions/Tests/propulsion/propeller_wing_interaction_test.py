
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
    sectional_lift_coeff_true   = np.array([7.59385503e-01, 7.59944841e-01, 7.63353100e-01, 8.14141690e-01,
                                            7.01223610e-01, 6.42235327e-01, 6.47070388e-01, 6.34965315e-01,
                                            6.14946412e-01, 5.87218574e-01, 5.50760649e-01, 5.03547689e-01,
                                            4.35530303e-01, 9.24085758e-02, 1.04277209e-01, 7.50549741e-01,
                                            7.29812303e-01, 6.95986806e-01, 6.17559984e-01, 6.52018943e-01,
                                            6.65818347e-01, 6.38610162e-01, 6.25874588e-01, 6.06919777e-01,
                                            5.80337596e-01, 5.44888278e-01, 4.98542170e-01, 4.31356569e-01,
                                            9.14433237e-02, 1.03122528e-01, 2.37207711e-02, 2.24575496e-02,
                                            2.11723503e-02, 1.97043155e-02, 1.79589437e-02, 1.59241539e-02,
                                            1.36545309e-02, 1.12579987e-02, 8.87836699e-03, 6.66942318e-03,
                                            4.76227132e-03, 3.23374075e-03, 2.08928554e-03, 1.26989159e-03,
                                            6.96985388e-04, 2.50114360e-02, 2.61888383e-02, 2.72702345e-02,
                                            2.83203377e-02, 2.92863547e-02, 3.00495349e-02, 3.04687624e-02,
                                            3.03709349e-02, 2.95676335e-02, 2.79027601e-02, 2.52887844e-02,
                                            2.17263139e-02, 1.73167830e-02, 1.22807765e-02, 7.18900042e-03,
                                            2.36176114e-07, 5.20216058e-07, 5.79267948e-07, 7.87659512e-07,
                                            1.05579269e-06, 1.30071533e-06, 1.48683369e-06, 1.60182715e-06,
                                            1.64224134e-06, 1.60537757e-06, 1.48405420e-06, 1.26324771e-06,
                                            9.28278966e-07, 5.18139114e-07, 1.85526669e-07])

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

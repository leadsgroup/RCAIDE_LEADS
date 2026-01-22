'''
# Plots.py
# 
# Created: May 2019, M Clarke
#          Sep 2020, M. Clarke 

'''
#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------  

from RCAIDE.Visualization.Performance.Aerodynamics.Vehicle                 import *  
from RCAIDE.Visualization.Performance.Mission                              import *  
from RCAIDE.Visualization.Performance.Aerodynamics.Rotor import *  
from RCAIDE.Visualization.Performance.Energy.Battery                       import *   
from RCAIDE.Visualization.Performance.Noise                                import *   
from RCAIDE.Visualization.Geometry                                         import *  

# ----------------------------------------------------------------------
#   Plot Results
# ----------------------------------------------------------------------  
def plot_results(results,run_noise_model,save_figure_flag):  
    
    plot_flight_conditions(results) 
    
    # Plot Aerodynamic Coefficients
    plot_aerodynamic_coefficients(results)  
    
    # Plot Aircraft Flight Speed
    plot_aircraft_velocities(results) 

    # Plot Aircraft Stability
    plot_stability_coefficients(results) 
    
    # Plot Aircraft Electronics
    plot_battery_pack_conditions(results)
    
    # Plot Propeller Conditions 
    plot_rotor_conditions(results) 
    
    # Plot Electric Motor and Propeller Efficiencies 
    plot_electric_motor_and_rotor_efficiencies(results)
    
    # Plot rotor Disc and Power Loading
    plot_disc_power_loading(results)   
    
    # Plot Battery Degradation  
    plot_battery_degradation(results)   
    
    if run_noise_model: 
        # Plot noise level
        plot_ground_noise_levels(results)
        
        # Plot noise contour
        plot_flight_profile_noise_contours(results) 
                        
    return

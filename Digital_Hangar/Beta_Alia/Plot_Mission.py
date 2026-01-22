# Plot_Mission.py
# 
# Created: Dec 2021, E. Botero
# Modified: 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

from RCAIDE.Plots.Performance.Energy.Battery import *
from RCAIDE.Plots.Performance.Energy.Common import *
from RCAIDE.Plots.Performance.Aerodynamics import *
from RCAIDE.Plots.Performance.Mission import *

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(nexus):

    results_range   = nexus.results.base
    #results_sprint  = nexus.results.sprint

    # Plot Flight Conditions
    plot_flight_conditions(results_range)

    # Plot Aerodynamic Coefficients
    plot_aerodynamic_coefficients(results_range)

    # Plot Aircraft Flight Speed
    plot_aircraft_velocities(results_range)

    # Plot Aircraft Electronics
    plot_battery_pack_conditions(results_range)

    # Plot Propeller Conditions
    plot_lift_cruise_network(results_range)
    
    
    ## Plot the sprint conditions
    
    ## Plot Flight Conditions
    #plot_flight_conditions(results_sprint, line_style)

    ## Plot Aerodynamic Coefficients
    #plot_aerodynamic_coefficients(results_sprint, line_style)

    ## Plot Aircraft Flight Speed
    #plot_aircraft_velocities(results_sprint, line_style)

    ## Plot Aircraft Electronics
    #plot_battery_pack_conditions(results_sprint, line_style)

    ## Plot Propeller Conditions
    #plot_propeller_conditions(results_sprint, line_style)

    ## Plot Electric Motor and Propeller Efficiencies
    #plot_eMotor_Prop_efficiencies(results_sprint, line_style)

    ## Plot propeller Disc and Power Loading
    ##plot_disc_power_loading(results_sprint, line_style)    

    return

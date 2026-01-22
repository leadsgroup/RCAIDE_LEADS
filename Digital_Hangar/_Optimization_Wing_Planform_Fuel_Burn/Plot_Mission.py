# Plot_Mission.py
# 
# Created:  May 2015, E. Botero
# Modified: 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    
from RCAIDE.Library.Plots  import *    

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(nexus):

    results   = nexus.results.base

    # Plot Aerodynamic Coefficients 
    plot_aerodynamic_coefficients(results)
    
    # Drag Components
    plot_drag_components(results)

    # Plot Altitude, sfc, vehicle weight 
    plot_altitude_sfc_weight(results)

    return
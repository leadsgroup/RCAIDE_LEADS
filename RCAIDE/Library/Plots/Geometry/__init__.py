## @defgroup Library-Plots-Geometry Geometry
# RCAIDE/Library/Plots/Geometry/__init__.py
# 

"""
RCAIDE Package Setup for Geometry Plotting Functions

This module provides a collection of functions for visualizing vehicle geometry and components in 2D and 3D.

See Also
--------
RCAIDE.Library.Plots.Energy : Energy system visualization functions
RCAIDE.Library.Plots.Performance : Vehicle performance visualization functions
RCAIDE.Library.Plots.Mission : Mission profile visualization functions
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# @ingroup Visualization

from .Common            import * 
from .plot_3d_vehicle                  import plot_3d_vehicle
from .plot_3d_vehicle                  import plot_3d_energy_network 
from .plot_3d_vehicle                  import generate_3d_vehicle_geometry_data 
from .plot_3d_rotor                    import plot_3d_rotor
from .plot_3d_rotor                    import generate_3d_blade_points 
from .plot_3d_nacelle                  import plot_3d_nacelle 
from .plot_3d_nacelle                  import generate_3d_basic_nacelle_points
from .plot_3d_nacelle                  import generate_3d_BOR_nacelle_points
from .plot_3d_nacelle                  import generate_3d_stack_nacelle_points
from .plot_3d_wing                     import plot_3d_wing
from .plot_3d_wing                     import generate_3d_wing_points
from .plot_3d_vehicle_vlm_panelization import plot_3d_vehicle_vlm_panelization
from .plot_layout_of_passenger_accommodations import plot_layout_of_passenger_accommodations
from .plot_airfoil                     import plot_airfoil
from .plot_rotor                       import plot_rotor 
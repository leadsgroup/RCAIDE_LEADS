# RCAIDE/Library/Methods/Mass_Properties/Moment_of_Inertia/__init__.py 

""" RCAIDE Package Setup
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from .compute_cabin_moment_of_inertia                   import compute_cabin_moment_of_inertia
from .compute_cuboid_moment_of_inertia                  import compute_cuboid_moment_of_inertia
from .compute_bwb_moment_of_inertia                     import compute_bwb_moment_of_inertia
from .compute_fuselage_moment_of_inertia                import compute_fuselage_moment_of_inertia
from .compute_fuselage_integral_tank_moment_of_inertia  import compute_fuselage_integral_tank_moment_of_inertia
from .compute_cylinder_moment_of_inertia                import compute_cylinder_moment_of_inertia
from .compute_rounded_end_cylinder_moment_of_inertia    import compute_rounded_end_cylinder_moment_of_inertia
from .compute_wing_integral_tank_moment_of_inertia      import compute_wing_integral_tank_moment_of_inertia
from .compute_vehicle_moment_of_inertia                 import compute_vehicle_moment_of_inertia
from .compute_wing_moment_of_inertia                    import compute_wing_moment_of_inertia, compute_wing_section_moment_of_intertia
from .update_moments_of_inertia                         import * 
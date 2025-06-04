# RCAIDE/Methods/Aerodynamics/Common/Drag/__init__.py
# 

"""
Common drag calculation methods for aerodynamic analysis across multiple frameworks.

This module provides fundamental drag computation utilities including parasite 
drag estimation, compressibility corrections, and component-specific drag 
calculations. These methods serve as building blocks for comprehensive drag 
analysis in various aerodynamic frameworks within RCAIDE.

See Also
--------
RCAIDE.Library.Methods.Aerodynamics.Common : Parent common aerodynamics module
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

from .parasite_drag_wing                        import parasite_drag_wing
from .parasite_drag_pylon                       import parasite_drag_pylon
from .parasite_total                            import parasite_total
from .induced_drag                              import induced_drag
from .wave_drag                                 import wave_drag 
from .lift_wave_drag                            import lift_wave_drag 
from .parasite_drag_fuselage                    import parasite_drag_fuselage
from .compressibility_drag                      import compressibility_drag
from .miscellaneous_drag                        import miscellaneous_drag  
from .parasite_drag_nacelle                     import parasite_drag_nacelle   
from .spoiler_drag                              import spoiler_drag 
from .wave_drag                                 import wave_drag 
from .total_drag                                import total_drag 
from .drag_divergence                           import drag_divergence
from .supersonic_wave_drag_volume_raymer        import supersonic_wave_drag_volume_raymer
from .supersonic_wave_drag_volume_sears_haack   import supersonic_wave_drag_volume_sears_haack
from .cooling_drag                              import cooling_drag
from .windmilling_drag                          import windmilling_drag
from .asymmetry_drag                            import asymmetry_drag
from .wave_drag_lift                            import wave_drag_lift
from .compressible_mixed_flat_plate             import compressible_mixed_flat_plate
from .estimate_2ndseg_lift_drag_ratio           import estimate_2ndseg_lift_drag_ratio
from .compressible_turbulent_flat_plate         import compressible_turbulent_flat_plate
from .compressible_mixed_flat_plate             import compressible_mixed_flat_plate
# RCAIDE/Components/Wings/Control_Surfaces/Flaperon.py
#
# Created:  Aug 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from .Control_Surface import Control_Surface

# ----------------------------------------------------------------------------------------------------------------------
#  Flaperon
# ----------------------------------------------------------------------------------------------------------------------
class Flaperon(Control_Surface):
    """
    A compound control surface combining flap (lift augmentation) and aileron (roll)
    authority on a single physical hinge.

    Attributes
    ----------
    tag : str
        Unique identifier for the flaperon, defaults to 'flaperon'

    hinge_fraction : float
        Location of the hinge line as fraction of chord, defaults to 0.0

    sign_duplicate : float
        Sign convention applied to the primary (flap) deflection, defaults to 1.0
        (synchronized deflection, matching Flap). The secondary (roll) deflection's
        antisymmetric mirroring is applied downstream in the VLM panel-geometry code,
        not via this field.

    Notes
    -----
    `deflection` (inherited) carries the flap (symmetric) command; `secondary_deflection`
    (inherited) carries the aileron (antisymmetric) command. Both are independent
    pilot/trim inputs mixed onto one physical panel deflection per wing side.

    See Also
    --------
    RCAIDE.Library.Components.Wings.Control_Surfaces.Control_Surface
        Base class providing common control surface functionality
    RCAIDE.Library.Components.Wings.Control_Surfaces.Flap
        Lift-augmentation-only implementation
    RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron
        Roll-only control implementation
    """

    def __defaults__(self):
        """
        Sets default values for the flaperon attributes.

        Notes
        -----
        See Control_Surface.__defaults__ for additional inherited attributes.
        """
        self.tag            = 'flaperon'
        self.hinge_fraction = 0.0
        self.sign_duplicate = 1.0

        pass

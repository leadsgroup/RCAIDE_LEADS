# RCAIDE/Components/Wings/Control_Surfaces/Elevon.py
#
# Created:  Aug 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from .Control_Surface import Control_Surface

# ----------------------------------------------------------------------------------------------------------------------
#  Elevon
# ----------------------------------------------------------------------------------------------------------------------
class Elevon(Control_Surface):
    """
    A compound control surface combining elevator (pitch) and aileron (roll) authority
    on a single physical hinge, as used on tailless/delta-wing aircraft.

    Attributes
    ----------
    tag : str
        Unique identifier for the elevon, defaults to 'elevon'

    hinge_fraction : float
        Location of the hinge line as fraction of chord, defaults to 0.0

    sign_duplicate : float
        Sign convention applied to the primary (pitch) deflection, defaults to 1.0
        (synchronized deflection, matching Elevator). The secondary (roll) deflection's
        antisymmetric mirroring is applied downstream in the VLM panel-geometry code,
        not via this field.

    Notes
    -----
    `deflection` (inherited) carries the pitch (symmetric) command; `secondary_deflection`
    (inherited) carries the roll (antisymmetric) command. Both are independent pilot/trim
    inputs mixed onto one physical panel deflection per wing side.

    See Also
    --------
    RCAIDE.Library.Components.Wings.Control_Surfaces.Control_Surface
        Base class providing common control surface functionality
    RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator
        Pitch-only control implementation
    RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron
        Roll-only control implementation
    """

    def __defaults__(self):
        """
        Sets default values for the elevon attributes.

        Notes
        -----
        See Control_Surface.__defaults__ for additional inherited attributes.
        """
        self.tag            = 'elevon'
        self.hinge_fraction = 0.0
        self.sign_duplicate = 1.0

        pass

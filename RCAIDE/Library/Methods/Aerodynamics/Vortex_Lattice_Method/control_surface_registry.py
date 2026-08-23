# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/control_surface_registry.py
#
# Created:  Aug 2026, M. Clarke

"""
Single source of truth for the control-surface mapping used by train_VLM_surrogates.py,
build_VLM_surrogates.py, and evaluate_VLM.py so that every control surface is handled through
one generic code path instead of being branched on by type. Deflection of any control surface
couples into every force/moment axis (lift, drag, X, Y, Z, L, M, N), so all surface types are
treated identically here rather than each being hard-coded to a fixed subset of derivatives.
Mirrors the letter-tag convention already used in
RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.train_AVL_surrogates.

Each row of CONTROL_SURFACE_TYPES is (class, letter, name, channel, flag_attribute, deflection_attribute):
    class                 : the RCAIDE.Library.Components.Wings.Control_Surfaces class this row matches
    letter                : one-letter tag used to build derivative names, e.g. 'dCM_ddelta_<letter>'
    name                  : the control_surfaces.<name> results/conditions block this row writes into
    channel               : the aerodynamics.training.<channel>_deflection sweep array to train against
    flag_attribute        : the aerodynamics.<flag> attribute gating whether this row is active
    deflection_attribute  : which field on the control surface object holds this row's commanded
                             deflection ('deflection' for a primary/symmetric channel,
                             'secondary_deflection' for a secondary/antisymmetric channel)

Simple (single-channel) surfaces have name == channel and one row each. Compound surfaces
(Flaperon, Elevon, Ruddervator) are a single physical hinge that responds to two independent
trim channels at once, so they appear as two rows sharing one `name` (one results block) but
reusing the channel and training data of the two simple surfaces they combine, e.g. Elevon's
primary channel reuses Elevator's training range, its secondary channel reuses Aileron's.
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from RCAIDE.Library.Components.Wings.Control_Surfaces import Aileron, Elevator, Rudder, Flap, Slat, Flaperon, Elevon, Ruddervator

# ----------------------------------------------------------------------------------------------------------------------
#  Control Surface Registry
# ----------------------------------------------------------------------------------------------------------------------
CONTROL_SURFACE_TYPES = [
    (Aileron,     'a',  'aileron',     'aileron',  'aileron_flag',      'deflection'          ),
    (Elevator,    'e',  'elevator',    'elevator', 'elevator_flag',     'deflection'          ),
    (Rudder,      'r',  'rudder',      'rudder',   'rudder_flag',       'deflection'          ),
    (Flap,        'f',  'flap',        'flap',     'flap_flag',         'deflection'          ),
    (Slat,        's',  'slat',        'slat',     'slat_flag',         'deflection'          ),
    (Flaperon,    'pf', 'flaperon',    'flap',     'flaperon_flag',     'deflection'          ),
    (Flaperon,    'sf', 'flaperon',    'aileron',  'flaperon_flag',     'secondary_deflection'),
    (Elevon,      'pe', 'elevon',      'elevator', 'elevon_flag',       'deflection'          ),
    (Elevon,      'se', 'elevon',      'aileron',  'elevon_flag',       'secondary_deflection'),
    (Ruddervator, 'pr', 'ruddervator', 'elevator', 'ruddervator_flag',  'deflection'          ),
    (Ruddervator, 'sr', 'ruddervator', 'rudder',   'ruddervator_flag',  'secondary_deflection'),
]

def lookup(control_surface):
    """
    Returns every registry row matching a control surface instance's type.

    Simple surfaces match exactly one row. Compound surfaces (Flaperon, Elevon, Ruddervator)
    match two rows — one per independent channel they respond to.

    Args:
        control_surface : a RCAIDE.Library.Components.Wings.Control_Surfaces instance

    Returns:
        list of (letter, name, channel, flag_attribute, deflection_attribute) tuples;
        empty list if the control surface type is not registered
    """
    return [(letter, name, channel, flag, deflection_attribute)
            for cls, letter, name, channel, flag, deflection_attribute in CONTROL_SURFACE_TYPES
            if type(control_surface) == cls]

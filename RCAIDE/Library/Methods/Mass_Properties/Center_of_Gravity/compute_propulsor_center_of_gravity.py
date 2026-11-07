# RCAIDE/Library/Methods/Mass_Properties/Center_of_Gravity/compute_propulsor_center_of_gravity.py
#
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  Compute Propulsor Center of Gravity
# ----------------------------------------------------------------------------------------------------------------------
def compute_propulsor_center_of_gravity(propulsor):
    """
    Places an undefined engine and nacelle CG at 52.5 percent of the engine length.

    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Powertrain.Propulsors.Propulsor
        Propulsor whose origin is the engine front

    Notes
    -----
    52.5 percent is the midpoint of the 45-60 percent engine-length range given for the engine/nacelle group. The
    engine length is the propulsor length, or the nacelle length when the propulsor length is not set. Each CG is
    stored relative to its own component origin. A CG already set by the user is left unchanged.

    References
    ----------
    Chai, S., Crisafulli, P., and Mason, W. H., "Aircraft Center of Gravity Estimation in Conceptual/Preliminary
    Design," AIAA Paper 95-3882, 1995, Table 4.
    """
    nacelle        = propulsor.nacelle
    nacelle_length = nacelle.length if nacelle is not None else 0.0
    engine_length  = propulsor.get('length', 0.0)

    if engine_length > 0:
        x_group = propulsor.origin[0][0] + 0.525 * engine_length
    elif nacelle_length > 0:
        x_group = nacelle.origin[0][0] + 0.525 * nacelle_length
    else:
        return

    if propulsor.mass_properties.center_of_gravity[0][0] == 0:
        propulsor.mass_properties.center_of_gravity = [[x_group - propulsor.origin[0][0], 0.0, 0.0]]
    if nacelle is not None and nacelle.mass_properties.center_of_gravity[0][0] == 0:
        nacelle.mass_properties.center_of_gravity = [[x_group - nacelle.origin[0][0], 0.0, 0.0]]
    return

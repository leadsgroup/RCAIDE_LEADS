# RCAIDE/Library/Methods/Powertrain/Converters/EHD_Electrode_Array/compute_electrode_array_mass.py
#
# Created:  Oct 2026, RCAIDE EHD MVP

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

# NACA 0010 section area and perimeter per chord (spec 2.4.2, integrated from Abbott & von Doenhoff)
NACA_0010_AREA_COEFFICIENT      = 0.0685  # area = 0.0685·c^2
NACA_0010_PERIMETER_COEFFICIENT = 2.029   # perimeter = 2.029·c

# ----------------------------------------------------------------------------------------------------------------------
#  compute_electrode_array_mass
# ----------------------------------------------------------------------------------------------------------------------
def compute_electrode_array_mass(electrode_array):
    """
    Computes emitter-wire and collector mass of an EHD electrode array (spec 3.3, Mass).

    Parameters
    ----------
    electrode_array : RCAIDE.Library.Components.Powertrain.Converters.EHD_Electrode_Array
        Uses number_of_units N, span b, emitter_diameter D_w, emitter_density rho_w,
        collector_chord c, collector_foam_density rho_f, collector_foil_areal_density sigma_foil

    Returns
    -------
    wire_mass : float
        m_wire = N·rho_w·pi·a^2·b [kg]
    collector_mass : float
        m_collector = N·b·(rho_f·0.0685c^2 + sigma_foil·2.029c) [kg]

    Notes
    -----
    Also sets electrode_array.mass_properties.mass to the sum. The NACA 0010 area and perimeter
    coefficients come from integrating the 4-digit thickness equation (spec 2.4.2).

    References
    ----------
    [1] Abbott & von Doenhoff, Theory of Wing Sections, Dover, 1959.
    """
    a = electrode_array
    for key in ('collector_foam_density', 'collector_foil_areal_density', 'emitter_density',
                'number_of_units', 'collector_chord'):
        if a[key] is None:
            raise ValueError("EHD_Electrode_Array." + key + " must be set to compute mass.")
    N  = a.number_of_units
    b  = a.span
    c  = a.collector_chord
    r  = a.emitter_diameter / 2.0
    wire_mass      = N * a.emitter_density * np.pi * r**2 * b
    collector_mass = N * b * (a.collector_foam_density * NACA_0010_AREA_COEFFICIENT * c**2 +
                              a.collector_foil_areal_density * NACA_0010_PERIMETER_COEFFICIENT * c)
    a.mass_properties.mass = wire_mass + collector_mass
    return wire_mass, collector_mass

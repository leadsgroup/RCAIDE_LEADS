# RCAIDE/Library/Methods/Powertrain/Converters/EHD_Electrode_Array/check_ehd_electrode_array_inputs.py
#
# Created:  Oct 2026, RCAIDE EHD MVP

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import warnings

# Group 4 of spec 3.3.1: fixed in the MVP, raise if changed
EHD_FIXED_VALUES = {'collector_airfoil'      : 'NACA 0010',
                    'polarity'               : 'positive',
                    'emitters_per_collector' : 1}

# Group 1 optimizer permissions (spec 3.3.1): unit spacing and collector chord are locked
EHD_OPTIMIZER_PERMISSIONS = {'gap'                          : 'yes',
                             'emitter_diameter'             : 'yes, with warning',
                             'span'                         : 'yes',
                             'number_of_units'              : 'yes',
                             'unit_spacing'                 : 'no',
                             'collector_chord'              : 'no',
                             'maximum_voltage'              : 'yes',
                             'sparkover_voltage'            : 'no',
                             'emitter_density'              : 'yes',
                             'collector_foam_density'       : 'yes',
                             'collector_foil_areal_density' : 'yes',
                             'additional_mass'              : 'yes',
                             'efficiency'                   : 'yes',
                             'specific_power'               : 'yes',
                             'rated_power'                  : 'yes'}

# Sourced gap range (spec 3.7 step 3: 10-300 mm; Kahol et al. 2023, Xu et al. 2019)
EHD_GAP_RANGE = (0.010, 0.300)

# Geometry recorded in calibration_geometry and compared against the current design (spec 3.3.1 group 3)
EHD_CALIBRATION_KEYS = ('gap', 'emitter_diameter', 'collector_chord', 'unit_spacing')

# Kahol et al. 2023 Eq. 31 coefficients expected by the Stage 2 spacing hook
EHD_SPACING_CORRECTION_KEYS = ('k1', 'k2', 'k3', 'k4', 'k5', 'k6')

# ----------------------------------------------------------------------------------------------------------------------
#  check_fixed_value
# ----------------------------------------------------------------------------------------------------------------------
def check_fixed_value(key, value):
    """
    Raises if a Group 4 quantity (spec 3.3.1) is set to anything other than its MVP value.

    Parameters
    ----------
    key : str
        Attribute name
    value : any
        Value being assigned

    Notes
    -----
    Collector airfoil is fixed to NACA 0010 (t = 0.10c), polarity to positive (negative corona is not
    covered by the sources), one emitter per collector (no double emitter, no multistaging).
    """
    if key in EHD_FIXED_VALUES and value != EHD_FIXED_VALUES[key]:
        raise ValueError("EHD_Electrode_Array." + key + " is fixed to " + repr(EHD_FIXED_VALUES[key]) +
                         " in the Stage 1 MVP (spec 3.3.1 group 4); got " + repr(value) + ".")
    return

# ----------------------------------------------------------------------------------------------------------------------
#  check_ehd_electrode_array_inputs
# ----------------------------------------------------------------------------------------------------------------------
def check_ehd_electrode_array_inputs(electrode_array):
    """
    Validates EHD electrode-array inputs and issues the spec 3.3.1 / 3.7 warnings.

    Parameters
    ----------
    electrode_array : RCAIDE.Library.Components.Powertrain.Converters.EHD_Electrode_Array

    Returns
    -------
    None

    Notes
    -----
    Raises ValueError for changed Group 4 values, missing required inputs or non-positive geometry.
    Warns when:
        * the gap is outside the sourced 10-300 mm range
        * the maximum voltage is not below the sparkover voltage
        * k-factors differ from 1 with no calibration provenance recorded
        * the geometry differs from calibration_geometry by more than calibration_tolerance
    Raises NotImplementedError if the Stage 2 spacing_correction hook is supplied (spec 3.3.2: the
    interface exists now but stays disabled in the MVP).
    """
    a = electrode_array

    for key in EHD_FIXED_VALUES:
        check_fixed_value(key, a[key])

    for key in ('gap', 'emitter_diameter', 'span', 'number_of_units', 'unit_spacing', 'collector_chord',
                'maximum_voltage', 'ion_mobility'):
        if a[key] is None:
            raise ValueError("EHD_Electrode_Array." + key + " must be set by the user (spec 3.3.1).")
        if a[key] <= 0:
            raise ValueError("EHD_Electrode_Array." + key + " must be positive; got " + repr(a[key]) + ".")
    if a.emitter_diameter / 2.0 >= a.gap:
        raise ValueError("EHD_Electrode_Array.gap must exceed the emitter wire radius.")

    if not (EHD_GAP_RANGE[0] <= a.gap <= EHD_GAP_RANGE[1]):
        warnings.warn("EHD gap " + str(a.gap) + " m is outside the sourced 10-300 mm range; "
                      "the 1-D relations are extrapolated.", stacklevel=2)

    if a.sparkover_voltage is not None and a.maximum_voltage >= a.sparkover_voltage:
        warnings.warn("EHD maximum_voltage is not below sparkover_voltage; operating points at full "
                      "throttle will be flagged.", stacklevel=2)

    k_factors_default = (a.k_T == 1.0 and a.k_P == 1.0 and a.k_Vi == 1.0)
    if a.calibration_geometry is None:
        if not k_factors_default:
            warnings.warn("EHD k-factors differ from 1 but no calibration_geometry is recorded; their "
                          "provenance is unknown.", stacklevel=2)
    else:
        for key in EHD_CALIBRATION_KEYS:
            ref = a.calibration_geometry[key]
            if abs(a[key] - ref) > a.calibration_tolerance * abs(ref):
                warnings.warn("EHD " + key + " differs from calibration_geometry by more than " +
                              str(100 * a.calibration_tolerance) + "%; the k-factors are being "
                              "extrapolated.", stacklevel=2)

    check_spacing_correction(a.spacing_correction)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  check_spacing_correction
# ----------------------------------------------------------------------------------------------------------------------
def check_spacing_correction(spacing_correction):
    """
    Stage 2 hook for the unit-spacing correction (spec 3.3.2), present but disabled.

    Parameters
    ----------
    spacing_correction : None or dict
        Kahol et al. 2023 Eq. 31 coefficients k1..k6 fitted to bench data at fixed gap and voltage
        (T/b = k1(1 - k2·exp(-k3·S)), P/b = k4(1 - k5·exp(-k6·S))). No default coefficients exist.

    Notes
    -----
    None keeps the 1-D cell with unit_spacing locked. Supplying coefficients raises
    NotImplementedError: how the fitted functions scale away from their fitted gap and voltage is a
    Stage 2 item, so the hook cannot be enabled in the MVP.

    References
    ----------
    [1] Kahol et al., J. Electrostatics 123, 103815 (2023), Eq. (31).
    """
    if spacing_correction is None:
        return
    missing = [k for k in EHD_SPACING_CORRECTION_KEYS if k not in spacing_correction]
    if missing:
        raise ValueError("spacing_correction requires Kahol et al. Eq. 31 coefficients " +
                         str(EHD_SPACING_CORRECTION_KEYS) + "; missing " + str(missing) + ".")
    raise NotImplementedError("spacing_correction is a Stage 2 hook and is disabled in the MVP "
                              "(spec 3.3.2). Leave it as None.")

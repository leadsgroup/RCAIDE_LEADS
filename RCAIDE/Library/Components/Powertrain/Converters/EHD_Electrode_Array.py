# RCAIDE/Library/Components/Powertrain/Converters/EHD_Electrode_Array.py
#
# Created:  Oct 2026, RCAIDE EHD MVP

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from .Converter import Converter
from RCAIDE.Library.Methods.Powertrain.Converters.EHD_Electrode_Array.append_ehd_electrode_array_conditions import append_ehd_electrode_array_conditions
from RCAIDE.Library.Methods.Powertrain.Converters.EHD_Electrode_Array.check_ehd_electrode_array_inputs      import check_fixed_value, EHD_FIXED_VALUES, EHD_OPTIMIZER_PERMISSIONS

# ----------------------------------------------------------------------------------------------------------------------
#  EHD_Electrode_Array
# ----------------------------------------------------------------------------------------------------------------------
class EHD_Electrode_Array(Converter):
    """
    Geometry and physics inputs of a wire-to-NACA 0010 electrohydrodynamic (EHD) electrode array:
    N units, each one positive emitter wire at gap d upstream of one NACA 0010 collector of chord c,
    stacked at spacing S, all of span b (spec 2.4, 3.3).

    Attributes
    ----------
    Group 1, design inputs (set by the user)
        gap : float
            Wire centre to collector leading edge d [m], default 0.060 (MIT)
        emitter_diameter : float
            D_w [m], default 0.2e-3 (MIT). Optimizer: yes, with warning
        span : float
            Electrode span per unit b [m], default 3.0 (MIT)
        number_of_units : int
            N [-], required
        unit_spacing : float
            S [m], required. **Not optimizable** (wrong trend in the 1-D cell, spec 3.3.2)
        collector_chord : float
            c [m], required; t = 0.10c. **Not optimizable** (only drag effect modelled)
        maximum_voltage : float
            V_max [V], default 40.3e3 (MIT); must be below sparkover_voltage
        sparkover_voltage : float or None
            V_spark [V], measured or conservative; no sourced law. Not optimizable
        emitter_density : float
            rho_w [kg/m^3], default 8000 (stainless, typical)
        collector_foam_density : float or None
            rho_f [kg/m^3], required for mass
        collector_foil_areal_density : float or None
            sigma_foil [kg/m^2], required for mass
    Group 2, physics inputs with defaults
        ion_mobility : float
            mu [m^2/(V·s)], default 2.0e-4 (Vaddi et al. 2020); dry 2.15e-4, saturated 1.6e-4 (R11)
        peek_surface_factor : float
            m [-], default 1.0 (R19)
        collector_drag_coefficient : float or None
            c_d [-], default None -> R24 estimate; a user value is strongly recommended at low Re
        collector_boundary_layer : str
            Skin-friction law used by R24, 'laminar' (default) or 'turbulent'
        emitter_drag_coefficient : float
            C_D,w [-], default 1.0 (spec: ~1 for a cylinder at Re ~1e2, [UNVERIFIED])
    Group 3, calibration inputs (fitted in Stage 2)
        inception_voltage_measured : float or None
            V_i,meas [V], overrides R20
        k_T, k_P, k_Vi : float
            Thrust, power and inception multipliers [-], default 1
        calibration_geometry : dict or None
            gap, emitter_diameter, collector_chord, unit_spacing at which the k-factors were fitted
        calibration_tolerance : float
            Relative difference that triggers the extrapolation warning, default 0.10
        spacing_correction : dict or None
            Stage 2 hook for Kahol et al. Eq. 31 coefficients k1..k6; disabled (must stay None)
    Group 4, fixed in the MVP (assigning any other value raises ValueError)
        collector_airfoil : 'NACA 0010'
        polarity : 'positive'
        emitters_per_collector : 1

    Notes
    -----
    Group 5 (freestream and throttle) is read from the segment and Group 6 (computed outputs) is stored
    in conditions.energy.converters[tag]; neither is an attribute here. Optimizer permissions are
    listed in EHD_OPTIMIZER_PERMISSIONS and enforced by
    RCAIDE.Library.Methods.Powertrain.Propulsors.EHD_Thruster.check_ehd_optimizer_inputs.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Propulsors.EHD_Thruster
    RCAIDE.Library.Methods.Powertrain.Converters.EHD_Electrode_Array
    """

    optimizer_permissions = EHD_OPTIMIZER_PERMISSIONS

    def __defaults__(self):
        self.tag                          = 'ehd_electrode_array'

        # Group 1: design inputs
        self.gap                          = 0.060
        self.emitter_diameter             = 0.2e-3
        self.span                         = 3.0
        self.number_of_units              = None
        self.unit_spacing                 = None
        self.collector_chord              = None
        self.maximum_voltage              = 40.3e3
        self.sparkover_voltage            = None
        self.emitter_density              = 8000.
        self.collector_foam_density       = None
        self.collector_foil_areal_density = None

        # Group 2: physics inputs with defaults
        self.ion_mobility                 = 2.0e-4
        self.peek_surface_factor          = 1.0
        self.collector_drag_coefficient   = None
        self.collector_boundary_layer     = 'laminar'
        self.emitter_drag_coefficient     = 1.0

        # Group 3: calibration inputs
        self.inception_voltage_measured   = None
        self.k_T                          = 1.0
        self.k_P                          = 1.0
        self.k_Vi                         = 1.0
        self.calibration_geometry         = None
        self.calibration_tolerance        = 0.10
        self.spacing_correction           = None

        # Group 4: fixed in the MVP
        for key, value in EHD_FIXED_VALUES.items():
            self[key] = value

    def __setitem__(self, key, value):
        check_fixed_value(key, value)
        super().__setitem__(key, value)

    def append_operating_conditions(self, segment, energy_conditions, noise_conditions=None):
        """
        Allocates per-control-point electrode-array results.
        """
        append_ehd_electrode_array_conditions(self, segment, energy_conditions)
        return

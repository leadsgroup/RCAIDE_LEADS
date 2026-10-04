# RCAIDE/Library/Methods/Powertrain/Propulsors/EHD_Thruster/check_ehd_optimizer_inputs.py
#
# Created:  Oct 2026, RCAIDE EHD MVP

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Library.Methods.Powertrain.Converters.EHD_Electrode_Array.check_ehd_electrode_array_inputs import EHD_OPTIMIZER_PERMISSIONS

import warnings

# ----------------------------------------------------------------------------------------------------------------------
#  check_ehd_optimizer_inputs
# ----------------------------------------------------------------------------------------------------------------------
def check_ehd_optimizer_inputs(problem):
    """
    Enforces the optimizer permissions of spec 3.3.1 group 1 on a Nexus optimization problem.

    Parameters
    ----------
    problem : RCAIDE.Framework.Core.Data
        Nexus optimization_problem with inputs ([tag, initial, lb, ub, scaling, units] rows) and
        aliases ([tag, path or list of paths] rows)

    Returns
    -------
    None

    Notes
    -----
    Raises ValueError if an optimizer input maps to an EHD attribute marked 'no' (unit_spacing,
    collector_chord, sparkover_voltage) and warns for 'yes, with warning' (emitter_diameter: V_i
    direction reliable, magnitude approximate). Call it after defining problem.inputs and
    problem.aliases. Unit spacing is locked because the 1-D cell makes thrust proportional to S with no
    optimum; chord is locked because only its drag effect is modelled (spec 3.3.2).
    """
    input_tags = [row[0] for row in problem.inputs]
    for alias in problem.aliases:
        tag, paths = alias[0], alias[1]
        if tag not in input_tags:
            continue
        for path in ([paths] if isinstance(paths, str) else paths):
            attribute  = path.split('.')[-1]
            permission = EHD_OPTIMIZER_PERMISSIONS.get(attribute)
            if permission == 'no':
                raise ValueError("Optimizer input '" + tag + "' maps to " + path + ", which is not "
                                 "optimizable in the EHD MVP (spec 3.3.1/3.3.2).")
            if permission == 'yes, with warning':
                warnings.warn("Optimizer input '" + tag + "' maps to " + path + ": inception-voltage "
                              "magnitude is approximate (R19, R20).", stacklevel=2)
    return

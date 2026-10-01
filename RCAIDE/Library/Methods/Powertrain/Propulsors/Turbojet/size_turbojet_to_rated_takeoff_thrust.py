# RCAIDE/Library/Methods/Powertrain/Propulsors/Turbojet/size_turbojet_to_rated_takeoff_thrust.py
#
#
# Created:  Sep 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Library.Methods.Powertrain                                  import setup_operating_conditions
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.design_turbojet import design_turbojet

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  size_turbojet_to_rated_takeoff_thrust
# ----------------------------------------------------------------------------------------------------------------------
def size_turbojet_to_rated_takeoff_thrust(turbojet, relative_tolerance=1e-6, maximum_iterations=20):
    """
    Sizes a turbojet so that its dry sea-level static thrust at the takeoff rating equals its rated
    takeoff thrust, solving for the design-point thrust.

    Parameters
    ----------
    turbojet : RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet
        Turbojet with its design-point cycle defined and the following attributes:
            - rated_takeoff_thrust : float
                Rated (maximum dry) sea-level static takeoff thrust [N]
            - takeoff_combustor_exit_temperature_ratio : float
                Combustor exit temperature at the takeoff rating divided by its design-point value
    relative_tolerance : float, optional
        Convergence tolerance on the sea-level static thrust.
    maximum_iterations : int, optional
        Maximum number of sizing iterations.

    Returns
    -------
    None
        Sets turbojet.design_thrust and runs design_turbojet at the converged size.

    Notes
    -----
    Same approach as size_turbofan_to_rated_takeoff_thrust. The rating is the dry (afterburner off)
    rating: the gas generator is sized dry and afterburner thrust follows from the afterburner model.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.size_turbofan_to_rated_takeoff_thrust
    """
    rated_takeoff_thrust = turbojet.rated_takeoff_thrust
    takeoff_throttle     = turbojet.takeoff_combustor_exit_temperature_ratio

    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    velocity   = atmosphere.compute_values(0.0, 0.0).speed_of_sound[0][0] * 0.01
    fuel_line  = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()

    def sea_level_static_thrust():
        operating_state = setup_operating_conditions(turbojet, fuel_line, velocity_range=np.array([velocity]), altitude=0, angle_of_attack=0, temperature_deviation=0)
        operating_state.conditions.energy.propulsors[turbojet.tag].throttle[:,0] = takeoff_throttle
        operating_state.unknowns.network['electrical_power'] = np.array([[turbojet.design_power_offtake]])
        _, outputs, _, _ = turbojet.compute_performance(operating_state)
        return outputs.thrust[0][0]

    # size at the design point alone while iterating (design_thrust only), without the idle_fallback deck,
    # and dry (the rating is the dry rating)
    afterburner_active            = turbojet.afterburner_active
    turbojet.afterburner_active   = False
    turbojet.rated_takeoff_thrust = 0.0
    turbojet.design_thrust        = 0.25 * rated_takeoff_thrust
    try:
        for iteration in range(maximum_iterations):
            design_turbojet(turbojet, build_idle_fallback=False)
            thrust = sea_level_static_thrust()
            if abs(thrust / rated_takeoff_thrust - 1.0) < relative_tolerance:
                break
            turbojet.design_thrust *= rated_takeoff_thrust / thrust
        else:
            raise ValueError(f"Turbojet '{turbojet.tag}': sizing to the rated takeoff thrust did not converge in {maximum_iterations} iterations.")
    finally:
        turbojet.rated_takeoff_thrust = rated_takeoff_thrust
        turbojet.afterburner_active   = afterburner_active

    # final design at the converged size: solves rated_takeoff_temperature_ratio, evaluates sealevel_static_thrust
    # at it and builds the idle_fallback deck
    design_turbojet(turbojet)
    return

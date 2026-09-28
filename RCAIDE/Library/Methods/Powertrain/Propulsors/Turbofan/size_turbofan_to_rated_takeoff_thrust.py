# RCAIDE/Library/Methods/Powertrain/Propulsors/Turbofan/size_turbofan_to_rated_takeoff_thrust.py
#
#
# Created:  Sep 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Library.Methods.Powertrain                                  import setup_operating_conditions
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.design_turbofan import design_turbofan

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  size_turbofan_to_rated_takeoff_thrust
# ----------------------------------------------------------------------------------------------------------------------
def size_turbofan_to_rated_takeoff_thrust(turbofan, relative_tolerance=1e-6, maximum_iterations=20):
    """
    Sizes a turbofan so that its sea-level static thrust at the takeoff rating equals its rated
    takeoff thrust, solving for the design-point thrust.

    Parameters
    ----------
    turbofan : RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan
        Turbofan with its design-point cycle defined and the following attributes:
            - rated_takeoff_thrust : float
                Rated sea-level static takeoff thrust [N]
            - takeoff_combustor_exit_temperature_ratio : float
                Combustor exit temperature at the takeoff rating divided by its design-point value
    relative_tolerance : float, optional
        Convergence tolerance on the sea-level static thrust.
    maximum_iterations : int, optional
        Maximum number of sizing iterations.

    Returns
    -------
    None
        Sets turbofan.design_thrust and runs design_turbofan at the converged size, which sets
        rated_takeoff_temperature_ratio, sealevel_static_thrust and the off-design matching model.

    Notes
    -----
    The engine cycle is defined at its design point (pressure ratios, bypass ratio, combustor exit
    temperature) and the engine is sized by its rated sea-level static thrust, the conventional
    measure of engine size in aircraft sizing. The takeoff rating runs hotter than the design point
    by takeoff_combustor_exit_temperature_ratio; the corresponding throttle ratio of Ref. [1],
    Sec. 8.3.3, sets how thrust lapses away from sea level. The thrust at the design point is a
    result (the engine's climb/cruise capability) rather than an input.

    For a fixed cycle the thrust scales linearly with the design mass flow, so the design thrust
    is rescaled by the thrust error until converged (a couple of iterations; shaft power offtake
    makes the scaling slightly nonlinear).

    References
    ----------
    [1] Mattingly, J. D., "Elements of Propulsion: Gas Turbines and Rockets", AIAA Education
        Series, 2006, Sec. 8.3.3.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.design_turbofan
    """
    rated_takeoff_thrust = turbofan.rated_takeoff_thrust
    takeoff_throttle     = turbofan.takeoff_combustor_exit_temperature_ratio

    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    velocity   = atmosphere.compute_values(0.0, 0.0).speed_of_sound[0][0] * 0.01
    fuel_line  = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()

    def sea_level_static_thrust():
        operating_state = setup_operating_conditions(turbofan, fuel_line, velocity_range=np.array([velocity]), altitude=0, angle_of_attack=0, temperature_deviation=0)
        operating_state.conditions.energy.propulsors[turbofan.tag].throttle[:,0] = takeoff_throttle
        operating_state.unknowns.network['electrical_power'] = np.array([[turbofan.design_power_offtake]])
        _, outputs, _, _ = turbofan.compute_performance(operating_state, RCAIDE.Framework.Networks.Fuel())
        return outputs.thrust[0][0]

    # size at the design point alone while iterating (design_thrust only), without the idle_fallback deck
    turbofan.rated_takeoff_thrust = 0.0
    turbofan.design_thrust        = 0.25 * rated_takeoff_thrust
    try:
        for iteration in range(maximum_iterations):
            design_turbofan(turbofan, build_idle_fallback=False)
            thrust = sea_level_static_thrust()
            if abs(thrust / rated_takeoff_thrust - 1.0) < relative_tolerance:
                break
            turbofan.design_thrust *= rated_takeoff_thrust / thrust
        else:
            raise ValueError(f"Turbofan '{turbofan.tag}': sizing to the rated takeoff thrust did not converge in {maximum_iterations} iterations.")
    finally:
        turbofan.rated_takeoff_thrust = rated_takeoff_thrust

    # final design at the converged size: solves rated_takeoff_temperature_ratio (equal to the requested ratio to
    # within tolerance), evaluates sealevel_static_thrust at it, and builds the idle_fallback deck
    design_turbofan(turbofan)
    return

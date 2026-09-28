# RCAIDE/Library/Methods/Powertrain/Propulsors/Turboprop/size_turboprop_to_rated_takeoff_power.py
#
#
# Created:  Sep 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Library.Methods.Powertrain                                    import setup_operating_conditions
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.design_turboprop import design_turboprop

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  size_turboprop_to_rated_takeoff_power
# ----------------------------------------------------------------------------------------------------------------------
def size_turboprop_to_rated_takeoff_power(turboprop, relative_tolerance=1e-6, maximum_iterations=20):
    """
    Sizes a turboprop so that its sea-level static shaft power at the takeoff rating equals its rated
    takeoff power, solving for the design-point thrust.

    Parameters
    ----------
    turboprop : RCAIDE.Library.Components.Powertrain.Propulsors.Turboprop
        Turboprop with its design-point cycle defined and the following attributes:
            - rated_takeoff_power : float
                Rated sea-level static takeoff shaft power at the propeller shaft [W]
            - takeoff_combustor_exit_temperature_ratio : float
                Combustor exit temperature at the takeoff rating divided by its design-point value
    relative_tolerance : float, optional
        Convergence tolerance on the sea-level static shaft power.
    maximum_iterations : int, optional
        Maximum number of sizing iterations.

    Returns
    -------
    None
        Sets turboprop.design_thrust and runs design_turboprop at the converged size, which sets
        rated_takeoff_throttle, sealevel_static_power, sealevel_static_thrust and the off-design
        matching model.

    Notes
    -----
    The engine cycle is defined at its design point (compressor pressure ratio, combustor exit
    temperature, free-turbine pressure ratio) and the engine is sized by its rated takeoff shaft
    power, the quantity a turboprop is certified and quoted at. The design-point thrust is a result
    rather than an input.

    For a fixed cycle the shaft power scales linearly with the design mass flow, so the design thrust
    is rescaled by the power error until converged (a couple of iterations; gas-generator offtake
    makes the scaling slightly nonlinear).

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.design_turboprop
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.size_turbofan_to_rated_takeoff_thrust
    """
    rated_takeoff_power = turboprop.rated_takeoff_power
    takeoff_throttle    = turboprop.takeoff_combustor_exit_temperature_ratio

    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    velocity   = atmosphere.compute_values(0.0, 0.0).speed_of_sound[0][0] * 0.01
    fuel_line  = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()

    def sea_level_static_shaft_power():
        operating_state = setup_operating_conditions(turboprop, fuel_line, velocity_range=np.array([velocity]), altitude=0, angle_of_attack=0, temperature_deviation=0)
        operating_state.conditions.energy.propulsors[turboprop.tag].throttle[:,0] = takeoff_throttle
        operating_state.unknowns.network['electrical_power'] = np.array([[turboprop.design_power_offtake]])
        _, outputs, _, _ = turboprop.compute_performance(operating_state)
        return outputs.power.mechanical[0][0]

    # size at the design point alone while iterating (design_thrust only), without the idle_fallback deck
    turboprop.rated_takeoff_power = 0.0
    # first guess: cruise thrust of about 0.55 x rated power / design velocity (propulsive efficiency
    # times cruise-to-takeoff power ratio); rescaled below
    design_velocity               = float(np.ravel(atmosphere.compute_values(turboprop.design_altitude, turboprop.design_isa_deviation).speed_of_sound)[0]) * \
        float(np.ravel(turboprop.design_mach_number)[0]) if turboprop.design_mach_number is not None else turboprop.design_freestream_velocity
    turboprop.design_thrust       = 0.55 * rated_takeoff_power / design_velocity
    try:
        for iteration in range(maximum_iterations):
            design_turboprop(turboprop, build_idle_fallback=False)
            power = sea_level_static_shaft_power()
            if abs(power / rated_takeoff_power - 1.0) < relative_tolerance:
                break
            turboprop.design_thrust *= rated_takeoff_power / power
        else:
            raise ValueError(f"Turboprop '{turboprop.tag}': sizing to the rated takeoff power did not converge in {maximum_iterations} iterations.")
    finally:
        turboprop.rated_takeoff_power = rated_takeoff_power

    # final design at the converged size: solves rated_takeoff_throttle, evaluates the sea-level static
    # power and thrust at it and builds the idle_fallback deck
    design_turboprop(turboprop)
    return

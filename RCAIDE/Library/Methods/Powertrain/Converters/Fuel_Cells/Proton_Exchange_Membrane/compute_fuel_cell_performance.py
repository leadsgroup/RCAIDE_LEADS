# RCAIDE/Library/Methods/Powertrain/Converters/Fuel_Cells/Proton_Exchange_Membrane/compute_fuel_cell_performance.py
#
# Created:  Jan 2025, M. Clarke
# Modified: Jun 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
from scipy.optimize import minimize_scalar
from RCAIDE.Framework.Core import Data, array_namespace

# ----------------------------------------------------------------------------------------------------------------------
#  Compute Fuel Cell Performance
# ----------------------------------------------------------------------------------------------------------------------
def compute_fuel_cell_performance(fuel_cell_stack, state, network):
    """Computes the performance of a PEM fuel cell stack across all control points.

    Iteratively solves for the current density at each control point that matches
    the electrical power demand, then stores the converged electrochemical and
    thermodynamic results in the segment conditions.

    Parameters
    ----------
    fuel_cell_stack : RCAIDE.Library.Components.Powertrain.Converters.Fuel_Cell
        Fuel cell stack object containing cell-level properties (membrane area,
        catalyst loading, electrical configuration, etc.).
    state : RCAIDE.Framework.Mission.Common.State
        Mission segment state containing freestream conditions, numerics,
        and energy converter conditions.
    network : RCAIDE.Framework.Networks.Network
        Energy network to which this converter belongs.

    Returns
    -------
    inputs : Conditions
        Input power conditions for the fuel cell stack.
    outputs : Conditions
        Output power conditions (electrical power produced).
    stored_results_flag : bool
        Always True; indicates results are available for reuse by identical converters.
    stored_converter_tag : str
        Tag of this fuel cell stack, used to look up stored results.

    Notes
    -----
    The power-matching uses a fixed-point iteration:

    .. math::
        i^{(k+1)} = i^{(k)} + \\alpha \\left( P_{demand} - P_{net}(i^{(k)}) \\right)

    where :math:`\\alpha = 0.05` is the relaxation factor and :math:`P_{net}` is
    the net cell power from ``evaluate_PEM``.

    The power this stack must supply is its own electrical distributor's
    total demand, split between battery/fuel-cell sources by that
    distributor's psi and between multiple fuel-cell stacks on the same bus
    by ``power_split_ratio``. The distributor is looked up from
    ``fuel_cell_stack.assigned_distributors`` rather than assumed, since a
    fuel cell stack may share the vehicle with other electrically-isolated
    buses resolved to a different psi.

    **Major Assumptions**
        * Uniform temperature across all cells in the stack
        * Hydrogen supplied at constant rated pressure
        * Standard atmospheric air composition (23.3 %  O2 by mass)
        * Ideal water management (no flooding or dehydration)

    References
    ----------
    [1] O'Hayre, R., Cha, S.-W., Colella, W., & Prinz, F. B. (2016).
        *Fuel Cell Fundamentals* (3rd ed.). John Wiley & Sons.
    """
    # ---------------------------------------------------------------------------------
    # Unpack freestream conditions
    # ---------------------------------------------------------------------------------
    M0 = state.conditions.freestream.mach_number
    P0 = state.conditions.freestream.pressure
    T0 = state.conditions.freestream.temperature
    a  = state.conditions.freestream.speed_of_sound
    R  = 287  # specific gas constant for air [J/(kg·K)]

    # Ratio of specific heats from speed of sound
    gamma = (a ** 2) / (T0 * R)

    # Isentropic stagnation quantities
    stagnation_pressure    = P0 * ((1. + (gamma - 1.) / 2. * M0 * M0) ** (gamma / (gamma - 1.)))
    stagnation_temperature = T0 * (1. + ((gamma - 1.) / 2. * M0 * M0))

    # ---------------------------------------------------------------------------------
    # Fuel cell stack configuration
    # ---------------------------------------------------------------------------------
    fuel_cell = fuel_cell_stack.fuel_cell
    n_series  = fuel_cell_stack.electrical_configuration.series
    n_parallel = fuel_cell_stack.electrical_configuration.parallel
    n_total   = n_series * n_parallel

    # ---------------------------------------------------------------------------------
    # Retrieve converter conditions and power demand
    # ---------------------------------------------------------------------------------
    fc_conds = state.conditions.energy.converters[fuel_cell_stack.tag]

    electrical_distributor_tag = None
    for d_tag in fuel_cell_stack.assigned_distributors[0]:
        if network.distributors[d_tag].domain == 'electrical':
            electrical_distributor_tag = d_tag
    psi = state.conditions.energy.battery_fuel_cell_power_split_ratio[electrical_distributor_tag]
    total_electrical_demand = state.conditions.energy.distributors[electrical_distributor_tag].outputs.power.electrical
    P_stack  = total_electrical_demand * fuel_cell_stack.power_split_ratio * (1. - psi)
    P_cell   = P_stack[:, 0] / n_total  # power demand per individual cell [W] 

    xp = array_namespace(P_cell, stagnation_temperature)
    fc_conds.stagnation_temperature = xp.reshape(stagnation_temperature * xp.ones_like(P_stack), (-1, 1))
    fc_conds.stagnation_pressure    = xp.reshape(stagnation_pressure * xp.ones_like(P_stack), (-1, 1))

    # ---------------------------------------------------------------------------------
    # Current density matching the power demand on the rising branch of the net power curve
    # ---------------------------------------------------------------------------------
    # The net power rises from i_floor to a maximum, then falls toward the limiting current density, where the
    # concentration losses diverge. The bracket ends just below the limiting current density.
    i_floor   = 1e-6
    stack_T   = fc_conds.stack_temperature[:, 0]
    P_O2_ceiling = calculate_P_O2(fuel_cell_stack, fuel_cell.rated_air_pressure, stack_T,
                                   fuel_cell.oxygen_relative_humidity, fuel_cell.air_excess_ratio, 0., 0.)
    if fuel_cell.type == "LT":
        i_ceiling = calculate_limiting_current_density_LT(fuel_cell_stack, stack_T, P_O2_ceiling,
                                                            fuel_cell.oxygen_relative_humidity, fuel_cell.air_excess_ratio, 0., 0.)
    else:
        i_ceiling = calculate_limiting_current_density_HT(fuel_cell_stack, stack_T, P_O2_ceiling,
                                                            fuel_cell.oxygen_relative_humidity, fuel_cell.air_excess_ratio, 0., 0.)
    i_ceiling = 0.999 * i_ceiling * xp.ones_like(P_cell)

    i_vec = compute_current_density(fuel_cell_stack, fc_conds, P_cell, i_floor + 0 * i_ceiling, i_ceiling)

    # ---------------------------------------------------------------------------------
    # Final evaluation at the matched current densities
    # ---------------------------------------------------------------------------------
    fc_conds.current_density = xp.reshape(i_vec, (-1, 1))
    pem = evaluate_PEM(fuel_cell_stack, fc_conds, i_vec)
    store_PEM_conditions(fc_conds, pem)

    # Scale single-cell quantities to full stack
    I_cell  = pem.net_power / pem.voltage   # cell current [A]
    I_stack = I_cell * n_parallel           # stack current (parallel cells share current) [A]

    fc_conds.power                     = xp.reshape(pem.net_power * n_total, (-1, 1))
    fc_conds.current                   = xp.reshape(I_stack, (-1, 1))
    fc_conds.voltage_open_circuit      = xp.reshape(pem.voltage * n_series, (-1, 1))
    fc_conds.voltage_under_load        = xp.reshape(pem.voltage * n_series, (-1, 1))
    fc_conds.H2_mass_flow_rate         = xp.reshape(pem.mdot_H2 * n_total, (-1, 1))
    fc_conds.inlet_H2_mass_flow_rate   = xp.reshape(pem.mdot_H2, (-1, 1))

    # Chemical (hydrogen) power draw, fed to the fuel line so assigned fuel tanks
    # can compute their own mass depletion.
    fc_conds.inputs.power.chemical = fc_conds.H2_mass_flow_rate * fuel_cell_stack.fuel_cell.propellant.lower_heating_value
    fc_conds.fuel_mass_flow_rate   = fc_conds.H2_mass_flow_rate

    stored_results_flag  = True
    stored_converter_tag = fuel_cell_stack.tag

    fc_conds.outputs.power.electrical = fc_conds.power * fuel_cell_stack.electrical_efficiency

    return fc_conds.inputs, fc_conds.outputs, stored_results_flag, stored_converter_tag


# ----------------------------------------------------------------------------------------------------------------------
#  Evaluate PEM Cell Performance
# ----------------------------------------------------------------------------------------------------------------------
def evaluate_PEM(fuel_cell_stack, fc_conds, i):
    """Evaluates PEM fuel cell electrochemistry for all control points simultaneously.

    Computes cell voltage via the Nernst equation with activation, ohmic, and
    concentration losses, then determines gross/net power and hydrogen consumption.
    The compressor-expander module (CEM) parasitic losses are subtracted to yield
    the net electrical power.

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack object with cell-level electrochemical properties.
    fc_conds : Conditions
        Fuel cell operating conditions (current density, temperatures, pressures).

    Returns
    -------
    mdot_H2 : ndarray
        Hydrogen mass flow rate per cell [kg/s].
    voltage : ndarray
        Cell output voltage [V].
    net_power : ndarray
        Net electrical power per cell after CEM and parasitic losses [W].
    gross_power : ndarray
        Gross electrical power per cell (voltage × current density × area) [W].
    gross_heat : ndarray
        Heat generated per cell from voltage losses [W].
    compressor_power : ndarray
        Power consumed by the compressor-expander module [W].
    mdot_air_in : ndarray
        Air mass flow rate entering the cathode [kg/s].
    mdot_air_out : ndarray
        Air mass flow rate exiting the cathode (less consumed O2) [kg/s].
    expander_power : ndarray
        Power recovered by the expander [W].
    """
    fuel_cell        = fuel_cell_stack.fuel_cell
    xp               = array_namespace(i)
    air_excess_ratio = fuel_cell.air_excess_ratio

    # Pressure drops across stack and humidifier
    pressure_drop            = calculate_P_drop_stack(fuel_cell_stack, i)
    humidifier_pressure_drop = calculate_P_drop_hum(fuel_cell_stack, i) if fuel_cell.type == "LT" else 0 * i

    # Stoichiometric mass flow rates (Faraday's law)
    mdot_air_in = i * fuel_cell.interface_area * fuel_cell.O2_molar_mass / (4 * fuel_cell.Faraday_constant * fuel_cell.O2_mass_frac) * air_excess_ratio
    mdot_H2     = i * fuel_cell.interface_area / (2 * fuel_cell.Faraday_constant) * fuel_cell.H2_molar_mass

    # Electrochemical voltage model
    voltage, V_loss = calculate_voltage(i, fuel_cell_stack, fc_conds, pressure_drop)
    gross_power = voltage * i * fuel_cell.interface_area
    gross_heat  = V_loss  * i * fuel_cell.interface_area

    # Compressor-expander module losses
    CEM = evaluate_CEM(fuel_cell_stack, fc_conds.stagnation_temperature[:, 0], fc_conds.stagnation_pressure[:, 0], mdot_air_in,
                       humidifier_pressure_drop, pressure_drop)

    # Net power = gross - CEM parasitic - other parasitic
    parasitic_power = fuel_cell.gamma_para * gross_power
    net_power       = gross_power - CEM.compressor_expander_module_power - parasitic_power

    return Data(mdot_H2 = mdot_H2, voltage = voltage, net_power = net_power, gross_power = gross_power, gross_heat = gross_heat,
                mdot_air_in = mdot_air_in, mdot_air_out = CEM.expander_outlet_mass_flow_rate, pressure_drop = pressure_drop,
                humidifier_pressure_drop = humidifier_pressure_drop, CEM = CEM)


def store_PEM_conditions(fc_conds, pem):
    """Writes an evaluate_PEM result into the fuel cell stack conditions."""
    xp = array_namespace(pem.net_power)
    fc_conds.pressure_drop              = xp.reshape(pem.pressure_drop, (-1, 1))
    fc_conds.humidifier.pressure_drop   = xp.reshape(pem.humidifier_pressure_drop, (-1, 1))
    fc_conds.inlet_air_mass_flow_rate   = xp.reshape(pem.mdot_air_in, (-1, 1))
    fc_conds.outlet_air_mass_flow_rate  = xp.reshape(pem.mdot_air_out, (-1, 1))
    for field, value in pem.CEM.items():
        fc_conds[field] = xp.reshape(value, (-1, 1))
    return


# ----------------------------------------------------------------------------------------------------------------------
#  Current density at a power demand
# ----------------------------------------------------------------------------------------------------------------------
# bisection steps for the maximum net power, false-position steps for the power match, Newton steps that refine the
# maximum on JAX, and the relative step of the central difference whose sign locates the maximum
PEAK_BISECTION_STEPS = 60
FALSE_POSITION_STEPS = 60
PEAK_NEWTON_STEPS    = 4
PEAK_SLOPE_STEP      = 1e-5

def compute_current_density(fuel_cell_stack, fc_conds, P_cell, i_floor, i_ceiling):
    """
    Current density at which each cell delivers its net power demand, on the rising branch of the net power curve.

    The net power rises from i_floor to a single maximum below the limiting current density. The maximum is found by
    bisection on the sign of a central-difference slope (refined on JAX by Newton steps on the exact slope); a demand at or above it is held at the maximum, one at or below the power at i_floor at
    i_floor, and otherwise the demand is met on [i_floor, maximum] by the Illinois false-position method, which keeps
    a bracket and needs no derivative. The search is the same on NumPy and JAX; on JAX the result is differentiable
    through the implicit function theorem on the power match, on the zero slope at the maximum, or on the floor
    (jax.lax.custom_root, one independent equation per point).

    Inputs:
       fuel_cell_stack    PEM fuel cell stack
       fc_conds           its conditions (stack temperature, stagnation state, degradation)
       P_cell             net power demand of one cell at each control point [W]
       i_floor, i_ceiling current density bracket at each control point [A/cm^2]

    Outputs:
       current density at each control point [A/cm^2]
    """
    xp = array_namespace(P_cell, i_ceiling)

    def net_power(i):
        return evaluate_PEM(fuel_cell_stack, fc_conds, i).net_power

    def loop(steps, body, carry):
        if xp is np:
            for step in range(steps):
                carry = body(step, carry)
            return carry
        import jax
        return jax.lax.fori_loop(0, steps, body, carry)

    # maximum net power: bisection on the sign of the slope, which resolves the maximum far closer than comparing
    # power values, whose differences vanish into rounding near a flat maximum
    def bisect_slope(step, bracket):
        a, b   = bracket
        middle = 0.5 * (a + b)
        rising = net_power(middle * (1 + PEAK_SLOPE_STEP)) > net_power(middle * (1 - PEAK_SLOPE_STEP))
        return xp.where(rising, middle, a), xp.where(rising, b, middle)
    a, b  = loop(PEAK_BISECTION_STEPS, bisect_slope, (i_floor, i_ceiling))
    peak  = 0.5 * (a + b)
    if xp is not np:
        # Newton steps on the exact slope place the maximum where the implicit-function residual below vanishes
        import jax
        def slope(i):
            return jax.jvp(net_power, (i,), (xp.ones_like(i),))[1]
        for step in range(PEAK_NEWTON_STEPS):
            value, curvature = jax.jvp(slope, (peak,), (xp.ones_like(peak),))
            peak = xp.clip(peak - value / xp.where(curvature < 0., curvature, -1.), a, b)
        peak = jax.lax.stop_gradient(peak)

    P_peak    = net_power(peak)
    P_floor   = net_power(i_floor)
    saturated = P_cell >= P_peak
    at_floor  = P_cell <= P_floor

    def solve(residual, guess):
        # Illinois false position on f(i) = net power - demand, negative at i_floor and positive at the peak
        def false_position(step, bracket):
            a, fa, b, fb = bracket
            c       = xp.where(fb != fa, b - fb * (b - a) / xp.where(fb != fa, fb - fa, 1.), b)
            fc      = net_power(c) - P_cell
            crossed = fc * fb < 0.
            a, fa   = xp.where(crossed, b, a), xp.where(crossed, fb, 0.5 * fa)
            return a, fa, c, fc
        bracket = (i_floor, P_floor - P_cell, peak, P_peak - P_cell)
        a, fa, b, fb = loop(FALSE_POSITION_STEPS, false_position, bracket)
        matched = xp.where(xp.abs(fa) < xp.abs(fb), a, b)
        return xp.where(saturated, peak, xp.where(at_floor, i_floor, matched))

    if xp is np:
        return solve(None, peak)

    import jax
    saturated, at_floor, peak = jax.lax.stop_gradient(saturated), jax.lax.stop_gradient(at_floor), jax.lax.stop_gradient(peak)

    def residual(i):
        slope = jax.jvp(net_power, (i,), (xp.ones_like(i),))[1]
        return xp.where(saturated, slope, xp.where(at_floor, i - i_floor, net_power(i) - P_cell))

    def tangent_solve(linear_map, y):
        return y / linear_map(xp.ones_like(y))

    return jax.lax.custom_root(residual, peak, solve, tangent_solve)


# ----------------------------------------------------------------------------------------------------------------------
#  Evaluate Compressor-Expander Module (CEM)
# ----------------------------------------------------------------------------------------------------------------------
def evaluate_CEM(fuel_cell_stack, Tt_in, Pt_in, mdot_air_in, p_drop_hum, pressure_drop):
    """
    Compressor-expander module of the cathode air supply: compressor power to raise the air to the cathode supply
    pressure, expander power recovered from the cathode exhaust, and their net electrical power.

    Inputs:
       fuel_cell_stack   PEM fuel cell stack (compressor_expander_module, rated_air_pressure, air_excess_ratio)
       Tt_in, Pt_in      stagnation temperature and pressure of the inlet air [K, Pa]
       mdot_air_in       cathode air mass flow rate [kg/s]
       p_drop_hum        humidifier pressure drop [Pa]
       pressure_drop     stack pressure drop [Pa]

    Outputs:
       Data with the CEM conditions, including compressor_expander_module_power (net, positive when drawn) [W]
    """
    Cp  = 1004   # specific heat of air [J/(kg·K)]
    gam = 1.4    # ratio of specific heats for air

    fuel_cell        = fuel_cell_stack.fuel_cell
    CEM              = fuel_cell.compressor_expander_module
    xp               = array_namespace(Tt_in, Pt_in, mdot_air_in, pressure_drop)
    FC_air_p         = fuel_cell.rated_air_pressure
    air_excess_ratio = fuel_cell.air_excess_ratio

    # Compressor: raise ambient air to cathode supply pressure (p_air_FC already includes
    # the humidifier pressure drop the compressor must overcome, so it is not added again here).
    # Ram air can already meet or exceed the required cathode pressure at low altitude/high
    # speed; the compressor cannot do negative work in that case, so the ratio is floored at 1.
    p_air_FC          = FC_air_p + p_drop_hum
    pressure_ratio     = xp.maximum(p_air_FC / Pt_in, 1.0)
    comp_p_req         = mdot_air_in * Cp * Tt_in * (pressure_ratio ** ((gam - 1) / gam) - 1) / CEM.compressor_efficiency
    input_p    = comp_p_req / CEM.motor_efficiency

    # Expander: recover energy from cathode exhaust
    p_exp        = p_air_FC - pressure_drop - p_drop_hum
    Tt_exp       = Tt_in * (p_exp / Pt_in) ** ((gam - 1) / gam)
    mdot_air_out = mdot_air_in - mdot_air_in / air_excess_ratio * 0.233  # O2 consumed
    exp_p_ext    = mdot_air_out * Cp * Tt_exp * (1 - (Pt_in / p_exp) ** ((gam - 1) / gam)) * CEM.expander_efficiency
    output_p     = exp_p_ext * CEM.generator_efficiency

    return Data(outlet_air_pressure              = p_air_FC + 0 * Pt_in,
                compressor_inlet_pressure        = Pt_in,
                compressor_pressure_ratio        = pressure_ratio,
                compressor_inlet_mass_flow_rate  = mdot_air_in,
                compressor_power                 = comp_p_req,
                expander_outlet_mass_flow_rate   = mdot_air_out,
                expander_inlet_pressure          = p_exp,
                expander_pressure_ratio          = Pt_in / p_exp,
                expander_power                   = exp_p_ext,
                motor_compressor_power           = input_p,
                expander_generator_power         = output_p,
                compressor_expander_module_power = input_p - output_p)


def calculate_voltage(i, fuel_cell_stack, fc_conds, pressure_drop):
    """Computes the cell output voltage and total voltage loss.

    Applies the Nernst equation for the reversible potential, then subtracts
    activation, ohmic, and concentration overpotentials plus any degradation.

    .. math::
        V_{cell} = E_{cell} - \\eta_{act} - \\eta_{ohmic} - \\eta_{conc} - V_{deg}

    Parameters
    ----------
    i : ndarray
        Current density at each control point [A/cm^2].
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack object with electrochemical parameters.
    fc_conds : Conditions
        Operating conditions (stack temperature, pressure drops, degradation).

    Returns
    -------
    V_cell : ndarray
        Net cell voltage [V].
    V_loss : ndarray
        Total voltage loss (activation + ohmic + concentration + degradation) [V].
    """
    fuel_cell         = fuel_cell_stack.fuel_cell
    stack_temperature = fc_conds.stack_temperature[:, 0]
    P_H2_input        = fuel_cell.rated_H2_pressure
    P_air             = fuel_cell.rated_air_pressure
    RH                = fuel_cell.oxygen_relative_humidity
    air_excess_ratio  = fuel_cell.air_excess_ratio
    degradation       = fc_conds.degradation[:, 0]
    # Partial pressures at electrode surfaces
    P_O2   = calculate_P_O2(fuel_cell_stack, P_air, stack_temperature, RH, air_excess_ratio, pressure_drop, i)
    P_H2   = calculate_P_H2(fuel_cell_stack, P_H2_input, stack_temperature, RH, i)

    # Reversible Nernst potential
    E_cell = calculate_E_cell(fuel_cell_stack, stack_temperature, P_H2, P_O2)

    # Overpotentials (model selection based on LT vs HT PEM)
    if fuel_cell.type == "LT":
        eta_ohmic = calculate_ohmic_losses_LT(fuel_cell_stack, stack_temperature, i)
        eta_conc  = calculate_concentration_losses_LT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, pressure_drop, i)
    elif fuel_cell.type == "HT":
        eta_ohmic = calculate_ohmic_losses_HT(fuel_cell_stack, stack_temperature, i)
        eta_conc  = calculate_concentration_losses_HT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, pressure_drop, i)
    eta_act = calculate_activation_losses(fuel_cell_stack, stack_temperature, P_O2, i)

    V_cell = E_cell - eta_act - eta_ohmic - eta_conc - degradation * fuel_cell.maximum_deg
    V_loss = eta_act + eta_ohmic + eta_conc + degradation * fuel_cell.maximum_deg
    return V_cell, V_loss


# ----------------------------------------------------------------------------------------------------------------------
#  Rated Current Density Setter
# ----------------------------------------------------------------------------------------------------------------------
def set_rated_current_density(fuel_cell_stack, rated_current_density, rated_power_density):
    """Stores the rated current density and power density on the fuel cell object.

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack whose rated point is being set.
    rated_current_density : float
        Current density at the maximum-power operating point [A/cm^2].
    rated_power_density : float
        Power density at the maximum-power operating point [W/cm^2].
    """
    fuel_cell                       = fuel_cell_stack.fuel_cell
    fuel_cell.rated_current_density = rated_current_density
    fuel_cell.rated_power_density   = rated_power_density
    return


# ----------------------------------------------------------------------------------------------------------------------
#  Pressure Drop Models
# ----------------------------------------------------------------------------------------------------------------------
def calculate_P_drop_hum(fuel_cell_stack, i):
    """Calculates humidifier pressure drop, scaled quadratically from rated conditions.

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack with rated pressure drop properties.
    i : ndarray or float
        Current density [A/cm^2].

    Returns
    -------
    ndarray or float
        Humidifier pressure drop [bar].
    """
    fuel_cell = fuel_cell_stack.fuel_cell
    return (i / fuel_cell.rated_current_density) ** 2 * fuel_cell.rated_p_drop_hum


def calculate_P_drop_stack(fuel_cell_stack, i):
    """Calculates stack cathode pressure drop, scaled quadratically from rated conditions.

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack with rated pressure drop properties.
    i : ndarray or float
        Current density [A/cm^2].

    Returns
    -------
    ndarray or float
        Stack pressure drop [bar].
    """
    fuel_cell = fuel_cell_stack.fuel_cell
    return (i / fuel_cell.rated_current_density) ** 2 * fuel_cell.rated_p_drop_fc


# ----------------------------------------------------------------------------------------------------------------------
#  Partial Pressure Models
# ----------------------------------------------------------------------------------------------------------------------
def calculate_P_O2(fuel_cell_stack, P_air, stack_temperature, RH, air_excess_ratio, P_drop, i):
    """Calculates oxygen partial pressure at the cathode catalyst layer.

    Accounts for water vapor pressure, cathode pressure drop, air excess
    ratio, and current-dependent concentration gradient (empirical N factor).

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack (unused directly, kept for API consistency).
    P_air : ndarray or float
        Cathode inlet air pressure [bar].
    stack_temperature : ndarray or float
        Stack temperature [K].
    RH : float
        Relative humidity of the cathode air stream [0-1].
    air_excess_ratio : float
        Cathode stoichiometric ratio (> 1 means excess air).
    P_drop : ndarray or float
        Cathode pressure drop [bar].
    i : ndarray or float
        Current density [A/cm^2].

    Returns
    -------
    ndarray or float
        Oxygen partial pressure at the catalyst layer [bar].
    """
    xp = array_namespace(P_air, stack_temperature, RH, air_excess_ratio, P_drop, i)
    T_C       = stack_temperature - 273.15
    log_P_H2O = -2.1794 + 0.02953 * T_C - 9.1837e-5 * T_C**2 + 1.4454e-7 * T_C**3
    P_H2O     = 10 ** log_P_H2O
    N         = 0.291 * i / (stack_temperature ** 0.832)
    P_O2      = 0.21 * (P_air - P_drop / 2 - RH * P_H2O) * ((1 + (air_excess_ratio - 1) / air_excess_ratio) / 2) / xp.exp(N)
    return P_O2


def calculate_P_H2(fuel_cell_stack, P_H2_input, stack_temperature, RH, i):
    """Calculates hydrogen partial pressure at the anode catalyst layer.

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack (unused directly, kept for API consistency).
    P_H2_input : float
        Anode inlet hydrogen pressure [bar].
    stack_temperature : ndarray or float
        Stack temperature [K].
    RH : float
        Relative humidity of the anode stream [0-1].
    i : ndarray or float
        Current density [A/cm^2].

    Returns
    -------
    ndarray or float
        Hydrogen partial pressure at the catalyst layer [bar].
    """
    xp = array_namespace(P_H2_input, stack_temperature, RH, i)
    T_C       = stack_temperature - 273.15
    log_P_H2O = -2.1794 + 0.02953 * T_C - 9.1837e-5 * T_C**2 + 1.4454e-7 * T_C**3
    P_H2O     = 10 ** log_P_H2O
    P_H2      = 0.5 * (P_H2_input / xp.exp(1.653 * i / stack_temperature**1.334) - RH * P_H2O)
    return P_H2


# ----------------------------------------------------------------------------------------------------------------------
#  Nernst Potential
# ----------------------------------------------------------------------------------------------------------------------
def calculate_E_cell(fuel_cell_stack, stack_temperature, P_H2, P_O2):
    """Calculates the reversible (Nernst) cell potential.

    .. math::
        E = 1.229 - 8.45 \\times 10^{-4}(T - 298.15)
            + \\frac{RT}{4\\alpha F}\\left(\\ln P_{H_2} + 0.5 \\ln P_{O_2}\\right)

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack with thermodynamic constants.
    stack_temperature : ndarray or float
        Stack temperature [K].
    P_H2 : ndarray or float
        Hydrogen partial pressure [bar].
    P_O2 : ndarray or float
        Oxygen partial pressure [bar].

    Returns
    -------
    ndarray or float
        Reversible cell potential [V].
    """
    xp = array_namespace(stack_temperature, P_H2, P_O2)
    fuel_cell = fuel_cell_stack.fuel_cell
    # P_H2 and P_O2 can go non-positive when the Newton-Raphson current-density
    # solve (in compute_fuel_cell_performance) overshoots into an unphysical
    # region (e.g. demand far exceeding the stack's rated capacity). Floor them
    # so log() returns a large negative value (very low, but finite, cell
    # voltage) instead of NaN, keeping the solve well-defined everywhere.
    P_H2_safe = xp.maximum(P_H2, 1e-8)
    P_O2_safe = xp.maximum(P_O2, 1e-8)
    E_cell    = 1.229 - 8.45e-4 * (stack_temperature - 298.15) + \
        fuel_cell.Universal_gas_constant * stack_temperature / (4 * fuel_cell.alpha * fuel_cell.Faraday_constant) * (xp.log(P_H2_safe) + 0.5 * xp.log(P_O2_safe))
    return E_cell


# ----------------------------------------------------------------------------------------------------------------------
#  Activation Losses
# ----------------------------------------------------------------------------------------------------------------------
def calculate_activation_losses(fuel_cell_stack, stack_temperature, P_O2, i):
    """Calculates activation overpotential using the Tafel equation.

    .. math::
        \\eta_{act} = \\frac{RT}{2\\alpha F} \\ln\\left(\\frac{i}{i_0}\\right)

    where the exchange current density :math:`i_0` depends on catalyst loading,
    specific area, O2 partial pressure, and temperature (Arrhenius form).

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack with catalyst and kinetic parameters.
    stack_temperature : ndarray or float
        Stack temperature [K].
    P_O2 : ndarray or float
        Oxygen partial pressure [bar].
    i : ndarray or float
        Current density [A/cm^2].

    Returns
    -------
    ndarray or float
        Activation voltage loss [V].
    """
    xp = array_namespace(stack_temperature, P_O2, i)
    fuel_cell = fuel_cell_stack.fuel_cell
    A_const   = fuel_cell.Universal_gas_constant * stack_temperature / (2 * fuel_cell.alpha * fuel_cell.Faraday_constant)
    i0        = fuel_cell.i0ref * fuel_cell.L_c * fuel_cell.a_c * (P_O2 / fuel_cell.i0ref_P_ref) ** fuel_cell.gamma * \
        xp.exp(-fuel_cell.E_C / (fuel_cell.Universal_gas_constant * stack_temperature) * (1 - (stack_temperature / fuel_cell.i0ref_T_ref)))
    eta_act   = A_const * xp.log(i / i0)
    return eta_act


# ----------------------------------------------------------------------------------------------------------------------
#  Ohmic Losses
# ----------------------------------------------------------------------------------------------------------------------
def calculate_ohmic_losses_LT(fuel_cell_stack, stack_temperature, i):
    """Calculates ohmic losses for low-temperature PEM using Nafion membrane conductivity.

    Uses the empirical Springer model for Nafion membrane resistivity as a function
    of temperature, current density, and effective water content (lambda_eff).

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack with membrane properties (thickness, lambda_eff).
    stack_temperature : ndarray or float
        Stack temperature [K].
    i : ndarray or float
        Current density [A/cm^2].

    Returns
    -------
    ndarray or float
        Ohmic voltage loss [V].
    """
    xp = array_namespace(stack_temperature, i)
    fuel_cell  = fuel_cell_stack.fuel_cell
    t_m        = fuel_cell.t_m
    lambda_eff = fuel_cell.lambda_eff
    num        = 181.6 * (1 + 0.03 * i + 0.062 * (stack_temperature / 303) ** 2 * i ** 2.5)
    denom      = (lambda_eff - 0.634 - 3 * i) * xp.exp(4.18 * (stack_temperature - 303) / stack_temperature)
    rho        = num / denom
    eta_ohmic  = (rho * t_m) * i
    return eta_ohmic


def calculate_ohmic_losses_HT(fuel_cell_stack, stack_temperature, i):
    """Calculates ohmic losses for high-temperature PEM using PBI membrane conductivity.

    Interpolates membrane conductivity linearly between 100 C and 200 C using
    the c1 and c2 fitting parameters.

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack with PBI membrane properties (thickness, c1, c2).
    stack_temperature : ndarray or float
        Stack temperature [K].
    i : ndarray or float
        Current density [A/cm^2].

    Returns
    -------
    ndarray or float
        Ohmic voltage loss [V].
    """
    fuel_cell = fuel_cell_stack.fuel_cell
    t_m       = fuel_cell.t_m
    c1        = fuel_cell.c1
    c2        = fuel_cell.c2
    s         = (stack_temperature - 373.15) / 100 * (c2 - c1) + c1
    rho       = 1 / s
    eta_ohmic = (rho * t_m) * i
    return eta_ohmic


# ----------------------------------------------------------------------------------------------------------------------
#  Concentration Losses
# ----------------------------------------------------------------------------------------------------------------------
def calculate_concentration_losses_LT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, P_drop, i):
    """Calculates concentration overpotential for low-temperature PEM.

    Returns a large penalty (10 V) when current density exceeds the limiting
    current density, preventing non-physical solutions.

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack with charge-transfer coefficient (alpha).
    stack_temperature : ndarray or float
        Stack temperature [K].
    P_O2 : ndarray or float
        Oxygen partial pressure [bar].
    RH : float
        Relative humidity [0-1].
    air_excess_ratio : float
        Cathode stoichiometric ratio.
    P_drop : ndarray or float
        Cathode pressure drop [bar].
    i : ndarray or float
        Current density [A/cm^2].

    Returns
    -------
    ndarray or float
        Concentration voltage loss [V].
    """
    xp = array_namespace(stack_temperature, P_O2, RH, air_excess_ratio, P_drop, i)
    fuel_cell = fuel_cell_stack.fuel_cell
    i_lim     = calculate_limiting_current_density_LT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, P_drop, i)
    # xp.where for vectorized branching; xp.maximum prevents log(0)
    eta_conc  = xp.where(
        i >= i_lim,
        10.0,
        (1 + 1 / fuel_cell.alpha) * fuel_cell.Universal_gas_constant * stack_temperature / (2 * fuel_cell.Faraday_constant) * xp.log(i_lim / xp.maximum(i_lim - i, 1e-30))
    )
    return eta_conc


def calculate_limiting_current_density_LT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, P_drop, i):
    """Calculates limiting current density for LT-PEM via linear interpolation.

    Interpolates between reference O2 partial pressures at 1 atm and 2.5 atm
    to estimate the limiting current density at the actual O2 partial pressure.

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack with current density limit multiplier.
    stack_temperature : ndarray or float
        Stack temperature [K].
    P_O2 : ndarray or float
        Oxygen partial pressure [bar].
    RH : float
        Relative humidity [0-1].
    air_excess_ratio : float
        Cathode stoichiometric ratio.
    P_drop : ndarray or float
        Cathode pressure drop [bar].
    i : ndarray or float
        Current density [A/cm^2].

    Returns
    -------
    ndarray or float
        Limiting current density [A/cm^2].
    """
    fuel_cell        = fuel_cell_stack.fuel_cell
    P_O2_ref_1_atm   = calculate_P_O2(fuel_cell_stack, 1, stack_temperature, RH, air_excess_ratio, P_drop, i)
    P_O2_ref_2_5_atm = calculate_P_O2(fuel_cell_stack, 2.5, stack_temperature, RH, air_excess_ratio, P_drop, i)
    i_lim            = (2.25 - 1.65) * (P_O2 - P_O2_ref_1_atm) / (P_O2_ref_2_5_atm - P_O2_ref_1_atm) + 1.65
    return fuel_cell.current_density_limit_multiplier * i_lim


def calculate_concentration_losses_HT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, P_drop, i):
    """Calculates concentration overpotential for high-temperature PEM.

    Same structure as the LT version but uses a different charge-transfer
    coefficient weighting (1.8/alpha instead of 1/alpha).

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack with charge-transfer coefficient (alpha).
    stack_temperature : ndarray or float
        Stack temperature [K].
    P_O2 : ndarray or float
        Oxygen partial pressure [bar].
    RH : float
        Relative humidity [0-1].
    air_excess_ratio : float
        Cathode stoichiometric ratio.
    P_drop : ndarray or float
        Cathode pressure drop [bar].
    i : ndarray or float
        Current density [A/cm^2].

    Returns
    -------
    ndarray or float
        Concentration voltage loss [V].
    """
    xp = array_namespace(stack_temperature, P_O2, RH, air_excess_ratio, P_drop, i)
    fuel_cell = fuel_cell_stack.fuel_cell
    i_lim     = calculate_limiting_current_density_HT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, P_drop, i)
    eta_conc  = xp.where(
        i >= i_lim,
        10.0,
        (1 + 1.8 / fuel_cell.alpha) * fuel_cell.Universal_gas_constant * stack_temperature / (2 * fuel_cell.Faraday_constant) * xp.log(i_lim / xp.maximum(i_lim - i, 1e-30))
    )
    return eta_conc


def calculate_limiting_current_density_HT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, P_drop, i):
    """Calculates limiting current density for HT-PEM via linear interpolation.

    Interpolates between reference O2 partial pressures at 1 atm and 2 atm.

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack with current density limit multiplier.
    stack_temperature : ndarray or float
        Stack temperature [K].
    P_O2 : ndarray or float
        Oxygen partial pressure [bar].
    RH : float
        Relative humidity [0-1].
    air_excess_ratio : float
        Cathode stoichiometric ratio.
    P_drop : ndarray or float
        Cathode pressure drop [bar].
    i : ndarray or float
        Current density [A/cm^2].

    Returns
    -------
    ndarray or float
        Limiting current density [A/cm^2].
    """
    fuel_cell      = fuel_cell_stack.fuel_cell
    P_O2_ref_1_atm = calculate_P_O2(fuel_cell_stack, 1, stack_temperature, RH, air_excess_ratio, P_drop, i)
    P_O2_ref_2_atm = calculate_P_O2(fuel_cell_stack, 2, stack_temperature, RH, air_excess_ratio, P_drop, i)
    i_lim          = (2.15 - 1.4) * (P_O2 - P_O2_ref_1_atm) / (P_O2_ref_2_atm - P_O2_ref_1_atm) + 1.4
    return fuel_cell.current_density_limit_multiplier * i_lim


# ----------------------------------------------------------------------------------------------------------------------
#  Sizing Helpers (scalar — used with scipy.optimize.minimize_scalar)
# ----------------------------------------------------------------------------------------------------------------------
def evaluate_max_gross_power(fuel_cell_stack, fuel_cell_conditions, t_idx):
    """Finds the current density that maximizes gross power for a single operating point.

    Used during fuel cell sizing to determine the rated operating point.
    Calls ``scipy.optimize.minimize_scalar`` on the negative power-density curve
    bounded between 20 % and 95 % of the limiting current density.

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack to size.
    fuel_cell_conditions : Conditions
        Conditions at the design point (scalar, indexed by t_idx).
    t_idx : int
        Time index for the single design point.

    Returns
    -------
    rated_current_density : float
        Current density at peak power [A/cm^2].
    rated_power_density : float
        Peak power density [W/cm^2].
    """
    fuel_cell         = fuel_cell_stack.fuel_cell
    FC_air_p          = fuel_cell.rated_air_pressure
    stack_temperature = fuel_cell_conditions.stack_temperature[t_idx, 0]
    RH                = fuel_cell.oxygen_relative_humidity
    air_excess_ratio  = fuel_cell.air_excess_ratio

    P_O2 = calculate_P_O2(fuel_cell_stack, FC_air_p, stack_temperature, RH, air_excess_ratio, fuel_cell.rated_p_drop_fc, 0)

    if fuel_cell.type == "LT":
        i_lim = calculate_limiting_current_density_LT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, 0, i=0)
    elif fuel_cell.type == "HT":
        i_lim = calculate_limiting_current_density_HT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, 0, i=0)

    res = minimize_scalar(evaluate_power_func, args=(fuel_cell_stack, fuel_cell_conditions, t_idx), bounds=(0.2 * i_lim, 0.95 * i_lim))
    rated_current_density = res.x
    rated_power_density   = -res.fun

    return rated_current_density, rated_power_density


def evaluate_power_func(i, fuel_cell_stack, fuel_cell_conditions, t_idx):
    """Objective function for sizing: returns negative power density at current density i.

    Scalar function passed to ``minimize_scalar``. Computes the full voltage model
    at a single operating point and returns ``-V_cell * i`` (negative because
    the optimizer minimizes).

    Parameters
    ----------
    i : float
        Current density to evaluate [A/cm^2].
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack with electrochemical parameters.
    fuel_cell_conditions : Conditions
        Conditions at the design point.
    t_idx : int
        Time index for the single design point.

    Returns
    -------
    float
        Negative power density ``-V_cell * i`` [W/cm^2].
    """
    fuel_cell         = fuel_cell_stack.fuel_cell
    stack_temperature = fuel_cell_conditions.stack_temperature[t_idx, 0]
    P_H2_input        = fuel_cell.rated_H2_pressure
    P_air             = fuel_cell.rated_air_pressure
    RH                = fuel_cell.oxygen_relative_humidity
    air_excess_ratio  = fuel_cell.air_excess_ratio
    pressure_drop     = fuel_cell_conditions.pressure_drop[t_idx, 0]
    degradation       = fuel_cell_conditions.degradation[t_idx, 0]

    P_O2   = calculate_P_O2(fuel_cell_stack, P_air, stack_temperature, RH, air_excess_ratio, pressure_drop, i)
    P_H2   = calculate_P_H2(fuel_cell_stack, P_H2_input, stack_temperature, RH, i)
    E_cell = calculate_E_cell(fuel_cell_stack, stack_temperature, P_H2, P_O2)

    if fuel_cell.type == "LT":
        eta_ohmic = calculate_ohmic_losses_LT(fuel_cell_stack, stack_temperature, i)
        eta_conc  = calculate_concentration_losses_LT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, pressure_drop, i)
    elif fuel_cell.type == "HT":
        eta_ohmic = calculate_ohmic_losses_HT(fuel_cell_stack, stack_temperature, i)
        eta_conc  = calculate_concentration_losses_HT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, pressure_drop, i)
    eta_act = calculate_activation_losses(fuel_cell_stack, stack_temperature, P_O2, i)

    V_cell = E_cell - eta_act - eta_ohmic - eta_conc - degradation * fuel_cell.maximum_deg
    return -V_cell * i

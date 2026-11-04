# RCAIDE/Library/Methods/Powertrain/Propulsors/Common/compute_gas_generator_shaft_power_offtake.py
#
#
# Created:  Sep 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  compute_gas_generator_shaft_power_offtake
# ----------------------------------------------------------------------------------------------------------------------
def compute_gas_generator_shaft_power_offtake(propulsor, state, omega):
    """
    Computes the shaft power a gas-turbine propulsor's integrated drive generator takes from, and its
    integrated drive motor delivers to, the gas-generator (high-pressure) spool, and stores the motor
    and generator operating conditions.

    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan, Turbojet or Turboprop
        Propulsor with optional integrated_drive_motor and integrated_drive_generator, and
        design_power_offtake [W].
    state : RCAIDE.Framework.Mission.Common.State
        Operating state. In a mission (time differentiation present) the electrical power is read
        from state.unknowns.network['electrical_power'] when present, else from
        conditions.energy.inputs.power.electrical.
    omega : numpy.ndarray
        Angular velocity of the spool the motor and generator are on [rad/s], for their torque.

    Returns
    -------
    external_shaft_power : numpy.ndarray
        Net shaft power taken from the gas-generator spool [W]: positive for a generator, negative
        for a motor.
    motor_electrical_power : numpy.ndarray
        Electrical power drawn by the motor from the bus [W].
    generator_electrical_power : numpy.ndarray
        Electrical power supplied by the generator to the bus [W].
    in_mission : bool
        True when the motor and generator were evaluated at the mission's electrical power; False
        when the design-point offtake was used.

    Notes
    -----
    Outside a mission (design point, sea-level static and sizing evaluations, reading back the
    design point for the off-design matching model) the engine runs with its design-point offtake,
    with the motor and generator at design_power_offtake as in the design functions (e.g.
    design_turbofan). The returned power is absolute; the cycle converts it to specific work with
    the core mass flow.
    """
    conditions                 = state.conditions
    propulsor_conditions       = conditions.energy.propulsors[propulsor.tag]
    integrated_drive_motor     = propulsor.integrated_drive_motor
    integrated_drive_generator = propulsor.integrated_drive_generator
    in_mission                 = len(state.numerics.time.differentiate) > 0
    external_shaft_power       = 0*state.ones_row(1)
    motor_electrical_power     = 0*state.ones_row(1)
    generator_electrical_power = 0*state.ones_row(1)

    if in_mission:
        if 'electrical_power' in state.unknowns.network:
            electrical_power = state.unknowns.network['electrical_power']
        else:
            electrical_power = conditions.energy.inputs.power.electrical

    # Motor: consumes electrical power from the bus, delivers mechanical power to the shaft
    if integrated_drive_motor != None:
        if in_mission:
            motor_electrical_power = electrical_power * conditions.energy.hybrid_power_split_ratio
        else:
            motor_electrical_power = propulsor.design_power_offtake * state.ones_row(1)
        motor_mechanical_power = motor_electrical_power * integrated_drive_motor.efficiency
        external_shaft_power   = external_shaft_power - motor_mechanical_power
        if in_mission:
            motor_conditions = conditions.energy.converters[integrated_drive_motor.tag]
            propulsor_conditions.inputs.power.electrical = motor_electrical_power
            motor_conditions.inputs.power.electrical     = motor_electrical_power
            motor_conditions.outputs.power.mechanical    = motor_mechanical_power
            motor_conditions.outputs.efficiency          = integrated_drive_motor.efficiency * state.ones_row(1)
            motor_conditions.outputs.omega               = omega
            motor_conditions.outputs.torque              = motor_mechanical_power / omega

    # Generator: extracts mechanical power from the shaft, provides electrical power to the bus
    if integrated_drive_generator != None:
        if in_mission:
            generator_electrical_power = electrical_power * integrated_drive_generator.power_split_ratio
        else:
            generator_electrical_power = propulsor.design_power_offtake * state.ones_row(1)
        generator_mechanical_power = generator_electrical_power / integrated_drive_generator.efficiency
        external_shaft_power       = external_shaft_power + generator_mechanical_power
        if in_mission:
            generator_conditions = conditions.energy.converters[integrated_drive_generator.tag]
            propulsor_conditions.outputs.power.electrical = generator_electrical_power
            generator_conditions.outputs.power.electrical = generator_electrical_power
            generator_conditions.inputs.power.mechanical  = generator_mechanical_power
            generator_conditions.outputs.efficiency       = integrated_drive_generator.efficiency * state.ones_row(1)
            generator_conditions.inputs.omega             = omega
            generator_conditions.inputs.torque            = generator_mechanical_power / omega

    return external_shaft_power, motor_electrical_power, generator_electrical_power, in_mission

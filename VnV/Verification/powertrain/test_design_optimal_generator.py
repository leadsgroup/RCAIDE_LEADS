# VnV/Verification/powertrain/test_design_optimal_generator.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Library.Methods.Powertrain.Converters.Generator  import design_optimal_generator, compute_generator_performance
from RCAIDE.Library.Methods.Powertrain                       import setup_operating_conditions
import numpy as np

# A sized generator, run by the performance model at its nominal voltage and design speed, must return the current
# and efficiency of its design point. The performance model is the independent reference.

# ----------------------------------------------------------------------------------------------------------------------
#  Main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    check_megawatt_generator()
    check_unattainable_efficiency()
    check_zero_design_power()
    check_negative_design_power()
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def sized_generator(voltage, power, angular_velocity, efficiency, no_load_current=1.0):
    generator                         = RCAIDE.Library.Components.Powertrain.Converters.Generator()
    generator.voltage_type            = 'DC'
    generator.nominal_voltage         = voltage
    generator.design_power            = power
    generator.design_angular_velocity = angular_velocity
    generator.efficiency              = efficiency
    generator.no_load_current         = no_load_current
    design_optimal_generator(generator)
    return generator


def design_point(generator):
    """Current and efficiency the performance model gives at the nominal voltage and design speed."""
    bus                             = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    generator.assigned_distributors = [[bus.tag]]
    state                           = setup_operating_conditions(generator, bus)
    conditions                      = state.conditions.energy.converters[generator.tag]
    conditions.inputs.omega[:, 0]            = generator.design_angular_velocity
    conditions.inputs.power.mechanical[:, 0] = generator.design_power
    conditions.outputs.voltage[:, 0]         = generator.nominal_voltage
    compute_generator_performance(generator, state.conditions)
    return conditions.outputs.current[0, 0], conditions.outputs.efficiency[0, 0]

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_megawatt_generator():
    # 2 MW at 0.93: the current is the power over the efficiency and the voltage
    generator           = sized_generator(voltage=800.0, power=2.0e6, angular_velocity=1500.0, efficiency=0.93)
    current, efficiency = design_point(generator)
    assert generator.speed_constant > 0 and generator.resistance > 0
    assert abs(current / (2.0e6 / (0.93 * 800.0)) - 1) < 1e-9
    assert abs(efficiency - 0.93) < 1e-9
    return


def check_unattainable_efficiency():
    # 9 kW at 400 V with a no-load current of 1 A cannot reach 0.98: 0.02 of 23 A is 0.46 A. The design point
    # must then be the generator's most efficient one: the design power is met, the efficiency is
    # (1 - no-load current / current)^2, and it is lower at currents 5% below and above
    generator           = sized_generator(voltage=400.0, power=9000.0, angular_velocity=100.0, efficiency=0.98)
    current, efficiency = design_point(generator)
    power               = 100.0 * (current - 1.0) / generator.speed_constant
    assert abs(power / 9000.0 - 1) < 1e-9
    assert efficiency < 0.98 and abs(efficiency - (1 - 1.0 / current)**2) < 1e-9

    def at(i):
        return (1 - 1.0 / i) * (1 - i * generator.resistance / 400.0)
    assert at(0.95 * current) < efficiency and at(1.05 * current) < efficiency
    return


def check_zero_design_power():
    # a generator sized before its load is known reaches the design speed at the nominal voltage
    generator = sized_generator(voltage=400.0, power=0.0, angular_velocity=100.0, efficiency=0.98)
    assert np.isfinite(generator.speed_constant) and abs(400.0 * generator.speed_constant - 100.0) < 1e-9
    assert generator.resistance > 0
    return


def check_negative_design_power():
    # a negative design power is not a generator's design point: sized as one without a load
    generator = sized_generator(voltage=400.0, power=-9000.0, angular_velocity=100.0, efficiency=0.98)
    unloaded  = sized_generator(voltage=400.0, power=0.0,     angular_velocity=100.0, efficiency=0.98)
    assert generator.speed_constant == unloaded.speed_constant and generator.resistance == unloaded.resistance
    assert generator.speed_constant > 0 and generator.resistance > 0
    return


if __name__ == '__main__':
    main()
    print('test_design_optimal_generator passed')

# VnV/Verification/powertrain/test_design_optimal_motor.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Library.Methods.Powertrain.Converters.Motor  import design_optimal_motor, compute_motor_performance
from RCAIDE.Library.Methods.Powertrain                   import setup_operating_conditions
import numpy as np

# A sized motor, run by the performance model at its nominal voltage and design current, must return its design
# torque, angular velocity and efficiency. The performance model is the independent reference.

# ----------------------------------------------------------------------------------------------------------------------
#  Main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    check_small_motor()
    check_megawatt_motor()
    check_unattainable_efficiency()
    check_zero_design_torque()
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def sized_motor(voltage, torque, angular_velocity, efficiency, no_load_current=1.0):
    motor                         = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor()
    motor.nominal_voltage         = voltage
    motor.design_torque           = torque
    motor.design_angular_velocity = angular_velocity
    motor.efficiency              = efficiency
    motor.no_load_current         = no_load_current
    design_optimal_motor(motor)
    return motor


def check_design_point(motor):
    bus                         = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    motor.assigned_distributors = [[bus.tag]]
    state                       = setup_operating_conditions(motor, bus)
    conditions                  = state.conditions.energy.converters[motor.tag]
    conditions.inputs.voltage[:, 0] = motor.nominal_voltage
    conditions.inputs.current[:, 0] = motor.design_current
    compute_motor_performance(motor, state.conditions)

    assert motor.speed_constant > 0 and motor.resistance > 0
    assert abs(conditions.outputs.torque[0, 0] / motor.design_torque - 1) < 1e-9
    assert abs(conditions.outputs.omega[0, 0] / motor.design_angular_velocity - 1) < 1e-9
    assert abs(conditions.efficiency[0, 0] - motor.efficiency) < 1e-9
    # energy: shaft power is the efficiency times the electrical power
    assert abs(conditions.outputs.power.mechanical[0, 0] / (motor.efficiency * motor.nominal_voltage * motor.design_current) - 1) < 1e-9
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_small_motor():
    # about 8 kW, the size of a small electric aircraft's motor
    check_design_point(sized_motor(voltage=400.0, torque=130.0, angular_velocity=63.0, efficiency=0.90))
    return


def check_megawatt_motor():
    # 1.85 MW, the size of a regional turboprop's engine; the optimizer this sizing replaced did not converge here
    motor = sized_motor(voltage=567.0, torque=14578.0, angular_velocity=126.6, efficiency=0.95)
    check_design_point(motor)
    assert abs(motor.design_torque * motor.design_angular_velocity - 1.8456e6) < 1e3
    return


def check_unattainable_efficiency():
    # the no-load current alone loses more than a 0.98 efficiency allows: 0.02 of 20.9 A is 0.42 A.
    # The motor must still meet its design torque and speed, and the design point must be its most efficient:
    # at the nominal voltage the efficiency is lower at currents 5% below and above the design current
    motor            = sized_motor(voltage=400.0, torque=130.0, angular_velocity=63.0, efficiency=0.98, no_load_current=5.0)
    reached          = motor.design_torque * motor.design_angular_velocity / (motor.nominal_voltage * motor.design_current)
    motor.efficiency = reached
    check_design_point(motor)
    assert reached < 0.98 and abs(reached - (1 - 5.0 / motor.design_current)**2) < 1e-12

    def efficiency(current):
        return (1 - motor.no_load_current / current) * (1 - current * motor.resistance / motor.nominal_voltage)
    assert efficiency(0.95 * motor.design_current) < reached and efficiency(1.05 * motor.design_current) < reached
    return


def check_zero_design_torque():
    # a motor sized before its load is known spins at the design speed on its no-load current
    motor = sized_motor(voltage=400.0, torque=0.0, angular_velocity=63.0, efficiency=0.98)
    assert np.isfinite(motor.speed_constant) and motor.speed_constant > 0 and motor.design_current == 1.0
    assert abs(motor.nominal_voltage * motor.speed_constant - 63.0) < 1e-9
    return


if __name__ == '__main__':
    main()
    print('test_design_optimal_motor passed')

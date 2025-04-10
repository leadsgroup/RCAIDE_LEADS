# DC_motor_validation.py
# 
# Created:  April 2025, M. Clarke, M. Guidotti

#----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import RCAIDE
from   RCAIDE.Library.Methods.Powertrain                  import setup_operating_conditions 
from   RCAIDE.Library.Methods.Powertrain.Converters       import Motor
import matplotlib.pyplot as plt
import numpy as np

#----------------------------------------------------------------------
#   Reference Values
# ----------------------------------------------------------------------

# DC Motor: https://www.raveo.cz/sites/default/files/dkm/katalogy/motory/DC%20MOTOR%20(15W~120W).pdf

#----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main(): 

    motor_current       = np.linspace(0, 48, 20) # [A]
    motor_rpm_vector    = []                     # [rpm]
    motor_torque_vector = []                     # [kgfcm]

    motor = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor()
    
    # Input data from Datasheet 
    motor.nominal_voltage               = 12     # [V]            nominal voltage
    motor.resistance                    = 0.25   # [Ω]            resistance
    motor.no_load_current               = 1.8    # [A]            no load current
    motor.speed_constant                = 38     # [rpm/V]        speed constant
    motor.efficiency                    = 0.8    # [-]            efficiency
    motor.gearbox.gear_ratio            = 0.8    # [-]            gear ratio
    motor.design_angular_velocity       = 325    # [rad/s]        design angular velocity
    motor.design_torque                 = 0.081  # [Nm]           design torque
    motor.design_current                = 3.3    # [A]            design current

    for i in range(len(motor_current)):
        # set up default operating conditions 
        operating_state  = setup_operating_conditions(motor) 
        
        # Assign conditions to the motor
        motor_conditions = operating_state.conditions.energy.converters[motor.tag]
        motor_conditions.inputs.voltage[:, 0] = motor.nominal_voltage # [V]
        motor_conditions.inputs.current[:, 0] = motor_current[i]      # [A]

        # run analysis 
        Motor.compute_motor_performance(motor,operating_state.conditions)       
    
        # Extract results
        motor_rpm_vector.append(motor_conditions.outputs.omega[0][0]*(60/(2*np.pi)))    # [rpm]
        motor_torque_vector.append(motor_conditions.outputs.torque[0][0]*10.1971621298) # [kgfcm]
        
    # Literature values
    x_current = [ 0, 0.9800000000000004, 1.9799999999999995, 2.9799999999999995, 3.9800000000000004, 4.979999999999999, 5.979999999999999, 6.979999999999999, 7.98, 8.979999999999999, 9.98, 10.979999999999999, 11.979999999999999, 12.979999999999999, 13.979999999999999, 14.979999999999999, 15.219999999999997]  
    y_current = [2.1929824561403564, 5.175438596491226, 8.24561403508772, 11.31578947368422, 14.298245614035096, 17.368421052631582, 20.43859649122807, 23.50877192982456, 26.578947368421055, 29.56140350877193, 32.631578947368425, 35.70175438596492, 38.771929824561404, 41.8421052631579, 44.91228070175438, 47.98245614035088, 48.68421052631579]
    x_rpm     = [0, 0.9800000000000004, 1.9799999999999995, 2.9799999999999995, 3.9800000000000004, 4.979999999999999, 5.979999999999999, 7, 7.98, 8.979999999999999, 9.999999999999998, 10.979999999999999, 11.979999999999999, 12.979999999999999, 13.979999999999999, 14.979999999999999, 15.399999999999997]
    y_rpm     = [3315.708644142167, 3100.7020623080293, 2885.6954804738916, 2670.6888986397535, 2455.682316805616, 2240.675734971478, 2025.6691531373401, 1810.6625713032022, 1595.655989469065, 1380.6494076349268, 1165.642825800789, 950.6362439666509, 735.6296621325127, 520.623080298376, 305.6164984642378, 90.60991663010009, -1.5357612988168512]

    # Plot results
    plot_power_and_torque(x_current, y_current, x_rpm, y_rpm, motor_torque_vector, motor_current, motor_rpm_vector)

    error = np.abs((motor_current[-1] - y_current[-1]) / y_current[-1]) * 100
    print("\nError in Current [%]:", error)
    assert error < 10

def plot_power_and_torque(x_current, y_current, x_rpm, y_rpm, motor_torque_vector, motor_current, motor_rpm_vector):
    
    fig, ax1 = plt.subplots(figsize=(10, 5))

    # Increase font size for labels and tick marks
    label_fontsize = 16
    tick_fontsize = 14

    color = 'tab:blue'
    ax1.set_xlabel('Torque [kgfcm]', fontsize=label_fontsize)
    ax1.set_ylabel('Speed [RPM]', color=color, fontsize=label_fontsize)
    ax1.set_xlim(0, 16)
    ax1.plot(x_rpm, y_rpm, '-', color=color, label='8DCG12-25-30 rpm')
    ax1.plot(motor_torque_vector, motor_rpm_vector, 'o', color='tab:cyan', label='RCAIDE DC Motor rpm')
    ax1.tick_params(axis='y', labelcolor=color, labelsize=tick_fontsize)

    ax2 = ax1.twinx()
    color = 'tab:orange'
    ax2.set_ylabel('Current [A]', color=color, fontsize=label_fontsize)
    ax2.set_xlim(0, 16)
    ax2.plot(x_current, y_current, '-', color=color, label='8DCG12-25-30 current')
    ax2.plot(motor_torque_vector, motor_current, 'o', color='tab:orange', label='RCAIDE DC Motor current')
    ax2.tick_params(axis='y', labelcolor=color, labelsize=tick_fontsize)

    ax1.tick_params(axis='x', labelsize=tick_fontsize)
    # Combine legends from both axes and place on top
    lines, labels = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines + lines2, labels + labels2, loc='upper center', bbox_to_anchor=(0.5, 1.35), fontsize=label_fontsize, ncol=2)
    fig.tight_layout()

# ----------------------------------------------------------------------        
#   Call Main
# ----------------------------------------------------------------------    

if __name__ == '__main__':
    main()
    plt.show()
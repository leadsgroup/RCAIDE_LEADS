# PMSM_motor_validation.py
# 
# Created:  April 2025, M. Clarke, M. Guidotti

#----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import RCAIDE
from   RCAIDE.Library.Methods.Powertrain            import setup_operating_conditions 
from   RCAIDE.Library.Methods.Powertrain.Converters import Motor
import matplotlib.pyplot as plt
import numpy as np

#----------------------------------------------------------------------
#   Reference Values
# ----------------------------------------------------------------------

# PMSM Motor: https://emrax.com/e-motors/emrax-348/

#----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main(): 

    motor_voltage       = np.linspace(0, 610, 20) # [V]
    motor_rpm_vector    = []                      # [rpm]
    motor_torque_vector = []                      # [Nm]
    motor_power_vector  = []                      # [W]

    motor = RCAIDE.Library.Components.Powertrain.Converters.PMSM_Motor()
    
    motor.nominal_current       = 600             # [A]      nominal current
    motor.efficiency            = 0.96            # [-]      efficiency
    motor.speed_constant        = 6.56            # [rpm/V]  speed constant
    motor.stator_inner_diameter = 0.068           # [m]      stator inner diameter
    motor.stator_outer_diameter = 0.16            # [m]      stator outer diameter
    motor.resistance            = 0.0042          # [Ω]      resistance
    motor.winding_factor        = 0.95            # [-]      winding factor
    motor.motor_stack_length    = 11.4            # [m]      motor stack length 
    motor.number_of_turns       = 80              # [-]      number of turns  
    motor.length_of_path        = 0.4             # [m]      length of the path  
    motor.mu_0                  = 1.256637061e-6  # [N/A**2] permeability of free space
    motor.mu_r                  = 1005            # [N/A**2] relative permeability of the magnetic material 

    for i in range(len(motor_voltage)):

        # set up default operating conditions 
        operating_state  = setup_operating_conditions(motor) 
        
        # Assign conditions to the motor
        motor_conditions = operating_state.conditions.energy.converters[motor.tag]

        motor_conditions.inputs.voltage[:, 0] = motor_voltage[i]      # [V]
        motor_conditions.inputs.current[:, 0] = motor.nominal_current # [A]

        # run analysis 
        Motor.compute_motor_performance(motor,operating_state.conditions)       
    
        # Extract results
        motor_rpm_vector.append(motor_conditions.outputs.omega[0][0]*(60/(2*np.pi))) # [rpm]
        motor_torque_vector.append(motor_conditions.outputs.torque[0][0])            # [Nm]
        motor_power_vector.append(motor_conditions.outputs.power[0][0]/1000)         # [W]

    # Literature values
    x_cont_pow = [0,504.2283298097251,1003.6997885835095,1503.1712473572939,2002.6427061310778,2502.1141649048623,3006.3424947145872,3505.8139534883717,3795.9830866807606,3895.877378435517,4005.2854122621557]  
    y_cont_pow = [0,22.293577981651424,44.587155963302735,66.88073394495416,89.17431192660553,111.4678899082569,134.58715596330273,156.05504587155963,170.09174311926608,175.04587155963304,177.52293577981652]
    x_cont_tq  = [0,123.54804646251324,247.09609292502648,370.64413938753967,498.94403379091875,627.2439281942978,1002.6399155227035,1496.8321013727564,2000.5279831045405,2504.223864836325,2998.416050686378,3502.1119324181623,3872.7560718057025,3939.281942977825,4001.0559662090814]
    y_cont_tq  = [398.5321100917432,405.1376146788991,413.9449541284405,418.34862385321105,424.95412844036707,424.95412844036707,424.95412844036707,424.95412844036707,424.95412844036707,424.95412844036707,422.75229357798173,427.1559633027523,427.1559633027523,427.1559633027523,424.95412844036707]

    plot_power_and_torque(x_cont_pow, y_cont_pow, motor_rpm_vector, motor_power_vector, x_cont_tq, y_cont_tq, motor_torque_vector)

def plot_power_and_torque(x_cont_pow, y_cont_pow, motor_rpm_vector, motor_power_vector, x_cont_tq, y_cont_tq, motor_torque_vector):
    
    fig, ax1 = plt.subplots(figsize=(10, 5))
    label_fontsize = 16
    tick_fontsize = 14
    color = 'tab:blue'
    ax1.set_xlabel('[RPM]', fontsize=label_fontsize)
    ax1.set_ylabel('Power [kW]', color=color, fontsize=label_fontsize)
    ax1.plot(x_cont_pow, y_cont_pow, '-', color=color, label='EMRAX 348 Power')
    ax1.plot(motor_rpm_vector, motor_power_vector, 'o', color='tab:cyan', label='RCAIDE PMSM Power')
    ax1.tick_params(axis='y', labelcolor=color, labelsize=tick_fontsize)
    ax2 = ax1.twinx()
    color = 'tab:orange'
    ax2.set_ylabel('Torque [Nm]', color=color, fontsize=label_fontsize)
    ax2.set_ylim(0, 600)
    ax2.plot(x_cont_tq, y_cont_tq, '-', color=color, label='EMRAX 348 Torque')
    ax2.plot(motor_rpm_vector, motor_torque_vector, 'o', color='tab:orange', label='RCAIDE PMSM Torque')
    ax2.tick_params(axis='y', labelcolor=color, labelsize=tick_fontsize)
    ax1.tick_params(axis='x', labelsize=tick_fontsize)
    lines, labels = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines + lines2, labels + labels2, loc='upper center', bbox_to_anchor=(0.5, 1.35), fontsize=label_fontsize, ncol=2)
    fig.tight_layout()
    fig.savefig('PMSM_motor_validation.png', dpi=300)

# ----------------------------------------------------------------------        
#   Call Main
# ----------------------------------------------------------------------    

if __name__ == '__main__':
    main()
    plt.show()
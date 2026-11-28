# RCAIDE/Library/Methods/Powertrain/Converters/DC_Motor/design_optimal_motor.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  design motor 
# ----------------------------------------------------------------------------------------------------------------------     
def design_optimal_motor(motor, minimum_resistance=0.001):
    """
    Sizes a DC motor: the speed constant and resistance that give the design torque at the design
    angular velocity and nominal voltage with the specified efficiency.
    
    Parameters
    ----------
    motor : RCAIDE.Library.Components.Powertrain.Converters.DC_Motor
        Motor component with the following attributes:
            - no_load_current : float
                No-load current [A]
            - nominal_voltage : float
                Nominal voltage [V]
            - design_angular_velocity : float
                Angular velocity [radians/s]
            - efficiency : float
                Efficiency [unitless]
            - design_torque : float
                Design torque [N·m]
            - gearbox : Data
                Gearbox component
                    - gear_ratio : float
                        Gear ratio [unitless]
    minimum_resistance : float, optional
        Resistance given to a motor sized without a design load [ohms]. Default is 0.001.
    
    Returns
    -------
    motor : RCAIDE.Library.Components.Powertrain.Converters.DC_Motor
        Motor with updated attributes:
            - speed_constant : float
                Speed constant [rad/s/V]
            - resistance : float
                Resistance [ohms]
            - design_current : float
                Design current [A]
    
    Notes
    -----
    The two design conditions fix the two unknowns, so the solution is direct.
    
    **Major Assumptions**
        * The motor follows a DC motor model
    
    **Theory**
    The motor model is that of Drela [1], Eqs. (1), (2), (5) and (6):
        - Current: :math:`I = (V - \\omega/Kv)/R`
        - Torque: :math:`Q = (I - I_0)/Kv`
        - Efficiency: :math:`\\eta = (1 - I_0/I)\\cdot(\\omega/(V\\cdot Kv)) = Q\\omega/(V I)`
    
    so that
        - :math:`I = Q\\omega/(\\eta V)`
        - :math:`Kv = (I - I_0)/Q`
        - :math:`R = (V - \\omega/Kv)/I`
    
    where V is the nominal voltage, ω the angular velocity of the motor, Kv the speed constant,
    R the resistance, I₀ the no-load current, Q the torque and η the efficiency.
    
    This solution exists only if the no-load current is below :math:`(1 - \\eta) I`, since the no-load
    loss alone must be smaller than the total loss the efficiency allows. Otherwise the motor is sized so
    that the design point is its most efficient one. From Eq. (5) of [1], the efficiency at a given voltage
    peaks where :math:`I^2 R = I_0 V`, and is then :math:`(1 - I_0/I)^2`. With the design torque this gives
    :math:`(I - I_0)^2 = (Q\\omega/V) I`, then :math:`R = I_0 V/I^2` and :math:`Kv = (I - I_0)/Q`.
    
    A motor with no design torque, or a negative one (a rotor design that did not converge can return one),
    is given a speed constant that reaches the design speed at the nominal voltage, and the minimum resistance.
    
    References
    ----------
    [1] Drela, M., "DC Motor / Propeller Matching," Lab 5 Lecture Notes, MIT, 3 Mar. 2005.
        https://web.mit.edu/drela/Public/web/qprop/motorprop.pdf
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Motor.compute_motor_performance
    """ 
    # design properties of the motor 
    io     = motor.no_load_current
    v      = motor.nominal_voltage
    G      = motor.gearbox.gear_ratio      
    omega  = motor.design_angular_velocity * G
    etam   = motor.efficiency 
    Q      = motor.design_torque
    
    current = Q * omega / (etam * v)
    if Q <= 0:
        motor.speed_constant = omega / v
        motor.resistance     = minimum_resistance
        current              = io
    elif io < (1 - etam) * current:
        motor.speed_constant = (current - io) / Q
        motor.resistance     = (v - omega / motor.speed_constant) / current
    else:
        # the efficiency cannot be met: make the design point the motor's most efficient one
        c                    = Q * omega / v
        current              = ((2 * io + c) + np.sqrt((2 * io + c)**2 - 4 * io**2)) / 2
        motor.resistance     = io * v / current**2
        motor.speed_constant = (current - io) / Q
        print('\n Motor ' + motor.tag + ' cannot reach an efficiency of ' + str(etam) + ' with a no-load current of ' + str(io)
              + ' A. Sized for its peak efficiency of ' + str((1 - io / current)**2) + ' at the design point')
    
    motor.design_current   = current
    return motor  

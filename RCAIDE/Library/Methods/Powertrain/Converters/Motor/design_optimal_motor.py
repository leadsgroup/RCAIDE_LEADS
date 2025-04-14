# RCAIDE/Library/Methods/Powertrain/Converters/DC_Motor/design_optimal_motor.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE

# python imports  
from scipy.optimize import minimize 

# ----------------------------------------------------------------------------------------------------------------------
#  design motor 
# ----------------------------------------------------------------------------------------------------------------------     
def design_optimal_motor(motor):
    """
    Sizes a DC motor to obtain the best combination of speed constant and resistance values
    by sizing the motor for a design RPM value.
    
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
    
    Returns
    -------
    motor : RCAIDE.Library.Components.Powertrain.Converters.DC_Motor
        Motor with updated attributes:
            - speed_constant : float
                Speed constant [unitless]
            - resistance : float
                Resistance [ohms]
            - design_current : float
                Design current [A]
    
    Notes
    -----
    This function uses numerical optimization to find the optimal values of speed constant
    and resistance that satisfy the motor's design requirements. It attempts to solve
    the optimization problem with hard constraints first, and if that fails, it uses slack
    constraints.
    
    The optimization process follows these steps:
        1. Extract motor design parameters (voltage, angular velocity, efficiency, torque)
        2. Define optimization bounds for speed constant and resistance
        3. Set up hard constraints for efficiency and torque
        4. Attempt optimization with hard constraints
        5. If optimization fails, retry with slack constraints
        6. Update the motor with the optimized parameters
    
    The objective function minimizes the current draw for a given voltage, angular velocity,
    and torque requirement.
    
    **Major Assumptions**
        * The motor follows a DC motor model
        * The optimization bounds are appropriate for the motor size
        * If hard constraints cannot be satisfied, slack constraints are acceptable
    
    **Theory**
    The motor model uses the following relationships:
        - Current: :math:`I = (V - \\omega/Kv)/R`
        - Torque: :math:`T = (I - I₀)/Kv`
        - Efficiency: :math:`\\eta = (1 - (I₀\\cdot R)/(V - \\omega/Kv))\\cdot(\\omega/(V\\cdot Kv))`
    
    where:
        - V is the nominal voltage
        - ω is the angular velocity
        - Kv is the speed constant
        - R is the resistance
        - I₀ is the no-load current
        - T is the torque
        - η is the efficiency
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Motor.compute_motor_performance
    """

    if type(motor) != RCAIDE.Library.Components.Powertrain.Converters.DC_Motor:
        raise Exception('function only supports low-fidelity (DC) motor')
    
    # design properties of the motor 
    io     = motor.no_load_current
    v      = motor.nominal_voltage
    G      = motor.gearbox.gear_ratio      
    omega  = motor.design_angular_velocity 
    etam   = motor.efficiency 
    Q      = motor.design_torque
    
    # define optimizer bounds 
    KV_lower_bound  = 0.01
    KV_upper_bound  = 100
    Res_lower_bound = 0.001
    Res_upper_bound = 10
    
    args       = (v , omega,  etam , Q , io,G ) 
    hard_cons  = [{'type':'eq', 'fun': hard_constraint_1,'args': args},{'type':'eq', 'fun': hard_constraint_2,'args': args}] 
    slack_cons = [{'type':'eq', 'fun': slack_constraint_1,'args': args},{'type':'eq', 'fun': slack_constraint_2,'args': args}]  
    bnds       = ((KV_lower_bound, KV_upper_bound), (Res_lower_bound , Res_upper_bound)) 
    
    # try hard constraints to find optimum motor parameters
    sol = minimize(objective, [0.5, 0.1], args=(v , omega,  etam , Q , io,G) , method='SLSQP', bounds=bnds, tol=1e-6, constraints=hard_cons) 
    
    if sol.success == False:
        # use slack constraints if optimizer fails and motor parameters cannot be found 
        print('\n Optimum motor design failed. Using slack constraints')
        sol = minimize(objective, [0.5, 0.1], args=(v , omega,  etam , Q , io,G) , method='SLSQP', bounds=bnds, tol=1e-6, constraints=slack_cons) 
        if sol.success == False:
            assert('\n Slack contraints failed')  
    
    motor.speed_constant   = sol.x[0]
    motor.resistance       = sol.x[1]
    internal_omega         = omega * G
    motor.design_current   = (v-(internal_omega)/motor.speed_constant)/motor.resistance  
    return motor  
  
# objective function
def objective(x, v , omega,  etam , Q , io,G ): 
    internal_omega = omega * G    
    return (v - internal_omega/x[0])/x[1]   

# hard efficiency constraint
def hard_constraint_1(x, v , omega,  etam , Q , io,G ): 
    internal_omega = omega * G    
    return etam - (1- (io*x[1])/(v - internal_omega/x[0]))*(internal_omega/(v*x[0]))   

# hard torque equality constraint
def hard_constraint_2(x, v , omega,  etam , Q , io,G ): 
    internal_omega = omega * G    
    return ((v - internal_omega/x[0])/x[1] - io)/x[0] - Q  

# slack efficiency constraint 
def slack_constraint_1(x, v , omega,  etam , Q , io,G ): 
    internal_omega = omega * G   
    return abs(etam - (1- (io*x[1])/(v - internal_omega/x[0]))*(internal_omega/(v*x[0]))) - 0.2

# slack torque equality constraint 
def slack_constraint_2(x, v , omega,  etam , Q , io,G ): 
    internal_omega = omega * G   
    return  abs(((v - internal_omega/x[0])/x[1] - io)/x[0] - Q) - 200 
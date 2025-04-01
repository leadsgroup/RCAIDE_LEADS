# RCAIDE/Library/Methods/Powertrain/Converters/DC_Motor/design_motor.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------    
# python imports  
from scipy.optimize import minimize 

# ----------------------------------------------------------------------------------------------------------------------
#  design motor 
# ----------------------------------------------------------------------------------------------------------------------     
def design_DC_motor(motor):
    """
    Sizes a DC motor by optimizing speed constant and resistance values for a given design point.
    
    Parameters
    ----------
    motor : DCMotor
        The DC motor component with the following attributes:
            - no_load_current : float
                Current drawn by the motor with no load [A]
            - nominal_voltage : float
                Nominal operating voltage [V]
            - angular_velocity : float
                Design angular velocity [rad/s]
            - efficiency : float
                Target efficiency at design point [unitless]
            - design_torque : float
                Design torque [N·m]
    
    Returns
    -------
    motor : DCMotor
        The same motor object with the following additional attributes:
            - speed_constant : float
                Optimized speed constant [rad/(V·s)]
            - resistance : float
                Optimized internal resistance [Ω]
    
    Notes
    -----
    This function uses numerical optimization to find the best combination of 
    speed constant and resistance values that satisfy the design requirements.
    
    The optimization attempts to find values that exactly meet the efficiency and
    torque constraints. If this fails, the function falls back to slack constraints
    that allow for small deviations from the target values.
    
    **Theory**
    
    The DC motor model uses the following relationships:
    
    .. math::
        I = \\frac{V - \\omega/K_V}{R} \\quad \\text{(Current)}
    
    .. math::
        \\eta = \\left(1 - \\frac{I_0 R}{V - \\omega/K_V}\\right)\\frac{\\omega}{V K_V} \\quad \\text{(Efficiency)}
    
    .. math::
        Q = \\frac{I - I_0}{K_V} \\quad \\text{(Torque)}
    
    where:
    - :math:`K_V` is the speed constant
    - :math:`R` is the internal resistance
    - :math:`I_0` is the no-load current
    - :math:`\\omega` is the angular velocity
    - :math:`V` is the nominal voltage
    - :math:`\\eta` is the efficiency
    - :math:`Q` is the torque
    
    See Also
    --------
    scipy.optimize.minimize : The optimization function used internally
    """
    # design properties of the motor 
    io    = motor.no_load_current
    v     = motor.nominal_voltage
    omeg  = motor.angular_velocity     
    etam  = motor.efficiency 
    Q     = motor.design_torque 
    
    # define optimizer bounds 
    KV_lower_bound  = 0.01
    KV_upper_bound  = 100
    Res_lower_bound = 0.001
    Res_upper_bound = 10
    
    args       = (v , omeg,  etam , Q , io ) 
    hard_cons  = [{'type':'eq', 'fun': hard_constraint_1,'args': args},{'type':'eq', 'fun': hard_constraint_2,'args': args}] 
    slack_cons = [{'type':'eq', 'fun': slack_constraint_1,'args': args},{'type':'eq', 'fun': slack_constraint_2,'args': args}]  
    bnds       = ((KV_lower_bound, KV_upper_bound), (Res_lower_bound , Res_upper_bound)) 
    
    # try hard constraints to find optimum motor parameters
    sol = minimize(objective, [0.5, 0.1], args=(v , omeg,  etam , Q , io) , method='SLSQP', bounds=bnds, tol=1e-6, constraints=hard_cons) 
    
    if sol.success == False:
        # use slack constraints if optimizer fails and motor parameters cannot be found 
        print('\n Optimum motor design failed. Using slack constraints')
        sol = minimize(objective, [0.5, 0.1], args=(v , omeg,  etam , Q , io) , method='SLSQP', bounds=bnds, tol=1e-6, constraints=slack_cons) 
        if sol.success == False:
            assert('\n Slack contraints failed')  
    
    motor.speed_constant   = sol.x[0]
    motor.resistance       = sol.x[1]    
    
    return motor  
  
# objective function
def objective(x, v , omeg,  etam , Q , io ): 
    return (v - omeg/x[0])/x[1]   

# hard efficiency constraint
def hard_constraint_1(x, v , omeg,  etam , Q , io ): 
    return etam - (1- (io*x[1])/(v - omeg/x[0]))*(omeg/(v*x[0]))   

# hard torque equality constraint
def hard_constraint_2(x, v , omeg,  etam , Q , io ): 
    return ((v - omeg/x[0])/x[1] - io)/x[0] - Q  

# slack efficiency constraint 
def slack_constraint_1(x, v , omeg,  etam , Q , io ): 
    return abs(etam - (1- (io*x[1])/(v - omeg/x[0]))*(omeg/(v*x[0]))) - 0.2

# slack torque equality constraint 
def slack_constraint_2(x, v , omeg,  etam , Q , io ): 
    return  abs(((v - omeg/x[0])/x[1] - io)/x[0] - Q) - 200 
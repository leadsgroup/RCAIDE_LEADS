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
    ''' Sizes a DC motor to obtain the best combination of speed constant and resistance values
    by sizing the motor for a design RPM value. Note that this design RPM value can be compute
    from design tip mach. The following properties are computed.  
      motor.speed_constant  (float): speed-constant [untiless] 
      motor.resistance      (float): resistance     [ohms]        
    
    Assumptions:
        None 
    
    Source:
        None
    
    Args:
        motor.no_load_current  (float): no-load current  [A]
        motor.nominal_voltage  (float): nominal voltage  [V]
        motor.angular_velocity (float): angular velocity [radians/s]    
        motor.efficiency       (float): efficiency       [unitless]
        motor.design_torque    (float): design torque    [Nm]
       
    Returns:
       None 
    
    '''     
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
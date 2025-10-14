# RCAIDE/Library/Methods/Powertrain/Converters/DC_generator/design_optimal_generator.py
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE

# python imports  
from scipy.optimize import minimize 

# ----------------------------------------------------------------------------------------------------------------------
#  design generator 
# ----------------------------------------------------------------------------------------------------------------------     
def design_optimal_generator(generator):
    """
    Sizes a generator to obtain the best combination of speed constant and resistance values
    by sizing the generator for a design RPM value.
    
    Parameters
    ----------
    generator : RCAIDE.Library.Components.Powertrain.Converters.DC_Generator
        Generator component with the following attributes:
            - no_load_current : float
                No-load current [A]
            - nominal_voltage : float
                Nominal voltage [V]
            - angular_velocity : float
                Angular velocity [radians/s]
            - efficiency : float
                Efficiency [unitless]
            - design_torque : float
                Design torque [N·m]
            - gearbox : Data
                Gearbox component
                    - gear_ratio : float
                        Gear ratio [unitless]
            - design_angular_velocity : float
                Design angular velocity [radians/s]
            - design_power : float
                Design power [W]
    
    Returns
    -------
    generator : RCAIDE.Library.Components.Powertrain.Converters.DC_Generator
        Generator with updated attributes:
            - speed_constant : float
                Speed constant [unitless]
            - resistance : float
                Resistance [ohms]
    
    Notes
    -----
    This function uses numerical optimization to find the optimal values of speed constant
    and resistance that satisfy the generator's design requirements. It attempts to solve
    the optimization problem with hard constraints first, and if that fails, it uses slack
    constraints.
    
    The optimization process follows these steps:
        1. Extract generator design parameters (voltage, angular velocity, efficiency, power)
        2. Define optimization bounds for speed constant and resistance
        3. Set up hard constraints for efficiency and power
        4. Attempt optimization with hard constraints
        5. If optimization fails, retry with slack constraints
        6. Update the generator with the optimized parameters
    
    The objective function maximizes the current output for a given voltage and angular velocity.
    
    **Major Assumptions**
        * The generator follows a DC generator model
        * The optimization bounds are appropriate for the generator size
        * If hard constraints cannot be satisfied, slack constraints are acceptable
    
    **Theory**
    The generator model uses the following relationships:
        - Current: :math:`I = (V - \\omega/Kv)/R - I₀`
        - Efficiency: :math:`\\eta = (1 - (I₀\\cdot R)/(V - \\omega/Kv))\\cdot(\\omega/(V\\cdot Kv))`
        - Power: :math:`P = \\omega\\cdot I/Kv`
    
    where:
        - V is the nominal voltage
        - ω is the angular velocity
        - Kv is the speed constant
        - R is the resistance
        - I₀ is the no-load current
        - η is the efficiency
        - P is the power
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Generator.compute_generator_performance
    """
    
    if type(generator) != RCAIDE.Library.Components.Powertrain.Converters.DC_Generator:
        raise Exception('function only supports low-fidelity (DC) generator')
    
    # design properties of the generator 
    io     = generator.no_load_current
    v      = generator.nominal_voltage  
    G      = generator.gearbox.gear_ratio      
    omega  = generator.design_angular_velocity /G     
    etam   = generator.efficiency 
    P      = generator.design_power
    
    # define optimizer bounds 
    KV_lower_bound  = 0.01
    KV_upper_bound  = 100
    Res_lower_bound = 0.001
    Res_upper_bound = 10
    
    args       = (v , omega,  etam , P , io ) 
    hard_cons  = [{'type':'eq', 'fun': hard_constraint_1,'args': args},{'type':'eq', 'fun': hard_constraint_2,'args': args}] 
    slack_cons = [{'type':'eq', 'fun': slack_constraint_1,'args': args},{'type':'eq', 'fun': slack_constraint_2,'args': args}]  
    bnds       = ((KV_lower_bound, KV_upper_bound), (Res_lower_bound , Res_upper_bound)) 
    
    # try hard constraints to find optimum generator parameters
    sol = minimize(objective, [0.5, 0.1], args=(v , omega,  etam , P , io) , method='SLSQP', bounds=bnds, tol=1e-6, constraints=hard_cons) 
    
    if sol.success == False:
        # use slack constraints if optimizer fails and generator parameters cannot be found 
        print('\n Optimum generator design failed. Using slack constraints')
        sol = minimize(objective, [0.5, 0.1], args=(v , omega,  etam , P , io) , method='SLSQP', bounds=bnds, tol=1e-6, constraints=slack_cons) 
        if sol.success == False:
            assert('\n Slack contraints failed')  
    
    generator.speed_constant   = sol.x[0]
    generator.resistance       = sol.x[1]    
    
    return generator  
  
# objective function
def objective(x, v , omega,  etam , P , io ): 
    return (v - omega/x[0])/x[1]   

# hard efficiency constraint
def hard_constraint_1(x, v , omega,  etam , P , io ): 
    return etam - (1- (io*x[1])/(v - omega/x[0]))*(omega/(v*x[0]))   

# hard torque equality constraint
def hard_constraint_2(x, v , omega,  etam , P , io ): 
    P_guess = omega * ((v - omega/x[0])/x[1] - io)/x[0]
    return P_guess - P  

# slack efficiency constraint 
def slack_constraint_1(x, v , omega,  etam , P , io ): 
    return abs(etam - (1- (io*x[1])/(v - omega/x[0]))*(omega/(v*x[0]))) - 0.2

# slack torque equality constraint 
def slack_constraint_2(x, v , omega,  etam , P , io ): 
    P_guess = omega * ((v - omega/x[0])/x[1] - io)/x[0] 
    return  abs(P_guess - P) - 200 
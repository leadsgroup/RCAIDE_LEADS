# RCAIDE/Methods/Energy/Propulsors/design_lift_rotor.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  

# RCAIDE Imports   
from RCAIDE.Framework.Optimization.Packages.scipy                                                import scipy_setup       
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Design.optimization_setup       import optimization_setup
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Design.set_optimized_parameters import set_optimized_parameters

# Python package imports   
import time 
import os
import sys

# ----------------------------------------------------------------------------------------------------------------------  
#  Design Lift-rotor
# ----------------------------------------------------------------------------------------------------------------------  
def design_lift_rotor(rotor, number_of_stations=20, solver_name='SLSQP', iterations=200,
                      solver_sense_step=1E-6, solver_tolerance=1E-5, print_iterations=False):
    """
    Optimizes rotor chord and twist distribution to meet design power or thrust requirements.
    
    Parameters
    ----------
    rotor : RCAIDE.Library.Components.Powertrain.Converters.Rotor
        Rotor component with the following attributes:
            - tag : str
                Identifier for the rotor
            - hub_radius : float
                Hub radius of the rotor [m]
            - tip_radius : float
                Tip radius of the rotor [m]
            - rotation_rate : float
                Rotation rate [rad/s]
            - freestream_velocity : float
                Freestream velocity [m/s]
            - number_of_blades : int
                Number of blades on the rotor
            - design_lift_coefficient : float
                Design lift coefficient
            - airfoil_data : dict
                Dictionary of airfoil data
            - optimization_parameters : Data
                Optimization parameters
                    - slack_constraint : float
                        Slack constraint value
                    - ideal_SPL_dbA : float
                        Ideal sound pressure level [dBA]
                    - multiobjective_aeroacoustic_weight : float
                        Weight for multiobjective aeroacoustic optimization
    number_of_stations : int, optional
        Number of radial stations for blade discretization, default 20
    solver_name : str, optional
        Name of the optimization solver, default 'SLSQP'
    iterations : int, optional
        Maximum number of iterations, default 200
    solver_sense_step : float, optional
        Step size for finite difference gradient calculation, default 1E-6
    solver_tolerance : float, optional
        Convergence tolerance for the optimizer, default 1E-5
    print_iterations : bool, optional
        Flag to print optimization iterations, default False
    
    Returns
    -------
    rotor : RCAIDE.Library.Components.Powertrain.Converters.Rotor
        Optimized rotor with updated chord and twist distributions
    
    Notes
    -----
    This function optimizes the chord and twist distributions of a lift rotor to meet
    either design power or thrust requirements. It uses RCAIDE's native optimization
    framework with an objective function that balances aerodynamic performance and
    acoustic characteristics.
    
    The optimization process follows these steps:
        1. Set up the optimization problem using the optimization_setup function
        2. Solve the optimization problem using the specified solver
        3. Report optimization results
        4. Update the rotor with the optimized parameters
    
    The objective function is formulated as an aeroacoustic function that considers
    both efficiency and radiated noise, with the balance controlled by the
    multiobjective_aeroacoustic_weight parameter.
    
    **Major Assumptions**
        * Rotor blade design considers one engine inoperative scenario
        * Either design power or thrust must be specified (not both)
        * The optimization balances aerodynamic performance and acoustic characteristics
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Design.optimization_setup
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Design.set_optimized_parameters
    RCAIDE.Framework.Optimization.Packages.scipy.scipy_setup
    """    
    # Unpack rotor geometry  
    rotor_tag     = rotor.tag
    rotor.tag     = 'rotor'
    
    # start optimization 
    ti                   = time.time()   
    optimization_problem = optimization_setup(rotor,number_of_stations,print_iterations)

    # Commense suppression of console window output  
    devnull = open(os.devnull,'w')
    sys.stdout = devnull 
    outputs               = scipy_setup.SciPy_Solve(optimization_problem,
                                                   solver=solver_name,
                                                   iter = iterations ,
                                                   sense_step = solver_sense_step,
                                                   tolerance = solver_tolerance)
    # Terminate suppression of console window output   
    sys.stdout = sys.__stdout__

    if outputs[3] != 0:  
        print('Lift-rotor Optimization Failed: ', outputs[4] )   
    else:
        print('Lift-rotor Optimization Successful')  
        
    tf                   = time.time()
    elapsed_time         = round((tf-ti)/60,2)
    print('Simulation Time: ' + str(elapsed_time) + ' mins')   
     
    # set remaining rotor variables using optimized parameters 
    rotor     = set_optimized_parameters(rotor,optimization_problem)
    rotor.tag = rotor_tag
    
    return rotor 
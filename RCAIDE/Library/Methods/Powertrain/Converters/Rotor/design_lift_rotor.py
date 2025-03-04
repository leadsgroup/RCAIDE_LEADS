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

# ----------------------------------------------------------------------------------------------------------------------  
#  Design Lift-rotor
# ----------------------------------------------------------------------------------------------------------------------  
def design_lift_rotor(rotor,number_of_stations = 20,solver_name= 'SLSQP',iterations = 200,
                      solver_sense_step = 1E-5,solver_tolerance = 1E-4,print_iterations = False):  
    """ Optimizes rotor chord and twist given input parameters to meet either design power or thurst. 
        This scrip adopts RCAIDE's native optimization style where the objective function is expressed 
        as an aeroacoustic function, considering both efficiency and radiated noise.
          
          Inputs: 
          prop_attributes.
              hub radius                             [m]
              tip radius                             [m]
              rotation rate                          [rad/s]
              freestream velocity                    [m/s]
              number of blades                       [None]       
              number of stations                     [None]
              design lift coefficient                [None]
              airfoil data                           [None]
              optimization_parameters.         
                 slack_constaint                     [None]
                 ideal_SPL_dbA                       [dBA]
                 multiobjective_aeroacoustic_weight  [None]
            
          Outputs:
          Twist distribution                         [array of radians]
          Chord distribution                         [array of meters]
              
          Assumptions: 
             Rotor blade design considers one engine inoperative 
        
          Source:
             None 
    """    
    # Unpack rotor geometry  
    rotor_tag     = rotor.tag
    rotor.tag     = 'rotor'
    
    # start optimization 
    ti                   = time.time()   
    optimization_problem = optimization_setup(rotor,number_of_stations,print_iterations)
    output               = scipy_setup.SciPy_Solve(optimization_problem,solver=solver_name, iter = iterations , sense_step = solver_sense_step,tolerance = solver_tolerance)    
    tf                   = time.time()
    elapsed_time         = round((tf-ti)/60,2)
    print('Lift-rotor Optimization Simulation Time: ' + str(elapsed_time) + ' mins')   
    
    # print optimization results 
    print (output)  
    
    # set remaining rotor variables using optimized parameters 
    rotor     = set_optimized_parameters(rotor,optimization_problem)
    rotor.tag = rotor_tag
    
    return rotor 
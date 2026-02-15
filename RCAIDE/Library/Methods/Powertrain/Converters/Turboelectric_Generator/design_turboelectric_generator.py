# RCAIDE/Library/Methods/Energy/Converters/Turboelectric_Generator/design_turboshaft.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
# RCAIDE Imports      
from RCAIDE.Library.Methods.Powertrain.Converters.Turboshaft         import design_turboshaft
from RCAIDE.Library.Methods.Powertrain.Converters.Generator          import design_optimal_generator  

# ----------------------------------------------------------------------------------------------------------------------  
#  Design Turboshaft
# ----------------------------------------------------------------------------------------------------------------------   
def design_turboelectric_generator(turboelectric_generator):  
    """ Turboelectric generator design script. Sequentially calls the functions that
    design a turboshaft and optimally sizes a generator 
    """
    # call the turboshaft script 
    design_turboshaft(turboelectric_generator.turboshaft ) 

    # call the generator design script 
    design_optimal_generator(turboelectric_generator.generator)    

    return
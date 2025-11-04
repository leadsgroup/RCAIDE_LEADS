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
def design_turboelectric_generator(turboelectric_generator, network):  
    """ Turboelectric generator design script. Sequentially calls the functions that
    design a turboshaft and optimally sizes a generator 
    """
    # call the turboshaft script 
    design_turboshaft(network.converters[turboelectric_generator.assigned_converters.turboshaft_tag[0][0]], network) 

    # call the generator design script 
    design_optimal_generator(network.converters[turboelectric_generator.assigned_converters.generator_tag[0][0]])    

    return
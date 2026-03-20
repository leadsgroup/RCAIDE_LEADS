# RCAIDE/Library/Missions/Common/Update/network.py
# 
# 
# Created:  Sep 2025, S Shekar
import RCAIDE
from RCAIDE.Framework.Core import Data  
from scipy.optimize import fsolve, least_squares

# ----------------------------------------------------------------------------------------------------------------------
#  Solve Network
# ---------------------------------------------------------------------------------------------------------------------- 
def network(segment):
    """ Updates the **********
        
        Assumptions:
        N/A
        
        Inputs:
            None 
                 
        Outputs: 
            None
      
        Properties Used:
        N/A
                    
    """  
    # unpack
    energy_model = segment.analyses.energy
    for network  in segment.analyses.vehicle.networks:

        unknown_keys = list(segment.state.unknowns.network.keys())
        unknown_keys.remove('tag') 
        full_unkn_vals = Data()
        unknown_value  = Data()
        
        for unkn in unknown_keys:
            unknown_value[unkn]  = segment.state.unknowns.network[unkn]  
            full_unkn_vals[unkn] = unknown_value[unkn]         
    
            # segment.process.initialize.expand_state(segment)                        # NEED TO CHECK     it is not needed to expand it here, it is already expanded at this point
            # segment.process.initialize.expand_state = RCAIDE.Library.Methods.skip    # NEED TO CHECK     
            
        if segment.state.numerics.network_solver.type  == 'least_squares':       
            result = least_squares(energy_model.evaluate, 
                        full_unkn_vals.pack_array(),
                        args=(segment,network),
                        method= segment.state.numerics.network_solver.method,
                        verbose = 2 if segment.state.numerics.network_solver.print_output is True else 0,
                        xtol=segment.state.numerics.network_solver.tolerance,) 

            segment.state.numerics.network_solver.converged = result.success
            if result.success is False:
                print('The network solver fails with exit condition: ',result.message)
        elif segment.state.numerics.network_solver.type  == 'root_finder':
            result,_,ier,error_message = fsolve(energy_model.evaluate, 
                        full_unkn_vals.pack_array(),
                        args   = (segment,network),
                        xtol   = segment.state.numerics.network_solver.tolerance,
                        maxfev = segment.state.numerics.mission_solver.max_evaluations,
                        epsfcn = segment.state.numerics.mission_solver.step_size,
                        full_output = 1) 

            segment.state.numerics.network_solver.converged = False if ier == 0 else True
            if ier == 0:
                print('The network solver fails with exit condition: ',error_message)
                
        else: # If a network solver is not needed it will unpack values from the misison solver
            energy_model.evaluate(full_unkn_vals.pack_array(), segment, network)
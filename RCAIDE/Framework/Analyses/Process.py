# RCAIDE/Framework/Analyses/Process.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
from RCAIDE.Framework.Core import ContainerOrdered
from RCAIDE.Framework.Core import Data 
import numpy as np
# ----------------------------------------------------------------------------------------------------------------------
# Process
# ----------------------------------------------------------------------------------------------------------------------  
class Process(ContainerOrdered):
    """ RCAIDE.Framework.Analyses.Process()
    
        The Top Level Process Container Class
        
            Assumptions:
            None
            
            Source:
            N/A
    """     
    
    def evaluate(self,*args,**kwarg):
        """This is used to execute the evaluate functions of the analyses
            stored in the container.
        
                Assumptions:
                None
        
                Source:
                N/A
        
                Inputs:
                None
        
                Outputs:
                Results of the Evaluate Functions
        
                Properties Used:
                N/A
            """        
        
        results = Data() 

        for tag, step in self.items():

            if tag == 'thrust' and 'aerodynamics' in self.keys():
                MAX_PASSES   = args[0].analyses.vehicle.aero_prop_iterations if args else 0
                if MAX_PASSES > 0:
                    CONV_TOL     = args[0].analyses.vehicle.aero_prop_iterations_tolerance if args else 1e-6  
                    conditions   = args[0].conditions if args else None
                    use_surrogate = args[0].analyses.aerodynamics.settings.use_surrogate if args else False  
                    cl_wing      = None
                    prev_cl_wing = None
                    cl_prop      = None
                    prev_cl_prop = None

                    pass_count = 0
                    while True:
                        result = step.evaluate(*args,**kwarg) if hasattr(step,'evaluate') else step(*args,**kwarg)
                        results[f'thrust_pass{pass_count}'] = result
                        cl_prop = np.concatenate([
                        conv.lift_coefficient.flatten()
                        for conv_tag, conv in conditions.energy.converters.items()
                        if 'lift_coefficient' in conv.keys()
                        ])
                        if pass_count == 0 or not use_surrogate:  
                            aero_step   = self['aerodynamics']
                            aero_result = aero_step.evaluate(*args,**kwarg) if hasattr(aero_step,'evaluate') else aero_step(*args,**kwarg)
                            results[f'aero_pass{pass_count}'] = aero_result
                            if conditions is not None and 'VD' in conditions.aerodynamics:
                                cl_wing = conditions.aerodynamics.coefficients.lift.total.copy()

                        if prev_cl_wing is not None and prev_cl_prop is not None:
                            delta1 = np.linalg.norm(prev_cl_wing - cl_wing)
                            delta2 = np.linalg.norm(prev_cl_prop - cl_prop)
                            #print(f"pass {pass_count}: max |delta cl_wing| = {delta1:.6e}")
                            #print(f"pass {pass_count}: max |delta cl_prop| = {delta2:.6e}")
                            if delta1 < CONV_TOL and delta2 < CONV_TOL:
                                #print("wing and propeller aerodynamics converged!")
                                break

                        prev_cl_wing = cl_wing
                        prev_cl_prop = cl_prop

                        pass_count += 1
                        if pass_count > MAX_PASSES:
                            #print("max passes reached without convergence")
                            break

                    continue
                else:
                    # MAX_PASSES == 0: run thrust normally, fall through
                    result = step.evaluate(*args,**kwarg) if hasattr(step,'evaluate') else step(*args,**kwarg)
                    results[tag] = result
                    continue

            if tag == 'aerodynamics' and 'thrust' in self.keys():
                MAX_PASSES = args[0].analyses.vehicle.aero_prop_iterations if args else 0
                if MAX_PASSES > 0:
                    continue  # already run inside thrust-aero coupling

            result = step.evaluate(*args,**kwarg) if hasattr(step,'evaluate') else step(*args,**kwarg)
            results[tag] = result

        '''
        for tag,step in self.items():  
            if hasattr(step,'evaluate'): 
                result = step.evaluate(*args,**kwarg)
            else:
                result = step(*args,**kwarg)
            results[tag] = result
        '''
        return results
        
    def __call__(self,*args,**kwarg):
        """This is used to set the class' call behavior to the evaluate functions.
        
                Assumptions:
                None
        
                Source:
                N/A
        
                Inputs:
                None
        
                Outputs:
                None
        
                Properties Used:
                N/A
            """                        
        return self.evaluate(*args,**kwarg) 
    

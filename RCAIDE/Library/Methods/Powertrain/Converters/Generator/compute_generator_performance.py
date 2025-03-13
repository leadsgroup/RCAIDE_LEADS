# RCAIDE/Library/Methods/Powertrain/Converters/Generator/compute_generator_performance.py

# 
# Created: Feb 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE
from RCAIDE.Framework.Core import  Units
# package imports 
import numpy as np
 
# ----------------------------------------------------------------------------------------------------------------------
#  compute_omega_and_Q_from_Cp_and_V
# ----------------------------------------------------------------------------------------------------------------------    
def compute_generator_performance(generator,generator_conditions,conditions):
    """
    Computes generator performance characteristics including electrical, mechanical and thermal parameters.

    Parameters
    ----------
    generator : Converter
        Generator component for which performance is being computed
    generator_conditions : Conditions
        Container for generator operating conditions
    conditions : Conditions 
        Mission segment conditions containing freestream properties

    Returns
    -------
    None
        Updates generator_conditions in-place with computed performance parameters

    Notes
    -----
    This function handles both PMSM and DC generator types with different computation approaches:
     
    - Uses speed-torque relationships
    - Accounts for gearbox effects
    - Computes electrical parameters (current, voltage)
    - Determines overall efficiency

    **Major Assumptions**
        * Steady state operation
        * Uniform temperature distribution
        * No magnetic saturation effects
        * Linear speed-torque characteristics for DC generators
        * Constant material properties

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters.DC_generator
    RCAIDE.Library.Components.Powertrain.Converters.PMSM_generator
    """
 
    if type(generator) == RCAIDE.Library.Components.Powertrain.Converters.DC_Generator:   
        if generator.mode == "forward":  # THIS IS TOO VAGUE. NEED TO FIND BETTER WAY 
            power  = generator_conditions.inputs.power 
            Res    = generator.resistance  
            Kv     = generator.speed_constant
            G      = generator.gearbox.gear_ratio 
            io     = generator.no_load_current  
            v      = generator_conditions.voltage 
            omega  = generator_conditions.outputs.omega           
            i      = (v - (omega/G) /Kv)/Res  
            Q      = power / omega  
            etam   = (1-io/i)*(1-i*Res/v)
            
        elif generator.mode == 'reverse':  # THIS IS TOO VAGUE. NEED TO FIND BETTER WAY 
            Res    = generator.resistance  
            Kv     = generator.speed_constant
            G      = generator.gearbox.gear_ratio 
            io     = generator.no_load_current  
            v      = generator_conditions.voltage             
            P_elec = generator_conditions.outputs.power
            i      = P_elec /v
            omega  = ((v - (Res * i)) * Kv)   
            Q      = (((v-omega /Kv)/Res -io)/Kv)
            omega  = omega / G
            Q      = Q * G
            etam   = (1-io/i)*(1-i*Res/v)
        
        generator_conditions.outputs.current    = i 
        generator_conditions.outputs.power      = i *v   
        generator_conditions.inputs.power       = Q * omega  
        generator_conditions.outputs.efficiency = etam          
        
    elif type(generator) == RCAIDE.Library.Components.Powertrain.Converters.PMSM_Generator: 
        G      = generator.gearbox.gear_ratio 
        omega  = generator_conditions.outputs.omega  
        power  = generator_conditions.inputs.power  
        Kv     = generator.speed_constant                  
        D_in   = generator.inner_diameter         
        kw     = generator.winding_factor     
        Res    = generator.resistance                      
        L      = generator.stack_length                    
        l      = generator.length_of_path                  
        mu_0   = generator.mu_0                            
        mu_r   = generator.mu_r   
        Q      = power/omega                               
        i      = np.sqrt((2*(Q/G)*l)/(D_in*mu_0*mu_r*L*kw))           
        v      = (omega * G)/((2 * np.pi / 60)*Kv) + i*Res        
        etam   = (1-io/i)*(1-i*Res/v) 
    
        generator_conditions.outputs.current    = i 
        generator_conditions.outputs.power      = i *v   
        generator_conditions.inputs.power       = Q * omega 
        generator_conditions.outputs.efficiency = etam       
   
 
    return
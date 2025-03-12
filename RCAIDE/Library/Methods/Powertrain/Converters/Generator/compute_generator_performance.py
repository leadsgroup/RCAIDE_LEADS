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
def compute_generator_performance(electric_machine,electric_machine_conditions,conditions):
    """
    Computes electric_machine performance characteristics including electrical, mechanical and thermal parameters.

    Parameters
    ----------
    electric_machine : Converter
        Generator component for which performance is being computed
    electric_machine_conditions : Conditions
        Container for electric_machine operating conditions
    conditions : Conditions 
        Mission segment conditions containing freestream properties

    Returns
    -------
    None
        Updates electric_machine_conditions in-place with computed performance parameters

    Notes
    -----
    This function handles both PMSM and DC electric_machine types with different computation approaches:
     
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
 
    if type(electric_machine) == RCAIDE.Library.Components.Powertrain.Converters.DC_Generator:   
        if electric_machine.mode == "forward": 
            power  = electric_machine_conditions.inputs.shaft_power 
            Res    = electric_machine.resistance  
            Kv     = electric_machine.speed_constant
            G      = electric_machine.gearbox.gear_ratio
            etaG   = electric_machine.gearbox.efficiency
            exp_i  = electric_machine.expected_current
            io     = electric_machine.no_load_current + exp_i*(1-etaG) 
            v      = electric_machine_conditions.voltage 
            omega  = electric_machine_conditions.outputs.omega           
            i      = (v - (omega/G) /Kv)/Res  
            Q      = power / omega  
            etam   = (1-io/i)*(1-i*Res/v)
            
        elif electric_machine.mode == 'reverse':  
            Res    = electric_machine.resistance  
            Kv     = electric_machine.speed_constant
            G      = electric_machine.gearbox.gear_ratio
            etaG   = electric_machine.gearbox.efficiency
            exp_i  = electric_machine.expected_current
            io     = electric_machine.no_load_current + exp_i*(1-etaG) 
            v      = electric_machine_conditions.voltage             
            P_elec = electric_machine_conditions.outputs.power
            i      = P_elec /v
            omega  = ((v - (Res * i)) * Kv)   
            Q      = (((v-omega /Kv)/Res -io)/Kv)
            omega  = omega / G
            Q      = Q * G
            etam   = (1-io/i)*(1-i*Res/v) 
        
    elif type(electric_machine) == RCAIDE.Library.Components.Powertrain.Converters.PMSM_Generator: 
        G      = electric_machine.gearbox.gear_ratio 
        omega  = electric_machine_conditions.outputs.omega  
        power  = electric_machine_conditions.inputs.shaft_power  
        Kv     = electric_machine.speed_constant                  
        D_in   = electric_machine.inner_diameter         
        kw     = electric_machine.winding_factor     
        Res    = electric_machine.resistance                      
        L      = electric_machine.stack_length                    
        l      = electric_machine.length_of_path                  
        mu_0   = electric_machine.mu_0                            
        mu_r   = electric_machine.mu_r   
        Q      = power/omega                               
        i      = np.sqrt((2*(Q/G)*l)/(D_in*mu_0*mu_r*L*kw))           
        v      = (omega * G)/((2 * np.pi / 60)*Kv) + i*Res        
        etam   = (1-io/i)*(1-i*Res/v)                              
   
           
    elif (type(electric_machine) ==  RCAIDE.Library.Components.Powertrain.Converters.DC_Motor):
        if electric_machine.mode == "forward": 
            G      = electric_machine.gearbox.gear_ratio 
            omega  = electric_machine_conditions.outputs.omega  
            power  = electric_machine_conditions.outputs.power  
            Res    = electric_machine.resistance  
            Kv     = electric_machine.speed_constant 
            etaG   = electric_machine.gearbox.efficiency
            exp_i  = electric_machine.expected_current
            io     = electric_machine.no_load_current + exp_i*(1-etaG) 
            v      = electric_machine_conditions.voltage             
            i      = (v - (omega * G)/Kv)/Res  
            Q      = (power / omega) 
            etam   = (1-io/i)*(1-i*Res/v)
            
        elif electric_machine.mode == 'reverse':
            G      = electric_machine.gearbox.gear_ratio 
            omega  = electric_machine_conditions.outputs.omega 
            Res    = electric_machine.resistance  
            Kv     = electric_machine.speed_constant
            G      = electric_machine.gearbox.gear_ratio
            etaG   = electric_machine.gearbox.efficiency
            exp_i  = electric_machine.expected_current
            io     = electric_machine.no_load_current + exp_i*(1-etaG) 
            v      = electric_machine_conditions.voltage 
            P_elec = electric_machine_conditions.outputs.power
            i      = P_elec /v
            Q      = (((v- omega /Kv)/Res -io)/Kv)  
            omega  = ((v - (Res * i)) * Kv)  
            etam   = (1-io/i)*(1-i*Res/v)
        
    elif (type(electric_machine) ==  RCAIDE.Library.Components.Powertrain.Converters.PMSM_Motor): 
        G      = electric_machine.gearbox.gear_ratio 
        omega  = electric_machine_conditions.outputs.omega 
        power  = electric_machine_conditions.outputs.power  
        Kv     = electric_machine.speed_constant 
        D_in   = electric_machine.inner_diameter  
        kw     = electric_machine.winding_factor  
        Res    = electric_machine.resistance                      
        L      = electric_machine.stack_length                    
        l      = electric_machine.length_of_path                  
        mu_0   = electric_machine.mu_0                            
        mu_r   = electric_machine.mu_r   
        Q      = (power/omega)                                      
        i      = np.sqrt((2*(Q/G)*l)/(D_in*mu_0*mu_r*L*kw))           
        v      = (omega * G)/((2 * np.pi / 60)*Kv) + i*Res   
        etam   = (1-io/i)*(1-i*Res/v)                              
   
    electric_machine_conditions.outputs.torque     = Q   
    electric_machine_conditions.outputs.current    = i 
    electric_machine_conditions.outputs.power      = i *v 
    electric_machine_conditions.outputs.efficiency = etam 
    electric_machine_conditions.inputs.shaft_power = Q * omega 
 
    return
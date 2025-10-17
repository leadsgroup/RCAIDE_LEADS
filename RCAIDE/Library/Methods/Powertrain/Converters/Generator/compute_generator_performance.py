# RCAIDE/Library/Methods/Powertrain/Converters/Generator/compute_generator_performance.py

# 
# Created: Feb 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE

# package imports 
import numpy as np
 
# ----------------------------------------------------------------------------------------------------------------------
#  compute_omega_and_Q_from_Cp_and_V
# ----------------------------------------------------------------------------------------------------------------------    
def compute_generator_performance(generator,state):
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
    RCAIDE.Library.Components.Powertrain.Converters.DC_Generator
    RCAIDE.Library.Components.Powertrain.Converters.PMSM_Generator
    """
    
    # unpack generator conditions 
    generator_conditions = state.conditions.energy.converters[generator.tag]    
 
    if generator.generator_type == 'DC':   
        if generator.inverse_calculation == False:
            P_mech          = generator_conditions.inputs.power 
            Res            = generator.resistance  
            Kv             = generator.speed_constant
            G              = generator.gearbox.gear_ratio 
            io             = generator.no_load_current  
            v              = generator_conditions.outputs.voltage 
            omega          = generator_conditions.inputs.omega
            omega_internal = omega * G
            i              = (v - (omega_internal) /Kv)/Res  
            Q              = P_mech / omega  
            etam           = (1-io/i)*(1-i*Res/v)
            P_elec         = i * v
            
        else:
            Res             = generator.resistance  
            Kv              = generator.speed_constant
            G               = generator.gearbox.gear_ratio 
            io              = generator.no_load_current  
            v               = generator_conditions.outputs.voltage             
            i               = generator_conditions.outputs.current
            omega_internal  = ((v - (Res * i)) * Kv)   
            Q_internal      = (((v-omega_internal /Kv)/Res -io)/Kv)
            omega           = omega_internal / G
            Q               = Q_internal * G
            etam            = (1-io/i)*(1-i*Res/v)
            P_elec          = i * v
            P_mech          = Q * omega       
        
    elif generator.generator_type == 'AC': 
        if generator.inverse_calculation == False:
            io     = generator.no_load_current
            G      = generator.gearbox.gear_ratio 
            omega  = generator_conditions.inputs.omega  
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
            P_elec = i * v
            P_mech = Q * omega

        else:
            Res            = generator.resistance
            io             = generator.no_load_current
            G              = generator.gearbox.gear_ratio
            i              = generator_conditions.outputs.current
            v              = generator_conditions.outputs.voltage
            I_turn         = i/generator.number_of_turns                                                             # [A]            current in each turn
            omega          = (generator.speed_constant*(v - i*Res)) /G                                     # [RPM -> rad/s] rotor angular velocity
            A              = np.pi * ((generator.stator_outer_diameter**2 - generator.stator_inner_diameter**2) / 4)     # [m**2]         cross-sectional area of the reluctance path perpendicular to length 𝑙    
            MMF_coil       = generator.number_of_turns*I_turn                                                        # [A*turns]      magnetomotive force applied to the reluctance path for a coil (Eq.14)  
            R              = generator.length_of_path/(A*generator.mu_0*generator.mu_r)                                      # [A*turn/Wb]    reluctance of a given path or given reluctant element (Eq.16) 
            phi            = MMF_coil/R                                                                          # [Wb]           magnetic flux through the reluctance path (Eq.12)
            B_sign         = phi/A                                                                               # [V*s/m**2]     ranges from 0.5 to 1.2, average magnitude of the radial flux density produced by the rotor
            A_sign         = (generator.winding_factor*i)/(np.pi*generator.stator_inner_diameter)                        # [-]            stator electrical loading (Eq.2)        
            Q              = (np.pi/2)*(B_sign*A_sign)*(generator.inner_diameter**2)*generator.stack_length # [Nm]           torque (Eq.1)                                                                        # [W]            power (Eq.1)        
            A              = np.pi * ((generator.stator_outer_diameter**2 - generator.stator_inner_diameter**2) / 4)     # [m**2]         cross-sectional area of the reluctance path perpendicular to length 𝑙    
            P_elec         = v*i 
            etam           = (1-io/i)*(1-i*Res/v)                                                                                           # [W]            electrical power (Eq.11)
            P_mech         = Q * omega                                                                                     # [W]            mechanical power (Eq.10)

        generator_conditions.outputs.power      = P_elec
        generator_conditions.outputs.current    = i
        generator_conditions.outputs.voltage    = v
        generator_conditions.outputs.efficiency = etam
        generator_conditions.inputs.power       = P_mech
        generator_conditions.inputs.torque      = Q
        generator_conditions.inputs.omega       = omega
        
        stored_results_flag    = True
        stored_converter_tag   = generator.tag

        P_hydr  = 0 * state.ones_row(1)
        P_therm = 0 * state.ones_row(1)
        m_dot_fuel = 0 * state.ones_row(1)
   
    return P_mech,P_elec,P_hydr,P_therm, m_dot_fuel, stored_results_flag,stored_converter_tag
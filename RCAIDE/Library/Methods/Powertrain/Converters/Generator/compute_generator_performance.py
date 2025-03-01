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

    G     = generator.gearbox_ratio    
     
    if type(generator) == RCAIDE.Library.Components.Powertrain.Converters.DC_Generator:  
        omega  = generator_conditions.inputs.omega*G
        power = generator_conditions.inputs.shaft_power 
        fidelity = "Simple_DC_Electric_Machine" 
    elif type(generator) == RCAIDE.Library.Components.Powertrain.Converters.PMSM_Generator:  
        omega  = generator_conditions.inputs.omega*G
        power = generator_conditions.inputs.shaft_power 
        fidelity = "PMSM_Electric_Machine" 
    elif (type(generator) ==  RCAIDE.Library.Components.Powertrain.Converters.DC_Motor):
        omega  = generator_conditions.outputs.omega*G
        power = generator_conditions.outputs.shaft_power
        fidelity = "Simple_DC_Electric_Machine"
    elif (type(generator) ==  RCAIDE.Library.Components.Powertrain.Converters.PMSM_Motor): 
        omega  = generator_conditions.outputs.omega*G
        power = generator_conditions.outputs.shaft_power
        fidelity = "PMSM_Electric_Machine"
        
    if fidelity == 'Simple_DC_Electric_Machine':
        Res   = generator.resistance  
        Kv    = generator.speed_constant
        Res   = generator.resistance
        G     = generator.gear_ratio
        etaG  = generator.gearbox_efficiency
        exp_i = generator.expected_current
        io    = generator.no_load_current + exp_i*(1-etaG) 
        v     = generator_conditions.voltage
        
        if generator.mode == "forward":   
            i = (omega/Kv - v)/Res  
            Q = power / omega
        elif generator.mode == 'reverse':
            P_elec = generator_conditions.outputs.power
            i      = P_elec /v
            Q      = ((v-omega /Kv)/Res -io)/Kv  
            omega  = (v - (Res * i)) * Kv / G 
            generator_conditions.inputs.shaft_power = Q * omega

        etam=(1-io/i)*(1-i*Res/v)             

    elif fidelity == 'PMSM_Electric_Machine':

        Kv    = generator.speed_constant                          # [rpm/V]        speed constant
        omega = omega*60/(2*np.pi)                                # [rad/s -> rpm] nominal speed
        D_in  = generator.inner_diameter                          # [m]            stator inner diameter  
    
        # Input data from Literature
        kw    = generator.winding_factor                          # [-]            winding factor
        
        # Input data from Assumptions
        Res   = generator.resistance                              # [Ω]            resistance
        L     = generator.stack_length                            # [m]            (It should be around 0.14 m) generator stack length 
        l     = generator.length_of_path                          # [m]            length of the path  
        mu_0  = generator.mu_0                                    # [N/A**2]       permeability of free space
        mu_r  = generator.mu_r                                    # [N/A**2]       relative permeability of the magnetic material 
    
        Q     = power/omega                                       # [Nm]           torque                  
        i     = np.sqrt((2*Q*l)/(D_in*mu_0*mu_r*L*kw))            # [A]            total current 
        v     = omega/((2 * np.pi / 60)*Kv) + i*Res               # [V]            total voltage

        etam  = (1-io/i)*(1-i*Res/v)                              # [-]            efficiency
   
    generator_conditions.outputs.torque     = Q   
    generator_conditions.outputs.current    = i 
    generator_conditions.outputs.power      = i *v 
    generator_conditions.outputs.efficiency = etam 
 
    return
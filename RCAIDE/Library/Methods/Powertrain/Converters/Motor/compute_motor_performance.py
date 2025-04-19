# RCAIDE/Library/Methods/Powertrain/Converters/Motor/compute_motor_performance.py

# 
# Created:  Jan 2025, M. Clarke, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import  RCAIDE

# package imports 
import numpy as np
 
# ----------------------------------------------------------------------------------------------------------------------
#  compute_omega_and_Q_from_Cp_and_V
# ----------------------------------------------------------------------------------------------------------------------    
def compute_motor_performance(motor,motor_conditions,conditions):
    """
    Computes motor performance characteristics including electrical, mechanical and thermal parameters.

    Parameters
    ----------
    motor : Converter
        Motor component (DC_Motor or PMSM_Motor) for which performance is being computed
    motor_conditions : Conditions
        Container for motor operating conditions
    conditions : Conditions 
        Mission segment conditions containing freestream properties

    Returns
    -------
    None
        Updates motor_conditions in-place with computed performance parameters

    Notes
    -----
    This function handles both PMSM and DC motor types with different computation approaches:

    For PMSM motors:
        - Computes electromagnetic torque and power
        - Calculates thermal resistances and heat transfer
        - Determines cooling flow characteristics
        - Evaluates airgap and endspace heat transfer

    For DC motors:
        - Uses speed-torque relationships
        - Accounts for gearbox effects
        - Computes electrical parameters (current, voltage)
        - Determines overall efficiency

    **Major Assumptions**
        * Steady state operation
        * Uniform temperature distribution
        * No magnetic saturation effects
        * Linear speed-torque characteristics for DC motors
        * Constant material properties

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters.DC_Motor
    RCAIDE.Library.Components.Powertrain.Converters.PMSM_Motor
    """           
    # Unpack 
    rho   = conditions.freestream.density[:,0,None]
    Res   = motor.resistance

    if (type(motor) == RCAIDE.Library.Components.Powertrain.Converters.PMSM_Motor): 

        I              = motor_conditions.current
        V              = motor_conditions.voltage
        I_turn         = I/motor.number_of_turns                                                         # [A]            current in each turn
        omega          = motor.speed_constant*(V - I*Res)* (2 * np.pi / 60)                # [RPM -> rad/s] rotor angular velocity
        A              = np.pi * ((motor.stator_outer_diameter**2 - motor.stator_inner_diameter**2) / 4)                    # [m**2]         cross-sectional area of the reluctance path perpendicular to length 𝑙    
        MMF_coil       = motor.number_of_turns*I_turn                                                    # [A*turns]      magnetomotive force applied to the reluctance path for a coil (Eq.14)  
        R              = motor.length_of_path/(A*motor.mu_0*motor.mu_r)                                 # [A*turn/Wb]    reluctance of a given path or given reluctant element (Eq.16) 
        phi            = MMF_coil/R                                                        # [Wb]           magnetic flux through the reluctance path (Eq.12)
        B_sign         = phi/A                                                             # [V*s/m**2]     ranges from 0.5 to 1.2, average magnitude of the radial flux density produced by the rotor
        A_sign         = (motor.winding_factor*I)/(np.pi*motor.stator_inner_diameter)                                  # [-]            stator electrical loading (Eq.2)        
        TQ             = (np.pi/2)*(B_sign*A_sign)*(motor.stator_inner_diameter**2)*motor.motor_stack_length                 # [Nm]           torque (Eq.1)
        P              = omega*TQ/1000                                                     # [kW]           power (Eq.1)  
        
        A              = np.pi * ((motor.stator_outer_diameter**2 - motor.stator_inner_diameter**2) / 4)                    # [m**2]         cross-sectional area of the reluctance path perpendicular to length 𝑙    
        R_cond_path    = motor.length_of_conductive_path/(motor.thermal_conductivity*A)                                     # [K/W]          Conductive Path Thermal Resistance (Eq.68)

        if motor.Conduction_laminar_flow == True:   
            Nu     = 0.453*(motor.Re_cooling_flow**0.5)*(motor.Prandtl_number**(1/3))                               # Laminar Nusselt number (Eq.71)
        else:
            Nu   = 0.0308*(motor.Re_cooling_flow**(4/5))*(motor.Prandtl_number**(1/3))                              # Turbulent Nusselt number (Eq.71) 
        h    = motor.characteristic_length_of_flow*Nu*motor.thermal_conductivity_fluid               # [W/m**2*K]     convection coefficient of the flow at a liquid to solid interfaced
        R_conv_path    = 1/(h*A)                                                                       # [K/W]          Fluid Flow Thermal Resistance (Eq.69)

        if motor.Re_cooling_flow < 3000:
            Nu_cooling_flow     = 1.051*np.log(motor.height_of_duct/motor.width_of_duct) + 2.89         # Nusselt number for cooling flow in rectangular ducts and Re_d < 3000 (Eq.72)
            h_cooling_flow      = motor.characteristic_length_of_flow*Nu_cooling_flow*motor.thermal_conductivity_fluid                   # [W/m**2*K]     convection coefficient of the flow at a liquid to solid interfaced
            R_conv_path_cooling_flow    = 1/(h_cooling_flow*A)                             # [K/W]          Fluid Flow Thermal Resistance (Eq.69)
        else:
            if motor.Convection_laminar_flow == True:
                f = 64/motor.Re_cooling_flow
            else:
                f = (0.79*np.log(motor.Re_cooling_flow) - 1.64)**(-2)                                   # Turbulent Moody friction factor (Eq.73)
            Nu_cooling_flow   = ((f/8)*(motor.Re_cooling_flow - 1000)*motor.Prandtl_number)/(1 + 12.7*((f/8)**0.5)*(motor.Prandtl_number**(2/3) - 1)) # Nusselt number for cooling flow in rectangular ducts and Re_d >= 3000 (Eq.72)
            h_cooling_flow    = motor.characteristic_length_of_flow*Nu_cooling_flow*motor.thermal_conductivity_fluid                     # [W/m**2*K]     convection coefficient of the flow at a liquid to solid interfaced
            R_conv_path_cooling_flow    = 1/(h_cooling_flow*A)                                                                             # [K/W]          Fluid Flow Thermal Resistance (Eq.69)

        Delta_P_flow               = ((f*motor.density_of_fluid*motor.velocity_of_fluid**2)/(2*motor.hydraulic_diameter_of_duct))*motor.length_of_channel # Flow pressure drop (Eq.74)
        Loss_cooling               = Delta_P_flow*motor.volume_flow_rate_of_fluid                              # Power needed to pump the fluid through the duct (Eq.75)

        if motor.Taylor_number < 41: 
            Nu_airgap  = 2                                                                 # Nusselt number for the airgap convection and Ta < 41 (Eq.76)
        elif motor.Taylor_number> 41 and motor.Taylor_number < 100:
            Nu_airgap = 0.202*(motor.Taylor_number**(0.63))*(motor.Prandtl_number**0.27)                          # Nusselt number for the airgap convection and 41 < Ta < 100 (Eq.76)
        else:
            Nu_airgap = 0.386*(motor.Taylor_number**0.5)*(motor.Prandtl_number**0.27)                             # Nusselt number for the airgap convection and 100 < Ta (Eq.76)
        h_airgap    = motor.characteristic_length_of_flow*Nu_airgap*motor.thermal_conductivity_fluid                                     # [W/m**2*K]     convection coefficient of the flow at a liquid to solid interfaced
        R_airgap    = 1/(h_airgap*A)                                                       # [K/W]          Fluid Flow Thermal Resistance (Eq.69)

        if motor.axial_gap_to_radius_of_rotor == 0.01:
            if motor.Re_airgap < 1e5:
                Nu_G          = 7.46*motor.Re_airgap**(0.32)                               # Nusselt number for laminar flow and G = 0.01 (Eq.77)
            else:
                Nu_G        = 0.044*motor.Re_airgap**(0.75)                                # Nusselt number for turbulent flow and G = 0.01 (Eq.78)  
        elif motor.axial_gap_to_radius_of_rotor> 0.02 and motor.axial_gap_to_radius_of_rotor < 0.06:
            if motor.Re_airgap < 1e5:
                Nu_G     = 0.5*(1 + 5.47*(10**-4)*np.exp(112*motor.axial_gap_to_radius_of_rotor))*(motor.Re_airgap**0.5) # Nusselt number for laminar flow and G = 0.02 - 0.06 (Eq.77)
            else:    
                Nu_G  = 0.5*(12.57*np.exp(-33.18*motor.axial_gap_to_radius_of_rotor))*(motor.Re_airgap**(0.6 + 25*motor.axial_gap_to_radius_of_rotor**(12/7))) # Nusselt number for turbulent flow and G = 0.02 - 0.06 (Eq.78)    
        elif motor.axial_gap_to_radius_of_rotor > 0.06:
            if motor.Re_airgap < 1e5:
                Nu_G = 0.35*(motor.Re_airgap**0.5)                                         # Nusselt number for laminar flow and G > 0.06 (Eq.77)
            else:    
                Nu_G = 0.0151*(motor.Re_airgap**0.6)                                       # Nusselt number for turbulent flow and G > 0.06 (Eq.78)
        h_endspace    = motor.characteristic_length_of_flow*Nu_G*motor.thermal_conductivity_fluid                                        # [W/m**2*K]     convection coefficient of the flow at a liquid to solid interfaced
        R_endspace    = 1/(h_endspace*A)                                                   # [K/W]          Fluid Flow Thermal Resistance (Eq.69)

        Q_cond_path              = motor.Delta_T/R_cond_path                               # heat through a conductive thermal path (Eq.67)
        Q_conv_path              = motor.Delta_T/R_conv_path                               # heat through a thermal path (Eq.67)
        Q_conv_path_cooling_flow = motor.Delta_T/R_conv_path_cooling_flow                  # heat through a thermal path (Eq.67)
        Q_conv_airgap            = motor.Delta_T/R_airgap                                  # heat through a thermal path (Eq.67)
        Q_conv_endspace          = motor.Delta_T/R_endspace                                # heat through a thermal path (Eq.67)
        
        motor_conditions.Q_cond_path              = Q_cond_path                            
        motor_conditions.Q_conv_path              = Q_conv_path                            
        motor_conditions.Q_conv_path_cooling_flow = Q_conv_path_cooling_flow               
        motor_conditions.Q_conv_airgap            = Q_conv_airgap                          
        motor_conditions.Q_conv_endspace          = Q_conv_endspace                        
        motor_conditions.Loss_cooling             = Loss_cooling                           
        motor_conditions.torque                   = TQ
        motor_conditions.omega                    = omega 
        motor_conditions.power                    = P   
    else:
        eta_G = motor.gearbox_efficiency
        exp_I = motor.expected_current
        I0    = motor.no_load_current + exp_I*(1-eta_G)
        G     = motor.gear_ratio
        KV    = motor.speed_constant/G
        R     = motor.rotor_radius
        v     = motor_conditions.voltage 
        Cp    = motor_conditions.rotor_power_coefficient 
    
        # compute angular velocity, omega 
        omega   =   ((np.pi**(3./2.))*((- 16.*Cp*I0*rho*(KV*KV*KV)*(R*R*R*R*R)*(Res*Res) +
                    16.*Cp*rho*v*(KV*KV*KV)*(R*R*R*R*R)*Res + (np.pi*np.pi*np.pi))**(0.5) - 
                    np.pi**(3./2.)))/(8.*Cp*(KV*KV)*(R*R*R*R*R)*Res*rho)
        omega [np.isnan(omega )] = 0.0
        
        # compute torque 
        Q = ((v-omega /KV)/Res -I0)/KV 
        I    = (v-(omega*G)/KV)/Res 
         
        motor_conditions.torque     = Q
        motor_conditions.omega      = omega  
        motor_conditions.current    = I
        motor_conditions.power      = omega *Q 
        motor_conditions.efficiency = (1-I0/I)*(1-I*Res/v) 
     
    return
# RCAIDE/Library/Methods/Thermal_Management/Heat_Exchangers/Cross_Flow_Heat_Exchanger/cross_flow_hex_rating_model.py
#
#
# Created:  Apr 2024, S. Shekar
# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import numpy as np

# ----------------------------------------------------------------------
#  Methods
# ----------------------------------------------------------------------
def compute_cross_flow_heat_exchanger_performance(HEX,coolant_line,T_coolant,state):
    """ Computes the net heat rejected by a cross flow heat exchanger from the
        coolant loop to ambient air, at every control point simultaneously.

          Inputs:
          HEX.
              (all optimized and default properties)
          T_coolant
              Coolant temperature at every control point                [Kelvin]
          Outputs:
               Q_rejected  (heat pulled from the coolant loop)           [Watts]

          Assumptions:
               None

          Source:
            Shah RK, Sekulić DP. Fundamentals of Heat Exchanger Design. John Wiley & Sons; 2003
    """

    air              = HEX.air
    coolant          = HEX.coolant

    # take in variables from the output of sizing problem
    H     = HEX.stack_height
    L_c   = HEX.stack_length
    L_h   = HEX.stack_width

    # Inital assumed efficiency of HEX
    eff_hex     = 0.75

    # Hydraulic Diameters
    d_h_c       = HEX.coolant_hydraulic_diameter
    d_h_h       = HEX.air_hydraulic_diameter

    # Fin Height/Spaceing
    b_c         = HEX.fin_spacing_cold
    b_h         = HEX.fin_spacing_hot

    # Fin metal thickness
    delta_h     = HEX.fin_metal_thickness_hot
    delta_c     = HEX.fin_metal_thickness_cold

    # Platethickness
    delta_w     = HEX.t_w

    # Strip edge exposed
    l_s_h       = HEX.fin_exposed_strip_edge_hot
    l_s_c       = HEX.fin_exposed_strip_edge_cold

    #Fin and wall Conductivity
    fin_conductivity          = HEX.fin_conductivity
    wall_conductivity         = HEX.wall_conductivity

    # Ratio of finned area to total area
    Af_A_h      = HEX.finned_area_to_total_area_hot
    Af_A_c      = HEX.finned_area_to_total_area_cold

    # Finned area density
    beta_h      = HEX.fin_area_density_hot
    beta_c      = HEX.fin_area_density_cold

    # Assumes N passages for hot air and N+1 fpr cold air
    N_p   = (H-b_c+2*delta_w)/(b_h+b_c+2*delta_w)

    # Frontal areas on hot and cold sides
    A_fr_h = L_c*H
    A_fr_c = L_h*H

    # Heat exchnager volume between plates on each fluid side.
    V_p_h       = L_h*L_c*b_h*N_p
    V_p_c       = L_h*L_c*b_c*(N_p+1)

    # The heat transfer areas
    A_h         = beta_h*V_p_h
    A_c         = beta_c*V_p_c

    #The minimum free flow area
    A_o_h       = d_h_h*A_h/(4*L_h)
    A_o_c       = d_h_c*A_c/(4*L_c)

    # Minimum Free flow area
    sigma_h    = A_o_h/A_fr_h
    sigma_c    = A_o_c/A_fr_c

    # Inlet temperatures
    T_i_h           = T_coolant
    T_i_c           = state.conditions.freestream.temperature
    m_dot_h         = HEX.design_coolant_mass_flow_rate * np.ones_like(T_i_h)
    m_dot_c         = HEX.design_air_mass_flow_rate * np.ones_like(T_i_h)
    P_i_c           = HEX.design_air_inlet_pressure * np.ones_like(T_i_h)
    P_i_h           = HEX.design_coolant_inlet_pressure * np.ones_like(T_i_h)
    rho_c_i         = air.compute_density(T_i_h,P_i_c)
    rho_h_i         = coolant.compute_density(T_i_h)

    # Thermal Performance Calculation
    T_o_h       = T_i_h-eff_hex*(T_i_h-T_i_c)
    T_o_c       = T_i_c+eff_hex*(m_dot_h/m_dot_c)*(T_i_h-T_i_c)

    for _ in range(10):
        T_m_h       = (T_i_h+T_o_h)/2
        T_m_c       = (T_i_c+T_o_c)/2

        #Prandtl Number
        Pr_h    =  coolant.compute_prandtl_number(T_m_h)
        Pr_c    =  air.compute_prandtl_number(T_m_c)

        #Absolute viscosity
        mu_h    = coolant.compute_absolute_viscosity(T_m_h)
        mu_c    = air.compute_absolute_viscosity(T_m_c)

        #Specific heat
        c_p_h   = coolant.compute_cp(T_m_h)/1000 #KJ/kg-K
        c_p_c   = air.compute_cp(T_m_c)/1000     #KJ/kg-K

        # Core mass velcoity
        G_h   = m_dot_h/A_o_h
        G_c   = m_dot_c/A_o_c

        # Calculate Reynolds Number
        Re_h       = G_h * d_h_h / mu_h
        Re_c       = G_c * d_h_c / mu_c

        # Calculate the colburn factor and friction factor using curve fitted values
        j_c            = 0.0131 * (Re_c / 1000)**(-0.415)
        j_h            = 0.0131 * (Re_h / 1000)**(-0.415)

        f_c            = 0.0514 * (Re_c / 1000)**(-0.471)
        f_h            = 0.0514 * (Re_h / 1000)**(-0.471)

        # Heat Transfer Coefficients
        h_h = j_h * G_h * c_p_h / (Pr_h**(2/3))
        h_c = j_c * G_c * c_p_c / (Pr_c**(2/3))

        m_f_h = (np.sqrt((2*h_h)/(fin_conductivity*delta_h)))*np.sqrt(1+(delta_h/l_s_h))
        m_f_c = (np.sqrt((2*h_c)/(fin_conductivity*delta_c)))*np.sqrt(1+(delta_c/l_s_c))

        l_f_h = b_h / 2 - delta_h
        l_f_c = b_c / 2 - delta_c

        # Fin Efficiency
        eta_f_h = np.tanh(m_f_h * l_f_h) / (m_f_h * l_f_h)
        eta_f_c = np.tanh(m_f_c * l_f_c) / (m_f_c * l_f_c)

        # Overall Efficiency
        eta_o_h = 1 - (1 - eta_f_h) * Af_A_h
        eta_o_c = 1 - (1 - eta_f_c) * Af_A_c

        # Wall ressistance
        A_w   = L_c*L_h*(2*N_p+2)
        R_w   = delta_w/(wall_conductivity*A_w)

        # Calculate overall heat transfer without fouling
        UA    = 1 / ((1 / (eta_o_h * h_h*A_h)) +R_w+ (1 / (eta_o_c* h_c*A_c)))

        # Heat Capacity
        C_h            = m_dot_h*c_p_h
        C_c            = m_dot_c*c_p_c

        C_min = np.minimum(C_h, C_c)
        C_max = np.maximum(C_h, C_c)
        C_r   = C_min / C_max

        # NTU
        NTU            = UA/C_min

        # Updated effectiveness, neglecting longitudinal conduction
        eff_hex_updated = (1 - np.exp(((NTU**0.22)/C_r)*(np.exp(-C_r*(NTU**(0.78))) - 1 )))

        # Heat transfer rate
        q             = eff_hex*(T_i_h-T_i_c)*C_min

        # Updated Outlet temperatures
        T_o_h_updated = T_i_h-(q/C_h)
        T_o_c_updated = T_i_c+(q/C_c)

        residual = np.max(np.abs(T_o_c-T_o_c_updated)) + np.max(np.abs(T_o_h-T_o_h_updated)) + np.max(np.abs(eff_hex-eff_hex_updated))

        T_o_c   = T_o_c_updated
        T_o_h   = T_o_h_updated
        eff_hex = eff_hex_updated

        if residual < 0.01:
            break

    # ----------------------------------------------------------------------------------------------------------
    # Pressure Drop Calculation
    # ----------------------------------------------------------------------------------------------------------
    P_o_h   = P_i_h
    P_o_c   = P_i_c

    for i in range(10):
        rho_h_o  =  coolant.compute_density(T_o_h)
        rho_c_o  =  air.compute_density(T_o_c,P_o_c)
        rho_h_m  = 2 / (1 / rho_h_i + 1 / rho_h_o)
        rho_c_m  = 2 / (1 / rho_c_i + 1 / rho_c_o)

        k_c_c = 0.36
        k_c_h = 0.36
        k_e_c = 0.42
        k_e_h = 0.42

        # Thermal Resistance on the hot and cold fluid sides
        R_h = 1 / (eta_o_h * h_h * A_h)
        R_c = 1 / (eta_o_c * h_c * A_c)

        # Compute Wall temperature
        T_w = (T_m_h + (R_h / R_c) * T_m_c) / (1 + R_h / R_c)

        # Considering temperature at wall effecting f value of 0.81 changes
        f_h_wall = f_h * np.power(((T_w + 273) / (273 + T_m_h)), 0.81)
        f_c_wall = f_c * np.power(((T_w + 273) / (273 + T_m_c)), 1)

        delta_p_c_new = np.power(G_c, 2) / (2 * rho_c_i) * ((1 - np.power(sigma_c, 2) + k_c_c)
                                             + 2 * (rho_c_i / rho_c_o - 1) + f_c_wall * 4 * L_c / d_h_c *
                                                     rho_c_i / rho_c_m
                                                   - (1 - np.power(sigma_c, 2) - k_e_c) * rho_c_i / rho_c_o)

        delta_p_h_new = np.power(G_h, 2) / (2 * rho_h_i) * ((1 - np.power(sigma_h, 2) + k_c_h)
                                             + 2 * (rho_h_i / rho_h_o - 1) + f_h_wall * 4 * L_h / d_h_h *
                                                     rho_h_i / rho_h_m
                                                    - (1 - np.power(sigma_h, 2) - k_e_h) * rho_h_i / rho_h_o)

        if i >= 1:
            residual_pressure = np.max(np.abs(delta_p_c_new-delta_p_c)) + np.max(np.abs(delta_p_h_new-delta_p_h))
        else:
            residual_pressure = np.inf

        delta_p_c = delta_p_c_new
        delta_p_h = delta_p_h_new
        P_o_c     = -delta_p_c+P_i_c
        P_o_h     = -delta_p_h+P_i_h

        if residual_pressure < 0.01:
            break

    # Calculate Power drawn by HEX
    P_coolant = (m_dot_h*delta_p_h)/(HEX.pump.efficiency*rho_h_m)
    P_air     = np.where(state.conditions.freestream.velocity > HEX.minimum_air_speed, 0., (m_dot_c*delta_p_c/rho_c_m)/HEX.fan.efficiency)
    P_hex     = P_air+P_coolant

    Q_rejected = q

    HEX_conditions                             = state.conditions.energy.distributors[coolant_line.tag][HEX.tag]
    HEX_conditions.pressure_diff_air           = delta_p_c
    HEX_conditions.coolant_mass_flow_rate      = m_dot_h
    HEX_conditions.power                       = P_hex
    HEX_conditions.inlet_air_temperature       = T_i_c
    HEX_conditions.outlet_coolant_temperature  = T_o_h
    HEX_conditions.air_mass_flow_rate          = m_dot_c
    HEX_conditions.air_inlet_pressure          = P_i_c
    HEX_conditions.coolant_inlet_pressure      = P_i_h
    HEX_conditions.effectiveness_HEX           = eff_hex

    return Q_rejected

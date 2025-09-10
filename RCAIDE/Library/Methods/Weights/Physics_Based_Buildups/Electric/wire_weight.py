def cable_mass(V, E0, r_cond, rho, rho_theta_insul,L, rho_cond, rho_insul, theta_a, I, T_4):

    # Equation (18): Cable Insulation Radius based on voltage and electric field constraints
    # E0 is the electric field
    r_insul = r_cond * np.exp(V / (E0 * r_cond))  # Equation (18)

    # Equation (20): Conductor Resistance (thermal constraint based on material properties)
    R_prime = rho / (np.pi * r_cond ** 2)  # Equation (20)

    # Equation (21): Thermal Resistance of the insulation
    T_1 = rho_theta_insul / (2 * np.pi) * np.log(r_insul / r_cond)  # Equation (21)

    # Equation (22): Total Cable Mass calculation based on conductor and insulation volume and density
    M_cable = np.pi * L * (r_cond ** 2 * rho_cond + (r_insul ** 2 - r_cond ** 2) * rho_insul)  # Equation (22)

    # Equation (19): Maximum Temperature (conductor temperature based on current, resistance, and thermal resistances)
    theta_max = theta_a + I**2 * R_prime * (T_1 + T_4)  # Equation (19)
    
    return M_cable, theta_max
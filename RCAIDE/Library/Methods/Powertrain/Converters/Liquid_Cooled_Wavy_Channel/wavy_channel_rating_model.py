# RCAIDE/Methods/Energy/Thermal_Management/Batteries/Heat_Acquisition_Systems/wavy_channel_rating_model.py
#
#
# Created:  Apr 2024, M. Clarke, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Wavy Channel Rating Model
# ----------------------------------------------------------------------------------------------------------------------
def  wavy_channel_rating_model(HAS,battery_module,coolant_line,Q_heat_gen,T_cell,state):
    """ Computes the rate of temperature change of a battery module cooled by a wavy
        channel heat acquisition system, and the heat delivered to the coolant loop.

    Assumptions:
    1) Battery pack cell heat transfer can be modelled as a cooling columns in a cross-flow
    2) Isothermal battery_module cell - the temperature at the center of the cell is the same at
    the surface of the cell

    Source:
    Zhao, C., Clarke, M., Kellermann H., Verstraete D.,
    “Design of a Liquid Cooling System for Lithium-Ion Battery Packs for eVTOL Aircraft"

    Inputs:
              T_cell                    (battery_module cell temperature)   [Kelvin]
              heat_transfer_efficiency                               [unitless]
          HAS.
              channel_side_thickness                                 [meter]
              channel_width                                          [meter]
              channel_contact_angle                                  [degree]
              channel_top_thickness                                  [meter]
              channel                   (Properties of channel)     [unitless]
              coolant                   (Properties of coolant)    [unitless]
              coolant_flow_rate                                       [kg/s]


      Outputs:
             dT_dt                      (battery_module cell temperature rate) [Kelvin/s]
             Q_to_coolant               (heat delivered to the coolant loop)   [Watts]

    Properties Used:
    None
    """
    # Inlet coolant temperature comes from the reservoir's own implicit unknown, solved
    # simultaneously with every other network unknown (no explicit time marching).
    reservoir                = list(coolant_line.reservoirs)[0]
    T_inlet                  = state.unknowns.network[coolant_line.tag + '_' + reservoir.tag + '_coolant_temperature']
    heat_transfer_efficiency = HAS.heat_transfer_efficiency

    # Coolant Properties
    m_coolant                   = HAS.coolant_flow_rate
    coolant                     = HAS.coolant
    rho                         = coolant.compute_density(T_inlet)
    mu                          = coolant.compute_absolute_viscosity(T_inlet)
    cp                          = coolant.compute_cp(T_inlet)
    Pr                          = coolant.compute_prandtl_number(T_inlet)
    k                           = coolant.compute_thermal_conductivity(T_inlet)

    # Battery Properties
    d_cell                      = battery_module.cell.diameter
    h_cell                      = battery_module.cell.height
    A_cell                      = np.pi*d_cell*h_cell
    N_cells_geometric_config    = battery_module.geometric_configuration.parallel_count*battery_module.geometric_configuration.normal_count
    cell_mass                   = battery_module.cell.mass
    Nn_module_cells             = battery_module.electrical_configuration.series
    Np_module_cells             = battery_module.electrical_configuration.parallel
    number_of_cells_in_module   = Nn_module_cells*Np_module_cells
    Q_module                    = Q_heat_gen*number_of_cells_in_module
    Cp_bat                      = battery_module.cell.specific_heat_capacity

    # Channel Properties
    b         = HAS.channel_side_thickness
    d         = HAS.channel_width
    theta     = HAS.channel_contact_angle
    c         = battery_module.cell.height
    channel   = HAS.channel
    AR        = d/c
    k_chan    = channel.thermal_conductivity
    n_pump    = 0.7

    # Contact Surface area of the channel
    A_chan   = 2*N_cells_geometric_config*(theta)*A_cell

    # Length of Channel
    L_extra  =  battery_module.geometric_configuration.parallel_count*d_cell
    L_chan   = (N_cells_geometric_config*d_cell)+L_extra

    # Hydraulic diameter
    dh   = (4*c*d)/(2*(c+d))
    # calculate the velocity of the fluid in the channel
    v=rho*c*d*m_coolant

    # calculate the Reynolds Number
    Re=(rho*dh*v)/mu

    # fanning friction factor (eq 32),  Nusselt Number (eq 12)
    if np.all(Re < 2300):
        f= 24*(1-(1.3553*AR)+(1.9467*(AR**2))-(1.7012*(AR**3))+(0.9564*(AR**4))-(0.2537*(AR**5)))/Re
        Nu = 8.235*(1-(2.0421*AR)+(3.0853*(AR**2))-(2.4765*(AR**3))+(1.0578*(AR**4))-(0.1861*(AR**5)))
    else:
        f= (0.0791*(Re**(-0.25)))*(1.8075-0.1125*AR)
        Nu = ((f/2)*(Re-1000)*Pr)/(1+(12.7*((f/2)**0.5)*(Pr**(2/3)-1)))

    # Calculate the pressure drop in the channel
    dp     = 2*f*rho*v*v*L_chan/dh

    # heat transfer coefficient of the channeled coolant (eq 11)
    h = k*Nu/dh

    # Overall Heat Transfer Coefficient from battery_module surface to the coolant fluid (eq 10)
    U_total = 1/((1/h)+(b/k_chan))

    # Calculate NTU
    NTU = U_total*A_chan/(m_coolant*cp)

    # Log-mean-temperature-difference term, signed so that it is positive when the cell is
    # hotter than the inlet coolant (heat flows cell -> coolant) and negative when the
    # coolant is hotter than the cell (heat flows coolant -> cell). Written this way the
    # same expression covers both directions without branching or dividing by a
    # temperature difference that can be zero.
    T_lm = (T_cell-T_inlet)*(1-np.exp(-NTU))/NTU

    # Heat delivered from the cell to the coolant (can be negative, i.e. the coolant warms
    # the cell, if the coolant is hotter than the cell)
    Q_to_coolant = U_total*A_chan*T_lm*heat_transfer_efficiency

    # Net heat retained by the cell
    P_net = Q_module - Q_to_coolant

    dT_dt = P_net/(cell_mass*N_cells_geometric_config*Cp_bat)

    # Outlet coolant temperature and channel effectiveness, stored for reporting only
    T_outlet    = T_inlet + (T_cell-T_inlet)*(1-np.exp(-NTU))
    eff_channel = 1-np.exp(-NTU)

    # Calculate the Power consumed by the coolant pump
    Power = m_coolant * dp / (rho * n_pump)

    HAS_conditions                             = state.conditions.energy.distributors[coolant_line.tag][HAS.tag]
    HAS_conditions.heat_removed                = Q_to_coolant
    HAS_conditions.outlet_coolant_temperature  = T_outlet
    HAS_conditions.coolant_mass_flow_rate      = m_coolant * np.ones_like(T_cell)
    HAS_conditions.effectiveness               = eff_channel
    HAS_conditions.power                       = Power * np.ones_like(T_cell)

    return dT_dt, Q_to_coolant

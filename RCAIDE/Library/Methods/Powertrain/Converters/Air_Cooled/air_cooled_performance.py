# RCAIDE/Methods/Thermal_Management/Batteries/Heat_Acquisition_System/Direct_Air_Heat_Acquisition/direct_convection_model.py
#
#
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
from RCAIDE.Framework.Core import array_namespace

# ----------------------------------------------------------------------------------------------------------------------
#  Compute Net Convected Heat
# ----------------------------------------------------------------------------------------------------------------------
def air_cooled_performance(HAS,battery_module,coolant_line, Q_heat_gen,T_cell,state):
    '''Computes the rate of temperature change of a battery module cooled directly by
    ambient air, and the heat delivered to that air.

    Assumptions:
    1) Battery pack cell heat transfer can be modelled as a cooling columns in a cross-flow
    2) Isothermal battery cell - the temperature at the center of the cell is the same at
    the surface of the cell

    Source:
    Heat Transfer Model:
    Pakowski, Zdzisław. "Fundamentals of Heat and Mass Transfer, Frank P Incropera,
    David P DeWitt, Theodore L Bergman, Adrienne S Lavine, J. Wiley & Sons, Hoboken
    NJ (2007), 997 pp." (2007): 1683-1684.,  Chapter 7 pg 437-446
    Clarke, M. and Alonso, J. J., "Lithium-Ion Battery Modeling for Aerospace Applications", Journal of Aircraft,
    Vol. 58, No. 6 (2021), pp. 1323-1335, Eqs. 7 and 10-12: tube bank in cross flow, with the wall Prandtl number and the log-mean temperature
    difference taken at the cell surface temperature

    Inputs:
              h                         (heat transfer coefficient)  [W/(m^2*K)]
              As_cell                   (battery cell surface area)  [meters^2]
              H_cell                    (battery cell height)        [meters]
              T_ambient                 (ambient temperature)        [Kelvin]
              T_cell                    (battery cell temperature)   [Kelvin]
              heat_transfer_efficiency                               [unitless]


      Outputs:
             dT_dt                      (battery cell temperature rate) [Kelvin/s]
             Q_convec                   (heat delivered to the ambient air) [Watts]

    Properties Used:
    None
    '''


    # battery properties
    As_cell                  = battery_module.cell.surface_area
    D_cell                   = battery_module.cell.diameter
    H_cell                   = battery_module.cell.height
    cell_mass                = battery_module.cell.mass
    Cp                       = battery_module.cell.specific_heat_capacity
    Nn                       = battery_module.geometric_configuration.normal_count
    Np                       = battery_module.geometric_configuration.parallel_count
    n_total_module           = Nn*Np
    h                        = HAS.convective_heat_transfer_coefficient
    heat_transfer_efficiency = HAS.heat_transfer_efficiency
    T_ambient                = state.conditions.freestream.temperature

    if n_total_module == 1:
        # Using lumped model
        Q_convec       = h*As_cell*(T_cell - T_ambient)
        Q_heat_gen_tot = Q_heat_gen
        T_outlet       = T_ambient

    else:
        K_coolant                    = state.conditions.freestream.thermal_conductivity
        nu_coolant                   = state.conditions.freestream.kinematic_viscosity
        Pr_coolant                   = state.conditions.freestream.prandtl_number
        rho_coolant                  = state.conditions.freestream.density
        Cp_coolant                   = HAS.cooling_fluid.compute_cp(state.conditions.freestream.temperature,state.conditions.freestream.pressure )
        V_coolant                    = state.conditions.freestream.velocity

        # Chapter 7 pg 437-446 of Fundamentals of heat and mass transfer
        S_T             = battery_module.geometric_configuration.normal_spacing
        S_L             = battery_module.geometric_configuration.parallel_spacing

        S_D = (S_T**2+S_L**2)**0.5
        # the maximum velocity is in the diagonal gap when that is the narrower one
        xp       = array_namespace(V_coolant, T_cell, S_T, S_L, D_cell)
        diagonal = 2*(S_D-D_cell) < (S_T-D_cell)
        V_max    = V_coolant*xp.where(diagonal, S_T/(2*(S_D-D_cell)), S_T/(S_T-D_cell))

        # the cells are isothermal, so the tube-bank wall temperature is the cell temperature [Clarke & Alonso Eqs. 7, 10-11]
        T_wall   = T_cell
        Re_max   = V_max*D_cell/nu_coolant
        # one correlation range for every point, as chosen by the lowest Reynolds number
        upper    = xp.all(Re_max > 10E2)
        C        = xp.where(upper, 0.35*((S_T/S_L)**0.2), 0.51)
        m        = xp.where(upper, 0.6, 0.5)

        Pr_w_coolant          = HAS.cooling_fluid.compute_prandtl_number(T_wall)
        Nu                    = C*(Re_max**m)*(Pr_coolant**0.36)*((Pr_coolant/Pr_w_coolant)**0.25)
        h                     = Nu*K_coolant/D_cell
        Tw_Ti                 = (T_wall - T_ambient)
        decay                 = xp.exp((-np.pi*D_cell*n_total_module*h)/(rho_coolant*V_coolant*Nn*S_T*Cp_coolant))
        Tw_To                 = Tw_Ti * decay
        # no temperature difference, no heat; a unit difference keeps the unused log-mean finite
        uniform               = Tw_Ti == 0.
        Tw_Ti_safe            = xp.where(uniform, 1., Tw_Ti)
        Tw_To_safe            = Tw_Ti_safe * decay
        dT_lm                 = (Tw_Ti_safe - Tw_To_safe)/xp.log(Tw_Ti_safe/Tw_To_safe)
        Q_convec              = xp.where(uniform, 0., heat_transfer_efficiency*h*np.pi*D_cell*H_cell*n_total_module*dT_lm)
        Q_heat_gen_tot        = Q_heat_gen*n_total_module
        T_outlet              = T_ambient + Tw_To

    Q_net                     = Q_heat_gen_tot - Q_convec
    dT_dt                     = Q_net/(cell_mass*n_total_module*Cp)
    effectiveness             = (T_outlet - T_ambient) / array_namespace(T_cell, T_ambient).where(T_cell == T_ambient, 1., T_cell - T_ambient)

    HAS_conditions            = state.conditions.energy.distributors[coolant_line.tag][HAS.tag]
    HAS_conditions.heat_removed  = Q_convec
    HAS_conditions.effectiveness = effectiveness

    return dT_dt, Q_convec

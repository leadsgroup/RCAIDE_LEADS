# RCAIDE/Library/Methods/Thermal_Management/Reservoirs/Reservoir_Tank/compute_reservoir_performance.py
#
# Created: Nov 2026

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  compute_reservoir_performance
# ----------------------------------------------------------------------------------------------------------------------
def compute_reservoir_performance(reservoir,coolant_line,state,network):
    """
    Computes the reservoir coolant-temperature residual for the current control points.

    The reservoir is treated as a single lumped thermal mass that: gains heat from every
    battery module cooled by this coolant line (already computed once, during each battery
    module's own performance call, and stashed on that module's conditions as
    `heat_to_coolant`), loses heat through any heat exchangers on the coolant line, and
    loses/gains heat passively to the ambient environment through its walls.
    """
    D   = state.numerics.time.differentiate
    key = coolant_line.tag + '_' + reservoir.tag + '_coolant_temperature'

    T_scale          = 310.0
    T_coolant_unkn   = state.unknowns.network[key]
    T_coolant_scaled = T_coolant_unkn / T_scale

    # ---------------------------------------------------------------------------------
    # Heat absorbed from battery modules assigned to this coolant line
    # ---------------------------------------------------------------------------------
    Q_absorbed = np.zeros_like(T_coolant_unkn)
    for source in network.sources:
        if isinstance(source,RCAIDE.Library.Components.Powertrain.Sources.Batteries.Battery_Pack):
            for module in source.modules:
                if module.active and module.assigned_distributors != None and coolant_line.tag in module.assigned_distributors[0]:
                    Q_absorbed = Q_absorbed + state.conditions.energy.sources[source.tag][module.tag].heat_to_coolant

    # ---------------------------------------------------------------------------------
    # Heat rejected through heat exchangers on this coolant line
    # ---------------------------------------------------------------------------------
    Q_rejected = np.zeros_like(T_coolant_unkn)
    for HEX in coolant_line.heat_exchangers:
        Q_rejected = Q_rejected + HEX.compute_heat_exchanger_performance(coolant_line,T_coolant_unkn,state)

    # ---------------------------------------------------------------------------------
    # Passive heat loss to the ambient environment through the reservoir walls
    # ---------------------------------------------------------------------------------
    T_ambient  = state.conditions.freestream.temperature
    Q_env      = compute_reservoir_heat_loss_to_environment(reservoir,T_coolant_unkn,T_ambient)

    coolant_cp = reservoir.coolant.compute_cp(T_coolant_unkn)
    dT_dt      = (Q_absorbed - Q_rejected - Q_env) / (reservoir.mass_properties.mass * coolant_cp)

    dT_dt_scaled = dT_dt / T_scale
    R            = np.dot(D,T_coolant_scaled)[:,0] - dT_dt_scaled[:,0]

    reservoir_conditions = state.conditions.energy.distributors[coolant_line.tag][reservoir.tag]
    R[0] = T_coolant_scaled[0,0] - reservoir_conditions.coolant_temperature[0,0] / T_scale
    state.residuals.network[key] = R

    reservoir_conditions.coolant_temperature[1:,0] = T_coolant_unkn[1:,0]

    return


def compute_reservoir_heat_loss_to_environment(reservoir,T_reservoir,T_ambient):
    """ Passive heat loss from the reservoir to the ambient environment through
    conduction, natural convection, and radiation from its outer surface. """
    A_surface      = reservoir.surface_area
    thickness      = reservoir.thickness
    conductivity   = reservoir.material.conductivity
    emissivity_res = reservoir.material.emissivity

    sigma          = 5.69e-8  # Stefan Boltzmann Constant
    h              = 1000.    # [W/m^2-K]
    emissivity_air = 0.9

    dQ_dt_cond = conductivity * A_surface * (T_reservoir - T_ambient) / thickness
    dQ_dt_conv = h * A_surface * (T_reservoir - T_ambient)
    dQ_dt_rad  = sigma * A_surface * ((emissivity_res * T_reservoir**4) - (emissivity_air * T_ambient**4))

    return dQ_dt_cond + dQ_dt_conv + dQ_dt_rad

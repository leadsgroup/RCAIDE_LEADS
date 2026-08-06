# RCAIDE/Methods/Powertrain/Distributors/Fuel_Line/compute_fuel_line_distribution_losses.py
# 
# 
# Created: Mar 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# imports 
import numpy as np 
from scipy.optimize import fsolve
 
# ----------------------------------------------------------------------------------------------------------------------
# compute_fuel_line_distribution_losses
# ----------------------------------------------------------------------------------------------------------------------
def compute_fuel_line_distribution_losses(fuel_line,component_conditions,state):
    """
    Computes the pipe-friction and minor losses of a fuel line and accumulates the
    resulting ideal hydraulic power demand onto the fuel line's conditions.

    Parameters
    ----------
    fuel_line : RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line
        The fuel line whose losses are being computed
    component_conditions : Conditions
        Conditions of the propulsor, source, or converter assigned to this fuel
        line for which this call is computing a fuel flow contribution (this
        function is called once per assigned component, so it runs multiple
        times per fuel line per iteration)
    state : RCAIDE.Framework.Mission.Common.State
        Mission segment state

    Returns
    -------
    None

    Notes
    -----
    Head loss is computed from Darcy-Weisbach pipe friction (via the
    Colebrook-White equation, solved implicitly for Re >= 2300) plus minor
    losses from bends and fittings. The resulting ideal hydraulic power is
    accumulated, un-derated by any pump efficiency, onto
    ``fuel_line.inputs.power.hydraulic``.

    A connected pump converter (RCAIDE.Library.Components.Powertrain.Converters.Pump
    and its subclasses, e.g. Cryogenic_Pump) applies its own efficiency when it
    later pulls its ``distributor_split`` share of this demand -- see
    compute_cryogenic_pump_performance. If no pump converter is assigned to this
    fuel line, Fuel_Line.compute_performance() charges the network directly for
    this amount instead, so it is not silently dropped from the power budget.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line.compute_performance
    RCAIDE.Library.Methods.Powertrain.Converters.Cryogenic_Pump.compute_cryogenic_pump_performance
    """

    # unpack working fluid properties
    working_fluid   = fuel_line.working_fluid
    density         = working_fluid.density
    k_viscosity     = working_fluid.kinematic_viscosity

    # Extract current conditions for the fuel line
    fuel_line_conditions    = state.conditions.energy.distributors[fuel_line.tag]

    chemical_power      = abs(component_conditions.outputs.power.chemical - component_conditions.inputs.power.chemical)
    hydraulic_power     = abs(component_conditions.outputs.power.hydraulic - component_conditions.inputs.power.hydraulic)
    mass_flow_rate      = chemical_power /working_fluid.lower_heating_value

    # unpack fuel line properties
    length                 = fuel_line.length
    diameter               = fuel_line.pipe.diameters.internal
    surface_roughness      = fuel_line.pipe.surface_roughness 
    bend_90_deg_standard   = fuel_line.pipe.k_factors.bend_90_deg
    bend_45_deg            = fuel_line.pipe.k_factors.bend_45_deg              
    pipe_entrance_rounded  = fuel_line.pipe.k_factors.pipe_entrance_rounded
    pipe_exit              = fuel_line.pipe.k_factors.pipe_exit
  
    # Determine head loss due to pipe friction and minor losses, then compute the required pump performance to overcome these losses and deliver the required flow rate to the engine.
    # 1. Initialization
    volumetric_flow_rate = mass_flow_rate / density  # m^3/s 
    e                    = surface_roughness / 1000   # m
    area                 = np.pi * (diameter/2)**2
    flow_velocity        = volumetric_flow_rate / area              # velocity (m/s)
    g = 9.81

    number_of_90_deg_bends = 3
    number_of_45_deg_bends = 0

    # 2. Dynamically Calculate K_total 
    k_total =  bend_90_deg_standard*number_of_90_deg_bends + \
                bend_45_deg *number_of_45_deg_bends + \
                pipe_entrance_rounded  + \
                pipe_exit              
    
    # 3. Reynolds Number
    # Floored to avoid a 64/Re divide-by-zero (and the resulting inf*0 = nan in
    # h_major) when flow_velocity is exactly zero, e.g. at an idle/ground segment
    # or the solver's initial guess.
    Re = np.maximum((flow_velocity * diameter) / k_viscosity, 1e-6)

    # 4. Friction Factor (f)
    f =  np.zeros_like(Re)
    f_Re_leq_2300 = 64 / Re # Re < 2300

 
    def colebrook(f_initial): 
        vals =  1/np.sqrt(f_initial) + 2*np.log10((e/diameter)/3.7 + 2.51/(Re[:, 0]*np.sqrt(f_initial)))    
        return vals     
    
    # f_initial using Swamee-Jain explicit formula
    f_initial = 0.25 / (np.log10((e/diameter)/3.7 + 5.74/(Re[:, 0]**0.9)))**2
    f_Re_geq_2300 = fsolve(colebrook, f_initial)
    
    f[:,0]        = f_Re_geq_2300
    f[ Re < 2300] = f_Re_leq_2300[ Re < 2300] 

    # 5. Head Loss (m)
    h_major     = f * (length / diameter) * (flow_velocity**2 / (2 * g))
    h_minor     = k_total * (flow_velocity**2 / (2 * g)) 
    h_total     = h_major + h_minor 

    # 6. Pressure to overcome friction/gravity
    delta_p_losses  = h_total * density * g   

    # 7. Ideal hydraulic power a pump must add to the fluid to overcome line
    # losses and deliver this component's share of flow. 
    power_losses      = volumetric_flow_rate * delta_p_losses
    power_ideal_total = hydraulic_power + power_losses

    fuel_line_conditions.inputs.power.hydraulic  += power_ideal_total
    fuel_line_conditions.mass_flow_rate          += mass_flow_rate
    return
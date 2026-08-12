# RCAIDE/Library/Methods/Mass_Properties/Weight_Buildups/Hydrogen/BWB/Semi_Empirical/ccompute_propulsion_system_weight.py
#
#
# Created:  Sep 2024, M. Clarke
# Modified: Jul 2026, S. Sharma

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE
import  RCAIDE
from RCAIDE.Framework.Core    import Units ,  Data
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity.compute_distributor_center_of_gravity import compute_distributor_center_of_gravity

# python imports
import  numpy as  np
from copy import deepcopy

# ----------------------------------------------------------------------------------------------------------------------
#  Propulsion Systems Weight
# ----------------------------------------------------------------------------------------------------------------------
def compute_propulsion_system_weight(vehicle,ref_propulsor, settings):
    """ Calculate the weight of propulsion system, including:
        - dry engine weight
        - fuel system weight
        - thurst reversers weight
        - electrical system weight
        - starter engine weight
        - nacelle weight
        - cargo containers

        Assumptions:
            1) Rated thrust per scaled engine and rated thurst for baseline are the same
            2) Engine weight scaling parameter is 1.15
            3) Enginge inlet weight scaling exponent is 1
            4) Baseline inlet weight is 0 lbs as in example files FLOPS
            5) Baseline nozzle weight is 0 lbs as in example files FLOPS

        Source:
            The Flight Optimization System Weight Estimation Method

        Inputs:
            vehicle - data dictionary with vehicle properties                   [dimensionless]
                -.design_mach_number: design mach number for cruise flight
                -.systems.accessories: type of aircraft (short-range, commuter
                                                        medium-range, long-range,
                                                        sst, cargo)
            nacelle - data dictionary with propulsion system properties
                -.diameter: diameter of nacelle                                 [meters]
                -.length: length of complete engine assembly                    [meters]
            ref_propulsor.
                -.sealevel_static_thrust: thrust at sea level                   [N]


        Outputs:
            output - data dictionary with weights                               [kilograms]
                    - output.W_prop: total propulsive system weight
                    - output.W_thrust_reverser: thurst reverser weight
                    - output.starter: starter engine weight
                    - output.W_engine_controls: engine controls weight
                    - output.fuel_system: fuel system weight
                    - output.nacelle: nacelle weight
                    - output.W_engine: dry engine weight

        Properties Used:
            N/A
    """

    NENG   =  0
    WEC    =  0
    WNAC   =  0
    WSTART =  0
    number_of_tanks =  0
    ref_nacelle =  None
    for network in  vehicle.networks:
        for propulsor in network.propulsors:
            if isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan) \
               or  isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet)\
               or  isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turboprop):
                ref_propulsor = propulsor
                NENG  += 1
            if propulsor.nacelle !=  None:
                ref_nacelle =  propulsor.nacelle
        for source in network.sources:
            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank):
                number_of_tanks +=  1

    if ref_nacelle is not None:
        WNAC        = compute_nacelle_weight(ref_propulsor,ref_nacelle,NENG )
    WTANK, WLINE, WPUMP, WFC = compute_fuel_system_weight(vehicle,ref_propulsor)
    WENG            = compute_engine_weight(vehicle,ref_propulsor)
    if ref_nacelle is not None:
        WEC, WSTART = compute_misc_propulsion_system_weight(vehicle,ref_propulsor,ref_nacelle,NENG)
    WTHR            = compute_thrust_reverser_weight(ref_propulsor,NENG)
    WPRO            = NENG * WENG + WTANK + WLINE + WPUMP + WFC + WEC + WSTART + WTHR # Nacelle weight is not included in the propulsion system weight. it is included in the structural weight.

    output                      = Data()
    output.W_prop               = WPRO
    output.W_thrust_reverser    = WTHR
    output.W_starter            = WSTART
    output.W_engine_controls    = WEC
    output.W_tanks              = WTANK
    output.W_fuel_lines         = WLINE
    output.W_pumps              = WPUMP
    output.W_fuel_cells         = WFC
    output.W_nacelle            = WNAC
    output.W_engine             = WENG * NENG
    output.number_of_engines    = NENG
    output.number_of_fuel_tanks = number_of_tanks
    return output

def compute_fuel_system_weight(vehicle,ref_propulsor):
    """ Calculates the weight of the fuel system based on Wess ****update l
        Source:
            The Flight Optimization System Weight Estimation Method

        Inputs:
            vehicle - data dictionary with vehicle properties                   [dimensionless]
                -.design_mach_number: design mach number
                -   [kg]

        Outputs:
            WFSYS: Fuel system weight                                       [kg]

        Properties Used:
            N/A
    """
    WTANK = 0
    WLINE = 0
    WPUMP = 0
    WFC   = 0

    for network in vehicle.networks:
        for source in network.sources:
            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank):
                WTANK += source.tank_accesories_weight_factor * (source.insulation.mass_properties.mass + source.inner_structure.mass_properties.mass)

        for distributor in network.distributors:
            if isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line):
                compute_distributor_center_of_gravity(distributor, vehicle, length=0)
                WLINE += distributor.mass_properties.mass
                compute_transfer_pump_weight(network, distributor,ref_propulsor)

        for converter in network.converters:
            if issubclass(type(converter),RCAIDE.Library.Components.Powertrain.Converters.Pump):
                WPUMP += converter.mass_properties.mass
            elif issubclass(type(converter),RCAIDE.Library.Components.Powertrain.Converters.Generic_Fuel_Cell_Stack):
                if converter.mass_properties.mass == 0:
                    converter.mass_properties.mass = converter.design_power / converter.specific_power
                WFC += converter.mass_properties.mass
            elif issubclass(type(converter),RCAIDE.Library.Components.Powertrain.Converters.Reformer_Fuel_Cell):
                if converter.fuel_cell is not None:
                    if converter.fuel_cell.mass_properties.mass == 0:
                        converter.fuel_cell.mass_properties.mass = converter.fuel_cell.design_power / converter.fuel_cell.specific_power
                    WFC += converter.fuel_cell.mass_properties.mass
                if converter.reformer is not None:
                    if converter.reformer.mass_properties.mass == 0:
                        converter.reformer.mass_properties.mass = converter.reformer.design_power / converter.reformer.specific_power
                    WFC += converter.reformer.mass_properties.mass

    return WTANK, WLINE, WPUMP, WFC


def compute_transfer_pump_weight(network, fuel_line,ref_propulsor):
    """ Sizes and weighs a fuel line's transfer/boost pump and wires it into the network.
    Called from compute_fuel_system_weight for every Fuel_Line in a network, so this must be
    idempotent: safe to call again on the next MTOW-iteration pass without appending the pump
    twice into network.converters.

    Design point mostly comes from the pump's own attributes (set at vehicle_setup() time) --
    this is the same formula formerly in the standalone design_pump(), merged in here since
    design_pump() only ever ran once, up front, and this function's own mass==0 guard meant its
    result was silently overridden by design_pump()'s whenever both ran. Mass flow is the
    exception: rather than an explicit pump.design_mass_flow_rate, it's still estimated from the
    reference engine's thrust via a reference SFC, as this function did before the merge.

    Sizing chain (NASA SP-8107-style): fluid power Pf = mdot*dP/rho, shaft power Ps = Pf/eta.

    power_density (design_power / mass, before the casting_and_mount_factor margin) is a
    per-vehicle input, not derived -- set pump.power_density explicitly; falls back to
    15000 W/kg (a generic electric pump figure) if unset. See git history for the previous
    fuel-type-keyed (200/400 W/kg) SFC-derived default this replaced.

    Inputs:
            network                           - the vehicle's Fuel network
            fuel_line                         - Fuel_Line whose auto-created .pump is sized
            ref_propulsor                     - reference engine whose design_thrust (per-engine
                                                 cruise design point, not sealevel_static_thrust)
                                                 drives the reference-SFC mass-flow estimate

    Outputs:
            pump - the sized, positioned, network-wired Pump component

    Properties Used:
            N/A
    """
    reference_sfc = 0.08/3600
    reference_fuel_specific_energy = 48.632e6

    for converter in network.converters:
        if type(converter) is RCAIDE.Library.Components.Powertrain.Converters.Cryogenic_Pump:
            pump = converter

            # check if the pump's mass is defined or not
            if pump.mass_properties.mass == 0.0:

                # check to see if the pump is connected to the fuel line
                if fuel_line.tag in pump.assigned_distributors[0]:
                    fuel = pump.working_fluid

                    # pressure rise
                    pressure_rise = pump.design_outlet_pressure - pump.design_inlet_pressure

                    # mass flow, estimated from reference engine thrust via reference SFC
                    thermal_power_per_N = reference_sfc * reference_fuel_specific_energy
                    m_dot               = ref_propulsor.design_thrust * thermal_power_per_N / fuel.specific_energy

                    # volumetric flow rate
                    Q = m_dot / fuel.density

                    # hydraulic power
                    hydraulic_power = Q * pressure_rise

                    # shaft power
                    total_efficiency = pump.efficiency * pump.turbine_efficiency
                    shaft_power      = hydraulic_power / total_efficiency

                    if pump.power_density != None:
                        power_density = pump.power_density
                    else:
                        power_density = 15000 # W/kg

                    pump.design_power         = shaft_power
                    pump.mass_properties.mass = (shaft_power / power_density) * pump.casting_and_mount_factor

    return


def compute_nacelle_weight(ref_propulsor,ref_nacelle,NENG):
    """ Calculates the nacelle weight based on the FLOPS method

        Assumptions:
            1) All nacelles are identical
            2) The number of nacelles is the same as the number of engines

        Source:
            The Flight Optimization System Weight Estimation Method

        Inputs:
            ref_propulsor    - data dictionary for the specific network that is being estimated [dimensionless]
                -.number_of_engines: number of engines
                -.engine_lenght: total length of engine                                  [m]
                -.sealevel_static_thrust: sealevel static thrust of engine               [N]
            nacelle.
                -.diameter: diameter of nacelle                                          [m]
            WENG    - dry engine weight                                                  [kg]


        Outputs:
            WNAC: nacelle weight                                                         [kg]

        Properties Used:
            N/A
    """
    TNAC   = NENG + 0.5 * (NENG - 2 * np.floor(NENG / 2.))
    DNAC   = ref_nacelle.diameter / Units.ft
    XNAC   = ref_nacelle.length / Units.ft
    FTHRST = ref_propulsor.sealevel_static_thrust * 1 / Units.lbf
    WNAC   = 0.25 * TNAC * DNAC * XNAC * FTHRST ** 0.36
    return WNAC * Units.lbs


def compute_thrust_reverser_weight(ref_propulsor,NENG):
    """ Calculates the weight of the thrust reversers of the aircraft

        Assumptions:

        Source:
            The Flight Optimization System Weight Estimation Method

        Inputs:
            ref_propulsor    - data dictionary for the specific network that is being estimated [dimensionless]
                -.number_of_engines: number of engines
                -.sealevel_static_thrust: sealevel static thrust of engine  [N]

        Outputs:
            WTHR: Thrust reversers weight                                   [kg]

        Properties Used:
            N/A
    """
    TNAC = NENG + 1. / 2 * (NENG - 2 * np.floor(NENG / 2.))
    THRUST = ref_propulsor.sealevel_static_thrust * 1 / Units.lbf
    WTHR = 0.034 * THRUST * TNAC
    return WTHR * Units.lbs


def compute_misc_propulsion_system_weight(vehicle,ref_propulsor,ref_nacelle,NENG ):
    """ Calculates the miscellaneous engine weight based on the FLOPS method, electrical control system weight
        and starter engine weight

        Assumptions:
            1) All nacelles are identical
            2) The number of nacelles is the same as the number of engines

        Source:
            The Flight Optimization System Weight Estimation Method

        Inputs:
            vehicle - data dictionary with vehicle properties                            [dimensionless]
                 -.design_mach_number: design mach number
            ref_propulsor    - data dictionary for the specific network that is being estimated [dimensionless]
                -.number_of_engines: number of engines
                -.sealevel_static_thrust: sealevel static thrust of engine               [N]
            nacelle
                -.diameter: diameter of nacelle                                          [m]

        Outputs:
            WEC: electrical engine control system weight                                 [kg]
            WSTART: starter engine weight                                                [kg]

        Properties Used:
            N/A
    """
    THRUST  = ref_propulsor.sealevel_static_thrust * 1 / Units.lbf
    WEC     = 0.26 * NENG * THRUST ** 0.5
    FNAC    = ref_nacelle.diameter / Units.ft
    VMAX    = vehicle.flight_envelope.design_mach_number
    WSTART  = 11.0 * NENG * VMAX ** 0.32 * FNAC ** 1.6
    return WEC * Units.lbs, WSTART * Units.lbs


def compute_engine_weight(vehicle, ref_propulsor):
    """ Calculates the dry engine weight based on the FLOPS method
        Assumptions:
            Rated thrust per scaled engine and rated thurst for baseline are the same
            Engine weight scaling parameter is 1.15
            Enginge inlet weight scaling exponent is 1
            Baseline inlet weight is 0 lbs as in example files FLOPS
            Baseline nozzle weight is 0 lbs as in example files FLOPS

        Source:
            The Flight Optimization System Weight Estimation Method

        Inputs:
            vehicle - data dictionary with vehicle properties                   [dimensionless]
                -.systems.accessories: type of aircraft (short-range, commuter
                                                        medium-range, long-range,
                                                        sst, cargo)
            ref_propulsor    - data dictionary for the specific network that is being estimated [dimensionless]
                -.sealevel_static_thrust: sealevel static thrust of engine  [N]

        Outputs:
            WENG: dry engine weight                                         [kg]

        Properties Used:
            N/A
    """
    EEXP = 1.15
    EINL = 1
    ENOZ = 1
    THRSO = ref_propulsor.sealevel_static_thrust * 1 / Units.lbf
    THRUST = THRSO
    WENGB = THRSO / 5.5
    WINLB = 0 / Units.lbs
    WNOZB = 0 / Units.lbs
    WENGP = WENGB * (THRUST / THRSO) ** EEXP
    WINL = WINLB * (THRUST / THRSO) ** EINL
    WNOZ = WNOZB * (THRUST / THRSO) ** ENOZ
    WENG = WENGP + WINL + WNOZ
    return WENG * Units.lbs

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

    Design shaft power is estimated at vehicle_setup() time from design_thrust.

    Sizing chain (NASA SP-8107-style): fluid power Pf = mdot*dP/rho, shaft power Ps = Pf/eta.

    specific_power_density (design_power / mass) is selected from the fuel's own type, not
    passed in:
        - Cryogenic (Liquid_Hydrogen, Liquid_Natural_Gas): 200 W/kg.
        - All other fuels (Liquid_Petroleum_Gas, Jet_A, ...): 400 W/kg.
    Derived from real aircraft electric fuel boost pumps -- Eaton Type 9106 (B777:
    6.5 kg, 200V/400Hz 3-phase, 9.5A -> ~3.29 kVA apparent power -> ~506 W/kg) and Type 20004
    (B747: 4.2 kg, 7.8A -> ~2.70 kVA -> ~643 W/kg), derated ~85% for motor efficiency (apparent
    power overstates shaft power). The ambient-fuel figure uses that directly (400 W/kg); the
    cryogenic figure is further halved (200 W/kg) as a reasoned penalty for the cryo-compatible
    double-walled/vacuum-jacketed housing and seals those ambient-temperature Jet-A pumps don't need.

    Inputs:
            network                           - the vehicle's Fuel network
            fuel_line                         - Fuel_Line whose auto-created .pump is sized
            fuel                              - this fuel line's Propellant
            design_thrust                     - per-engine design thrust                         [N]
            engine_origins                    - [origin_1, origin_2, ...] of the engines this
                                                 line feeds, used to place the pump just aft of
                                                 their midpoint
            delta_pressure                    - assumed line-loss + NPSH margin                  [Pa]
            efficiency                        - assumed overall pump efficiency                  [-]
            reference_sfc                     - calibration point for mass flow rate estimation  [kg/N-s]
            reference_fuel_specific_energy    - specific energy of the fuel reference_sfc was
                                                 calibrated against, so other fuels' mass flow
                                                 rate scales correctly through their OWN
                                                 specific_energy (e.g. LH2 needs much less fuel
                                                 mass per unit thrust than LNG/LPG)              [J/kg]
            aft_offset                        - additional offset aft of the engine midpoint     [m]

    Outputs:
            pump - the sized, positioned, network-wired Pump component

    Properties Used:
            N/A
    """
    reference_sfc = 0.08/3600
    reference_fuel_specific_energy =48.632e6

    for converter in network.converters:
        if type(converter) is RCAIDE.Library.Components.Powertrain.Converters.Pump:
            pump = converter

            # check if the pump's mass is defined or not 
            if pump.mass_properties.mass  != 0.0:
                continue 
            else:
                # check to see if the pump is connected to the fuel line
                if fuel_line.tag in pump.assigned_distributors:
                    fuel           = fuel_line.working_fluid 
                    delta_pressure = pump.delta_pressure
                    efficiency     = pump.efficiency

                    if pump.specific_power_density != None:
                        specific_power_density = pump.specific_power_density
                    else:
                        specific_power_density = 15000 # W/kg

                    thermal_power_per_N  = reference_sfc * reference_fuel_specific_energy
                    mdot                 = ref_propulsor.design_thrust * thermal_power_per_N / fuel.specific_energy
                    fluid_power          = mdot * delta_pressure / fuel.density
                    pump.design_power    = fluid_power / efficiency
                    pump.mass_properties.mass = pump.design_power / specific_power_density
                   
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

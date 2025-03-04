# RCAIDE/Library/Methods/Weights/Correlation_Buildups/Raymer/compute_propulsion_system_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE 
import  RCAIDE 
from RCAIDE.Framework.Core    import Units, Data
# python imports 
import  numpy as  np
 
# ----------------------------------------------------------------------------------------------------------------------
# Propulsion System Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_propulsion_system_weight(vehicle,network):
    """ Calculate the weight of propulsion system using Raymer method, including:
        - fuel system weight
        - thurst reversers weight
        - electrical system weight
        - starter engine weight
        - nacelle weight
        - cargo containers
        The dry engine weight comes from the FLOPS relations since it is not listed in Raymer

        Assumptions:

        Source:
            Aircraft Design: A Conceptual Approach

        Inputs:
            vehicle - data dictionary with vehicle properties                   [dimensionless]
            network    - data dictionary for the specific network that is being estimated [dimensionless]

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

    NENG    =  0 
    WENG    =  0
    number_of_tanks =  0
    for network in  vehicle.networks:
        for fuel_line in network.fuel_lines:
            for fuel_tank in fuel_line.fuel_tanks:
                number_of_tanks +=  1
            for propulsor in network.propulsors:
                if isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan) or  isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet):
                    ref_propulsor = propulsor  
                    NENG  += 1
                    BPR =  propulsor.bypass_ratio
                    WENG   += 0.084 *  (propulsor.sealevel_static_thrust/Units.lbf)**1.1 * np.exp(-0.045*BPR) * Units.lbs # Raymer 3rd Edition eq. 10.4 
                if 'nacelle' in propulsor:
                    ref_nacelle =  propulsor.nacelle 
                    
    WFSYS           = compute_fuel_system_weight(vehicle, NENG)
    WNAC            = compute_nacelle_weight(vehicle,ref_nacelle, NENG, WENG)
    WEC, WSTART     = compute_misc_engine_weight(vehicle,NENG, WENG)
    WTHR            = 0
    WPRO            = WENG + WFSYS + WEC + WSTART + WTHR + WNAC

    output                      = Data()
    output.W_prop               = WPRO
    output.W_thrust_reverser    = WTHR
    output.W_starter            = WSTART
    output.W_engine_controls    = WEC
    output.W_fuel_system        = WFSYS
    output.W_nacelle            = WNAC
    output.W_engine             = WENG 
    output.number_of_engines    = NENG
    output.number_of_fuel_tanks = number_of_tanks  
    return output

def compute_nacelle_weight(vehicle,ref_nacelle, NENG, WENG):
    """ Calculates the nacelle weight based on the Raymer method
        Assumptions:
            1) All nacelles are identical
            2) The number of nacelles is the same as the number of engines 
            The engine weight is imported in Kg (The default RCAIDE mass unit)
        Source:
            Aircraft Design: A Conceptual Approach (2nd edition)

        Inputs:
            vehicle - data dictionary with vehicle properties                           [dimensionless]
                -.ultimate_load: ultimate load factor of aircraft
            nacelle  - data dictionary for the specific nacelle that is being estimated [dimensionless]
                -lenght: total length of engine                                         [m]
                -diameter: diameter of nacelle                                          [m]
            WENG    - dry engine weight                                                 [kg]


        Outputs:
            WNAC: nacelle weight                                                        [kg]

        Properties Used:
            N/A
    """ 
    Kng             = 1.017 # assuming the engine is pylon mounted
    Nlt             = ref_nacelle.length / Units.ft
    Nw              = ref_nacelle.diameter / Units.ft
    Wec             = 2.331 * (WENG/Units.lbs) ** 0.901 * 1.18
    Sn              = 2 * np.pi * Nw/2 * Nlt + np.pi * Nw**2/4 * 2
    WNAC            = 0.6724 * Kng * Nlt ** 0.1 * Nw ** 0.294 * vehicle.flight_envelope.ultimate_load ** 0.119 \
                      * Wec ** 0.611 * NENG ** 0.984 * Sn ** 0.224
    return WNAC * Units.lbs

def compute_misc_engine_weight(vehicle, NENG, WENG):
    """ Calculates the miscellaneous engine weight based on the Raymer method, electrical control system weight
        and starter engine weight
        Assumptions:

        Source:
            Aircraft Design: A Conceptual Approach

        Inputs:
            vehicle - data dictionary with vehicle properties                   [dimensionless]
                -.fuselages['fuselage'].lengths.total: length of fuselage   [m]
            network    - data dictionary for the specific network that is being estimated [dimensionless]
                -.number_of_engines: number of engines

        Outputs:
            WEC: electrical engine control system weight                    [kg]
            WSTART: starter engine weight                                   [kg]

        Properties Used:
            N/A
    """

    L =  0 
    for fuselage in vehicle.fuselages:
        if L < fuselage.lengths.total: 
            total_length = fuselage.lengths.total / 2 # approximately the distance from cockpit to engine           
    Lec     = NENG * total_length / Units.ft
    WEC     = 5 * NENG + 0.8 * Lec
    WSTART  = 49.19*((WENG/Units.lbs)/1000)**0.541
    return WEC * Units.lbs, WSTART * Units.lbs
 
def compute_fuel_system_weight(vehicle, NENG):
    """ Calculates the weight of the fuel system based on the Raymer method
        Assumptions:

        Source:
            Aircraft Design: A Conceptual Approach

        Inputs:
            vehicle - data dictionary with vehicle properties                   [dimensionless]
                -.design_mach_number: design mach number
                -.mass_properties.max_zero_fuel: maximum zero fuel weight   [kg]

        Outputs:
            WFSYS: Fuel system weight                                       [kg]

        Properties Used:
            N/A
    """
    Nt = 0
    Vt = 0
    for network in vehicle.networks:
        for fuel_line in network.fuel_lines:
            for fuel_tank in fuel_line.fuel_tanks:
                Nt +=1
                Vt += fuel_tank.volume / Units["gallon"]
    WFSYS = 2.405 * Vt**0.606 * 0.5 * Nt**0.5 
    return WFSYS * Units.lbs
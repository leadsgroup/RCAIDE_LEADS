# RCAIDE/Library/Methods/Weights/Correlation_Buildups/FLOPS/compute_operating_items_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE
import  RCAIDE 
from RCAIDE.Framework.Core    import Units ,  Data

# python imports 
import  numpy as  np
 
# ----------------------------------------------------------------------------------------------------------------------
#  Operating Items Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_operating_items_weight(vehicle):
    """ Calculate the weight of operating items, including:
        - crew
        - baggage
        - unusable fuel
        - engine oil
        - passenger service
        - ammunition and non-fixed weapons
        - cargo containers

        Assumptions:
            If no tanks are specified, 5 fuel tanks are assumed (includes main and auxiliary tanks)
            If the number of coach seats is not defined, then it assumed that 5% of
                of the seats are first class and an additional 10 % are business class.
            If the number of coach seats is defined, then the additional seats are 1/4 first class
                and 3/4 business class

        Source:
            The Flight Optimization System Weight Estimation Method

        Inputs:
            vehicle - data dictionary with vehicle properties                   [dimensionless]
                -.networks: data dictionary containing all propulsion properties
                    -.number_of_engines: number of engines
                    -.sealevel_static_thrust: thrust at sea level               [N]
                -.reference_area: wing surface area                             [m^2]
                -.mass_properties.max_takeoff: MTOW                             [kilograms]
                -.passengers: number of passengers in aircraft
                -.design_mach_number: design mach number for cruise flight
                -.design_range: design range of aircraft                        [nmi]
                -.mass_properties.cargo: weight of cargo carried                [kilograms]

        Outputs:
            output - data dictionary with weights                               [kilograms]
                    - output.oper_items: unusable fuel, engine oil, passenger service weight and cargo containers
                    - output.flight_crew: flight crew weight
                    - output.flight_attendants: flight attendants weight
                    - output.total: total operating items weight

        Properties Used:
            N/A
    """ 
    NENG =  0 
    for network in  vehicle.networks:
        for propulsor in network.propulsors:
            if isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan) or  isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet):
                ref_propulsor = propulsor  
                NENG  += 1   
    
    THRUST          = ref_propulsor.sealevel_static_thrust * 1 / Units.lbf
    SW              = vehicle.reference_area / Units.ft ** 2
    FMXTOT          = vehicle.mass_properties.max_zero_fuel / Units.lbs
    DESRNG          = vehicle.flight_envelope.design_range / Units.nmi
    VMAX            = vehicle.flight_envelope.design_mach_number   
    
    number_of_tanks = 0  
    for network in  vehicle.networks:
        for fuel_line in network.fuel_lines:
            for fuel_tank in fuel_line.fuel_tanks:
                number_of_tanks += 1 
    if number_of_tanks == 0:
        number_of_tanks = 5    
    
    WUF             = 11.5 * NENG * THRUST ** 0.2 + 0.07 * SW + 1.6 * number_of_tanks * FMXTOT ** 0.28  # unusable fuel weight
    WOIL            = 0.082 * NENG * THRUST ** 0.65  # engine oil weight
    
    for fuselage in  vehicle.fuselages: 
        if hasattr(fuselage, 'number_coach_seats'):
            NPT = fuselage.number_coach_seats  # number of economy passengers
            NPF = (vehicle.passengers - NPT) / 4.  # number of first clss passengers
            NPB = vehicle.passengers - NPF - NPT  # number of bussines passengers
        else:
            NPF = vehicle.passengers / 20.
            NPB = vehicle.passengers / 10.
            NPT = vehicle.passengers - NPF - NPB
    vehicle.NPF = NPF
    vehicle.NPB = NPB
    vehicle.NPT = NPT
    WSRV        = (5.164 * NPF + 3.846 * NPB + 2.529 * NPT) * (DESRNG / VMAX) ** 0.255  # passenger service weight
    WCON        = 175 * np.ceil(vehicle.mass_properties.cargo / Units.lbs * 1. / 950)  # cargo container weight

    if vehicle.passengers >= 150:
        NFLCR = 3  # number of flight crew
        NGALC = 1 + np.floor(vehicle.passengers / 250.)  # number of galley crew
    else:
        NFLCR = 2
        NGALC = 0
    if vehicle.passengers < 51:
        NFLA = 1  # number of flight attendants, NSTU in FLOPS
    else:
        NFLA = 1 + np.floor(vehicle.passengers / 40.)

    WFLAAB = NFLA * 155 + NGALC * 200  # flight attendant weight, WSTUAB in FLOPS
    WFLCRB = NFLCR * 225  # flight crew and baggage weight

    output                           = Data()
    output.misc = WUF * Units.lbs + WOIL * Units.lbs + WSRV * Units.lbs + WCON * Units.lbs
    output.flight_crew               = WFLCRB * Units.lbs
    output.flight_attendants         = WFLAAB * Units.lbs
    output.total                     = output.misc + output.flight_crew + \
                                       output.flight_attendants
    return output
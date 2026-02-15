# RCAIDE/Library/Methods/Mass_Properties/Weight_Buildups/Electric/Transport/Semi_Empirical/compute_operating_items_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE 
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

        Outputs:
            output - data dictionary with weights                               [kilograms]
                    - output.oper_items: unusable fuel, engine oil, passenger service weight and cargo containers
                    - output.flight_crew: flight crew weight
                    - output.flight_attendants: flight attendants weight
                    - output.total: total operating items weight

        Properties Used:
            N/A
    """  
    NPF  = vehicle.number_of_first_class_seats      
    NPB  = vehicle.number_of_business_class_seats   
    NPE  = vehicle.number_of_economy_class_seats   

    DESRNG          = vehicle.flight_envelope.design_range / Units.nmi
    VMAX            = vehicle.flight_envelope.design_mach_number   
            
    WSRV        = (5.164 * NPF + 3.846 * NPB + 2.529 * NPE) * (DESRNG / VMAX) ** 0.225  # passenger service weight
 

    W_cargo = 0
    WCON    = 0
    for cargo_bay in vehicle.cargo_bays:
        W_cargo     =  int(cargo_bay.mass_properties.mass)/ len(vehicle.cargo_bays)
        W_container = 175 * np.ceil(W_cargo/ Units.lbs * 1. / 950)  
        WCON        += W_container 

    if vehicle.number_of_passengers >= 150:
        NFLCR = 3  # number of flight crew
        NGALC = 1 + np.floor(vehicle.number_of_passengers / 250.)  # number of galley crew
    else:
        NFLCR = 2
        NGALC = 0
    if vehicle.number_of_passengers < 51:
        NFLA = 1  # number of flight attendants, NSTU in FLOPS
    else:
        NFLA = 1 + np.floor(vehicle.number_of_passengers / 40.)

    WFLAAB = NFLA * 155 + NGALC * 200  # flight attendant weight, WSTUAB in FLOPS
    WFLCRB = NFLCR * 225  # flight crew and baggage weight

    # Passenger Service Weight
    WSRV = (5.164*NPF + 3.846*NPB + 2.529*NPE)*(DESRNG/VMAX)**0.225 

    output                           = Data()
    output.container                 = WCON * Units.lbs
    output.flight_crew               = WFLCRB * Units.lbs
    output.flight_attendants         = WFLAAB * Units.lbs
    output.passenger_service         = WSRV   * Units.lbs
    output.total                     = output.container + output.flight_crew + output.flight_attendants + output.passenger_service 
    return output
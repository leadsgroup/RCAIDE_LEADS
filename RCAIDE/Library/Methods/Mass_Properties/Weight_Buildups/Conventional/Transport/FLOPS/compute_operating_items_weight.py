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
    """
    Computes the weight of operating items using NASA FLOPS weight estimation method. 
    Includes crew, baggage, unusable fuel, engine oil, passenger service items, weapons, and cargo containers.

    Parameters
    ----------
    vehicle : Vehicle
        The vehicle instance containing:
            - networks : list
                Propulsion systems with:
                    - propulsors : list
                        Engine data for thrust and count
                    - fuel_lines : list
                        Fuel tank data
            - reference_area : float
                Wing reference area [m²]
            - mass_properties.max_zero_fuel : float
                Maximum zero fuel weight [kg]
            - flight_envelope.design_range : float
                Design range [nmi]
            - flight_envelope.design_mach_number : float
                Design cruise Mach number
            - mass_properties.cargo : float
                Cargo weight [kg]
            - passengers : int
                Total passenger count
            - fuselages : list
                Fuselage data with optional:
                    - number_coach_seats : int
                        Number of economy seats

    Returns
    -------
    output : Data
        Container with weight breakdown:
            - misc : float
                Weight of unusable fuel, engine oil, passenger service, cargo containers [kg]
            - flight_crew : float
                Flight crew and baggage weight [kg]
            - flight_attendants : float
                Flight attendant and galley crew weight [kg]
            - total : float
                Total operating items weight [kg]

    Notes
    -----
    Uses FLOPS correlations developed from commercial transport aircraft data. For more details, 
    please refer to the FLOPS documentation: https://ntrs.nasa.gov/citations/20170005851  

    **Major Assumptions**
        * If no tanks specified, assumes 5 fuel tanks
        * Default seat class distribution if not specified:
            - 5% first class
            - 10% business class
            - 85% economy class
        * If coach seats specified:
            - Remaining seats split 1/4 first class, 3/4 business
        * Crew requirements based on passenger count:
            - 2-3 flight crew (>150 pax = 3)
            - 1 attendant per 40 pax (min 1)
            - Additional galley crew for >250 pax

    **Theory**
    Component weights computed using empirical correlations:
    
    Unusable fuel weight:
    .. math::
        W_{UF} = 11.5N_{eng}T_{SLS}^{0.2} + 0.07S_{ref} + 1.6N_{tank}W_{ZF}^{0.28}

    Engine oil weight:
    .. math::
        W_{oil} = 0.082N_{eng}T_{SLS}^{0.65}

    Passenger service weight:
    .. math::
        W_{srv} = (5.164N_{1st} + 3.846N_{bus} + 2.529N_{eco})(R/M)^{0.255}
    
    Cargo container weight:
    .. math::
        W_{con} = 175 \\lceil \\frac{W_{cargo}}{950} \\rceil

    where:
        * N_eng = number of engines
        * T_SLS = sea level static thrust
        * S_ref = wing reference area
        * N_tank = number of fuel tanks
        * W_ZF = zero fuel weight
        * N_1st/bus/eco = number of seats by class
        * R = design range
        * M = design Mach number
        * W_{cargo} = cargo weight
    References
    ----------
    [1] NASA Flight Optimization System (FLOPS)
    """ 
    NENG =  0 
    for network in  vehicle.networks:
        for propulsor in network.propulsors: 
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
            for _ in fuel_line.fuel_tanks:
                number_of_tanks += 1 
    if number_of_tanks == 0:
        number_of_tanks = 5    
    
    WUF             = 11.5 * NENG * THRUST ** 0.2 + 0.07 * SW + 1.6 * number_of_tanks * FMXTOT ** 0.28  # unusable fuel weight
    WOIL            = 0.082 * NENG * THRUST ** 0.65  # engine oil weight
    
    for fuselage in  vehicle.fuselages: 
        if len(fuselage.cabins) > 0:
            NPT =  0
            NPF =  0
            NPB =  0
            for cabin in fuselage.cabins:
                for cabin_class in cabin.classes:
                    if type(cabin_class) == RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy:
                        NPT =  cabin_class.number_of_seats_abrest *  cabin_class.number_of_rows
                    elif type(cabin_class) == RCAIDE.Library.Components.Fuselages.Cabins.Classes.Business:
                        NPB =  cabin_class.number_of_seats_abrest *  cabin_class.number_of_rows
                    elif type(cabin_class) == RCAIDE.Library.Components.Fuselages.Cabins.Classes.First:
                        NPF =  cabin_class.number_of_seats_abrest *  cabin_class.number_of_rows
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
    output.passenger_service         = WSRV   * Units.lbs
    output.total                     = output.misc + output.flight_crew +  output.flight_attendants + output.passenger_service 
    return output
# RCAIDE/Library/Methods/Mass_Properties/Weight_Buildups/Transport/Common/compute_operating_items_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke
# Modifed:  Feb 2025, A. Molloy, S. Shekar

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

        Source:
            http://aerodesign.stanford.edu/aircraftdesign/AircraftDesign.html

        Inputs:
            vehicle - data dictionary with vehicle properties                   [dimensionless]
                -.passengers: number of passengers
                -.systems.accessories: type of aircraft (short-range, commuter
                                                        medium-range, long-range,
                                                        sst, cargo)

        Outputs:
            output - data dictionary with weights                               [kilograms]
                    - output.oper_items: unusable fuel, engine oil, passenger service weight and cargo containers
                    - output.flight_crew: flight crew weight
                    - output.flight_attendants: flight attendants weight
                    - output.total: total operating items weight

        Properties Used:
            N/A
    """
    num_seats   = vehicle.passengers
    ac_type     = vehicle.systems.accessories
    if ac_type   == "short-range":  # short-range domestic, austere accomodations
        operitems_wt = 17.0 * num_seats * Units.lb
    elif ac_type == "medium-range":  # medium-range domestic
        operitems_wt = 28.0 * num_seats * Units.lb
    elif ac_type == "long-range":  # long-range overwater
        operitems_wt = 28.0 * num_seats * Units.lb
    elif ac_type == "business":  # business jet
        operitems_wt = 28.0 * num_seats * Units.lb
    elif ac_type == "cargo":  # all cargo
        operitems_wt = 56.0 * Units.lb
    elif ac_type == "commuter":  # commuter
        operitems_wt = 17.0 * num_seats * Units.lb
    elif ac_type == "sst":  # sst
        operitems_wt = 40.0 * num_seats * Units.lb
    else:
        operitems_wt = 28.0 * num_seats * Units.lb

    if vehicle.passengers >= 150:
        flight_crew = 3  # FLOPS: NFLCR
    else:
        flight_crew = 2

    if vehicle.passengers < 51:
        flight_attendants = 1  # FLOPS: NSTU
    else:
        flight_attendants = 1 + np.floor(vehicle.passengers / 40.)

    W_flight_attendants = flight_attendants * (170 + 40)  # FLOPS: WSTUAB
    W_flight_crew = flight_crew * (190 + 50)  # FLOPS: WFLCRB

    output                           = Data()
    output.misc = operitems_wt
    output.flight_crew               = W_flight_crew * Units.lbs
    output.flight_attendants         = W_flight_attendants * Units.lbs
    output.total                     = output.misc + output.flight_crew + \
                                       output.flight_attendants
    return output
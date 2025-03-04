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
    Calculate the weight of aircraft operating items using FLOPS methodology for a general aviation aircraft.

    Parameters
    ----------
    vehicle : RCAIDE.Vehicle()
        Vehicle data structure (not directly used in current implementation)

    Returns
    -------
    output : Data()
        Data structure containing:
            - misc : float
                Weight of miscellaneous items [kg], currently set to 0
            - flight_crew : float
                Weight of flight crew and their baggage [kg]
            - flight_attendants : float
                Weight of flight attendants [kg], currently set to 0
            - total : float
                Total operating items weight [kg]

    Notes
    -----
    Calculates weights for crew and operating items for a general aviation aircraft.
    Currently implements a simplified version focused on single-pilot operations.
    
    **Major Assumptions**
        * Single pilot operation (1 flight crew member)
        * Standard flight crew weight of 225 lb including baggage
        * No flight attendants
        * No additional passenger service items
    
    **Theory**

    For general aviation aircraft, the operating items weight is primarily the flight crew weight:

    .. math::
        W_{oi} = N_{fc} * W_{fc}

    Where:
        - W_{oi} is total operating items weight [lb]
        - N_{fc} is number of flight crew (1)
        - W_{fc} is flight crew weight with baggage (225 lb)

    References
    ----------
    [1] NASA. (1979). The Flight Optimization System Weights Estimation Method. 
        NASA Technical Report.
    """
    NFLCR = 1 # Number of flight crew. Assumed to be 1
    WFLCRB = NFLCR * 225  # flight crew and baggage weight

    output                           = Data()
    output.misc = 0
    output.flight_crew               = WFLCRB * Units.lbs
    output.flight_attendants         = 0.0
    output.total                     = output.misc + output.flight_crew + \
                                       output.flight_attendants
    return output



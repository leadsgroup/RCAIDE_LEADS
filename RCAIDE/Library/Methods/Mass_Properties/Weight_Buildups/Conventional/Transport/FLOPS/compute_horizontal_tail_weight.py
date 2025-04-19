# RCAIDE/Library/Methods/Weights/Correlation_Buildups/FLOPS/compute_horizontal_tail_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE 
from RCAIDE.Framework.Core    import Units
 
# ----------------------------------------------------------------------------------------------------------------------
#  Horizontal Tail Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_horizontal_tail_weight(vehicle, wing): 
    """ Calculate the horizontal tail weight

        Assumptions:
           Conventional tail configuration

        Source:
            The Flight Optimization System Weight Estimation Method

       Inputs:
            vehicle - data dictionary with vehicle properties                   [dimensionless]
                -.mass_properties.max_takeoff: MTOW                             [kilograms]
            wing - data dictionary with vertical tail properties                [dimensionless]
                -.taper: taper of wing
                -.areas.reference: surface area of wing                         [m^2]

       Outputs:
            WHT - vertical tail weight                                          [kilograms]

        Properties Used:
            N/A
        """
    SHT     = wing.areas.reference / Units.ft **2
    DG      = vehicle.mass_properties.max_takeoff / Units.lbs
    TRHT    = wing.taper
    WHT     = 0.53 * SHT * DG ** 0.2 * (TRHT + 0.5)
    return WHT * Units.lbs

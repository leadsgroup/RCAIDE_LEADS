# RCAIDE/Library/Methods/Weights/Correlation_Buildups/Raymer/compute_vertical_tail_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE 
from RCAIDE.Framework.Core    import Units

# python imports 
import  numpy as  np
 
# ----------------------------------------------------------------------------------------------------------------------
#  Vertical Tail Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_vertical_tail_weight(vehicle, wing, settings):
    """ Calculates vertical tail weight based on Raymer method

        Assumptions:

        Source:
            Aircraft Design: A Conceptual Approach (2nd edition)

        Inputs:
            vehicle - data dictionary with vehicle properties                    [dimensionless]
                -.mass_properties.max_takeoff: MTOW                             [kilograms]
                -.flight_envelope.ultimate_load: ultimate load factor (default: 3.75)
                -.wings['main_wing']: data dictionary with properties of main wing
                    -.aerodynamic_center: aerodynamic center as measured from root leading edge
                    -.origin: root of main wing as measured from nose of aircraft
            wing    - data dictionary with specific tail properties              [dimensionless]
                -.areas.reference: tail surface area                            [m^2}
                -.origin: location of tail measured from nose
                -.aerodynamic_center: location of ac measured from leading edge
                -.sweeps.quarter_chord: quarter chord sweep of tail             [rad]
                -.thickness_to_chord: t/c of tail

        Outputs:
              tail_weight: vertical tail weight                                [kilograms]

        Properties Used:
            N/A
    """
    DG          = vehicle.mass_properties.max_takeoff / Units.lbs
    t_tail_flag = wing.t_tail
    wing_origin = wing.origin[0][0] / Units.ft
    wing_ac     = wing.aerodynamic_center[0] / Units.ft
    main_origin = vehicle.wings['main_wing'].origin[0][0] / Units.ft
    main_ac     = vehicle.wings['main_wing'].aerodynamic_center[0] / Units.ft
    Svt         = wing.areas.reference / Units.ft ** 2
    sweep       = wing.sweeps.quarter_chord
    Av          = wing.aspect_ratio
    t_c         = wing.thickness_to_chord 
    Nult        = vehicle.flight_envelope.ultimate_load
    
    H = 0
    if t_tail_flag:
        H = 1
    Lt = (wing_origin + wing_ac - main_origin - main_ac)
    Kz = Lt
    tail_weight = 0.0026 * (1 + H) ** 0.225 * DG ** 0.556 * Nult ** 0.536 \
                  * Lt ** (-0.5) * Svt ** 0.5 * Kz ** 0.875 * np.cos(sweep) ** (-1) * Av ** 0.35 * t_c ** (-0.5)
    if settings.advanced_composites:
        tail_weight *= 0.85
    
    return tail_weight * Units.lbs

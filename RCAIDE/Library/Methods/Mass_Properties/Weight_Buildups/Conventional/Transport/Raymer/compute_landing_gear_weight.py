# RCAIDE/Library/Methods/Weights/Correlation_Buildups/Raymer/compute_landing_gear_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE
import RCAIDE
from RCAIDE.Framework.Core    import Units ,  Data 
 
# ----------------------------------------------------------------------------------------------------------------------
#  Horizontal Tail Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_landing_gear_weight(vehicle):
    """ Calculate the weight of the landing gear of a transport aircraft based on the Raymer method

        Assumptions:
            No fuselage mounted landing gear
            1 cargo door
            gear load factor = 3
            number of main gear shock struts = 2
            stall speed is estimated with a Cl max of 2.5 and at sea level
            Not a reciprocating engine

        Source:
            Aircraft Design: A Conceptual Approach (2nd edition)

        Inputs:
            vehicle - data dictionary with vehicle properties                   [dimensionless]
                -.design_range: design range of aircraft                        [m]
                -.mass_properties.max_takeoff: MTOW                             [kg]
                -.systems.accessories: type of aircraft (short-range, commuter
                                                        medium-range, long-range,
                                                        sst, cargo)
                -.landing_gear.main_strut_length: main strut length              [m]
                -.landing_gear.main_wheels: number of wheels on main landing gear
                -.landing_gear.nose_strut_length: nose strut length              [m]
                -.landing_gear.nose_wheels: number of wheels on nose landing gear

            fuse - data dictionary with specific fuselage properties            [dimensionless]

        Outputs:
            weight_fuse - weight of the fuselage                                [kilograms]

        Properties Used:
            N/A
    """
    Kmp         = 1  # assuming not a kneeling gear
    if vehicle.systems.accessories == "sst":
        RFACT   = 0.00009
    else:
        RFACT   = 0.00004
    DESRNG      = vehicle.flight_envelope.design_range / Units.nmi  # Design range in nautical miles
    WLDG        = vehicle.mass_properties.max_takeoff / Units.lbs * (1 - RFACT * DESRNG)
    Ngear       = 3  # gear load factor, usually around 3
    Nl          = Ngear * 1.5  # ultimate landing load factor
      
    Nmw  = 0
    Lm   = 0 
    Ln   = 0
    Nnw  = 0
    Nmss = 2  # number of main gear shock struts assumed to be 2
    for landing_gear in  vehicle.landing_gears:
        if isinstance(landing_gear,RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear):
            Nmw  = landing_gear.wheels * Nmss    
            Lm   = landing_gear.strut_length / Units.inch 
        elif isinstance(landing_gear,RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear): 
            Ln          = landing_gear.strut_length / Units.inch
            Nnw         = landing_gear.wheels  
        
    Vstall      = (2 * vehicle.mass_properties.max_takeoff * 9.81 / (vehicle.wings.main_wing.areas.reference * 1.225 * 2.5)) ** 0.5 # Assumes max Cl of 2.5, density of 1.225 kg/m^3
    Knp         = 1  # assuming not a kneeling gear
    W_main_landing_gear = 0.0106 * Kmp * WLDG ** 0.888 * Nl ** 0.25 * Lm ** 0.4 * Nmw ** 0.321 * Nmss ** (-0.5) * Vstall ** 0.1
    W_nose_landing_gear = 0.032 * Knp * WLDG ** 0.646 * Nl ** 0.2 * Ln ** 0.5 * Nnw ** 0.45

    output          = Data()
    output.main     = W_main_landing_gear * Units.lbs
    output.nose     = W_nose_landing_gear * Units.lbs
    return output
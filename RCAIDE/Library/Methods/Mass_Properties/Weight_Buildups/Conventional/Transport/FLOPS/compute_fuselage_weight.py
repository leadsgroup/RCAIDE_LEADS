# RCAIDE/Library/Methods/Weights/Correlation_Buildups/FLOPS/compute_fuselage_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE
import RCAIDE 
from RCAIDE.Framework.Core    import Units

# python imports 
import  numpy as  np
 
# ----------------------------------------------------------------------------------------------------------------------
# Fuselage Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_fuselage_weight(vehicle):
    """ Calculate the weight of the fuselage of a transport aircraft

        Assumptions:
            NFUSE = 1, only one fuselage (it's possible to modify this in future work)
            delta_isa = 0, for pressure calculations
            Fuselage is tagged as 'fuselage'

        Source:
            The Flight Optimization System Weight Estimation Method

        Inputs:
            vehicle - data dictionary with vehicle properties                    [dimensionless]
                -.networks: data dictionary containing all propulsion properties
                -.fuselages['fuselage'].lengths.total: fuselage total length      [meters]
                -.fuselages['fuselage'].width: fuselage width                    [meters]
                -.fuselages['fuselage'].heights.maximum: fuselage maximum height [meters]
                -.flight_envelope.ultimate_load: ultimate load factor (default: 3.75)
                -.systems.accessories: type of aircraft (short-range, commuter
                                                        medium-range, long-range,
                                                        sst, cargo)
                -.mass_properties.max_takeoff: MTOW                              [kilograms]
                -.design_mach_number: design mach number for cruise flight

        Outputs:
            WFUSE - weight of the fuselage                                      [kilograms]

        Properties Used:
            N/A
    """
    
    L =  0
    for fuselage in vehicle.fuselages:
        if L < fuselage.lengths.total: 
            total_length = fuselage.lengths.total
            width        = fuselage.width
            max_height   = fuselage.heights.maximum
    
    XL  = total_length / Units.ft  # Fuselage length, ft
    DAV = (width + max_height) / 2. * 1 / Units.ft
    NENG = 0
    FNEW = 0
    FNEF = 0 
    for network in  vehicle.networks:
        for propulsor in network.propulsors:
            if isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan) or  isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet):
                NENG += 1 
                if propulsor.wing_mounted: 
                    FNEW += 1  
                else:
                    FNEF += 1
    if vehicle.systems.accessories == 'freighter':
        CARGF = 1
    else:
        CARGF = 0  # Cargo aircraft floor factor [0 for passenger transport, 1 for cargo transport
    NFUSE = 1  # Number of fuselages
    WFUSE = 1.35 * (XL * DAV) ** 1.28 * (1 + 0.05 * FNEF) * (1 + 0.38 * CARGF) * NFUSE
    
    return WFUSE * Units.lbs
# RCAIDE/Library/Methods/Weights/Correlation_Buildups/FLOPS/compute_landing_gear_weight.py
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
#  Landing Gear Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_landing_gear_weight(vehicle):
    """ Calculate the weight of the main and nose landing gear of a transport aircraft

        Assumptions:
            No fighter jet, change DFTE to 1 for a fighter jet
            Aircraft is not meant for carrier operations, change CARBAS to 1 for carrier-based aircraft

        Source:
            The Flight Optimization System Weight Estimation Method

        Inputs:
            vehicle - data dictionary with vehicle properties                   [dimensionless]
                -.networks: data dictionary containing all propulsion properties
                -.design_range: design range of aircraft                        [nmi]
                -.systems.accessories: type of aircraft (short-range, commuter
                                                        medium-range, long-range,
                                                        sst, cargo)
                -.mass_properties.max_takeoff: MTOW                              [kilograms]
                -.nacelles['nacelle']                                            [meters]
                -.wings['main_wing'].dihedral                                    [radians]
                -.fuselages['fuselage'].width: fuselage width                    [meters]
                -.fuselages['fuselage'].lengths.total: fuselage total length     [meters]


        Outputs:
            output - data dictionary with main and nose landing gear weights    [kilograms]
                    output.main, output.nose

        Properties Used:
            N/A
    """
    DFTE    = 0
    CARBAS  = 0
    if vehicle.systems.accessories == "sst":
        RFACT = 0.00009
    else:
        RFACT = 0.00004
    DESRNG  = vehicle.flight_envelope.design_range / Units.nmi  # Design range in nautical miles
    WLDG    = vehicle.mass_properties.max_takeoff / Units.lbs * (1 - RFACT * DESRNG)
    
    for wing in vehicle.wings:
        if isinstance(wing,RCAIDE.Library.Components.Wings.Main_Wing):
            main_wing = wing
    
    l_f =  0
    for fuselage in vehicle.fuselages:
        if l_f < fuselage.lengths.total:
            main_fuselage = fuselage 
            l_f = main_fuselage.lengths.total
        
    for network in vehicle.networks:
        for propulsor in  network.propulsors:
            if propulsor.wing_mounted:
                nacelle =  propulsor.nacelle 
                FNAC    = nacelle.diameter / Units.ft
                DIH     = main_wing.dihedral
                YEE     = np.max(np.abs(np.array(network.origin)[:, 1])) / Units.inches
                WF      = main_fuselage.width / Units.ft
                XMLG    = 12 * FNAC + (0.26 - np.tan(DIH)) * (YEE - 6 * WF)  # length of extended main landing gear
            else:
                XMLG    = 0.75 * main_fuselage.lengths.total / Units.ft  # length of extended nose landing gear
    XNLG = 0.7 * XMLG
    WLGM = (0.0117 - 0.0012 * DFTE) * WLDG ** 0.95 * XMLG ** 0.43
    WLGN = (0.048 - 0.0080 * DFTE) * WLDG ** 0.67 * XNLG ** 0.43 * (1 + 0.8 * CARBAS)

    output      = Data()
    output.main = WLGM * Units.lbs
    output.nose = WLGN * Units.lbs
    return output
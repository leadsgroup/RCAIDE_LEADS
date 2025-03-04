# RCAIDE/Library/Methods/Weights/Correlation_Buildups/FLOPS/compute_landing_gear_weight.py
# 
# Created:  Sep 2024, M. Clarke
# Modified: Feb 2025, A. Molloy and S. Shekar

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
    """
    Calculates the weight of a general aviation aircraft's landing gear system using FLOPS methodology.

    Parameters
    ----------
    vehicle : RCAIDE.Vehicle()
        Vehicle data structure containing:
            - mass_properties.max_takeoff
            - landing_gear.nose_wheels
            - landing_gear.main_wheels
            - landing_gear.configuration_type
            - flight_envelope.design_range
            - wings.main_wing.dihedral
            - fuselages.fuselage.width
            - fuselages.fuselage.lengths.total

    Returns
    -------
    output : Data()
        Data structure containing:
            - output.main: weight of the main landing gear in kilograms
            - output.nose: weight of the nose landing gear in kilograms

    Notes
    -----
    The function uses FLOPS (Flight Optimization System) statistical regression models 
    based on historical aircraft data to estimate landing gear weight. The calculation 
    includes the weight of struts, wheels, tires, brakes, and actuation systems. This method 
    is common between general aviation and transport aircraft in FLOPS. Of note, FLOPS does
    not use user defined landing gear lengths and tire sizes.
    
    **Major Assumptions**
        * Landing gear is not for a fighter jet, carrier based aircraft, or supersonic transport
        * Landing gear length is based on the wingspan and fuselage length
        * Brake system, retraction mechanism, and other components are baked into the weight
    
    **Theory**

    The FLOPS weight estimation follows the relationship:

    .. math::
        W_{LGM} = (0.0117) * (W_{LDG})^{0.95} * (X_{MLG})^{0.43}
        W_{LGN} = (0.048) * (W_{LDG})^{0.67} * (X_{NLG})^{0.43}
    Where:
        - W_{LGM} is landing gear weight for the main landing gear
        - W_{LGN} is landing gear weight for the nose landing gear
        - W_{LDG} is landing weight for the entire aircraft
        - X_{MLG} is the length of the main landing gear
        - X_{NLG} is the length of the nose landing gear
    
    Reference:
    """
    RFACT = 0.00004 # Not a supersonic transport aircraft
    
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
                FNAC = 0
                if isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan) or  isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet):
                    nacelle =  propulsor.nacelle 
                    FNAC    = nacelle.diameter / Units.ft
                if isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.ICE_Propeller) or  isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Constant_Speed_ICE_Propeller): 
                    FNAC    = propulsor.propeller.tip_radius * 2 / Units.ft      
                DIH     = main_wing.dihedral
                YEE     = np.max(np.abs(np.array(network.origin)[:, 1])) / Units.inches
                WF      = main_fuselage.width / Units.ft
                XMLG    = 12 * FNAC + (0.26 - np.tan(DIH)) * (YEE - 6 * WF)  # length of extended main landing gear
            else:
                XMLG    = 0.75 * main_fuselage.lengths.total / Units.ft  # length of extended nose landing gear
    XNLG = 0.7 * XMLG
    WLGM = (0.0117) * WLDG ** 0.95 * XMLG ** 0.43
    WLGN = (0.048) * WLDG ** 0.67 * XNLG ** 0.43

    output      = Data()
    output.main = WLGM * Units.lbs
    output.nose = WLGN * Units.lbs
    return output
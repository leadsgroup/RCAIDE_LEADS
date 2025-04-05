# RCAIDE/Library/Methods/Weights/Correlation_Buildups/FLOPS/compute_systems_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE
import  RCAIDE 
from RCAIDE.Framework.Core    import Units, Data 

# python imports 
import  numpy as  np
 
# ----------------------------------------------------------------------------------------------------------------------
# Systems Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_systems_weight(vehicle):
    """
    Calculate the systems weight for general aviation aircraft using FLOPS methodology.

    Parameters
    ----------
    vehicle : RCAIDE.Vehicle()
        Vehicle data structure containing:
            - networks : list
                List of propulsion networks
            - wings['main_wing'] : Component
                Main wing data structure containing:
                    - sweeps.quarter_chord : float
                        Quarter chord sweep angle [deg]
                    - areas.reference : float
                        Wing reference area [m^2]
                    - spans.projected : float
                        Wing projected span [m]
                    - flap_ratio : float
                        Flap area to wing area ratio
            - fuselages : list
                List of fuselage components containing:
                    - lengths.total : float
                        Fuselage length [m]
                    - width : float
                        Fuselage width [m]
                    - heights.maximum : float
                        Maximum fuselage height [m]
            - flight_envelope : Component
                Flight envelope data containing:
                    - design_mach_number : float
                        Design cruise Mach number
                    - ultimate_load : float
                        Ultimate load factor
                    - design_range : float
                        Design range [nmi]
            - passengers : int
                Number of passengers
            - design_dynamic_pressure : float
                Design dynamic pressure [Pa]
            - mass_properties.max_takeoff : float
                Maximum takeoff weight [kg]

    Returns
    -------
    output : Data()
        Data structure containing:
            - W_flight_control : float
                Flight control system weight [kg]
            - W_hyd_pnu : float
                Hydraulics and pneumatics weight [kg]
            - W_instruments : float
                Instruments weight [kg]
            - W_avionics : float
                Avionics weight [kg]
            - W_apu : float
                APU weight [kg], currently set to 0
            - W_anti_ice : float
                Anti-ice system weight [kg], currently set to 0
            - W_electrical : float
                Electrical system weight [kg]
            - W_ac : float
                Air conditioning weight [kg], currently set to 0
            - W_furnish : float
                Furnishing weight [kg]
            - W_systems : float
                Total systems weight [kg]

    Notes
    -----
    Calculates weights for all aircraft systems using FLOPS methodology.
    
    **Major Assumptions**
        * Hydraulic system pressure is 3000 psf
        * Single fuselage configuration
        * Pressure ratio for cabin pressure (cruise to sea level) is 0.85. i.e. almostno cabin pressurization
        * Passenger cabin length is 25% of fuselage length
    
    **Theory**

    Flight Controls:
    .. math::
        W_{fc} = 0.404 * S_w^{0.317} * (W_{TO}/1000)^{0.602} * N_{ult}^{0.525} * q^{0.345}

    Instruments:
    .. math::
        W_{in} = 0.48 * A_f^{0.57} * M^{0.5} * (10 + 2.5N_{c} + N_{ew} + 1.5N_{ef})

    Hydraulic and Pneumatics:
    .. math::
        W_{hyd} = 0.57 * (A_f + 0.27 * S_w) * (1 + 0.03N_{ew} + 0.05N_{ef}) * (3000 / P)^{0.35} * M^{0.33}

    Electrical:
    .. math::
        W_{elec} = 92 * L_f^{0.4} * W_f^{0.14} * N_{fuse}^{0.27} * N_{eng}^{0.69} * (1 + 0.044N_{c} + 0.0015N_{pax})

    Avionics:
    .. math::
        W_{av} = 15.8 * D_{range}^{0.1} * N_{fc}^{0.7} * A_f^{0.43}

    Furnishing:
    .. math::
        W_{furn} = 127 * N_{fc} + 44 * N_{pax} + 2.6 * L_f * (W_f + D) * N_{fuse}
    
    Air Conditioning:
    .. math::
        W_{ac} = 3.2 * (A_f * D)^{0.6} + 9 * N_{pax}^{0.83} * M + 0.075 * W_{av}

    Where:
        - W_{fc} is flight controls weight [lb]
        - S_w is wing area [ft^2]
        - W_{TO} is takeoff weight [lb]
        - N_{ult} is ultimate load factor
        - q is dynamic pressure [psf]
        - A_f is fuselage planform area [ft^2]
        - M is design Mach number
        - N_{c} is number of crew
        - N_{ew} is number of wing-mounted engines
        - N_{ef} is number of fuselage-mounted engines
        - P is hydraulic system pressure [psf]
        - L_f is fuselage length [ft]
        - W_f is fuselage width [ft]
        - N_{fuse} is number of fuselages
        - N_{eng} is total number of engines
        - N_{pax} is number of passengers
        - D_{range} is design range [nmi]
        - N_{fc} is number of flight crew
        - D is fuselage depth/height [ft]

    References
    ----------
    [1] NASA. (1979). The Flight Optimization System Weights Estimation Method. 
        NASA Technical Report.
    """ 
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
                if 'nacelle' in propulsor:
                    nacelle =  propulsor.nacelle 
                    FNAC    = nacelle.diameter / Units.ft
                else:
                    FNAC    = 0                     
            
    VMAX     = vehicle.flight_envelope.design_mach_number
    SFLAP    = 0
    ref_wing = None 
    for wing in  vehicle.wings:
        if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing):
            SFLAP  += wing.areas.reference * wing.flap_ratio / Units.ft ** 2
            ref_wing  =  wing
    
    SW = 0
    if ref_wing == None:
        for wing in  vehicle.wings:
            if SW < wing.areas.reference / Units.ft ** 2:
                ref_wing = wing
                SW = ref_wing.areas.reference / Units.ft ** 2
    DELTA  = 0.85 # Pressure Ratio (cruise to sea level) is assumed to be 0.85
    ULF    = vehicle.flight_envelope.ultimate_load   
    QDIVE  = 1481.35*DELTA*VMAX**2
    DG    = vehicle.mass_properties.max_takeoff / Units.lbs
    WSC   = 0.404*SW**0.317 * (DG/1000)**0.602 * ULF**0.525 * QDIVE**0.345
     
    XL = 0
    WF = 0
    L_fus = 0
    for fuselage in vehicle.fuselages:
        if L_fus < fuselage.lengths.total:
            ref_fuselage = fuselage 
            XL  = fuselage.lengths.total / Units.ft
            WF  = fuselage.width / Units.ft
    FPAREA      = XL * WF
    NPASS       = vehicle.passengers

    
    NFLCR = 1 
    WIN     = 0.48 * FPAREA ** 0.57 * VMAX ** 0.5 * (10 + 2.5 * NFLCR + FNEW + 1.5 * FNEF)  # instrumentation weight

    SW      = vehicle.reference_area / Units.ft ** 2
    HYDR    = 3000  # Hydraulic system pressure
    VARSWP  = 0
    WHYD    = 0.57 * (FPAREA + 0.27 * SW) * (1 + 0.03 * FNEW + 0.05 * FNEF) * (3000 / HYDR) ** 0.35 * \
            (1 + 0.04 * VARSWP) * VMAX ** 0.33  # hydraulic and pneumatic system weight

    NFUSE   = len(vehicle.fuselages)
    WELEC   = 92. * XL ** 0.4 * WF ** 0.14 * NFUSE ** 0.27 * NENG ** 0.69 * \
            (1. + 0.044 * NFLCR + 0.0015 * NPASS)  # electrical system weight

    DESRNG  = vehicle.flight_envelope.design_range / Units.nmi
    WAVONC  = 15.8 * DESRNG ** 0.1 * NFLCR ** 0.7 * FPAREA ** 0.43  # avionics weight

    XLP     = 0.25 * XL # Assumption that pax cabin is 25% of fuselage length
    DF      = ref_fuselage.heights.maximum / Units.ft # D stands for depth
    WFURN   = 127 * NFLCR +  44 * vehicle.passengers \
                + 2.6 * XLP * (WF + DF) * NFUSE  # furnishing weight

    WAC     = (3.2 * (FPAREA * DF) ** 0.6 + 9 * NPASS ** 0.83) * VMAX + 0.075 * WAVONC  # ac weight



    output                      = Data()
    output.W_flight_control    = WSC * Units.lbs
    output.W_hyd_pnu           = WHYD * Units.lbs
    output.W_instruments       = WIN * Units.lbs
    output.W_avionics          = WAVONC * Units.lbs
    output.W_apu               = 0.0
    output.W_anti_ice          = 0.0
    output.W_electrical        = WELEC * Units.lbs
    output.W_ac                = 0.0
    output.W_furnish           = WFURN * Units.lbs
    output.W_systems           = WSC  + WIN + WHYD + WELEC + WAVONC + WFURN + WAC 
    return output
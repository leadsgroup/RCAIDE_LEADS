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
    Computes aircraft systems weights using NASA FLOPS weight estimation method. 
    Includes flight controls, APU, hydraulics, instruments, avionics, electrical, 
    air conditioning, furnishings, and anti-ice systems.

    Parameters
    ----------
    vehicle : Vehicle
        The vehicle instance containing:
            - networks : list
                Propulsion systems with:
                    - propulsors : list
                        Engine data with:
                            - wing_mounted : bool
                                Engine mounting location
                            - nacelle : Nacelle
                                Nacelle geometry
            - flight_envelope : FlightEnvelope
                - design_mach_number : float
                    Design cruise Mach number
                - design_range : float
                    Design range [nmi]
            - wings['main_wing'] : Wing
                Main wing with:
                    - sweeps.quarter_chord : float
                        Quarter chord sweep [rad]
                    - areas.reference : float
                        Reference area [m²]
                    - spans.projected : float
                        Projected span [m]
                    - flap_ratio : float
                        Flap to wing area ratio
            - fuselages : list
                Aircraft fuselages with:
                    - lengths.total : float
                        Total length [m]
                    - width : float
                        Maximum width [m]
                    - heights.maximum : float
                        Maximum height [m]
            - mass_properties.max_takeoff : float
                Maximum takeoff weight [kg]
            - passengers : int
                Total passenger count

    Returns
    -------
    output : Data
        Container with systems weight breakdown:
            - W_flight_control : float
                Flight control system weight [kg]
            - W_apu : float
                APU system weight [kg]
            - W_hyd_pnu : float
                Hydraulics and pneumatics weight [kg]
            - W_instruments : float
                Instruments weight [kg]
            - W_avionics : float
                Avionics weight [kg]
            - W_electrical : float
                Electrical system weight [kg]
            - W_ac : float
                Air conditioning weight [kg]
            - W_furnish : float
                Furnishings weight [kg]
            - W_anti_ice : float
                Anti-ice system weight [kg]
            - W_systems : float
                Total systems weight [kg]

    Notes
    -----
    Uses FLOPS correlations developed from transport aircraft database. For more information, see 
    https://ntrs.nasa.gov/citations/20170005851 

    **Major Assumptions**
        * No variable sweep (VARSWP = 0)
        * Hydraulic system pressure is 3000 psf
        * Single fuselage configuration

    **Theory**
    Component weights computed using empirical correlations:
    .. math::
        W_{sc} = 1.1V_{max}^{0.52}S_{flap}^{0.6}W_{to}^{0.32}

        W_{apu} = 54A_{f}^{0.3} + 5.4N_{pax}^{0.9}

        W_{hyd} = 0.57(A_{f} + 0.27S_{w})(1 + 0.03N_{ew} + 0.05N_{ef})
                  (3000/P_{hyd})^{0.35}V_{max}^{0.33}
        
        W_{inst} = 0.48A_{f}^{0.57}V_{max}^{0.5}(10 + 2.5N_{fcr} + N_{ew} + 1.5N_{ef})

        W_{av} = 15.8D_{r}^{0.1}N_{fcr}^{0.7}A_{f}^{0.43}

        W_{elec} = 92X_{l}^{0.4}W_{f}^{0.14}N_{f}^{0.27}N_{eng}^{0.69}(1 + 0.044N_{fcr} + 0.0015N_{pax})
        
        W_{ac} = 3.2(A_{f}D_{f})^{0.6} + 9N_{pax}^{0.83}V_{max} + 0.075W_{av}

        W_{furn} = 127N_{fcr} + 112N_{pf} + 78N_{pb} + 44N_{pt} + 2.6X_{l}(W_{f} + D_{f})N_{f}

        W_{ai} = 1.5W_{f} + 3.8D_{n}N_{eng} + 1.5W_{f}

    where:
        * V_max = design Mach number
        * S_flap = flap area
        * W_to = takeoff weight
        * A_f = fuselage area
        * N_pax = passenger count
        * S_w = wing area
        * N_ew = wing-mounted engines
        * N_ef = fuselage-mounted engines
        * P_hyd = hydraulic pressure
        * N_fcr = number of flight crew
        * N_pf = number of first class passenger
        * N_pb = number of business class passenger
        * N_pt = number of economy class passenger
        * X_l = fuselage length
        * D_f = fuselage depth
        * D_n = nacelle diameter

    References
    ----------
    [1] NASA Flight Optimization System (FLOPS)
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
    
    S = 0
    if ref_wing == None:
        for wing in  vehicle.wings:
            if S < wing.areas.reference:
                ref_wing = wing
                
    DG    = vehicle.mass_properties.max_takeoff / Units.lbs
    WSC   = 1.1 * VMAX ** 0.52 * SFLAP ** 0.6 * DG ** 0.32  # surface controls weight
    
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
    WAPU        = 54 * FPAREA ** 0.3 + 5.4 * NPASS ** 0.9  # apu weight

    if vehicle.passengers >= 150:
        NFLCR = 3  # number of flight crew
    else:
        NFLCR = 2 
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

    XLP     = 0.8 * XL
    DF      = ref_fuselage.heights.maximum / Units.ft # D stands for depth
    WFURN   = 127 * NFLCR + 112 * vehicle.NPF + 78 * vehicle.NPB + 44 * vehicle.NPT \
                + 2.6 * XLP * (WF + DF) * NFUSE  # furnishing weight

    WAC     = (3.2 * (FPAREA * DF) ** 0.6 + 9 * NPASS ** 0.83) * VMAX + 0.075 * WAVONC  # ac weight

    WAI     = ref_wing.spans.projected / Units.ft * 1. / np.cos(ref_wing.sweeps.quarter_chord) + 3.8 * FNAC * NENG + 1.5 * WF  # anti-ice weight

    output                      = Data()
    output.W_flight_control    = WSC * Units.lbs
    output.W_apu               = WAPU * Units.lbs
    output.W_hyd_pnu           = WHYD * Units.lbs
    output.W_instruments       = WIN * Units.lbs
    output.W_avionics          = WAVONC * Units.lbs
    output.W_electrical        = WELEC * Units.lbs
    output.W_ac                = WAC * Units.lbs
    output.W_furnish           = WFURN * Units.lbs
    output.W_anti_ice          = WAI * Units.lbs
    output.W_systems           = WSC + WAPU + WIN + WHYD + WELEC + WAVONC + WFURN + WAC + WAI
    return output
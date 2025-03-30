# RCAIDE/Library/Methods/Weights/Correlation_Buildups/Raymer/compute_systems_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke
# Modifed:  Feb 2025, A. Molloy, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE
import RCAIDE 
from RCAIDE.Framework.Core    import Units, Data 

# python imports 
import  numpy as  np
 
# ----------------------------------------------------------------------------------------------------------------------
# Systems Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_systems_weight(vehicle):
    """
    Calculates aircraft systems weights using Raymer's empirical correlations.

    Parameters
    ----------
    vehicle : RCAIDE.Vehicle()
        Vehicle data structure containing:
            - networks : list
                List of propulsion networks
            - wings : list
                List of wing components
            - fuselages : list
                List of fuselage components
            - mass_properties.max_takeoff : float
                Maximum takeoff weight [kg]
            - flight_envelope.design_mach_number : float
                Design mach number
            - passengers : int
                Number of passengers
            - payload.cargo.mass_properties.mass : float
                Cargo weight [kg]

    Returns
    -------
    output : Data()
        Systems weight breakdown:
            - W_flight_control : float
                Flight control system weight [kg]
            - W_apu : float
                APU weight [kg]
            - W_hyd_pnu : float
                Hydraulics and pneumatics weight [kg]
            - W_instruments : float
                Instruments weight [kg]
            - W_avionics : float
                Avionics weight [kg]
            - W_electrical : float
                Electrical system weight [kg]
            - W_ac : float
                Air conditioning system weight [kg]
            - W_furnish : float
                Furnishings weight [kg]
            - W_anti_ice : float
                Anti-ice system weight [kg]
            - W_systems : float
                Total systems weight [kg]

    Notes
    -----
    This method implements Raymer's correlations for transport aircraft systems
    weight estimation. Each system is calculated separately using specific correlations. 
    APU weight is calculated using a correlation from Pasquale Sforza's Commercial Airplane Design Principles.

    **Major Assumptions**
        * Number of flight control systems = 4
        * Not a reciprocating engine or turboprop
        * System electrical rating: 60 kVA (typically 40-60 for transports, 110-160 for fighters & bombers)
        * Uninstalled avionics weight: 1400 lb
        * Flight crew size based on passenger count
        * Air conditioning sized for passenger count and cabin volume

    **Theory**
    Key system weights are calculated using:
    .. math::
        W_{fc} = 36.28M^{0.003}S_{cs}^{0.489}N_s^{0.484}N_{crew}^{0.124}

    .. math::
        W_{elec} = 7.291(kVA)^{0.782}(2L)^{0.346}N_{eng}^{0.1}

    .. math::
        W_{furn} = 0.0577N_{crew}^{0.1}W_{cargo}^{0.393}S_f^{0.75} + 46N_{pax} + 75N_{crew} + 2.5N_{pax}^{1.33}

    .. math::
        W_{ac} = 62.36N_{pax}^{0.25}\left(\frac{V_{pr}}{1000}\right)^{0.604}W_{av}^{0.1}

    .. math::
        W_{ai} = 0.002W_{dg}

    References
    ----------
    [1] Raymer, D., "Aircraft Design: A Conceptual Approach", AIAA 
        Education Series, 2018. 
    [2] Sforza, P. M. (2014). Commercial Airplane Design Principles. Netherlands: Elsevier Science.

    See Also
    --------
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport.Raymer.compute_operating_empty_weight
    """
    flap_area = 0
    ref_wing  = None 
    for wing in  vehicle.wings:
        if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing):
            ref_wing  = wing
            rc        = wing.chords.root
            taper     = wing.taper
            for cs in wing.control_surfaces:
                if type(cs) == RCAIDE.Library.Components.Wings.Control_Surfaces.Flap: 
                    sfs  = cs.span_fraction_start    
                    sfe  = cs.span_fraction_end       
                    c_f  = cs.chord_fraction
                    span = (sfe - sfs) *wing.spans.projected 
                    y1s = c_f*(rc -  taper *sfs * rc)
                    y2e = c_f*(rc -  taper *sfe * rc)
                    flap_area =  span * ( y1s + y2e) /2 
    S = 0
    if ref_wing == None:
        for wing in  vehicle.wings:
            if S < wing.areas.reference:
                ref_wing = wing
                rc        = wing.chords.root
                taper     = wing.taper
                for cs in wing.control_surfaces:
                    if type(cs) == RCAIDE.Library.Components.Wings.Control_Surfaces.Flap: 
                        sfs  = cs.span_fraction_start    
                        sfe  = cs.span_fraction_end       
                        c_f  = cs.chord_fraction
                        span =  (sfe - sfs) *wing.spans.projected 
                        y1s = c_f*(rc -  taper *sfs * rc)
                        y2e = c_f*(rc -  taper *sfe * rc)
                        flap_area =  span * ( y1s + y2e) /2 
    L_fus = 0
    for fuselage in vehicle.fuselages:
        if L_fus < fuselage.lengths.total:
            ref_fuselage = fuselage
            L_fus = ref_fuselage.lengths.total
    flap_ratio     = flap_area / ref_wing.areas.reference
    L              = ref_fuselage.lengths.total / Units.ft
    Bw             = ref_wing.spans.projected / Units.ft
    DG             = vehicle.mass_properties.max_takeoff / Units.lbs
    Scs            = flap_ratio * vehicle.reference_area / Units.ft**2
    design_mach    = vehicle.flight_envelope.design_mach_number
    num_pax        = vehicle.passengers 
    NENG = 0 
    for network in  vehicle.networks:
        for propulsor in network.propulsors:
            if isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan) or  isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet):
                NENG += 1  
    fuse_w         = ref_fuselage.width / Units.ft
    fuse_h         = ref_fuselage.heights.maximum / Units.ft   
    cargo_weight   = vehicle.payload.cargo.mass_properties.mass / Units.lbs
    
    if vehicle.passengers >= 150:
        flight_crew = 3 # number of flight crew
    else:
        flight_crew = 2
    Ns      = 4  # Number of flight control systems (typically 4)
    Kr      = 1  # assuming not a reciprocating engine
    Ktp     = 1  # assuming not a turboprop
    Nf      = 7  # number of functions performed by controls (typically 4-7)
    Nm      = 1  # number of mechincal functions performed by controls (typically 0-2)
    Rkva    = 60  # system electrical rating
    Iy      = 10**7 # For a norminal transport aircraft
    Wuav    = 1400 # Uninstall avionics weight

    WSC     = 145.9*Nf**.554 * (1+Nm/Nf)**-1 * Scs**0.2 * (Iy*10e-6)**0.07 
    WAPUG   = 8*(DG**0.4) # APU group weight from Commercial Airplane Design Principles by Pasquale Sforza eq. 8.27. Which is an improvment from Kroo's estimation
    WIN     = 4.509 * Kr * Ktp * flight_crew ** 0.541 * NENG * (L + Bw) ** 0.5
    WHYD    = 0.2673 * Nf * (L + Bw) ** 0.937
    WELEC   = 7.291 * Rkva ** 0.782 * (2*L) ** 0.346 * NENG ** 0.1
    WAVONCG  = 0.09 * DG**0.8  # Avionics Group from Commercial Airplane Design Principles by Pasquale Sforza eq. 8.35. Which is an improvment from Kroo's estimation

    D       = (fuse_w + fuse_h) / 2.
    Sf      = np.pi * (L / D - 1.7) * D ** 2  # Fuselage wetted area, ft**2
    WFURN   = 0.0577 * flight_crew ** 0.1 * (cargo_weight) ** 0.393 * Sf ** 0.75 + 46 * num_pax
    WFURN  += 75 * flight_crew
    WFURN  += 2.5 * num_pax**1.33

    Vpr = D ** 2 * np.pi / 4 * L
    WAC = 62.36 * num_pax ** 0.25 * (Vpr / 1000) ** 0.604 * Wuav ** 0.1

    WAI = 0.002 * DG

    output                     = Data()
    output.W_flight_control    = WSC * Units.lbs
    output.W_apu               = WAPUG * Units.lbs
    output.W_hyd_pnu           = WHYD * Units.lbs
    output.W_instruments       = WIN * Units.lbs
    output.W_avionics          = WAVONCG * Units.lbs
    output.W_electrical        = WELEC * Units.lbs
    output.W_ac                = WAC * Units.lbs
    output.W_furnish           = WFURN * Units.lbs
    output.W_anti_ice          = WAI * Units.lbs
    output.W_systems           = WSC + WAPUG + WIN + WHYD + WELEC + WAVONCG + WFURN + WAC + WAI
    return output
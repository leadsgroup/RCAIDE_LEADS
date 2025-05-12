# RCAIDE/Library/Methods/Mass_Properties/Weight_Buildups/Conventional/BWB/FLOPS/compute_systems_weight.py
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
    Calculate the system weight of the aircraft using the FLOPS methodology.
    Parameters
    ----------
    vehicle : Data
        Data dictionary with vehicle properties
            - networks : list
                List of network objects containing propulsion properties
                - propulsors : list
                    List of propulsor objects
                    - wing_mounted : bool
                        Flag indicating if propulsor is wing-mounted
                    - nacelle : Data, optional
                        Nacelle properties
                        - diameter : float
                            Nacelle diameter [m]
        - wings : list
            List of wing objects
            - areas.reference : float
                Wing reference area [m²]
            - flap_ratio : float
                Flap surface area over wing surface area
            - spans.projected : float
                Projected span of wing [m]
            - sweeps.quarter_chord : float
                Quarter chord sweep [deg]
        - fuselages : list
            List of fuselage objects
            - lengths.total : float
                Fuselage total length [m]
            - width : float
                Fuselage width [m]
            - heights.maximum : float
                Fuselage maximum height [m]
        - mass_properties.max_takeoff : float
            Maximum takeoff weight [kg]
        - flight_envelope.design_mach_number : float
            Design mach number for cruise flight
        - flight_envelope.design_range : float
            Design range of aircraft [nmi]
        - passengers : int
            Number of passengers in aircraft
        - NPF : int
            Number of first class passengers
        - NPB : int
            Number of business class passengers
        - NPT : int
            Number of tourist/economy class passengers
        - reference_area : float
            Aircraft reference area [m²]
    Returns
    -------
    output : Data
        Data dictionary containing system weight components
        - W_flight_control : float
            Weight of the flight control system [kg]
        - W_apu : float
            Weight of the auxiliary power unit [kg]
        - W_hyd_pnu : float
            Weight of the hydraulics and pneumatics [kg]
        - W_instruments : float
            Weight of the instruments and navigational equipment [kg]
        - W_avionics : float
            Weight of the avionics [kg]
        - W_electrical : float
            Weight of the electrical items [kg]
        - W_ac : float
            Weight of the air conditioning system [kg]
        - W_furnish : float
            Weight of the furnishings in the fuselage [kg]
        - W_anti_ice : float
            Weight of anti-ice system [kg]
        - W_systems : float
            Total systems weight [kg]
    Notes
    -----
    This function implements the Flight Optimization System (FLOPS) weight estimation
    methodology for aircraft systems. The calculations are performed in imperial units
    and converted to metric for output.
    **Major Assumptions**
        * No variable sweep (VARSWP = 0)
        * Hydraulic pressure is 3000 psf (HYDR)
        * Flight crew is 2 for aircraft with less than 150 passengers, 3 otherwise
    References
    ----------
    [1] McCullers, L. A. (1984). "Aircraft Configuration Optimization Including Optimized Flight Profiles", NASA Symposium on Recent Experiences in Multidisciplinary Analysis and Optimization.
    [2] Ardema, M. D., Chambers, M. C., Patron, A. P., Hahn, A. S., Miura, H., & Moore, M. D. (1996). "Analytical Fuselage and Wing Weight Estimation of Transport Aircraft", NASA Technical Memorandum 110392.
    See Also
    --------
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport.FLOPS
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
                if propulsor.nacelle !=  None:                
                    nacelle =  propulsor.nacelle
                    FNAC    = nacelle.diameter / Units.ft
            else:
                FNAC    = 0
            
    VMAX     = vehicle.flight_envelope.design_mach_number
    SFLAP    = 0
    ref_wing = None
    for wing in  vehicle.wings:
        if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing) or isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
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
    NFUSE = 0
    
    for wing in  vehicle.wings:
        if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
            XL    = wing.chords.root / Units.ft
            DF    = (wing.chords.root*wing.thickness_to_chord) / Units.ft
            NFUSE   += 1
            
        for segment in wing.segments:
            if isinstance(segment, RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment):
                WF    += segment.percent_span_location * wing.spans.projected  / Units.ft        

    WF = 12 /Units.feet
    XL = 22/Units.feet 
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
   
    WELEC   = 92. * XL ** 0.4 * WF ** 0.14 * NFUSE ** 0.27 * NENG ** 0.69 * \
            (1. + 0.044 * NFLCR + 0.0015 * NPASS)  # electrical system weight
    DESRNG  = vehicle.flight_envelope.design_range / Units.nmi
    WAVONC  = 15.8 * DESRNG ** 0.1 * NFLCR ** 0.7 * FPAREA ** 0.43  # avionics weight
    XLP     = 0.8 * XL
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
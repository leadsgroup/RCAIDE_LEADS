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
    """ Calculates the system weight based on the Raymer method

        Assumptions:
            Number of flight control systems = 4
            Max APU weight = 70 lbs
            Assuming not a reciprocating engine and not a turboprop
            System Electrical Rating: 60 kv · A (typically 40-60 for transports, 110-160 for fighters & bombers)
            Uninstalled Avionics weight: 1400 lb (typically= 800-1400 lb)

        Source:
            Aircraft Design: A Conceptual Approach (3rd edition)
            Sforza, P. (2014). Commercial airplane design principles. Butterworth-Heinemann. https://doi.org/10.1016/B978-0-12-419953-8

        Inputs:
            vehicle - data dictionary with vehicle properties                   [dimensionless]
                -.networks: data dictionary containing all propulsion properties
                -.number_of_engines: number of engines
                -.sealevel_static_thrust: thrust at sea level               [N]
                -.fuselages['fuselage'].lengths.total: fuselage total length    [meters]
                -.fuselages['fuselage'].width: fuselage width                   [meters]
                -.fuselages['fuselage'].heights.maximum: fuselage maximum height[meters]
                -.mass_properties.max_takeoff: MTOW                             [kilograms]
                -.design_mach_number: design mach number for cruise flight
                -.design_range: design range of aircraft                        [nmi]
                -.passengers: number of passengers in aircraft
                -.wings['main_wing']: data dictionary with main wing properties
                    -.sweeps.quarter_chord: quarter chord sweep                 [deg]
                    -.areas.reference: wing surface area                        [m^2]
                    -.spans.projected: projected span of wing                   [m]
                    -.flap_ratio: flap surface area over wing surface area
                -.payload: payload weight of aircraft                           [kg]

        Outputs:
            output - a data dictionary with fields:
               W_flight_controls - weight of the flight control system                                [kilograms]
               W_apu - weight of the apu                                                       [kilograms]
               W_hyd_pnu - weight of the hydraulics and pneumatics                             [kilograms]
               W_instruments - weight of the instruments and navigational equipment            [kilograms]
               W_avionics - weight of the avionics                                             [kilograms]
               W_electrical - weight of the electrical items                                         [kilograms]
               W_ac - weight of the air conditioning and anti-ice system                       [kilograms]
               W_furnish - weight of the furnishings in the fuselage                           [kilograms]
               W_anti_ice - weight of anti-ice system                                          [kilograms]

        Properties Used:
            N/A
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
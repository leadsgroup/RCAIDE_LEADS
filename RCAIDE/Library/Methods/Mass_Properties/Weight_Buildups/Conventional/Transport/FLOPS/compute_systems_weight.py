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
    """ Calculate the system weight of the aircraft including:
        -  flight control system
        - apu weight
        - hydraulics and pneumatics weight
        - intstrumentation weight
        - avionics weight
        - electrical system weight
        - air-condtioning weight
        - furnishing weight
        - anti-ice weight

    Assumptions:
        1) No variable sweep, change VARSWP to 1 is variable sweep in aicraft
        2) Hydraulic pressure is 3000 psf (HYDR)
        3) 1 fuselage named fuselage. Function could be modified to allow multiple fuselages by
           relaxing this assumption.

    Source:
        The Flight Optimization System Weight Estimation Method

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
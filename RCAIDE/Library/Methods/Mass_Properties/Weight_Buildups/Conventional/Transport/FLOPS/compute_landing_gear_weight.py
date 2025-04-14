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
    """
    Computes the landing gear weight using NASA FLOPS weight estimation method. Accounts for 
    aircraft type, size, and operational requirements.

    Parameters
    ----------
    vehicle : Vehicle
        The vehicle instance containing:
            - networks : list
                Propulsion system data for nacelle dimensions
            - design_range : float
                Design range [nmi]
            - systems.accessories : str
                Aircraft type ('short-range', 'commuter', 'medium-range', 
                'long-range', 'sst', 'cargo')
            - mass_properties.max_takeoff : float
                Maximum takeoff weight [kg]
            - wings['main_wing'].dihedral : float
                Wing dihedral angle [rad]
            - fuselages['fuselage'] : Fuselage
                Primary fuselage with:
                    - width : float
                        Maximum width [m]
                    - lengths.total : float
                        Total length [m]

    Returns
    -------
    output : Data
        Container with weight breakdown:
            - main : float
                Main landing gear weight [kg]
            - nose : float
                Nose landing gear weight [kg]

    Notes
    -----
    Uses FLOPS correlations developed from transport aircraft database, with 
    adjustments for different aircraft types. Please refer to the FLOPS documentation 
    for more details: https://ntrs.nasa.gov/citations/20170005851  

    **Major Assumptions**
        * Average landing gear
        * Not designed for carrier operations (CARBAS = 0)
        * Not a fighter aircraft (DFTE = 0)
        * Retractable gear configuration

    **Theory**
    Main gear weight is computed using:
    .. math::
        W_{MLG} = (0.0117 - 0.0012D_{FTE})W_{L}^{0.95}X_{MLG}^{0.43}

    Nose gear weight is computed using:
    .. math::
        W_{NLG} = (0.048 - 0.0080D_{FTE})W_{L}^{0.67}X_{NLG}^{0.43}(1 + 0.8C_{B})

    where:
        * W_L = landing weight
        * X_MLG = extended main gear length
        * X_NLG = extended nose gear length
        * D_FTE = fighter aircraft flag
        * C_B = carrier-based flag

    References
    ----------
    [1] NASA Flight Optimization System (FLOPS)

    See Also
    --------
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport.FLOPS.compute_operating_empty_weight
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
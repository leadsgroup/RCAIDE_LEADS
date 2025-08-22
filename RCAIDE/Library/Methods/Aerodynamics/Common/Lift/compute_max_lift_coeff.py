
# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

#RCAIDE Imports
from RCAIDE import  * 
from RCAIDE.Framework.Core import Units,  Data
from RCAIDE.Library.Components import Wings

from RCAIDE.Library.Methods.Aerodynamics.Common.Lift.compute_slat_lift import compute_slat_lift
from RCAIDE.Library.Methods.Aerodynamics.Common.Lift.compute_flap_lift import compute_flap_lift

# ----------------------------------------------------------------------
#  compute_max_lift_coeff
# ----------------------------------------------------------------------

def compute_max_lift_coeff(state,settings,geometry):
    """
    Computes the maximum lift coefficient for an aircraft with high-lift system.

    Parameters
    ----------
    state : Data
        Flight conditions containing:
            - conditions.freestream.velocity : float
                Freestream velocity [m/s]
            - conditions.freestream.density : float
                Freestream density [kg/m³]
            - conditions.freestream.dynamic_viscosity : float
                Freestream dynamic viscosity [N·s/m²]
    settings : dict
        Aerodynamic analysis settings containing:
            - maximum_lift_coefficient_factor : float
                Factor to adjust maximum lift coefficient [unitless]
    geometry : Data
        Aircraft geometry containing:
            - reference_area : float
                Vehicle reference area [m²]
            - wings : list
                List of wing objects containing:
                    - high_lift : bool
                        Flag indicating if wing has high-lift devices
                    - areas.reference : float
                        Wing reference area [m²]
                    - thickness_to_chord : float
                        Wing thickness-to-chord ratio [unitless]
                    - chords.mean_aerodynamic : float
                        Mean aerodynamic chord [m]
                    - sweeps.quarter_chord : float
                        Quarter-chord sweep angle [radians]
                    - taper : float
                        Wing taper ratio [unitless]
                    - areas.affected : float
                        Wing area affected by high-lift devices [m²]
                    - control_surfaces : dict
                        Dictionary of control surfaces containing:
                            - slat : Data, optional
                                Slat control surface with:
                                    - deflection : float
                                        Slat deflection angle [radians]
                            - flap : Data, optional
                                Flap control surface with:
                                    - configuration_type : str
                                        Type of flap configuration
                                    - chord_fraction : float
                                        Flap chord as fraction of wing chord [unitless]
                                    - deflection : float
                                        Flap deflection angle [radians]

    Returns
    -------
    Cl_max_ls : float
        Maximum lift coefficient with high-lift system [unitless]
    Cd_ind : float
        Induced drag coefficient [unitless]

    Notes
    -----
    This function calculates the maximum lift coefficient for an aircraft
    considering airfoil characteristics, wing geometry, Reynolds number effects,
    and high-lift device contributions (slats and flaps).
    
    **Major Assumptions**
        * Only wings with high_lift flag are considered
        * Reynolds number effects follow power law relationship
        * Wing mounted engines reduce maximum lift by 0.2
        * FAR stall speed requirements increase maximum lift by 10%
        * High-lift device effects are additive
        * Induced drag is estimated as 0.01 per wing
    
    **Theory**

    The maxiumum lift coefficient is calculated by adding the effects of the airfoil, wing, 
    and high-lift devices one after the other.

    The airfoil maximum lift coefficient is calculated using a polynomial fit:

    :math:`C_{L,max,ref} = -0.0009(t/c)^3 + 0.0217(t/c)^2 - 0.0442(t/c) + 0.7005`

    Reynolds number correction is applied:

    :math:`C_{L,max,Re} = C_{L,max,ref} \\left(\\frac{Re}{Re_{ref}}\\right)^{0.1}`

    where :math:`Re_{ref} = 9 \\times 10^6`.

    The wing maximum lift coefficient includes geometry effects:

    :math:`C_{L,max,wing} = C_{L,max,Re} \\cdot f(\\lambda, \\Lambda)`

    where :math:`f(\\lambda, \\Lambda)` is a polynomial function of taper ratio and sweep.

    FAR stall speed requirements increase the maximum lift:

    :math:`C_{L,max,FAR} = 1.1 \\cdot C_{L,max,wing}`

    Wing mounted engines reduce maximum lift:

    :math:`C_{L,max,eng} = C_{L,max,FAR} - 0.2`

    High-lift device increments are added:

    :math:`C_{L,max,HL} = C_{L,max,eng} + \\Delta C_{L,slat} + \\Delta C_{L,flap}`

    The total aircraft maximum lift coefficient is area-weighted:

    :math:`C_{L,max,total} = \\sum_{i=1}^{n} C_{L,max,HL,i} \\cdot \\frac{S_{wing,i}}{S_{ref}} \\cdot f_{factor}`
    
    **Definitions**

    'Maximum Lift Coefficient'
        Highest attainable lift coefficient before stall occurs.
    
    References
    ----------
    [1] FAR stall speed requirements for aircraft certification
    [2] Unknown

    See Also
    --------
    RCAIDE.Library.Methods.Aerodynamics.Common.Lift.compute_slat_lift
    RCAIDE.Library.Methods.Aerodynamics.Common.Lift.compute_flap_lift
    """


    # initializing Cl and CDi
    Cl_max_ls = 0
    Cd_ind    = 0
    vehicle = geometry
    conditions = state.conditions

    #unpack
    max_lift_coefficient_factor = settings.maximum_lift_coefficient_factor
    for wing in vehicle.wings:
    
        if not wing.high_lift: continue
        #geometrical data
        Sref       = vehicle.reference_area
        Swing      = wing.areas.reference
        tc         = wing.thickness_to_chord * 100
        chord_mac  = wing.chords.mean_aerodynamic
        sweep      = wing.sweeps.quarter_chord
        sweep_deg  = wing.sweeps.quarter_chord / Units.degree # convert into degrees
        taper      = wing.taper
        
        # conditions data
        V    = conditions.freestream.velocity
        roc  = conditions.freestream.density
        nu   = conditions.freestream.dynamic_viscosity

        #--cl max based on airfoil t_c
        Cl_max_ref = -0.0009*tc**3 + 0.0217*tc**2 - 0.0442*tc + 0.7005
        #-reynolds number effect
        Reyn     =  V * roc * chord_mac / nu
        Re_ref   = 9*10**6
        op_Clmax = Cl_max_ref * ( Reyn / Re_ref ) **0.1

        #wing cl_max to outer panel Cl_max
        w_Clmax = op_Clmax* ( 0.919729714285715 -0.044504761904771*taper \
                             -0.001835900000000*sweep_deg +  0.247071428571446*taper**2 +  \
                              0.003191500000000*taper*sweep_deg -0.000056632142857*sweep_deg**2  \
                             -0.279166666666676*taper**3 +  0.002300000000000*taper**2*sweep_deg + \
                              0.000049982142857*taper*sweep_deg**2  -0.000000280000000* sweep_deg**3)

        #---FAR stall speed effect---------------
        #should be optional based on aircraft being modelled
        Cl_max_FAA = 1.1 * w_Clmax

        #-----------wing mounted engine ----
        Cl_max_w_eng = Cl_max_FAA - 0.2

        # Compute CL increment due to Flap
        if 'slat' in wing.control_surfaces.keys():
         # Compute CL increment due to Slat
            slat_angle = wing.control_surfaces.slat.deflection
            dcl_slat = compute_slat_lift(slat_angle, sweep)
        else:
            dcl_slat = 0.

        # Compute CL increment due to Flap
        if 'flap' in wing.control_surfaces.keys():
            flap_type  = wing.control_surfaces.flap.configuration_type
            flap_chord = wing.control_surfaces.flap.chord_fraction # correct !!! 
            flap_angle = wing.control_surfaces.flap.deflection
            Swf        = wing.areas.affected  # portion of wing area with flaps
            dcl_flap   = compute_flap_lift(tc,flap_type,flap_chord,flap_angle,sweep,Sref,Swf)
        else:
            dcl_flap = 0.0

        #results
        Cl_max_ls += (Cl_max_w_eng + dcl_slat + dcl_flap) * Swing / Sref
        Cd_ind += ( 0.01 ) * Swing / Sref

    Cl_max_ls = Cl_max_ls * max_lift_coefficient_factor
    return Cl_max_ls, Cd_ind

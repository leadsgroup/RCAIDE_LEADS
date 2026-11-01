# RCAIDE/Library/Methods/Aerodynamics/Common/Drag/miscellaneous_drag.py 
# 
# Created:  Jun 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
  
import RCAIDE
from RCAIDE.Framework.Core                    import Data, Units

# package imports
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
#  Miscellaneous Drag Total
# ----------------------------------------------------------------------------------------------------------------------   
def miscellaneous_drag(state,settings,geometry):
    """
    Computes the miscellaneous drag coefficient associated with aircraft components and configurations.

    Parameters
    ----------
    state : Data
        Flight conditions containing:
            - conditions.freestream.mach_number : float
                Freestream Mach number [unitless]
    settings : dict
        Aerodynamic analysis settings and parameters
    geometry : Data
        Aircraft geometry containing:
            - reference_area : float
                Reference area for drag coefficient calculation [m²]
            - landing_gears : list
                List of landing gear objects containing:
                    - gear_extended : bool
                        Flag indicating if landing gear is deployed
            - mass_properties.max_takeoff : float
                Maximum takeoff mass, the gross weight of the gear drag correlation [kg]
            - wings : list
                List of wing objects containing:
                    - areas.wetted : float
                        Wetted area of the wing [m²]
                    - control_surfaces : list
                        Flap deflection [rad], which reduces the gear drag increment
            - fuselages : list
                List of fuselage objects containing:
                    - areas.wetted : float
                        Wetted area of the fuselage [m²]
            - booms : list
                List of boom objects containing:
                    - areas.wetted : float
                        Wetted area of the boom [m²]
            - networks : list
                List of propulsion networks containing propulsors
                    - propulsors : list
                        List of propulsor objects with nacelle attributes
                            - nacelle : Nacelle, optional
                                Nacelle object containing:
                                    - areas.wetted : float
                                        Wetted area of the nacelle [m²]
                                    - diameter : float
                                        Diameter of the nacelle [m]

    Returns
    -------
    None
        Results are stored in state.conditions.aerodynamics.coefficients.drag.miscellaneous.total

    Notes
    -----
    This function calculates the miscellaneous drag coefficient including landing gear drag,
    subsonic miscellaneous drag based on wetted area, and supersonic nacelle base drag and
    fuselage upsweep drag. The calculation method varies between subsonic and supersonic
    flight regimes.
    
    **Major Assumptions**
        * Landing gear drag scales with gross weight (GASP correlation)
        * Subsonic miscellaneous drag correlates with total wetted area
        * Supersonic drag includes nacelle base drag and fuselage upsweep effects
        * Basic empirical correlations are valid for typical transport aircraft
    
    **Theory**

    Landing Gear Drag:
    With the gear extended, the NASA GASP correlation (as implemented in NASA Aviary 0.10,
    subsystems/aerodynamics/gasp_based/table_based.py, GearDragIncrement) gives the gear equivalent
    flat-plate area from the gross weight and reduces it with flap deflection:

    :math:`f_{LG} = 0.0033 W^{0.785}` [ft², W in lb],  :math:`C_{D,LG} = \\frac{f_{LG}}{S_{ref}} (1 - 0.4545 \\delta_f / 50)`

    where :math:`\\delta_f` is the main-wing flap deflection [deg]. It sits between Raymer's 0.02 gear-down
    increment (Aircraft Design, 1992, Sec. 5.3) and Roskam Part VI Fig. 4.60 for an E175-sized aircraft.

    Subsonic Miscellaneous Drag:
    For Mach ≤ 1.0, the miscellaneous drag is:

    :math:`C_{D,misc} = \\frac{0.40(0.0184 + 0.000469 S_{wet} - 1.13 \\times 10^{-7} S_{wet}^2)}{S_{ref}}`

    where :math:`S_{wet}` is the total wetted area [m²].

    Supersonic Miscellaneous Drag:
    For Mach > 1.0, the drag includes nacelle base drag and fuselage upsweep:

    :math:`C_{D,nacelle} = \\sum_{i=1}^{n} \\frac{0.5}{12} \\pi D_i \\cdot 0.2 / S_{ref}`

    :math:`C_{D,upsweep} = \\frac{0.006}{S_{ref}}`

    :math:`C_{D,misc} = C_{D,nacelle} + C_{D,upsweep}`
    
    **Definitions**

    'Miscellaneous Drag'
        Drag component not accounted for by other drag categories, including landing gear, nacelle base, and fuselage upsweep effects.
    
    'Landing Gear Drag'
        Additional drag of the extended landing gear, scaled from the gross weight (GASP correlation).
    
    'Nacelle Base Drag'
        Drag caused by the base area of nacelles in supersonic flow.

    References
    ----------
    [1] Stanford AA241 Course Notes. http://aerodesign.stanford.edu/aircraftdesign/aircraftdesign.html
    """ 

    conditions     = state.conditions  
    S_ref          = geometry.reference_area
    Mach           = conditions.freestream.mach_number 
   
    # landing gear drag, GASP correlation (see Notes)
    landing_gear_drag  =  np.zeros_like(Mach)
    if any(landing_gear.gear_extended for landing_gear in geometry.landing_gears):
        gross_weight   = geometry.mass_properties.max_takeoff / Units.lb
        flat_plate     = 0.0033 * gross_weight**0.785 * Units.ft**2
        flap_deg       = 0.0
        for wing in geometry.wings:
            for control_surface in wing.control_surfaces:
                if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Flap:
                    flap_deg = max(flap_deg, abs(control_surface.deflection) / Units.degrees)
        landing_gear_drag[:,0] = flat_plate / S_ref * max(0.0, 1 - 0.454545 * flap_deg / 50)
         
    # subsonic 
    miscellaneous_drag =  np.zeros_like(Mach) 
    swet_tot       = 0.
    for wing in geometry.wings:
        swet_tot += wing.areas.wetted 
    for fuselage in geometry.fuselages:
        swet_tot += fuselage.areas.wetted
    for boom in geometry.booms:
        swet_tot += boom.areas.wetted
    for network in geometry.networks: 
        for propulsor in network.propulsors:   
            if propulsor.nacelle !=  None:                    
                swet_tot += propulsor.nacelle.areas.wetted 
    miscellaneous_drag[:,0] =  (0.40* (0.0184 + 0.000469 * swet_tot - 1.13*10**-7 * swet_tot ** 2)) / S_ref
    
    # supersonic 
    fuselage_upsweep_drag   = 0.006 /S_ref  
    miscellaneous_drag[Mach>1] = fuselage_upsweep_drag
    
    # total misc drag 
    total_miscellaneous_drag = miscellaneous_drag + landing_gear_drag
        
    # Store results 
    conditions.aerodynamics.coefficients.drag.miscellaneous = Data(  total  = total_miscellaneous_drag)
    return  
    

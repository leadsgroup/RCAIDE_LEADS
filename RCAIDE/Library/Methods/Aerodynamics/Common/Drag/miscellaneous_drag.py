# RCAIDE/Library/Methods/Aerodynamics/Common/Drag/miscellaneous_drag.py 
# 
# Created:  Jun 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
  
from RCAIDE.Framework.Core                    import Data  

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
                    - fairing : bool
                        Flag indicating if landing gear has fairings
                    - wheels : int
                        Number of wheels on the landing gear
                    - tire_diameter : float
                        Diameter of landing gear tires [m]
                    - tire_width : float
                        Width of landing gear tires [m]
            - wings : list
                List of wing objects containing:
                    - areas.wetted : float
                        Wetted area of the wing [m²]
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
        * Landing gear drag is proportional to tire frontal area
        * Subsonic miscellaneous drag correlates with total wetted area
        * Supersonic drag includes nacelle base drag and fuselage upsweep effects
        * Landing gear fairings reduce drag by approximately 50%
        * Basic empirical correlations are valid for typical transport aircraft
    
    **Theory**

    Landing Gear Drag:
    The landing gear drag coefficient is calculated as:

    :math:`C_{D,LG} = \\sum_{i=1}^{n} N_{wheels,i} \\cdot C_{D,i} \\cdot \\frac{D_i \\cdot W_i}{S_{ref}}`

    where:
        - :math:`N_{wheels,i}` is the number of wheels on landing gear :math:`i`
        - :math:`C_{D,i}` is 0.15 for faired landing gear or 0.30 for unfaired
        - :math:`D_i` and :math:`W_i` are the tire diameter and width [m]
        - :math:`S_{ref}` is the reference area [m²]

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
        Additional drag caused by deployed landing gear, proportional to tire frontal area.
    
    'Nacelle Base Drag'
        Drag caused by the base area of nacelles in supersonic flow.

    References
    ----------
    [1] Stanford AA241 Course Notes. http://aerodesign.stanford.edu/aircraftdesign/aircraftdesign.html
    """ 

    conditions     = state.conditions  
    S_ref          = geometry.reference_area
    Mach           = conditions.freestream.mach_number 
   
    # landing gear drag  
    landing_gear_drag  =  np.zeros_like(Mach)
    for landing_gear in  geometry.landing_gears:
        if landing_gear.gear_extended == True:
            if landing_gear.fairing:
                cd_lg = 0.15
            else:
                cd_lg = 0.30
            landing_gear_drag[:,0] +=  landing_gear.wheels *  cd_lg * (landing_gear.tire_diameter * landing_gear.tire_width) /S_ref 
         
    # miscellaneous drag
    miscellaneous_drag =  np.zeros_like(Mach)
    if np.all((Mach<=1.0) == True):  # Subsonic
        swet_tot       = 0.
        for wing in geometry.wings:
            swet_tot += wing.areas.wetted 
        for fuselage in geometry.fuselages:
            swet_tot += fuselage.areas.wetted
        for boom in geometry.booms:
            swet_tot += boom.areas.wetted
        for network in geometry.networks: 
            for propulsor in network.propulsors:  
                if 'nacelle' in propulsor: 
                    if propulsor.nacelle !=  None:                    
                        swet_tot += propulsor.nacelle.areas.wetted
                            
        # Total miscellaneous drag 
        miscellaneous_drag[:,0] =  (0.40* (0.0184 + 0.000469 * swet_tot - 1.13*10**-7 * swet_tot ** 2)) / S_ref    
    else:  
        # Initialize drag
        total_nacelle_base_drag   = 0.0   
        # Estimating nacelle drag 
        for network in  geometry.networks: 
            for propulsor in network.propulsors:  
                if 'nacelle' in propulsor: 
                    if propulsor.nacelle !=  None:                    
                        nacelle_base_drag = 0.5/12. * np.pi * propulsor.nacelle.diameter * 0.2/S_ref  
                        total_nacelle_base_drag += nacelle_base_drag     

        # Fuselage upsweep drag 
        fuselage_upsweep_drag = 0.006 /S_ref  
        miscellaneous_drag[:,0] = total_nacelle_base_drag + fuselage_upsweep_drag
        
    total_miscellaneous_drag = miscellaneous_drag + landing_gear_drag
        
    # Store results 
    conditions.aerodynamics.coefficients.drag.miscellaneous = Data(  total  = total_miscellaneous_drag)
    return  
    

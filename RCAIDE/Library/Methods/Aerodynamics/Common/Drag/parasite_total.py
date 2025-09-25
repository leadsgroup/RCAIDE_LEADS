# RCAIDE/Methods/Aerodynamics/Common/Drag/parasite_drag_total.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
import numpy as np 
 
# package imports
import numpy as np  

# ----------------------------------------------------------------------------------------------------------------------  
#  Total Parasite Drag 
# ----------------------------------------------------------------------------------------------------------------------   
def parasite_total(state,settings,geometry):
    """
    Computes the total parasite drag coefficient by summing contributions from all aircraft components.

    Parameters
    ----------
    state : Data
        Flight conditions and aerodynamic state containing:
            - conditions.aerodynamics.coefficients.drag.parasite : dict
                Dictionary of parasite drag coefficients indexed by component tag containing:
                    - total : float
                        Parasite drag coefficient for each component [unitless]
    settings : dict
        Aerodynamic analysis settings containing:
            - drag_reduction_factors.parasite_drag : float
                Parasite drag reduction factor [unitless]
    geometry : Data
        Aircraft geometry containing:
            - reference_area : float
                Vehicle reference area for drag coefficient normalization [m²]
            - wings : list
                List of wing objects containing:
                    - tag : str
                        Unique identifier for the wing
                    - areas.reference : float
                        Reference area of the wing [m²]
            - fuselages : list
                List of fuselage objects containing:
                    - tag : str
                        Unique identifier for the fuselage
                    - areas.front_projected : float
                        Front projected area of the fuselage [m²]
            - booms : list
                List of boom objects containing:
                    - tag : str
                        Unique identifier for the boom
                    - areas.front_projected : float
                        Front projected area of the boom [m²]
            - networks : list
                List of propulsion networks containing propulsors
                    - propulsors : list
                        List of propulsor objects with nacelle attributes
                            - nacelle : Nacelle, optional
                                Nacelle object containing:
                                    - tag : str
                                        Unique identifier for the nacelle
                                    - diameter : float
                                        Diameter of the nacelle [m]

    Returns
    -------
    None
        Results are stored in state.conditions.aerodynamics.coefficients.drag.parasite.total

    Notes
    -----
    This function aggregates parasite drag contributions from all aircraft components
    (wings, fuselages, booms, nacelles, and pylons) and normalizes them to the vehicle
    reference area. The total is then adjusted by a drag reduction factor to account
    for interference effects and other corrections.
    
    **Major Assumptions**
        * Component drag coefficients are normalized to their respective reference areas
        * Vehicle reference area is used for final normalization
        * Drag reduction factor accounts for interference effects
        * Nacelle front area is calculated as circular cross-section
        * Component contributions are additive
    
    **Theory**

    The total parasite drag coefficient is calculated as the sum of all component contributions:

    :math:`C_{D,parasite,total} = \\sum_{i=1}^{n} C_{D,parasite,i} \\cdot \\frac{S_{ref,i}}{S_{ref,vehicle}}`

    where:
        - :math:`C_{D,parasite,i}` is the parasite drag coefficient of component :math:`i`
        - :math:`S_{ref,i}` is the reference area of component :math:`i`
        - :math:`S_{ref,vehicle}` is the vehicle reference area

    For wings:
    :math:`S_{ref,wing} = S_{wing}` (wing reference area)

    For fuselages and booms:
    :math:`S_{ref,fuselage} = S_{front,projected}` (front projected area)

    For nacelles:
    :math:`S_{ref,nacelle} = \\frac{\\pi d_{nacelle}^2}{4}` (circular cross-section area)

    The final total parasite drag coefficient includes a reduction factor (user defined):

    :math:`C_{D,parasite,final} = C_{D,parasite,total} \\cdot (1 - f_{reduction})`

    where :math:`f_{reduction}` is the parasite drag reduction factor.

    See Also
    --------
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_wing
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_fuselage
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_nacelle
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_pylon
    """
    # unpack
    conditions             = state.conditions 
    vehicle_reference_area = geometry.reference_area
    
    #compute parasite drag total
    total_parasite_drag = 0.0
    
    # renormalize parasite drag from wings using reference area of aircraft 
    for wing in geometry.wings:
        wing_parasite_drag = conditions.aerodynamics.coefficients.drag.parasite[wing.tag].total 
        conditions.aerodynamics.coefficients.drag.parasite[wing.tag].total  = wing_parasite_drag * wing.areas.reference/vehicle_reference_area
        total_parasite_drag += wing_parasite_drag * wing.areas.reference/vehicle_reference_area
 
    # renormalize parasite drag from fuselages using reference area of aircraft 
    for fuselage in geometry.fuselages:
        fuselage_parasite_drag = conditions.aerodynamics.coefficients.drag.parasite[fuselage.tag].total 
        conditions.aerodynamics.coefficients.drag.parasite[fuselage.tag].total = fuselage_parasite_drag * fuselage.areas.front_projected/vehicle_reference_area
        total_parasite_drag += fuselage_parasite_drag * fuselage.areas.front_projected/vehicle_reference_area
    
    # renormalize parasite drag from booms using reference area of aircraft         
    for boom in geometry.booms:
        boom_parasite_drag = conditions.aerodynamics.coefficients.drag.parasite[boom.tag].total 
        conditions.aerodynamics.coefficients.drag.parasite[boom.tag].total = boom_parasite_drag * boom.areas.front_projected/vehicle_reference_area
        total_parasite_drag += boom_parasite_drag * boom.areas.front_projected/vehicle_reference_area
    
    # renormalize parasite drag from nacelles and pylons using reference area of aircraft  
    for network in  geometry.networks:
        for nacelle in  network.nacelles: 
            front_area    = np.pi * (nacelle.diameter ** 2) /4  
            nacelle_parasite_drag = conditions.aerodynamics.coefficients.drag.parasite[nacelle.tag].total  
            conditions.aerodynamics.coefficients.drag.parasite[nacelle.tag].total  = nacelle_parasite_drag * front_area/vehicle_reference_area
            total_parasite_drag += nacelle_parasite_drag * front_area/vehicle_reference_area
                
    state.conditions.aerodynamics.coefficients.drag.parasite.total = total_parasite_drag *  (1 -  settings.drag_reduction_factors.parasite_drag)

    return 
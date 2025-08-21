# RCAIDE/Library/Methods/Aerodynamics/Common/Drag/pparasite_drag_pylon.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imporst 
from RCAIDE.Framework.Core import Data 

# python imports 
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
#  Pylon Drag Fuselage
# ----------------------------------------------------------------------------------------------------------------------   
def parasite_drag_pylon(state,settings,geometry):
    """
    Computes the parasite drag coefficient for pylons as a proportion of nacelle drag.

    Parameters
    ----------
    state : Data
        Flight conditions and aerodynamic state containing:
            - conditions.aerodynamics.coefficients.drag.parasite : dict
                Dictionary of parasite drag data indexed by nacelle tag containing:
                    - total : float
                        Total parasite drag coefficient [unitless]
                    - wetted_area : float
                        Wetted area [m²]
                    - skin_friction : float
                        Skin friction coefficient [unitless]
                    - compressibility_factor : float
                        Compressibility correction factor [unitless]
                    - reynolds_factor : float
                        Reynolds number correction factor [unitless]
                    - form_factor : float
                        Form factor [unitless]
    settings : dict
        Aerodynamic analysis settings and parameters. Not used.
    geometry : Data
        Aircraft geometry containing:
            - reference_area : float
                Reference area for drag coefficient calculation [m²]
            - networks : list
                List of propulsion networks containing propulsors
                    - propulsors : list
                        List of propulsor objects with nacelle attributes
                            - nacelle : Nacelle, optional
                                Nacelle object containing:
                                    - diameter : float
                                        Diameter of the nacelle [m]
                                    - has_pylon : bool
                                        Flag indicating if nacelle has an associated pylon
                                    - tag : str
                                        Unique identifier for the nacelle

    Returns
    -------
    None
        Results are stored in state.conditions.aerodynamics.coefficients.drag.parasite[nacelle.tag + '_pylon']

    Notes
    -----
    This function calculates the parasite drag coefficient for pylons based on the
    corresponding nacelle drag characteristics. The pylon drag is estimated as a
    proportion of the nacelle drag, accounting for the relative size and aerodynamic
    characteristics of the pylon compared to the nacelle.
    
    **Major Assumptions**
        * Pylon drag is proportional to nacelle drag
        * Pylon factor of 0.2 represents a typical pylon-to-nacelle drag ratio
        * Pylon inherits aerodynamic characteristics from associated nacelle
        * Pylon reference area is based on nacelle diameter
        * Pylon drag scales with the ratio of pylon to aircraft reference areas
    
    **Theory**

    The pylon drag is calculated as a proportion of the nacelle drag:

    :math:`C_{D,pylon} = f_{pylon} \\cdot C_{D,nacelle} \\cdot \\frac{S_{ref,pylon}}{S_{ref,aircraft}}`

    where:
        - :math:`f_{pylon} = 0.2` is the pylon factor
        - :math:`C_{D,nacelle}` is the nacelle parasite drag coefficient
        - :math:`S_{ref,pylon} = \\frac{\\pi d_{nacelle}^2}{4}` is the pylon reference area
        - :math:`S_{ref,aircraft}` is the aircraft reference area

    The pylon inherits the aerodynamic characteristics from the nacelle:

    :math:`C_{f,pylon} = C_{f,nacelle}`

    :math:`k_{comp,pylon} = k_{comp,nacelle}`

    :math:`k_{reyn,pylon} = k_{reyn,nacelle}`

    :math:`FF_{pylon} = FF_{nacelle}`

    The pylon wetted area is scaled proportionally:

    :math:`S_{wet,pylon} = f_{pylon} \\cdot S_{wet,nacelle}`
    
    **Definitions**

    'Pylon'
        Structural support connecting an engine nacelle to the aircraft wing or fuselage.

    References
    ----------
    [1] Stanford AA241 Course Notes
    """
    
    drag          = state.conditions.aerodynamics.coefficients.drag
    pylon_factor  = 0.2  

    # Estimating pylon drag
    for network in  geometry.networks: 
        for propulsor in network.propulsors:  
            if 'nacelle' in propulsor:
                if propulsor.nacelle !=  None:
                    nacelle   =  propulsor.nacelle
                    if nacelle.has_pylon:
                        ref_area             = nacelle.diameter**2 / 4 * np.pi
                        pylon_parasite_drag  = pylon_factor *  drag.parasite[nacelle.tag].total* (ref_area/geometry.reference_area)
                        pylon_wetted_area    = pylon_factor *  drag.parasite[nacelle.tag].wetted_area
                        pylon_cf             = drag.parasite[nacelle.tag].skin_friction
                        pylon_compr_fact     = drag.parasite[nacelle.tag].compressibility_factor
                        pylon_rey_fact       = drag.parasite[nacelle.tag].reynolds_factor
                        pylon_FF             = drag.parasite[nacelle.tag].form_factor
                        pylon_result         = Data(
                            wetted_area               = pylon_wetted_area   ,
                            reference_area            = geometry.reference_area   ,
                            total                     = pylon_parasite_drag ,
                            skin_friction             = pylon_cf  ,
                            compressibility_factor    = pylon_compr_fact   ,
                            reynolds_factor           = pylon_rey_fact   ,
                            form_factor               = pylon_FF   , )
                        drag.parasite[ nacelle.tag + '_pylon'] = pylon_result 
    return 
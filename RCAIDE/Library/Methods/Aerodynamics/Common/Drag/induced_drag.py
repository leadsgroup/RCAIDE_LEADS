# RCAIDE/Library/Methods/Aerodynamics/Common/Drag/induced_drag.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imporst 
from RCAIDE.Framework.Core import Data

# package imports
import numpy as np

# ----------------------------------------------------------------------
#  Induced Drag Aircraft
# ---------------------------------------------------------------------- 
def induced_drag(state,settings,geometry):
    """
    Determines induced drag coefficient for the full aircraft using multiple calculation methods.

    Parameters
    ----------
    state : Data
        Flight conditions and aerodynamic state containing:
            - conditions.aerodynamics.coefficients.lift.total : float
                Total lift coefficient [unitless]
            - conditions.aerodynamics.coefficients.drag.induced.inviscid : float
                Inviscid induced drag coefficient [unitless]
            - conditions.aerodynamics.coefficients.drag.parasite.total : float
                Total parasite drag coefficient [unitless]
            - conditions.aerodynamics.coefficients.drag.parasite : dict
                Dictionary of parasite drag coefficients by wing tag
                    - wing_tag : Data
                        Parasite drag data for each wing containing:
                            - parasite_drag_coefficient : float
                                Parasite drag coefficient [unitless]
                            - reference_area : float
                                Reference area [m²]
    settings : dict
        Aerodynamic analysis settings containing:
            - oswald_efficiency_factor : float, optional
                Vehicle-level Oswald efficiency factor [unitless]
            - viscous_lift_dependent_drag_factor : float
                Viscous lift-dependent drag factor K [unitless]
            - span_efficiency : float, optional
                Span efficiency factor [unitless]
    geometry : Data
        Aircraft geometry containing:
            - wings : list
                List of wing objects containing:
                    - tag : str
                        Unique identifier for the wing
                    - aspect_ratio : float
                        Aspect ratio of the wing [unitless]
                    - areas.reference : float
                        Reference area of the wing [m²]

    Returns
    -------
    None
        Results are stored in state.conditions.aerodynamics.coefficients.drag.induced

    Notes
    -----
    This function calculates the induced drag coefficient using one of three methods depending
    on the available input data. The calculation accounts for both inviscid and viscous
    components of induced drag, with the viscous component related to parasite drag through
    a lift-dependent factor.
    
    **Major Assumptions**
        * Three calculation methods available based on input data availability
        * Viscous induced drag is proportional to parasite drag and lift coefficient squared
        * Largest wing determines the effective aspect ratio when multiple wings are present
        * Fuselage-induced drag is not explicitly accounted for in span efficiency methods
    
    **Theory**

    Method 1: Oswald Efficiency Factor Provided
    The total induced drag is calculated directly:

    :math:`C_{D,i} = \\frac{C_L^2}{\\pi AR \\cdot e_{osw}}`

    where:
    - :math:`C_L` is the total lift coefficient
    - :math:`AR` is the aspect ratio of the largest wing
    - :math:`e_{osw}` is the Oswald efficiency factor

    Method 2: Span Efficiency Provided
    The inviscid induced drag is calculated from span efficiency (Note, this is not the same as the oswald efficiency factor):

    :math:`C_{D,i,inviscid} = \\frac{C_L^2}{\\pi AR \\cdot e_{span}}`

    The viscous induced drag is:

    :math:`C_{D,i,viscous} = K \\cdot C_{D,parasite} \\cdot C_L^2`

    where :math:`K` is the viscous lift-dependent drag factor.

    Method 3: Inviscid Induced Drag from Analysis
    Uses pre-computed inviscid induced drag and calculates viscous component:

    :math:`C_{D,i,viscous} = K \\cdot C_{D,parasite} \\cdot C_L^2`

    :math:`C_{D,i,total} = C_{D,i,inviscid} + C_{D,i,viscous}`

    The effective Oswald efficiency factor is back-calculated as:

    :math:`e_{osw} = \\frac{C_L^2}{\\pi AR \\cdot C_{D,i,total}}`
    
    **Definitions**

    'Induced Drag'
        Drag component caused by the production of lift, including both inviscid and viscous effects.
    
    'Oswald Efficiency Factor'
        Factor accounting for the efficiency of lift generation and its associated drag penalty of the entire aircraft.
    
    'Span Efficiency Factor'
        Factor accounting for the efficiency of lift generation and its associated drag penalty of the wing.

    'Viscous Lift-Dependent Drag'
        Additional drag caused by viscous effects that scale with lift coefficient squared.

    References
    ----------
    [1] Stanford AA241 Course Notes. adg.stanford.edu http://aerodesign.stanford.edu/aircraftdesign/aircraftdesign.html
    """
    # unpack inputs 
    wings         = geometry.wings 
    K             = settings.viscous_lift_dependent_drag_factor
    e_osw         = settings.oswald_efficiency_factor	 
    aero          = state.conditions.aerodynamics.coefficients
    CL            = aero.lift.total
    CDi           = aero.drag.induced.inviscid

    wing_viscous_induced_drags = Data()

    # If the oswald efficiency factor is not specified  
    if e_osw == None:

        # Prime totals
        area                        = 1E-12 
        AR                          = 1E-12 
        total_viscous_induced_drag  = K*aero.drag.parasite.total*(CL**2)

        # Go through each wing, and make calculations
        for wing in wings: 
            AR_wing    = wing.aspect_ratio
            S_wing     = aero.drag.parasite[wing.tag].reference_area  
            if S_wing > area:
                area = S_wing
                AR   = AR_wing
        
        # compute total induced drag 
        total_induced_drag = total_viscous_induced_drag +  CDi
        
        # Calculate the vehicle level oswald efficiency
        e_osw = (CL**2)/(np.pi*AR*total_induced_drag)

    # If the user specifies a vehicle level oswald efficiency factor
    else: 
        # Find the largest wing, use that for AR
        S  = 1E-12 
        AR = 1E-12 
        for wing in wings:
            if wing.areas.reference>S:
                AR = wing.aspect_ratio
                S  = wing.areas.reference 

        # Calculate the induced drag       
        total_induced_drag = CL **2 / (np.pi*AR*e_osw)
        total_viscous_induced_drag = total_induced_drag - CDi
        
    aero.drag.induced.total                    = total_induced_drag
    aero.drag.induced.viscous                  = total_viscous_induced_drag 
    aero.drag.induced.oswald_efficiency_factor = e_osw
    aero.drag.induced.viscous_wings_drag       = wing_viscous_induced_drags 
    
    return 
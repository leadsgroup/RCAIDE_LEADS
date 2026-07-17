# RCAIDE/Library/Methods/Aerodynamics/Common/Drag/trim_drag.py
# 
# Created:  Mar 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Spolier Drag 
# ---------------------------------------------------------------------------------------------------------------------- 
def trim_drag(state,settings,geometry):
    """
    Computes trim drag and updates aerodynamic coefficients due to control surface deflections.

    Parameters
    ----------
    state : Data
        Flight conditions and aerodynamic state of aircraft containing:
            - conditions.aerodynamics.coefficients.drag.parasite.total : float
                Parasite drag coefficient [unitless]
            - conditions.aerodynamics.coefficients.drag.induced.total : float
                Induced drag coefficient [unitless]
            - conditions.aerodynamics.coefficients.drag.compressible.total : float
                Compressibility drag coefficient [unitless]
            - conditions.aerodynamics.coefficients.drag.form.total : float
                Form drag coefficient [unitless]
            - conditions.control_surfaces.flap.deflection : float
                Flap deflection angle [degrees]
    settings : dict
        Aerodynamic analysis settings and parameters
    geometry : Data
        Aircraft geometry containing:
            - wings : list
                List of wing objects with control_surfaces attribute
                - control_surfaces : list
                    List of control surface objects (Spoiler, Flap, etc.)
                    - deflection : float
                        Control surface deflection angle [degrees]
                    - chord_fraction : float
                        Fraction of wing chord occupied by control surface [unitless]
                    - type : str
                        Type of flap ('split', 'plain', 'single_slotted', 'flowler', 'double_slotted')

    Returns
    -------
    None
        Results are stored in state.conditions.aerodynamics.coefficients.drag.trim.total

    Notes
    -----
    This function calculates the additional drag and aerodynamic effects caused by control surface
    deflections, particularly spoilers and flaps. The calculations are based on empirical correlations
    from wind tunnel testing and flight data.
    
    **Major Assumptions**
        * Linear relationship between control surface deflection and drag increment
        * Control surface effects are additive to baseline aerodynamic coefficients
        * Empirical coefficients are valid across typical flight conditions
    
    **Theory**

    For spoilers, the drag increment is calculated as:

    :math:`\\Delta C_D = 0.0011 \\frac{\\delta_{spoiler}}{1°}`

    where :math:`\\delta_{spoiler}` is the spoiler deflection angle in degrees.

    For flaps, the drag increment follows:

    :math:`\\Delta C_D = c_f \\cdot A \\cdot \\left(\\frac{\\delta_{flap}}{1°}\\right)^B`

    where:
    - :math:`c_f` is the chord fraction of the flap
    - :math:`A` and :math:`B` are empirical coefficients that depend on flap type
    - :math:`\\delta_{flap}` is the flap deflection angle in degrees

    The lift and moment effects for spoilers are:

    :math:`\\Delta C_L = -0.0075 \\frac{\\delta_{spoiler}}{1°}`

    :math:`\\Delta C_M = 0.0053 \\frac{\\delta_{spoiler}}{1°}`
    
    **Definitions**

    'Trim Drag'
        Additional drag caused by control surface deflections required to maintain aircraft equilibrium.
    
    'Spoiler'
        Control surface that reduces lift and increases drag by disrupting airflow over the wing.
    
    'Flap'
        High-lift device that increases wing camber and chord to improve low-speed performance.

    References
    ----------
    [1] Croom, D. R. (1976). "Low-speed wind-tunnel investigation of various segments of flight spoilers as trailing-vortex-alleviation devices on a transport aircraft model." NASA-TN-D-8162.
    [2] Sadraey, M. H. (2017). "Aircraft performance: an engineering approach." CRC Press.

    See Also
    --------
    RCAIDE.Library.Components.Wings.Control_Surfaces.Spoiler
    RCAIDE.Library.Components.Wings.Control_Surfaces.Flap
    """
    # unpack   
    parasite_total          = state.conditions.aerodynamics.coefficients.drag.parasite.total            
    induced_total           = state.conditions.aerodynamics.coefficients.drag.induced.total            
    compressibility_total   = state.conditions.aerodynamics.coefficients.drag.compressible.total    
    form_drag               = state.conditions.aerodynamics.coefficients.drag.form.total  

    # untrimmed drag 
    CD_0  =  parasite_total + induced_total  + compressibility_total + form_drag
    
    # control surface drag 
    control_surface_drag = np.zeros_like(CD_0)
    for wing in geometry.wings: 
        for cs in wing.control_surfaces:
            if type(cs) == RCAIDE.Library.Components.Wings.Control_Surfaces.Spoiler:
                control_surface_drag += CD_0 * (0.0011 * (cs.deflection / Units.degrees))  
                state.conditions.aerodynamics.coefficients.lift.total  +=  -0.0075 *(cs.deflection / Units.degrees)  
                state.conditions.static_stability.coefficients.M       +=  0.0053 *(cs.deflection / Units.degrees)
    state.conditions.aerodynamics.coefficients.drag.trim.total =  control_surface_drag 
    return  

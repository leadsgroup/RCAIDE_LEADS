

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# RCAIDE Imports
from   RCAIDE                    import * 
from   RCAIDE.Library.Components import Wings 

# ----------------------------------------------------------------------
#  Compute asymmetry drag due to engine failure 
# ----------------------------------------------------------------------
def asymmetry_drag(state, geometry, engine_out_location = 0,  single_engine_thrust = 0,  windmilling_drag_coefficient = 0.):
    """
    Computes asymmetry drag coefficient due to engine failure and resulting trim requirements.

    Parameters
    ----------
    state : Data
        Flight conditions and aerodynamic state containing:
            - conditions.freestream.dynamic_pressure : float
                Freestream dynamic pressure [Pa]
            - conditions.aerodynamics.coefficients.drag.windmilling.total : float, optional
                Windmilling drag coefficient [unitless]
    geometry : Data
        Vehicle geometry containing:
            - reference_area : float, optional
                Reference area for drag coefficient calculation [m²]
            - mass_properties.center_of_gravity : array
                Center of gravity location [m]
            - networks : list
                List of propulsion networks containing:
                    - number_of_engines : int
                        Total number of engines [unitless]
            - wings : list
                List of wing objects containing:
                    - tag : str
                        Unique identifier for the wing
                    - sref : float
                        Reference area of the wing [m²]
                    - spans.projected : float
                        Projected span of the wing [m]
                    - aerodynamic_center : array
                        Aerodynamic center location [m]
                    - origin : array
                        Wing origin location [m]
                    - vertical : bool
                        Flag indicating if wing is vertical tail
    engine_out_location : float, optional
        Lateral distance of failed engine from aircraft centerline [m]
    single_engine_thrust : float, optional
        Thrust produced by remaining operational engine [N]
    windmilling_drag_coefficient : float, optional
        Windmilling drag coefficient for failed engine [unitless]

    Returns
    -------
    asymm_trim_drag_coefficient : float
        Asymmetry trim drag coefficient [unitless]

    Notes
    -----
    This function calculates the additional drag required to trim the aircraft when one engine
    fails, creating an asymmetric thrust condition. The calculation accounts for the drag caused by yawing
    moment created by the asymmetric thrust and the counteracting moment from the vertical tail.
    
    **Major Assumptions**
        * Two-engine aircraft configuration
        * Vertical tail provides the primary yawing moment for trim
        * Linear relationship between trim drag and asymmetric thrust moment
        * Windmilling drag contributes to the asymmetric moment
    
    **Theory**

    The asymmetry drag is calculated from the trim requirement to balance the yawing moment:

    :math:`D_{trim} = \\frac{(y_{engine})^2 (T_{single} + D_{windmilling})^2}{q_{\\infty} \\pi (h_{vt} \\cdot l_{vt})^2}`

    where:
        - :math:`y_{engine}` is the lateral distance of the failed engine [m]
        - :math:`T_{single}` is the thrust of the remaining engine [N]
        - :math:`D_{windmilling}` is the windmilling drag force [N]
        - :math:`q_{\\infty}` is the freestream dynamic pressure [Pa]
        - :math:`h_{vt}` is the vertical tail height [m]
        - :math:`l_{vt}` is the moment arm of the vertical tail [m]

    The windmilling drag force is:

    :math:`D_{windmilling} = C_{D,windmilling} \\cdot q_{\\infty} \\cdot S_{ref}`

    The asymmetry drag coefficient is:

    :math:`C_{D,asymmetry} = \\frac{D_{trim}}{q_{\\infty} \\cdot S_{ref}}`
    
    **Definitions**

    'Asymmetry Drag'
        Additional drag required to trim the aircraft when thrust is asymmetric due to engine failure.
    
    'Windmilling Drag'
        Drag produced by a failed engine that continues to rotate due to incoming airflow.
    
    'Trim Drag'
        Drag increment required to maintain aircraft equilibrium in asymmetric flight conditions.

    References
    ----------
    [1] Unknown source

    See Also
    --------
    RCAIDE.Library.Components.Wings.Main_Wing
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.windmilling_drag
    """ 
    # ==============================================
	# Unpack
    # ==============================================
    vehicle    = geometry 
    wings      = vehicle.wings
    dyn_press  = state.conditions.freestream.dynamic_pressure
    
     # Defining reference area
    if vehicle.reference_area:
            reference_area = vehicle.reference_area
    else:
        n_wing = 0
        for wing in wings:
            if not (isinstance(wing,Wings.Main_Wing) or isinstance(wing,Wings.Blended_Wing_Body)): continue
            n_wing = n_wing + 1
            reference_area = wing.sref
        if n_wing > 1:
            print(' More than one Main_Wing in the vehicle. Last one will be considered.')
        elif n_wing == 0:
            print('No Main_Wing defined! Using the 1st wing found')
            for wing in wings:
                if not isinstance(wing,Wings.Wing): continue
                reference_area = wing.sref
                break
            
    # getting cg x position
    xcg = vehicle.mass_properties.center_of_gravity[0]  
    
    # finding vertical tail
    for idx,wing in enumerate(wings):
        if not wing.vertical: continue
        vertical_idx = wing.tag
        break
    # if vertical tail not found, raise error
    try:
        vertical_idx
    except AttributeError:
        print(' No vertical tail found! Error calculating one engine inoperative drag')

    # getting vertical tail data (span, distance to cg)
    vertical_height = wings[vertical_idx].spans.projected
    vertical_dist   = wings[vertical_idx].aerodynamic_center[0] + wings[vertical_idx].origin[0][0] - xcg[0]
    
    # colculating windmilling drag
    if windmilling_drag_coefficient == 0:
        try:
            windmilling_drag_coefficient = state.conditions.aerodynamics.coefficients.drag.windmilling.total  
        except: pass
    
    windmilling_drag = windmilling_drag_coefficient * dyn_press * reference_area
    
    # calculating Drag force due to trim     
    trim_drag = (engine_out_location**2 * (single_engine_thrust+windmilling_drag)**2 ) /      \
                (dyn_press * 3.141593* (vertical_height*vertical_dist)**2)

    # Compute asymmetry trim drag coefficient
    asymm_trim_drag_coefficient = trim_drag / dyn_press / reference_area

    # dump data to state
    state.conditions.aerodynamics.coefficients.drag.asymmetry_trim.total = asymm_trim_drag_coefficient

    return asymm_trim_drag_coefficient
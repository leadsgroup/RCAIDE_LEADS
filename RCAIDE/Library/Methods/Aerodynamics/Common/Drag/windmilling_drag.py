

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# RCAIDE Imports
from  RCAIDE import * 
from  RCAIDE                    import * 
from  RCAIDE.Library.Components import Wings
from  RCAIDE.Framework.Core     import Data

# ----------------------------------------------------------------------
#  Compute drag of turbofan in windmilling condition
# ---------------------------------------------------------------------- 
def windmilling_drag(geometry,state):
    """
    Computes windmilling drag coefficient for turbofan engines in windmilling condition.

    Parameters
    ----------
    geometry : Vehicle
        Vehicle geometry object containing:
            - reference_area : float, optional
                Reference area for drag coefficient calculation [m^2]
            - wings : list
                List of wing objects with sref attribute [m^2]
            - networks : list
                List of propulsion networks containing propulsors
                    - propulsors : list
                        List of propulsor objects with nacelle attributes
                            - nacelle : Nacelle, optional
                                Nacelle object with areas.wetted attribute [m^2]
    state : State
        State object to store results in conditions.aerodynamics.coefficients.drag.windmilling

    Returns
    -------
    windmilling_drag_coefficient : float
        Windmilling drag coefficient [unitless]

    Notes
    -----
    This function calculates the windmilling drag coefficient for turbofan engines
    when they are not producing thrust but are still rotating due to incoming airflow.
    The calculation is based on empirical correlations from wind tunnel testing.
    
    **Major Assumptions**
        * Turbofan engines are in windmilling condition
        * Drag is primarily due to nacelle wetted area
        * Linear relationship between wetted area and drag coefficient
    
    **Theory**

    The windmilling drag coefficient is calculated using an empirical correlation:

    :math:`C_{D,windmilling} = 0.007274 \\frac{S_{wet,nacelle}}{S_{ref}}`

    where:
        - :math:`C_{D,windmilling}` is the windmilling drag coefficient
        - :math:`S_{wet,nacelle}` is the total wetted area of all nacelles [m²]
        - :math:`S_{ref}` is the reference area [m²]
    
    **Definitions**

    'Windmilling'
        Condition where a turbofan engine is not producing thrust but continues to rotate due to incoming airflow, typically during engine failure or shutdown scenarios.
    
    'Wetted Area'
        Total surface area of the nacelle exposed to the airflow, used as a proxy for the drag-producing surface area.

    References
    ----------
    [1] Askin, T. (2002). "Aircraft Engine Integration and Installation Effects." Virginia Tech Thesis. http://www.dept.aoe.vt.edu/~mason/Mason_f/AskinThesis2002_13.pdf

    See Also
    --------
    RCAIDE.Library.Components.Wings.Main_Wing
    """
    # ==============================================
        # Unpack
    # ==============================================
    vehicle = geometry

    # Defining reference area
    if vehicle.reference_area:
        reference_area = vehicle.reference_area
    else:
        n_wing = 0
        for wing in vehicle.wings:
            if not isinstance(wing,Wings.Main_Wing): continue
            n_wing = n_wing + 1
            reference_area = wing.sref
        if n_wing > 1:
            print(' More than one Main_Wing in the vehicle. Last one will be considered.')
        elif n_wing == 0:
            print('No Main_Wing defined! Using the 1st wing found')
            for wing in vehicle.wings:
                if not isinstance(wing,Wings.Wing): continue
                reference_area = wing.sref
                break

    # getting geometric data from engine (estimating when not available)
    swet_nac = 0

    for network in vehicle.networks: 
        for propulsor in network.propulsors:   
            if propulsor.nacelle !=  None:                
                swet_nac += propulsor.nacelle.areas.wetted 
    # Compute
    windmilling_drag_coefficient = 0.007274 * swet_nac / reference_area

    # dump data to state
    windmilling_result = Data(
        wetted_area  = swet_nac    ,
        total        = windmilling_drag_coefficient ,
    )
    state.conditions.aerodynamics.coefficients.drag.windmilling = windmilling_result

    return windmilling_drag_coefficient
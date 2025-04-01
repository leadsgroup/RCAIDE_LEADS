# RCAIDE/Library/Methods/Powertrain/Converters/Shaft_Power_Offtake/compute_shaft_power_offtaker.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------    

# package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
# compute_offtake_shaft_performance
# ----------------------------------------------------------------------------------------------------------------------     
def compute_offtake_shaft_performance(offtake_shaft, offtake_shaft_conditions, conditions):
    """
    Computes the work done from the power draw of an offtake shaft.
    
    Parameters
    ----------
    offtake_shaft : OfftakeShaft
        The offtake shaft component with the following attributes:
            - power_draw : float
                Power extracted from the main shaft [W]
            - reference_temperature : float
                Reference temperature for mass flow calculations [K]
            - reference_pressure : float
                Reference pressure for mass flow calculations [Pa]
    offtake_shaft_conditions : Conditions
        Object containing offtake shaft inlet conditions with the following attributes:
            - inputs.total_temperature_reference : numpy.ndarray
                Reference total temperature [K]
            - inputs.total_pressure_reference : numpy.ndarray
                Reference total pressure [Pa]
            - inputs.mdhc : numpy.ndarray
                Compressor nondimensional mass flow [unitless]
    conditions : Conditions
        Object containing freestream conditions (not directly used in this function)
    
    Returns
    -------
    None
        This function modifies the offtake_shaft_conditions.outputs object in-place with the following attributes:
            - power : numpy.ndarray
                Power extracted from the main shaft [W]
            - work_done : numpy.ndarray
                Work done normalized by mass flow [J/(kg/s)]
    
    Notes
    -----
    This function calculates the specific work done by the offtake shaft by dividing
    the power draw by the core mass flow rate. The core mass flow rate is computed
    using the non-dimensional mass flow parameter and reference conditions.
    
    If the power draw is zero or the mass flow is zero, the work done is set to zero.
    
    **Theory**
    
    The core mass flow rate is calculated using:
    
    .. math::
        \\dot{m}_{core} = \\dot{m}_{dhc} \\cdot \\sqrt{\\frac{T_{ref}}{T_{t,ref}}} \\cdot \\frac{P_{t,ref}}{P_{ref}}
    
    The specific work done is:
    
    .. math::
        w = \\frac{P}{\\dot{m}_{core}}
    
    where:
        - :math:`\\dot{m}_{dhc}` is the non-dimensional mass flow
        - :math:`T_{ref}` is the reference temperature
        - :math:`T_{t,ref}` is the total temperature reference
        - :math:`P_{t,ref}` is the total pressure reference
        - :math:`P_{ref}` is the reference pressure
        - :math:`P` is the power draw
    
    References
    ----------
    [1] Cantwell, B., "AA283 Course Notes", Stanford University https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_BOOK/AA283_Aircraft_and_Rocket_Propulsion_BOOK_Brian_J_Cantwell_May_28_2024.pdf

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Offtake_Shaft.append_offtake_shaft_conditions
    """
    if offtake_shaft.power_draw == 0.0:
        offtake_shaft.outputs.work_done = np.array([0.0]) 
    else: 
        # unpack 
        total_temperature_reference = offtake_shaft_conditions.inputs.total_temperature_reference
        total_pressure_reference    = offtake_shaft_conditions.inputs.total_pressure_reference
        mdhc                        = offtake_shaft_conditions.inputs.mdhc
        Tref                        = offtake_shaft.reference_temperature
        Pref                        = offtake_shaft.reference_pressure
        
        # compute core mass flow rate 
        mdot_core = mdhc * np.sqrt(Tref / total_temperature_reference) * (total_pressure_reference / Pref)

        offtake_shaft_conditions.outputs.power     = offtake_shaft.power_draw
        offtake_shaft_conditions.outputs.work_done = offtake_shaft_conditions.outputs.power / mdot_core  # normalize 
        offtake_shaft_conditions.outputs.work_done[mdot_core == 0] = 0
        
    return 
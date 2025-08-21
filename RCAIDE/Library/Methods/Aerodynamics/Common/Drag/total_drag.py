# RCAIDE/Library/Methods/Aerodynamics/Common/Drag/total_drag.py
# 
# Created:  Jun 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  Wave Drag 
# ---------------------------------------------------------------------------------------------------------------------- 
def total_drag(state,settings,geometry):
    """
    Computes the total drag coefficient by summing all individual drag components.

    Parameters
    ----------
    state : Data
        Flight conditions and aerodynamic state containing:
            - conditions.aerodynamics.coefficients.drag : Data
                Drag coefficients containing:
                    - parasite.total : float
                        Total parasite drag coefficient [unitless]
                    - induced.total : float
                        Total induced drag coefficient [unitless]
                    - compressible.total : float
                        Compressibility drag coefficient [unitless]
                    - miscellaneous.total : float
                        Miscellaneous drag coefficient [unitless]
                    - cooling.total : float
                        Cooling drag coefficient [unitless]
                    - trim.total : float
                        Trim drag coefficient [unitless]
                    - form.total : float
                        Form drag coefficient [unitless]
                    - wave.total : float
                        Wave drag coefficient [unitless]
    settings : dict
        Aerodynamic analysis settings containing:
            - trim_drag_correction_factor : float
                Correction factor for trim drag [unitless]
            - drag_coefficient_increment : float
                Additional drag coefficient increment [unitless]
    geometry : Data
        Aircraft geometry data structure

    Returns
    -------
    None
        Results are stored in state.conditions.aerodynamics.coefficients.drag.total

    Notes
    -----
    This function aggregates all drag components into a total drag coefficient for the aircraft.
    The calculation includes parasite, induced, compressibility, miscellaneous, cooling, trim,
    form, and wave drag components, with optional correction factors and increments.
    
    **Major Assumptions**
        * All drag components are additive with no overlap
        * Trim drag correction factor accounts for trim effects
        * Drag coefficient increment allows for additional corrections
        * All drag coefficients are normalized to the same reference area
    
    **Theory**

    The total drag coefficient is calculated as the sum of all drag components with corrections:

    :math:`C_{D,total} = f_{trim} \\cdot (C_{D,parasite} + C_{D,induced} + C_{D,compressible} + C_{D,miscellaneous} + C_{D,cooling} + C_{D,trim} + C_{D,form} + C_{D,wave} + \\Delta C_D)`

    where:
        - :math:`f_{trim}` is the trim drag correction factor
        - :math:`C_{D,parasite}` is the parasite drag coefficient
        - :math:`C_{D,induced}` is the induced drag coefficient
        - :math:`C_{D,compressible}` is the compressibility drag coefficient
        - :math:`C_{D,miscellaneous}` is the miscellaneous drag coefficient
        - :math:`C_{D,cooling}` is the cooling drag coefficient
        - :math:`C_{D,trim}` is the trim drag coefficient
        - :math:`C_{D,form}` is the form drag coefficient
        - :math:`C_{D,wave}` is the wave drag coefficient
        - :math:`\\Delta C_D` is the drag coefficient increment

    See Also
    --------
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_total
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.induced_drag
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.compressibility_drag
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.miscellaneous_drag
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.cooling_drag
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.trim_drag
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.form_drag
    """

    # unpack inputs    
    drag                 = state.conditions.aerodynamics.coefficients.drag

    # various drag components
    parasite_total        = drag.parasite.total            
    induced_total         = drag.induced.total            
    compressibility_total = drag.compressible.total     
    miscellaneous_drag    = drag.miscellaneous.total
    cooling_drag          = drag.cooling.total 
    trim_drag             = drag.trim.total  
    form_drag             = drag.form.total  
    wave_drag             = drag.wave.total  
 
    # total drag 
    drag.total =  settings.trim_drag_correction_factor * (parasite_total + induced_total  + compressibility_total + miscellaneous_drag \
                  + cooling_drag + trim_drag + form_drag + wave_drag     + settings.drag_coefficient_increment)  

    return  
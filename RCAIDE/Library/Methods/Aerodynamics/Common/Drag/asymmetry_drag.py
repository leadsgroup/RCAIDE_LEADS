

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
    """Computes asymmetry drag due to engine failure

    Assumptions:
    Two engine aircraft

    Source:
    Unknown source

    Inputs:
    state.conditions.freestream.dynamic_pressure                                                [Pa]
    state.conditions.aerodynamics.drag_breakdown.windmilling_drag.windmilling_drag_coefficient  [Unitless]
    geometry.
      mass_properties.center_of_gravity                                                         [m]
      networks. 
        number_of_engines                                                                       [Unitless]
      wings.
        tag                                                                                     
        sref (this function probably doesn't run)                                               [m^2]
	spans.projected                                                                         [m]
      reference_area                                                                            [m^2]

    Outputs:
    asymm_trim_drag_coefficient                                                                 [Unitless]
    (packed in state.conditions.aerodynamics.drag_breakdown.asymmetry_trim_coefficient)

    Properties Used:
    N/A
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
# RCAIDE/Library/Missions/Common/Unpack_Unknowns/control_surfaces.py
# 
# 
# Created:  Jul 2023, M. Clarke
import RCAIDE

# ----------------------------------------------------------------------------------------------------------------------
#  Unpack Unknowns
# ----------------------------------------------------------------------------------------------------------------------
def control_surfaces(segment):
    """
    Updates control surface deflections from solver unknowns

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - assigned_control_variables : Data
                Control variable configurations
                - {control_type}.active : bool
                    Whether control is active
                - {control_type}.assigned_surfaces : list
                    Surface names for each control group
            - state.unknowns : Data
                Solver unknown values
            - state.conditions.control_surfaces : Data
                Results data structure
            - analyses : list
                Analysis modules containing vehicle definition
    
    Returns
    -------
    None
        Updates segment state and vehicle model directly

    Notes
    -----
    This function applies control surface deflection values from the solver's
    unknowns to both the vehicle model and results data structure. It handles
    all types of control surfaces including elevators, slats, rudders, flaps,
    and ailerons.

    The function processes:
        1. Elevator deflections
        2. Slat deflections
        3. Rudder deflections
        4. Flap deflections
        5. Aileron deflections
        6. Spoiler deflections

    **Control Surface Types**
    
    Supported controls:
        - Elevator
        - Slat
        - Rudder
        - Flap
        - Aileron
        - Spoiler

    **Major Assumptions**
        * Valid control surface definitions
        * Proper surface assignments
        * Compatible deflection values
        * Well-defined vehicle geometry

    See Also
    --------
    RCAIDE.Library.Components.Wings.Control_Surfaces
    RCAIDE.Framework.Mission.Segments
    """
    
    
    assigned_control_variables   = segment.assigned_control_variables
    control_surfaces             = segment.state.conditions.control_surfaces
    
    for analysis in segment.analyses:
        if analysis !=  None: 
            if 'vehicle' in analysis: 
                wings =  analysis.vehicle.wings 
                # loop through wings on aircraft
                for wing in wings:
                    # Elevator Control
                    if assigned_control_variables.elevator_deflection.active:
                        for control_surface in wing.control_surfaces:
                            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator: 
                                control_surfaces.elevator.deflection  = segment.state.unknowns["elevator"]
                            
                    # Slat Control
                    if assigned_control_variables.slat_deflection.active:
                        for control_surface in wing.control_surfaces:
                            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Slat: 
                                control_surfaces.slat.deflection  = segment.state.unknowns["slat"]

                    # Rudder Control
                    if assigned_control_variables.rudder_deflection.active:
                        for control_surface in wing.control_surfaces:
                            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder: 
                                control_surfaces.rudder.deflection  = segment.state.unknowns["rudder"]
            
                    # flap Control
                    if assigned_control_variables.flap_deflection.active:
                        for control_surface in wing.control_surfaces:
                            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Flap: 
                                control_surfaces.flap.deflection  = segment.state.unknowns["flap"]
            
                    # Aileron Control
                    if assigned_control_variables.aileron_deflection.active:
                        for control_surface in wing.control_surfaces:
                            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron: 
                                control_surfaces.aileron.deflection  = segment.state.unknowns["aileron"]
    return
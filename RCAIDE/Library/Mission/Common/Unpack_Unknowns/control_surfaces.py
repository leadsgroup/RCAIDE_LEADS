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
                                num_elev_ctrls = len(assigned_control_variables.elevator_deflection.assigned_surfaces)
                                for i in range(num_elev_ctrls):   
                                    for j in range(len(assigned_control_variables.elevator_deflection.assigned_surfaces[i])):
                                        elevator_name = assigned_control_variables.elevator_deflection.assigned_surfaces[i][j]
            
                                        # set deflection on vehicle
                                        wing.control_surfaces[elevator_name].deflection = segment.state.unknowns["elevator_" + str(i)]
            
                                        # set deflection in results data structure
                                        control_surfaces.elevator.deflection  = segment.state.unknowns["elevator_" + str(i)]
                            
                    # Slat Control
                    if assigned_control_variables.slat_deflection.active:
                        for control_surface in wing.control_surfaces:
                            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Slat:
                                num_slat_ctrls = len(assigned_control_variables.slat_deflection.assigned_surfaces)
                                for i in range(num_slat_ctrls):   
                                    for j in range(len(assigned_control_variables.slat_deflection.assigned_surfaces[i])):
                                        slat_name = assigned_control_variables.slat_deflection.assigned_surfaces[i][j]
            
                                        # set deflection on vehicle
                                        wing.control_surfaces[slat_name].deflection = segment.state.unknowns["slat_" + str(i)]
            
                                        # set deflection in results data structure
                                        control_surfaces.slat.deflection  = segment.state.unknowns["slat_" + str(i)]
            
            
                    # Rudder Control
                    if assigned_control_variables.rudder_deflection.active:
                        for control_surface in wing.control_surfaces:
                            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder:
                                num_rud_ctrls = len(assigned_control_variables.rudder_deflection.assigned_surfaces)
                                for i in range(num_rud_ctrls):
                                    for j in range(len(assigned_control_variables.rudder_deflection.assigned_surfaces[i])):
                                        rudder_name = assigned_control_variables.rudder_deflection.assigned_surfaces[i][j]
            
                                        # set deflection on vehicle
                                        wing.control_surfaces[rudder_name].deflection = segment.state.unknowns["rudder_" + str(i)]
            
                                        # set deflection in results data structure
                                        control_surfaces.rudder.deflection  = segment.state.unknowns["rudder_" + str(i)]
            
                    # flap Control
                    if assigned_control_variables.flap_deflection.active:
                        for control_surface in wing.control_surfaces:
                            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Flap:
                                num_flap_ctrls = len(assigned_control_variables.flap_deflection.assigned_surfaces)
                                for i in range(num_flap_ctrls):   
                                    for j in range(len(assigned_control_variables.flap_deflection.assigned_surfaces[i])):
                                        flap_name = assigned_control_variables.flap_deflection.assigned_surfaces[i][j]
            
                                        # set deflection on vehicle
                                        wing.control_surfaces[flap_name].deflection = segment.state.unknowns["flap_" + str(i)]
            
                                        # set deflection in results data structure
                                        control_surfaces.flap.deflection  = segment.state.unknowns["flap_" + str(i)]
            
                    # Aileron Control
                    if assigned_control_variables.aileron_deflection.active:
                        for control_surface in wing.control_surfaces:
                            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron:
                                num_aile_ctrls = len(assigned_control_variables.aileron_deflection.assigned_surfaces)
                                for i in range(num_aile_ctrls):   
                                    for j in range(len(assigned_control_variables.aileron_deflection.assigned_surfaces[i])):
                                        aileron_name = assigned_control_variables.aileron_deflection.assigned_surfaces[i][j]
            
                                        # set deflection on vehicle
                                        wing.control_surfaces[aileron_name].deflection = segment.state.unknowns["aileron_" + str(i)]
            
                                        # set deflection in results data structure
                                        control_surfaces.aileron.deflection  = segment.state.unknowns["aileron_" + str(i)]


    
    return
# RCAIDE/Library/Missions/Common/Unpack_Unknowns/control_surfaces.py
# 
# 
# Created:  Jul 2023, M. Clarke
import RCAIDE

# ----------------------------------------------------------------------------------------------------------------------
#  Unpack Unknowns
# ----------------------------------------------------------------------------------------------------------------------
def control_surfaces(segment):
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
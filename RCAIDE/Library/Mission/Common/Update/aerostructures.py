# RCAIDE/Library/Missions/Common/Update/aerostructures.py
# 
# 
# Created:  Mar 2026, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  Update Aerodynamics
# ---------------------------------------------------------------------------------------------------------------------- 
def aerostructures(segment):
    """ Gets aerodynamics conditions
    
        Assumptions:
        +X out nose
        +Y out starboard wing
        +Z down

        Inputs:
            segment.analyses.aerodynamics_model                    [Function]
            aerodynamics_model.settings.maximum_lift_coefficient   [unitless]
            aerodynamics_model.vehicle.reference_area             [meter^2]
            segment.state.conditions.freestream.dynamic_pressure   [pascals]

        Outputs:
            conditions.aerodynamics.coefficients.lift.total [unitless]
            conditions.aerodynamics.coefficients.drag.total [unitless]
            conditions.frames.wind.force_vector [newtons]
            conditions.frames.wind.drag_force_vector [newtons]

        Properties Used:
        N/A
    """ 
     
    return
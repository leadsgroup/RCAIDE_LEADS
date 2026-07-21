# RCAIDE/Library/Missions/Common/Update/aerostructures.py
#
#
# Created:  Mar 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  Update Aerostructures
# ----------------------------------------------------------------------------------------------------------------------
def aerostructures(segment):
    """Evaluates the aerostructural analysis for the current segment.

    Calls the aerostructures surrogate (or direct FEA) and writes deflections
    and elastic twist into segment.state.conditions.aerostructures[wing_tag].

    Inputs:
        segment.analyses.aerostructures   [Analysis]
        segment.analyses.vehicle          [Vehicle]
        segment.state.conditions          [Conditions]

    Outputs:
        conditions.aerostructures[wing_tag].deflection    (n_cpt × n_nodes × 3)
        conditions.aerostructures[wing_tag].elastic_twist (n_cpt × n_nodes × 1)
    """

    aerostructures_model = segment.analyses.aerostructures
    if aerostructures_model is None:
        return
    if aerostructures_model.process.compute.structural is None:
        return

    _ = aerostructures_model(segment, segment.analyses.vehicle)

    return

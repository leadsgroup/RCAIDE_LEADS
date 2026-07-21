# RCAIDE/Library/Missions/Common/Pre_Process/aerostructures.py
#
#
# Created:  Jul 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE

# ----------------------------------------------------------------------------------------------------------------------
#  aerostructures
# ----------------------------------------------------------------------------------------------------------------------
def aerostructures(mission):
    """Initializes FEA surrogate models for each mission segment.

    Runs after the aerodynamics pre-process so that VLM training data
    (including FEA deflections) is already populated.  For the first segment
    the FEA surrogate is built by copying structural training data from the
    already-initialized VLM analysis.  Subsequent segments reuse the surrogate.

    Parameters
    ----------
    mission : Mission
        Mission whose segments contain analyses.aerostructures and
        analyses.aerodynamics.
    """
    last_tag = None

    for tag, segment in mission.segments.items():

        # Skip pure vertical-flight segments
        if type(segment) in (RCAIDE.Framework.Mission.Segments.Vertical_Flight.Climb,
                             RCAIDE.Framework.Mission.Segments.Vertical_Flight.Hover,
                             RCAIDE.Framework.Mission.Segments.Vertical_Flight.Descent):
            continue

        if segment.analyses.aerostructures is None:
            continue

        if last_tag is not None and 'compute' in mission.segments[last_tag].analyses.aerostructures.process.keys():
            # Reuse surrogate already built for the first segment
            prev_fea = mission.segments[last_tag].analyses.aerostructures
            segment.analyses.aerostructures.process.compute.structural = prev_fea.process.compute.structural
            segment.analyses.aerostructures.surrogates                 = prev_fea.surrogates
            segment.analyses.aerostructures.training                   = prev_fea.training

        else:
            # First segment: VLM has already been trained (including FEA data)
            segment.analyses.aerostructures.initialize(
                segment.analyses.vehicle,
                segment.analyses.aerodynamics)
            last_tag = tag

    return
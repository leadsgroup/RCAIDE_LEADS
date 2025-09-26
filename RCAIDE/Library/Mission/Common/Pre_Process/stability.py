# RCAIDE/Library/Missions/Common/Pre_Process/stability.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports  
import  RCAIDE  
# ----------------------------------------------------------------------------------------------------------------------
#  stability
# ----------------------------------------------------------------------------------------------------------------------  
def stability(mission):
    """
    Initializes and processes stability models for mission segments

    Parameters
    ----------
    mission : Mission
        The mission containing segments to be analyzed
            - analyses.stability : Analysis
                Stability analysis module
                - vehicle : Vehicle
                    Aircraft geometry definition
                    - wings : list
                        Wing geometry definitions
                - process.compute.lift.inviscid_wings : Process
                    Lift computation process
                - surrogates : Data
                    Stability surrogate models 
        
    
    Returns
    -------
    None
        Updates mission segment analyses directly
    
    Notes
    -----
    This function prepares the stability analysis for each mission segment.
    It ensures proper wing geometry computation and manages stability
    surrogate models across segments for computational efficiency.

    The function performs the following steps:
        1. Computes wing planform properties
        2. Reuses previous segment's stability data when possible
        3. Initializes new stability analyses when needed

    **Wing Processing**
    
    For each wing: 
        - Uses wing_planform

    **Major Assumptions**
        * Valid wing geometry definitions
        * Compatible stability models between segments
        * Proper initialization of first segment
        * Continuous stability characteristics

    See Also
    --------
    RCAIDE.Library.Methods.Geometry.Planform
    RCAIDE.Framework.Mission.Segments
    """
    last_tag = None
    for tag,segment in mission.segments.items():

        if type(segment) ==  RCAIDE.Framework.Mission.Segments.Vertical_Flight.Climb or  \
           type(segment) ==  RCAIDE.Framework.Mission.Segments.Vertical_Flight.Hover or \
           type(segment) ==  RCAIDE.Framework.Mission.Segments.Vertical_Flight.Descent:
            pass
        else:    
            if segment.analyses.stability !=  None: 
                if last_tag!=  None and 'compute' in mission.segments[last_tag].analyses.stability.process.keys():  
                    segment.analyses.stability.surrogates                       = mission.segments[last_tag].analyses.aerodynamics.surrogates    
                    segment.analyses.stability.process.compute.static_stability = mission.segments[last_tag].analyses.stability.process.compute.static_stability
                    segment.analyses.stability.vehicle.neutral_point            = mission.segments[last_tag].analyses.stability.vehicle.neutral_point
                else: 
                    segment.analyses.stability.surrogates   = segment.analyses.aerodynamics.surrogates  
                    segment.analyses.stability.initialize() 
                    last_tag = tag 
    return 
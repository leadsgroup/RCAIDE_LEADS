# RCAIDE/Library/Missions/Common/Pre_Process/aerodynamics.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
import  RCAIDE  
import os, sys

# ----------------------------------------------------------------------------------------------------------------------
#  aerodynamics
# ----------------------------------------------------------------------------------------------------------------------  
def aerodynamics(mission):
    """
    Initializes and processes aerodynamic models for mission segments

    Parameters
    ----------
    mission : Mission
        The mission containing segments to be analyzed
            - analyses.aerodynamics : Analysis
                Aerodynamic analysis module
                - vehicle : Vehicle
                    Aircraft geometry definition
                    - wings : list
                        Wing geometry definitions
                - process.compute.lift.inviscid_wings : Process
                    Lift computation process
                - surrogates : Data
                    Aerodynamic surrogate models 

    Notes
    -----
    This function prepares the aerodynamic analysis for each mission segment.
    It ensures proper wing geometry computation and manages aerodynamic
    surrogate models across segments for computational efficiency.

    The function performs the following steps:
        1. Computes wing planform properties
        2. Reuses previous segment's aerodynamic data when possible
        3. Initializes new aerodynamic analyses when needed

    **Wing Processing**
    
    For each wing:
        - Uses wing_planform

    **Major Assumptions**
        * Valid wing geometry definitions
        * Compatible aerodynamic models between segments
        * Proper initialization of first segment
        * Continuous aerodynamic characteristics

    Returns
    -------
    None
        Updates mission segment analyses directly

    See Also
    --------
    RCAIDE.Library.Methods.Geometry.Planform
    """                    
    last_tag = None
    for tag,segment in mission.segments.items(): 
        if type(segment) ==  RCAIDE.Framework.Mission.Segments.Vertical_Flight.Climb or  \
           type(segment) ==  RCAIDE.Framework.Mission.Segments.Vertical_Flight.Hover or \
           type(segment) ==  RCAIDE.Framework.Mission.Segments.Vertical_Flight.Descent:
            pass
        else:        
            if segment.analyses.aerodynamics != None:
                if last_tag!=  None:
                    if segment.analyses.aerodynamics.settings.unique_segment_surrogate:
                        aero   = segment.analyses.aerodynamics
                        aero.filename =  os.path.join(os.path.dirname(os.path.abspath(sys.argv[0])), mission.tag + "_" + segment.tag +"_aerodynamic_training_data.pkl")
                        aero.initialize()   
                        last_tag = tag
                    else:
                        if 'compute' in mission.segments[last_tag].analyses.aerodynamics.process.keys(): 
                            segment.analyses.aerodynamics.process.compute.lift.inviscid_wings = mission.segments[last_tag].analyses.aerodynamics.process.compute.lift.inviscid_wings
                            segment.analyses.aerodynamics.surrogates                          = mission.segments[last_tag].analyses.aerodynamics.surrogates  
                            segment.analyses.aerodynamics.settings.vortex_distribution        = mission.segments[last_tag].analyses.aerodynamics.settings.vortex_distribution 
                else: 
                    aero   = segment.analyses.aerodynamics
                    aero.filename =  os.path.join(os.path.dirname(os.path.abspath(sys.argv[0])), mission.tag + "_" + segment.tag +"_aerodynamic_training_data.pkl")
                    aero.initialize()   
                    last_tag = tag  
    return 
# RCAIDE/Library/Missions/Common/Pre_Process/stability.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports  
import  RCAIDE 
from RCAIDE.Library.Methods.Geometry.Planform  import wing_planform
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
                - reference_values : Data
                    Reference stability parameters
        
    
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
        - If multi-segmented: Uses wing_segmented_planform
        - If single segment: Uses wing_planform

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
                
        if segment.analyses.stability !=  None: 
            # ensure all properties of wing are computed before drag calculations  
            vehicle =  segment.analyses.stability.vehicle
            for wing in  vehicle.wings:  
                wing_planform(wing)
                    
            if  (last_tag!=  None) and  ('compute' in mission.segments[last_tag].analyses.stability.process.keys()): 
                segment.analyses.stability.process.compute.lift.inviscid_wings = mission.segments[last_tag].analyses.stability.process.compute.lift.inviscid_wings
                segment.analyses.stability.surrogates       = mission.segments[last_tag].analyses.stability.surrogates 
                segment.analyses.stability.reference_values = mission.segments[last_tag].analyses.stability.reference_values   
            else: # use aerodynamic results that have been previously processed 
                if (type(segment.analyses.aerodynamics) == RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method) or\
                (type(segment.analyses.aerodynamics) == RCAIDE.Framework.Analyses.Aerodynamics.Athena_Vortex_Lattice) :
                    segment.analyses.stability.process.compute.lift.inviscid_wings = segment.analyses.aerodynamics.process.compute.lift.inviscid_wings 
                    segment.analyses.stability.surrogates       = segment.analyses.aerodynamics.surrogates 
                    segment.analyses.stability.reference_values = segment.analyses.aerodynamics.reference_values 
                    last_tag = tag                 
                else: # run new simulation 
                    stab = segment.analyses.stability
                    stab.initialize() 
                    last_tag = tag 
    return 
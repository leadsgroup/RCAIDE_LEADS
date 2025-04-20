# RCAIDE/Library/Missions/Common/Pre_Process/geometry.py
# 
# 
# Created:  Apr 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Library.Methods.Geometry.Planform  import  wing_planform
from copy import deepcopy

# ----------------------------------------------------------------------------------------------------------------------
#  aerodynamics
# ----------------------------------------------------------------------------------------------------------------------  
def geometry(mission):
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
                - reference_values : Data
                    Reference aerodynamic parameters

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
        if segment.analyses.aerodynamics != None:
            if last_tag!=  None:
                segment.analyses.aerodynamics.vehicle = vehicle 
            else: 
                vehicle  =  segment.analyses.aerodynamics.vehicle 
                
                # ensure all properties of wing are computed before drag calculations  
                for wing in vehicle.wings: 
                    if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing) or isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body): 
                        # compute planform properties 
                        wing_planform(wing, overwrite_reference=True) 
                        
                        vehicle.reference_area = wing.areas.reference
                    else:
                        # compute planform properties 
                        wing_planform(wing, overwrite_reference=True)  
                last_tag = tag 

            # update weights vehicle with correct geometric properties                  
            if segment.analyses.weights != None:
                segment.analyses.weights.vehicle = vehicle
    return 
# RCAIDE/Library/Missions/Segments/expand_state.py
# 
# 
# Created:  Jul 2023, M. Clarke   

# ----------------------------------------------------------------------------------------------------------------------
# Expand State
# ----------------------------------------------------------------------------------------------------------------------  
def expand_state(segment):
    
    """
    Expands all state vectors to match number of control points

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
             - state:
                numerics:
                    number_of_control_points : int
                        Number of discretization points [-]
                expand_rows : function
                    Method to expand state containers

    Returns
    -------
    None

    Notes
    -----
    This function ensures all state vectors in the segment have consistent dimensions
    by expanding them to match the number of control points used for discretization.
       
    **Calculation Process**
        1. Get required vector size from numerics
        2. Call expand_rows to resize all state containers:
            - conditions
            - unknowns
            - residuals
            - differentials

    **Major Assumptions**
        * All state vectors should have same length
        * Expansion preserves vector values
        * State containers support expand_rows method
        * Control points already properly set

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """     

    n_points = segment.state.numerics.number_of_control_points
    
    segment.state.expand_rows(n_points)
    
    return
    
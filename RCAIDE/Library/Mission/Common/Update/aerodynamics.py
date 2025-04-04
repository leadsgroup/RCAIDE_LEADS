# RCAIDE/Library/Missions/Common/Update/aerodynamics.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  Update Aerodynamics
# ----------------------------------------------------------------------------------------------------------------------
def aerodynamics(segment):
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
    
    # unpack
    conditions         = segment.state.conditions
    aerodynamics_model = segment.analyses.aerodynamics
    q                  = segment.state.conditions.freestream.dynamic_pressure
    Sref               = aerodynamics_model.vehicle.reference_area
    CLmax              = aerodynamics_model.settings.maximum_lift_coefficient 
    MAC                = aerodynamics_model.vehicle.wings.main_wing.chords.mean_aerodynamic
    span               = aerodynamics_model.vehicle.wings.main_wing.spans.projected 
    
    # call aerodynamics model
    _ = aerodynamics_model(segment)     

    # Forces 
    CL = conditions.aerodynamics.coefficients.lift.total
    CD = conditions.aerodynamics.coefficients.drag.total
    CY = conditions.static_stability.coefficients.Y

    CL[q<=0.0] = 0.0
    CD[q<=0.0] = 0.0
    CL[CL>CLmax] = CLmax
    CL[CL< -CLmax] = -CLmax

    # dimensionalize
    F      = segment.state.ones_row(3) * 0.0
    F[:,2] = ( -CL * q * Sref )[:,0]
    F[:,1] = ( -CY * q * Sref )[:,0]
    F[:,0] = ( -CD * q * Sref )[:,0]

    # rewrite aerodynamic CL and CD
    conditions.aerodynamics.coefficients.lift.total  = CL
    conditions.aerodynamics.coefficients.drag.total  = CD
    conditions.frames.wind.force_vector[:,:]   = F[:,:]

    # -----------------------------------------------------------------
    # Moments
    # -----------------------------------------------------------------
    C_M = conditions.static_stability.coefficients.M
    C_L = conditions.static_stability.coefficients.L
    C_N = conditions.static_stability.coefficients.N

    C_M[q<=0.0] = 0.0

    # dimensionalize
    M      = segment.state.ones_row(3) * 0.0
    M[:,0] = (C_L[:,0] * q[:,0] * Sref * span)
    M[:,1] = (C_M[:,0] * q[:,0] * Sref * MAC)
    M[:,2] = (C_N[:,0] * q[:,0] * Sref * span)

    # pack conditions
    conditions.frames.wind.moment_vector[:,:] = M[:,:] 

    return
# RCAIDE/Library/Missions/Common/Update/Aerodynamics.py
# 
# 
# Created:  Jul 2023, M. Clarke
# Modified: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
import numpy as np

from RCAIDE.Framework.Core import array_namespace 
    
# ----------------------------------------------------------------------------------------------------------------------
#  Update Freestream
# ----------------------------------------------------------------------------------------------------------------------
def freestream(segment):
    """ Updates the freestream conditions 
        
        Assumptions:
        N/A
        
        Inputs:
            segment.state.conditions.:
                 frames.inertial.velocity_vector   [m/s]
                 freestream.density                [kg/m^3]
                 freestream.speed_of_sound         [m/s]
                 freestream.dynamic_viscosity      [Pa-s]
        Outputs:
            segment.conditions.freestream
                 velocity                          [m/s]
                 mach_number                       [-]
                 reynolds_number                   [-]
                 dynamic_pressure                  [Pa] 
      
        Properties Used:
        N/A
                    
    """   
    
    # unpack
    conditions = segment.state.conditions
    V          = conditions.frames.inertial.velocity_vector
    rho        = conditions.freestream.density
    a          = conditions.freestream.speed_of_sound
    mu         = conditions.freestream.dynamic_viscosity

    # velocity magnitude
    xp    = array_namespace(V)
    Vmag2 = xp.sum( V**2, axis=1)[:,None]  
    Vmag  = xp.sqrt(Vmag2)

    # dynamic pressure
    q = 0.5 * rho * Vmag2 # Pa

    # Mach number
    M = Vmag / a

    # Reynolds number
    Re = rho * Vmag / mu  # per m

    # pack
    conditions.freestream.velocity         = Vmag
    conditions.freestream.mach_number      = M
    conditions.freestream.reynolds_number  = Re
    conditions.freestream.dynamic_pressure = q

    return


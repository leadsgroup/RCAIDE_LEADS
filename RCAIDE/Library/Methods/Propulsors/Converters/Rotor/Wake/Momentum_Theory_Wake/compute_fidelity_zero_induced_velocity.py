## @defgroup Methods-Propulsion-Rotor_Wake-Fidelity_Zero
# compute_fidelity_zero_induced_velocity.py
# 
# Created:  Jun 2021, R. Erhard 

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# package imports
import numpy as np 
from scipy.interpolate import interp1d

## @defgroup Methods-Propulsion-Rotor_Wake-Fidelity_Zero
def compute_fidelity_zero_induced_velocity(rotor,rotor_conditions,evaluation_points,ctrl_pts, identical_flag=False):  
    """ This computes the velocity induced by the fidelity zero wake
    on specified evaluation points.

    Assumptions:  
       The wake contracts following formulation by McCormick.
    
    Source:   
       Contraction factor: McCormick 1969, Aerodynamics of V/STOL Flight
       
    Inputs: 
       evaluation_points.
          XC              - X-location of evaluation points (vehicle frame)             [m] 
          YC              - Y-location of evaluation points (vehicle frame)             [m] 
          ZC              - Z-location of evaluation points (vehicle frame)             [m] 
          
       geometry    - SUAVE vehicle                                 [Unitless] 
       cpts        - control points in segment                     [Unitless]
    Properties Used:
       N/A
    """

    # extract vortex distribution
    n_cp = len(evaluation_points.XC)
    
    # initialize rotor wake induced velocities
    rotor_V_wake_ind = np.zeros((ctrl_pts,n_cp,3))
       
    R            = rotor.tip_radius
    r            = rotor_conditions.disc_radial_distribution[0,:,0]
    
    # Ignore points within hub or outside tip radius
    hub_y_center = rotor.origin[0][1]
    inboard_r    = np.flip(hub_y_center - r) 
    outboard_r   = hub_y_center + r 
    rotor_y_range = np.append(inboard_r, outboard_r)

    # within this range, add an induced x- and z- velocity from rotor wake
    bool_inboard  = ( evaluation_points.YC > inboard_r[0] )  * ( evaluation_points.YC < inboard_r[-1] )
    bool_outboard = ( evaluation_points.YC > outboard_r[0] ) * ( evaluation_points.YC < outboard_r[-1] )
    bool_in_range = bool_inboard + bool_outboard
    YC_in_range   = evaluation_points.YC[bool_in_range]

    y_vals  = YC_in_range
    val_ids = np.where(bool_in_range==True)
    
    s  = evaluation_points.XC[val_ids] - rotor.origin[0][0]
    kd = 1 + s/(np.sqrt(s**2 + R**2))    

    # extract radial and azimuthal velocities at blade
    va = rotor_conditions.blade_axial_induced_velocity[0]
    vt = rotor_conditions.blade_tangential_induced_velocity[0]
    
    
    va_y_range  = np.append(np.flipud(va), va)
    vt_y_range  = np.append(np.flipud(vt), vt)*rotor.rotation
    va_interp   = interp1d(rotor_y_range, va_y_range)
    vt_interp   = interp1d(rotor_y_range, vt_y_range)
    
    
    # preallocate va_new and vt_new
    va_new = kd*va_interp((y_vals))
    vt_new = np.zeros(np.size(val_ids))
    
    # invert inboard vt values
    inboard_bools                = (y_vals < hub_y_center)
    vt_new[inboard_bools]        = -kd[inboard_bools]*vt_interp((y_vals[inboard_bools]))
    vt_new[inboard_bools==False] = kd[inboard_bools==False]*vt_interp((y_vals[inboard_bools==False]))
    
    rotor_V_wake_ind[0,val_ids,0] = va_new  # axial induced velocity
    rotor_V_wake_ind[0,val_ids,1] = 0       # spanwise induced velocity; in line with rotor, so 0
    rotor_V_wake_ind[0,val_ids,2] = vt_new  # vertical induced velocity     
    
    return rotor_V_wake_ind
  
  
# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/train_VLM_surrogates.py
#
# Created: Aug 2025, M. Clarke
# Modified: Oct 2026, H. Hussien

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core import  Data, Units
from RCAIDE.Library.Plots import *
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.VLM   import VLM
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.compute_zero_lift_moment import compute_zero_lift_moment
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.control_surface_registry import lookup as cs_lookup, CONTROL_SURFACE_TYPES, HIGH_LIFT_LETTERS, HIGH_LIFT_DRAG_TERMS
from copy import deepcopy

# package imports
import numpy  as np

# ----------------------------------------------------------------------------------------------------------------------
#  Vortex_Lattice
# ---------------------------------------------------------------------------------------------------------------------- 
SIDESLIP_AND_RATE_DERIVATIVES = ["dCY_dbeta", "dCL_dbeta", "dCN_dbeta", "dCZ_dq", "dCM_dq", "dCY_dp", "dCL_dp", "dCN_dp", "dCY_dr", "dCL_dr", "dCN_dr"]

def train_VLM_surrogates(aerodynamics, vehicle):
    """Call methods to run VLM for sample point evaluation. 
    
    Assumptions:
        CY_beta multiplied by -1,  
        CN Rudder derivatives multiplied by -1, verified against literature (this is not multiplied here but is in the VLM.py)
        
    Source:
        None

    Args:
        aerodynamics       : VLM analysis          [unitless] 
        
    Returns: 
        None    
    """
 
    Mach          = aerodynamics.training.Mach 
    training      = aerodynamics.training  
    sub_len       = int(sum(Mach<1.))  
    sub_Mach      = Mach[:sub_len] 
    sup_Mach      = Mach[sub_len:] 

    training.subsonic    =  train_model(aerodynamics, sub_Mach, vehicle)

    # supersonic/transonic surrogates are trained on first need (see train_VLM_supersonic_surrogates)
    training.supersonic  = None
    training.transonic   = None
    return 

def train_VLM_supersonic_surrogates(aerodynamics, vehicle):
    """Trains the supersonic and transonic surrogates; called the first time a flight condition reaches
    the subsonic smoothing limit, so subsonic-only missions never pay for them.

    Args:
        aerodynamics       : VLM analysis, already trained subsonic
        vehicle            : vehicle the subsonic surrogates were trained on

    Returns:
        None
    """
    Mach          = aerodynamics.training.Mach
    training      = aerodynamics.training
    sub_len       = int(sum(Mach<1.))
    sub_Mach      = Mach[:sub_len]
    sup_Mach      = Mach[sub_len:]
    if len(sup_Mach) > 2:
        training.supersonic  =  train_model(aerodynamics, sup_Mach, vehicle)
        training.transonic   =  train_trasonic_model(aerodynamics, training.subsonic,training.supersonic,sub_Mach, sup_Mach, vehicle)
    return
    
def train_model(aerodynamics,Mach, vehicle): 
    """Sub function that call methods to run VLM for sample point evaluation. 
    
    Assumptions:
        None
        
    Source:
        None

    Args:
        aerodynamics       : VLM analysis          [unitless] 
        
    Returns: 
        None    
    """
    settings       = aerodynamics.settings
    AoA            = aerodynamics.training.angle_of_attack                  
    training       = Data()
    training.Mach  = Mach 
    
    # loop through wings to determine what control surfaces are present
    delta_0     = {}
    delta_train = {}
    for wing in vehicle.wings:
        for control_surface in wing.control_surfaces:
            for letter, name, channel, flag, deflection_attr in cs_lookup(control_surface):
                delta_0[letter]     = getattr(control_surface, deflection_attr)
                delta_train[letter] = getattr(aerodynamics.training, channel + '_deflection')
                setattr(aerodynamics, flag, True)
            control_surface.deflection           = 0 # set all control surfaces to be 0
            control_surface.secondary_deflection = 0
             
    len_Mach       = len(Mach)        
    len_AoA        = len(AoA)  
    
    # --------------------------------------------------------------------------------------------------------------
    # Alpha
    # --------------------------------------------------------------------------------------------------------------
    
    # Setup new array shapes for vectorization 
    # stakcing 9x9 matrices into one horizontal line(81)  
    AoAs       = np.atleast_2d(np.tile(AoA,len_Mach).T.flatten()).T 
    Machs      = np.atleast_2d(np.repeat(Mach,len_AoA)).T      
    conditions                                      = RCAIDE.Framework.Mission.Common.Results() 
    conditions.freestream.mach_number               = Machs
    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs)*AoAs 
    conditions.aerodynamics.angles.beta             = np.zeros_like(Machs)*AoAs  
    conditions.freestream.velocity                  = np.zeros_like(Machs)*AoAs  
    conditions.static_stability.pitch_rate          = np.zeros_like(Machs)*AoAs  
    conditions.static_stability.roll_rate           = np.zeros_like(Machs)*AoAs  
    conditions.static_stability.yaw_rate            = np.zeros_like(Machs)*AoAs 
   
    clean_wing_vehicle = deepcopy(vehicle)
    for wing in clean_wing_vehicle.wings:
        wing.control_surfaces = []
    VLM_results      = call_VLM(conditions,settings,clean_wing_vehicle)
    Clift_res        = VLM_results.CLift
    Cdrag_res        = VLM_results.CDrag_induced
    CX_res           = VLM_results.CX
    CY_res           = VLM_results.CY
    CZ_res           = VLM_results.CZ
    CL_res           = VLM_results.CL
    CM_res           = VLM_results.CM
    CN_res           = VLM_results.CN   

    training.Clift_spanwise  = VLM_results.sectional_CLift.reshape(len_Mach, len_AoA, np.shape(VLM_results.sectional_CLift)[1]).transpose(1, 0, 2)
    training.gamma_alpha     = VLM_results.gamma.reshape(len_Mach, len_AoA, np.shape(VLM_results.gamma)[1]).transpose(1, 0, 2)
    Clift_alpha              = np.reshape(Clift_res,(len_Mach,len_AoA)).T 
    Cdrag_induced_alpha      = np.reshape(Cdrag_res,(len_Mach,len_AoA)).T 
    CX_alpha                 = np.reshape(CX_res,(len_Mach,len_AoA)).T 
    CY_alpha                 = np.reshape(CY_res,(len_Mach,len_AoA)).T 
    CZ_alpha                 = np.reshape(CZ_res,(len_Mach,len_AoA)).T 
    CL_alpha                 = np.reshape(CL_res,(len_Mach,len_AoA)).T 
    CM_alpha                 = np.reshape(CM_res,(len_Mach,len_AoA)).T 
    CN_alpha                 = np.reshape(CN_res,(len_Mach,len_AoA)).T  
    
    # zero-lift moment from cases at +/- the reference perturbation (a VLM case at exactly zero incidence is noisy
    # for untwisted wings)
    reference     = reference_coefficients(aerodynamics, Mach, settings, clean_wing_vehicle)
    # STABILITY COEFFICIENTS  
    training.Clift_alpha               = Clift_alpha  
    training.Cdrag_induced_alpha       = Cdrag_induced_alpha   
    training.CX_alpha                  = CX_alpha
    training.CY_alpha                  = CY_alpha 
    training.CZ_alpha                  = CZ_alpha  
    training.CL_alpha                  = CL_alpha   
    training.CM_alpha                  = CM_alpha 
    training.CN_alpha                  = CN_alpha    
    training.CM_zero_lift              = reference.CM_zero_lift
            
    # derivatives at each training angle of attack
    training.update(sideslip_and_rate_derivatives(aerodynamics, Mach, settings, clean_wing_vehicle))

    # control surface derivatives: one deflected run per channel against a single undeflected reference
    # shared by all channels; see control_surface_registry.py for why every type is treated identically here.
    has_control_surfaces = any(len(wing.control_surfaces) > 0 for wing in vehicle.wings)
    if has_control_surfaces:
        VLM_ref = call_VLM(control_surface_conditions(Mach),settings,vehicle)  # all deflections are 0 here
    for wing in vehicle.wings:
        for control_surface in wing.control_surfaces:
            for letter, name, channel, flag, deflection_attr in cs_lookup(control_surface):
                delta = delta_train[letter][0]
                setattr(control_surface, deflection_attr, delta)
                VLM_results = call_VLM(control_surface_conditions(Mach),settings,vehicle)
                training['dClift_ddelta_' + letter] = (VLM_results.CLift[:,0]         - VLM_ref.CLift[:,0]        ) / delta
                training['dCdrag_ddelta_' + letter] = (VLM_results.CDrag_induced[:,0] - VLM_ref.CDrag_induced[:,0]) / delta
                training['dCX_ddelta_'    + letter] = (VLM_results.CX[:,0]            - VLM_ref.CX[:,0]           ) / delta
                training['dCY_ddelta_'    + letter] = (VLM_results.CY[:,0]            - VLM_ref.CY[:,0]           ) / delta
                training['dCZ_ddelta_'    + letter] = (VLM_results.CZ[:,0]            - VLM_ref.CZ[:,0]           ) / delta
                training['dCL_ddelta_'    + letter] = (VLM_results.CL[:,0]            - VLM_ref.CL[:,0]           ) / delta
                training['dCM_ddelta_'    + letter] = (VLM_results.CM[:,0]            - VLM_ref.CM[:,0]           ) / delta
                training['dCN_ddelta_'    + letter] = (VLM_results.CN[:,0]            - VLM_ref.CN[:,0]           ) / delta
                # reset to 0, not the real deflection, so later surfaces train in isolation
                setattr(control_surface, deflection_attr, 0)

    # high-lift induced drag at two deflections and two angles of attack (see train_high_lift_induced_drag)
    if has_control_surfaces:
        training.update(train_high_lift_induced_drag(settings, vehicle, Mach, VLM_ref.CDrag_induced[:,0]))

    # restore real deflections now that training is done
    for wing in vehicle.wings:
        for control_surface in wing.control_surfaces:
            for letter, name, channel, flag, deflection_attr in cs_lookup(control_surface):
                setattr(control_surface, deflection_attr, delta_0[letter])

    # reset vortex distribution after training
    # restore the clean-wing geometry in place: segments share this object, and the control-surface
    # sweeps above left the last deflected geometry in it (read by e.g. transonic_lift_wave_drag)
    return training

def train_trasonic_model(aerodynamics, training_subsonic,training_supersonic,sub_Mach, sup_Mach, vehicle): 
    """Sub function that call methods to run VLM for sample point evaluation. 
    
    Assumptions:
        None
        
    Source:
        None

    Args:
        aerodynamics       : VLM analysis          [unitless] 
        
    Returns: 
        None    
    """     
    training       = Data() 
    training.Mach  = np.array([sub_Mach[-1], sup_Mach[0]])
    
    # --------------------------------------------------------------------------------------------------------------
    # Alpha
    # --------------------------------------------------------------------------------------------------------------  
    Clift_alpha           =  np.concatenate((training_subsonic.Clift_alpha[:,-1][:,None] , training_supersonic.Clift_alpha[:,0][:,None] ), axis = 1)
    Clift_spanwise        =np.concatenate((training_subsonic.Clift_spanwise[:,-1][:,None] , training_supersonic.Clift_spanwise[:,0][:,None] ), axis = 1)
    if 'gamma_alpha' in training_subsonic and 'gamma_alpha' in training_supersonic:
        training.gamma_alpha = np.concatenate((training_subsonic.gamma_alpha[:,-1][:,None], training_supersonic.gamma_alpha[:,0][:,None]), axis = 1)
    Cdrag_induced_alpha   =  np.concatenate((training_subsonic.Cdrag_induced_alpha[:,-1][:,None]  , training_supersonic.Cdrag_induced_alpha[:,0][:,None] ), axis = 1) 
    CX_alpha              =  np.concatenate((training_subsonic.CX_alpha[:,-1][:,None]    , training_supersonic.CX_alpha[:,0][:,None] ), axis = 1)   
    CY_alpha              =  np.concatenate((training_subsonic.CY_alpha[:,-1][:,None]    , training_supersonic.CY_alpha[:,0][:,None] ), axis = 1)   
    CZ_alpha              =  np.concatenate((training_subsonic.CZ_alpha[:,-1][:,None]    , training_supersonic.CZ_alpha[:,0][:,None] ), axis = 1)   
    CL_alpha              =  np.concatenate((training_subsonic.CL_alpha[:,-1][:,None]    , training_supersonic.CL_alpha[:,0][:,None] ), axis = 1)   
    CM_alpha              =  np.concatenate((training_subsonic.CM_alpha[:,-1][:,None]    , training_supersonic.CM_alpha[:,0][:,None] ), axis = 1)   
    CN_alpha              =  np.concatenate((training_subsonic.CN_alpha[:,-1][:,None]    , training_supersonic.CN_alpha[:,0][:,None] ), axis = 1) 
    CM_zero_lift          =  np.concatenate((training_subsonic.CM_zero_lift[-1,None], training_supersonic.CM_zero_lift[0,None]))
 
    # STABILITY COEFFICIENTS 
    training.Clift_alpha               = Clift_alpha 
    training.Cdrag_induced_alpha       = Cdrag_induced_alpha   
    training.CX_alpha                  = CX_alpha   
    training.CY_alpha                  = CY_alpha
    training.CZ_alpha                  = CZ_alpha  
    training.CL_alpha                  = CL_alpha
    training.CM_alpha                  = CM_alpha  
    training.CN_alpha                  = CN_alpha
    training.CM_zero_lift              = CM_zero_lift
    training.Clift_spanwise            = Clift_spanwise
    
    
    # sideslip and rate derivatives carried from the bounding subsonic and supersonic Mach, like the tables above
    for key in SIDESLIP_AND_RATE_DERIVATIVES:
        training[key] = np.concatenate((training_subsonic[key][:,-1:], training_supersonic[key][:,:1]), axis=1)

    # for control surfaces, subtract influence WITHOUT control surface deflected from coefficients WITH control
    # surface deflected; see control_surface_registry.py for why every type is treated identically here.
    for cls, letter, name, channel, flag, deflection_attr in CONTROL_SURFACE_TYPES:
        if getattr(aerodynamics, flag):
            for coeff in ('Clift', 'Cdrag', 'CX', 'CY', 'CZ', 'CL', 'CM', 'CN'):
                key = 'd' + coeff + '_ddelta_' + letter
                training[key] = np.array([training_subsonic[key][-1], training_supersonic[key][0]])
    for letter in HIGH_LIFT_LETTERS:
        for term in HIGH_LIFT_DRAG_TERMS:
            key = 'high_lift_Cdrag_' + term + '_' + letter
            if key in training_subsonic and key in training_supersonic:
                training[key] = np.array([training_subsonic[key][-1], training_supersonic[key][0]])

    return training


def train_high_lift_induced_drag(settings, vehicle, Mach, CDi_reference):
    """
    Trains the induced-drag increment of each flap and slat as
    dCDi = p*delta + q*delta^2 + (r0 + r1*delta)*alpha*delta, from VLM runs at two deflections (15, 30 deg)
    and two angles of attack (0, 5 deg). Trefftz-plane induced drag is a quadratic form in the circulation,
    which the VLM solves linearly in alpha and delta; the alpha*delta cross term with the wing's own lift
    dominates at landing incidence and is missed by a zero-incidence derivative. r1 absorbs the departure from
    a pure quadratic at large deflection. All control surfaces must be undeflected on entry.

    Consistent with Roskam, Airplane Design Part VI (DARcorporation, 2004 printing), Sec. 4.6.2, where induced drag is computed from the
    total flaps-down lift coefficient and Eq. 4.74 adds only the span-loading term K^2 dCL_flap^2 cos(sweep):
    on an E175-like wing (flap 0.2-0.7 span, A 8.5, Fig. 4.53 K = 0.20) that build-up is within 7-12% of the
    VLM induced drag this model reproduces.

    Parameters
    ----------
    settings : Data
        Vortex lattice settings.
    vehicle : RCAIDE.Vehicle
        Vehicle with its control surfaces undeflected.
    Mach : numpy.ndarray
        Training Mach numbers [-].
    CDi_reference : numpy.ndarray
        Undeflected induced drag at zero incidence for each Mach number [-].

    Returns
    -------
    training : Data
        high_lift_Cdrag_<term>_<letter> for each term in HIGH_LIFT_DRAG_TERMS and each flap or slat letter.
    """
    training  = Data()
    high_lift = [(control_surface, letter, deflection_attr) for wing in vehicle.wings for control_surface in wing.control_surfaces
                 for letter, name, channel, flag, deflection_attr in cs_lookup(control_surface) if letter in HIGH_LIFT_LETTERS]
    if len(high_lift) == 0:
        return training
    alpha_training         = 5.0 * Units.deg
    delta_a, delta_b       = 15.0 * Units.deg, 30.0 * Units.deg
    conditions_alpha       = control_surface_conditions(Mach)
    conditions_alpha.aerodynamics.angles.alpha[:] = alpha_training
    CDi_alpha              = call_VLM(conditions_alpha,settings,vehicle).CDrag_induced[:,0]
    for control_surface, letter, deflection_attr in high_lift:
        increment = {}
        for delta in (delta_a, delta_b):
            setattr(control_surface, deflection_attr, delta)
            increment[delta] = (call_VLM(control_surface_conditions(Mach),settings,vehicle).CDrag_induced[:,0] - CDi_reference,
                                call_VLM(conditions_alpha,settings,vehicle).CDrag_induced[:,0] - CDi_alpha)
            setattr(control_surface, deflection_attr, 0)
        q   = (increment[delta_b][0] / delta_b - increment[delta_a][0] / delta_a) / (delta_b - delta_a)
        p   = increment[delta_a][0] / delta_a - q * delta_a
        r_a = (increment[delta_a][1] - increment[delta_a][0]) / (alpha_training * delta_a)
        r_b = (increment[delta_b][1] - increment[delta_b][0]) / (alpha_training * delta_b)
        r1  = (r_b - r_a) / (delta_b - delta_a)
        r0  = r_a - r1 * delta_a
        for term, value in zip(HIGH_LIFT_DRAG_TERMS, (p, q, r0, r1)):
            training['high_lift_Cdrag_' + term + '_' + letter] = value
    return training


def reference_coefficients(aerodynamics, Mach, settings, vehicle):
    """Coefficients at zero incidence per Mach, the mean of VLM cases at +/- training.reference_angle_perturbation,
    and the pitching moment at zero lift from the same cases."""
    delta                                  = aerodynamics.training.reference_angle_perturbation
    AoAs                                   = np.atleast_2d(np.tile([delta, -delta], len(Mach))).T
    Machs                                  = np.atleast_2d(np.repeat(Mach, 2)).T
    conditions                             = RCAIDE.Framework.Mission.Common.Results()
    conditions.expand_rows(len(Machs),override=False)
    conditions.aerodynamics.angles.alpha   = AoAs
    conditions.aerodynamics.angles.beta    = np.zeros_like(Machs)
    conditions.freestream.mach_number      = Machs
    conditions.freestream.velocity         = np.zeros_like(Machs)
    conditions.static_stability.pitch_rate = np.zeros_like(Machs)
    conditions.static_stability.roll_rate  = np.zeros_like(Machs)
    conditions.static_stability.yaw_rate   = np.zeros_like(Machs)
    VLM_results = call_VLM(conditions, settings, vehicle)
    reference   = Data()
    for key in ['CLift', 'CDrag_induced', 'CX', 'CY', 'CZ', 'CL', 'CM', 'CN']:
        reference[key] = np.reshape(VLM_results[key], (len(Mach), 2)).mean(axis=1)
    CLift, CM   = np.reshape(VLM_results.CLift, (len(Mach), 2)), np.reshape(VLM_results.CM, (len(Mach), 2))
    reference.CM_zero_lift = compute_zero_lift_moment(CLift[:, 0], CLift[:, 1], CM[:, 0], CM[:, 1])
    return reference


def sideslip_and_rate_derivatives(aerodynamics, Mach, settings, vehicle):
    """Sideslip and rate derivatives at each training angle of attack and Mach, shape (len(AoA), len(Mach)); rates are
    about body axes (see compute_lateral_derivatives). All cases share one call so each Mach reuses its C_mn."""
    training = aerodynamics.training
    AoA, Beta, q, p, r = training.angle_of_attack, training.sideslip_angle, training.pitch_rate, training.roll_rate, training.yaw_rate
    b, MAC   = vehicle.reference_span, vehicle.reference_chord

    # columns beta, q, p, r
    cases    = np.array([[Beta[0], 0, 0, 0], [Beta[1], 0, 0, 0], [0, q[0], 0, 0], [0, q[1], 0, 0],
                         [0, 0, p[0], 0], [0, 0, p[1], 0], [0, 0, 0, r[0]], [0, 0, 0, r[1]]])
    n_alpha, n_Mach, n_cases = len(AoA), len(Mach), len(cases)
    per_Mach = np.hstack((np.repeat(AoA, n_cases)[:, None], np.tile(cases, (n_alpha, 1))))
    table    = np.tile(per_Mach, (n_Mach, 1))
    Machs    = np.atleast_2d(np.repeat(Mach, n_alpha * n_cases)).T

    conditions                             = RCAIDE.Framework.Mission.Common.Results()
    conditions.expand_rows(len(Machs),override=False)
    conditions.freestream.mach_number      = Machs
    conditions.freestream.velocity         = Machs * 343 # speed of sound
    conditions.aerodynamics.angles.alpha   = table[:, 0:1]
    conditions.aerodynamics.angles.beta    = table[:, 1:2]
    conditions.static_stability.pitch_rate = table[:, 2:3]
    conditions.static_stability.roll_rate  = table[:, 3:4]
    conditions.static_stability.yaw_rate   = table[:, 4:5]
    VLM_results = call_VLM(conditions, settings, vehicle)

    def difference(key, case):
        values = np.reshape(VLM_results[key], (n_Mach, n_alpha, n_cases)).transpose(1, 0, 2)
        return values[:, :, case] - values[:, :, case + 1]

    V       = Mach * 343
    d_beta  = Beta[0] - Beta[1]
    d_q_hat = (q[0] - q[1]) * MAC / (2 * V)
    d_p_hat = (p[0] - p[1]) * b / (2 * V)
    d_r_hat = (r[0] - r[1]) * b / (2 * V)

    derivatives           = Data()
    derivatives.dCY_dbeta = difference('CY', 0) / d_beta
    derivatives.dCL_dbeta = difference('CL', 0) / d_beta
    derivatives.dCN_dbeta = difference('CN', 0) / d_beta
    derivatives.dCZ_dq    = difference('CZ', 2) / d_q_hat
    derivatives.dCM_dq    = difference('CM', 2) / d_q_hat
    derivatives.dCY_dp    = difference('CY', 4) / d_p_hat
    derivatives.dCL_dp    = difference('CL', 4) / d_p_hat
    derivatives.dCN_dp    = -difference('CN', 4) / d_p_hat
    derivatives.dCY_dr    = difference('CY', 6) / d_r_hat
    derivatives.dCL_dr    = difference('CL', 6) / d_r_hat
    derivatives.dCN_dr    = difference('CN', 6) / d_r_hat
    return derivatives


def control_surface_conditions(Mach):
    """Zero-incidence conditions, one case per Mach, for control-surface training."""
    Machs                                  = np.atleast_2d(Mach).T
    conditions                             = RCAIDE.Framework.Mission.Common.Results()
    conditions.expand_rows(len(Mach),override=False)
    conditions.aerodynamics.angles.alpha   = np.ones_like(Machs) *1E-12
    conditions.aerodynamics.angles.beta    = np.zeros_like(Machs)
    conditions.freestream.mach_number      = Machs
    conditions.freestream.velocity         = np.zeros_like(Machs)
    conditions.static_stability.pitch_rate = np.zeros_like(Machs)
    conditions.static_stability.roll_rate  = np.zeros_like(Machs)
    conditions.static_stability.yaw_rate   = np.zeros_like(Machs)
    return conditions


def neutral_point_objective(cg_location,conditions,settings,clean_wing_vehicle_np,Mach,AoA):

    len_Mach       = len(Mach)        
    len_AoA        = len(AoA)
    
    # update neutral point
    clean_wing_vehicle_np.mass_properties.center_of_gravity[0][0] =  cg_location[0]
    
    # run VLM 
    VLM_results = VLM(conditions,settings,clean_wing_vehicle_np)    
    
    AoA       = conditions.aerodynamics.angles.alpha
    CM_res    = VLM_results.CM
    CM        = np.reshape(CM_res,(len_Mach,len_AoA)).T 
    
    # compute dCM_dalpha 
    dCM_dalpha = ( CM[2, 0] - CM[1, 0]) /( AoA[2] - AoA[1])
     
    # find abs 
    return  abs(dCM_dalpha)


def call_VLM(full_conditions,settings,vehicle): 

    # geometry is fixed within this call, so cases at the same Mach share C_mn
    induced_velocity_cache = {}
    num_cases =  len(full_conditions.aerodynamics.angles.alpha)
    for i in  range(num_cases): 
        conditions                                      = RCAIDE.Framework.Mission.Common.Results() 
        conditions.freestream.mach_number               = np.atleast_2d(full_conditions.freestream.mach_number[i,:])    
        conditions.aerodynamics.angles.alpha            = np.atleast_2d(full_conditions.aerodynamics.angles.alpha[i,:])  
        conditions.aerodynamics.angles.beta             = np.atleast_2d(full_conditions.aerodynamics.angles.beta[i,:])     
        conditions.freestream.velocity                  = np.atleast_2d(full_conditions.freestream.velocity[i,:])          
        conditions.static_stability.pitch_rate          = np.atleast_2d(full_conditions.static_stability.pitch_rate[i,:])  
        conditions.static_stability.roll_rate           = np.atleast_2d(full_conditions.static_stability.roll_rate[i,:])   
        conditions.static_stability.yaw_rate            = np.atleast_2d(full_conditions.static_stability.yaw_rate[i,:])   

        VLM_results         = VLM(conditions,settings,vehicle,induced_velocity_cache)
        if i == 0: 
            RES                 = Data()
            RES.CLift           = VLM_results.CLift
            RES.CDrag_induced   = VLM_results.CDrag_induced
            RES.CX              = VLM_results.CX
            RES.CY              = VLM_results.CY
            RES.CZ              = VLM_results.CZ
            RES.CL              = VLM_results.CL
            RES.CM              = VLM_results.CM
            RES.CN              = VLM_results.CN
            RES.sectional_CLift = VLM_results.sectional_CLift         
            RES.gamma           = VLM_results.gamma
        else: 
            RES.CLift           = np.vstack((RES.CLift          ,VLM_results.CLift)) 
            RES.CDrag_induced   = np.vstack((RES.CDrag_induced  ,VLM_results.CDrag_induced))
            RES.CX              = np.vstack((RES.CX             ,VLM_results.CX))
            RES.CY              = np.vstack((RES.CY             ,VLM_results.CY))
            RES.CZ              = np.vstack((RES.CZ             ,VLM_results.CZ))
            RES.CL              = np.vstack((RES.CL             ,VLM_results.CL))
            RES.CM              = np.vstack((RES.CM             ,VLM_results.CM))
            RES.CN              = np.vstack((RES.CN             ,VLM_results.CN))
            RES.sectional_CLift = np.vstack((RES.sectional_CLift,VLM_results.sectional_CLift))   
            RES.gamma           = np.vstack((RES.gamma          ,VLM_results.gamma))
    
    return RES

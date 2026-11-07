# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/train_VLM_surrogates.py
#
# Created: Aug 2025, M. Clarke
# Modified: Oct 2026, H. Hussien
# Modified: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core import  Data, Units
from RCAIDE.Library.Plots import *
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.VLM   import VLM
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.generate_vortex_distribution import generate_vortex_distribution
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.compute_zero_lift_moment import compute_zero_lift_moment
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.FEA   import FEA
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.discretize_wing import deform_vortex_distribution
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.control_surface_registry import lookup as cs_lookup, CONTROL_SURFACE_TYPES, HIGH_LIFT_LETTERS, HIGH_LIFT_DRAG_TERMS
from copy import deepcopy

# package imports
import numpy  as np

# ----------------------------------------------------------------------------------------------------------------------
#  Vortex_Lattice
# ---------------------------------------------------------------------------------------------------------------------- 
SIDESLIP_AND_RATE_DERIVATIVES = ["dCY_dbeta", "dCL_dbeta", "dCN_dbeta", "dCZ_dq", "dCM_dq", "dCY_dp", "dCL_dp", "dCN_dp", "dCY_dr", "dCL_dr", "dCN_dr"]

def train_VLM_surrogates(aerodynamics, vehicle,aerostructural_analyses=None):
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

    # Skip supersonic training if the vehicle is known to stay subsonic.
    design_mach = getattr(vehicle.flight_envelope, 'design_mach_number', None)
    if design_mach is not None and design_mach < aerodynamics.surrogates.subsonic_smoothing_max and np.any(Mach >= 1.0):
        Mach = Mach[Mach < 1.0]
        aerodynamics.training.Mach = Mach

    training      = aerodynamics.training
    sub_len       = int(sum(Mach<1.))
    sub_Mach      = Mach[:sub_len]
    sup_Mach      = Mach[sub_len:]

    training.subsonic    =  train_model(aerodynamics, sub_Mach, vehicle, aerostructural_analyses)

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
    
def train_model(aerodynamics,Mach, vehicle,aerostructural_analyses=None): 
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
    settings_str   = aerostructural_analyses.settings if aerostructural_analyses is not None else None
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

    # Sea-level atmosphere: VLM only needs Mach, but FEA needs physical dynamic pressure
    atmosphere  = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo        = atmosphere.compute_values(aerodynamics.training.altitude)
    a0   =  atmo.speed_of_sound[0][0]
    rho0 =  atmo.density[0][0]
    mu0  =  atmo.dynamic_viscosity[0][0]
    T0   =  atmo.temperature[0][0]
    P0   =  atmo.pressure[0][0]    
    V    = Machs * a0

    conditions                                      = RCAIDE.Framework.Mission.Common.Results()
    conditions.freestream.mach_number               = Machs
    conditions.freestream.velocity                  = V
    conditions.freestream.density                   = rho0 * np.ones_like(Machs)
    conditions.freestream.dynamic_viscosity         = mu0  * np.ones_like(Machs)
    conditions.freestream.temperature               = T0   * np.ones_like(Machs)
    conditions.freestream.pressure                  = P0   * np.ones_like(Machs)
    conditions.freestream.dynamic_pressure          = 0.5 * rho0 * V**2
    conditions.freestream.gravitational_acceleration = 9.81 * np.ones_like(Machs)
    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs)*AoAs
    conditions.aerodynamics.angles.beta             = np.zeros_like(Machs)*AoAs
    conditions.static_stability.pitch_rate          = np.zeros_like(Machs)*AoAs
    conditions.static_stability.roll_rate           = np.zeros_like(Machs)*AoAs
    conditions.static_stability.yaw_rate            = np.zeros_like(Machs)*AoAs

    clean_wing_vehicle = deepcopy(vehicle)
    for wing in clean_wing_vehicle.wings:
        wing.control_surfaces = []

    # populate conditions fields FEA needs (point loads, structural container)
    # before call_VLM runs, so a coupled call can slice them per row too
    if aerostructural_analyses is not None:
        n_pts = int(AoAs.shape[0])
        conditions.aerostructures                    = Data()
        conditions.control_surfaces                  = Data()
        conditions.weights                           = Data()
        conditions.weights.components                = Data()
        conditions.weights.components.mass           = Data()
        for network in vehicle.networks:
            for source in network.sources:
                if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank):
                    conditions.weights.components.mass[source.fuel.tag] = (
                        source.mass_properties.mass * np.ones((n_pts, 1)))

    # run VLM (and, if coupled, VLM<->FEA to convergence) for all conditions
    coupled = (aerostructural_analyses is not None
               and settings_str.aeroelastic_coupling == 'coupled')
    VLM_results = call_VLM(conditions, settings, clean_wing_vehicle,
                            aerostructural_analyses=aerostructural_analyses,
                            coupled=coupled)

    # run FEA alongside VLM training if aerostructural_analyses is provided
    if aerostructural_analyses is not None:
        if coupled:
            # call_VLM already converged VLM<->FEA per row; reuse its result
            # instead of a second batched FEA pass (which would only see the
            # LAST row's deformed mesh - wrong once the mesh varies per row).
            FEA_results = VLM_results.structural
        else:
            FEA_results = FEA(conditions, VLM_results, VLM_results.VD,
                              settings_str, clean_wing_vehicle)

        # Normalise by dynamic pressure so the surrogate stores δ/q_dyn.
        # At query time evaluate_surrogate multiplies by the mission q_dyn,
        # giving the correct deflection at any altitude without a third
        # surrogate dimension.
        q_dyn_train = 0.5 * rho0 * V**2   # shape (n_pts, 1)

        training.deflection_u  = Data()
        training.deflection_v  = Data()
        training.deflection_w  = Data()
        training.elastic_twist = Data()
        for wing in clean_wing_vehicle.wings:
            n_nodes = FEA_results[wing.tag].deflection.shape[1]
            training.deflection_u[wing.tag]  = (FEA_results[wing.tag].deflection[:, :, 0] / q_dyn_train).reshape(len_Mach, len_AoA, n_nodes).transpose(1, 0, 2)
            training.deflection_v[wing.tag]  = (FEA_results[wing.tag].deflection[:, :, 1] / q_dyn_train).reshape(len_Mach, len_AoA, n_nodes).transpose(1, 0, 2)
            training.deflection_w[wing.tag]  = (FEA_results[wing.tag].deflection[:, :, 2] / q_dyn_train).reshape(len_Mach, len_AoA, n_nodes).transpose(1, 0, 2)
            training.elastic_twist[wing.tag] = (FEA_results[wing.tag].elastic_twist[:, :, 0] / q_dyn_train).reshape(len_Mach, len_AoA, n_nodes).transpose(1, 0, 2)

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
        if aerostructural_analyses is not None:
            structural_ref = Data(FEA(control_surface_structural_conditions(Mach, a0, rho0, vehicle), VLM_ref, VLM_ref.VD, settings_str, vehicle))
            q_dyn_cs       = 0.5 * rho0 * (np.atleast_2d(Mach).T * a0)**2
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

                # structural derivatives per unit dynamic pressure; gravity loads cancel in the difference
                if aerostructural_analyses is not None:
                    structural_d = FEA(control_surface_structural_conditions(Mach, a0, rho0, vehicle), VLM_results, VLM_results.VD, settings_str, vehicle)
                    for field in ('ddeflection_u_ddelta_', 'ddeflection_v_ddelta_', 'ddeflection_w_ddelta_', 'delastic_twist_ddelta_'):
                        training[field + letter] = Data()
                    for wing2 in vehicle.wings:
                        d_deflection = (structural_d[wing2.tag].deflection    - structural_ref[wing2.tag].deflection   ) / q_dyn_cs[:, :, None]
                        d_twist      = (structural_d[wing2.tag].elastic_twist - structural_ref[wing2.tag].elastic_twist) / q_dyn_cs[:, :, None]
                        training['ddeflection_u_ddelta_'  + letter][wing2.tag] = d_deflection[:, :, 0] / delta
                        training['ddeflection_v_ddelta_'  + letter][wing2.tag] = d_deflection[:, :, 1] / delta
                        training['ddeflection_w_ddelta_'  + letter][wing2.tag] = d_deflection[:, :, 2] / delta
                        training['delastic_twist_ddelta_' + letter][wing2.tag] = d_twist[:, :, 0]      / delta
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
    derivatives.dCN_dp    = difference('CN', 4) / d_p_hat
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


def control_surface_structural_conditions(Mach, a0, rho0, vehicle):
    """Zero-incidence conditions with the sea-level freestream and full tank masses FEA needs, one case per Mach."""
    Machs                                            = np.atleast_2d(Mach).T
    conditions                                       = control_surface_conditions(Mach)
    conditions.freestream.velocity                   = Machs * a0
    conditions.freestream.density                    = rho0 * np.ones_like(Machs)
    conditions.freestream.dynamic_pressure           = 0.5 * rho0 * (Machs * a0)**2
    conditions.freestream.gravitational_acceleration = 9.81 * np.ones_like(Machs)
    conditions.aerostructures                        = Data()
    conditions.weights                               = Data()
    conditions.weights.components                    = Data()
    conditions.weights.components.mass               = Data()
    for network in vehicle.networks:
        for source in network.sources:
            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank):
                conditions.weights.components.mass[source.fuel.tag] = source.mass_properties.mass * np.ones_like(Machs)
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


def _converge_aeroelastic(conditions, settings, vehicle, aerostructural_analyses, jig_vd, tol, max_iter):
    """Iterate VLM<->FEA on a single (AoA,Mach) row until deflection converges.

    Returns the converged VLM_results, structural_results (keyed by wing.tag,
    same shape FEA() returns), and a 1D history array of the relative
    (delta-deflection / span) convergence metric at each iteration - the
    rigid-wing (iteration 0) solve is the starting point, not part of the
    history, since there is no prior iterate to diff it against.
    """
    span = vehicle.reference_span

    # FEA() returns conditions.aerostructures itself (not a copy) - calling FEA
    # again on the SAME conditions object rebinds its wing-tag entries out from
    # under any earlier reference. Wrap each result in Data(...) (a shallow
    # copy of the outer dict, per RCAIDE.Framework.Core.Data's Data(d) ==
    # copy-of-d semantics) so the convergence check below diffs a real
    # snapshot of the previous iterate, not the same live-mutating object
    # against itself (which always reads delta==0 and "converges" instantly).
    VLM_results        = VLM(conditions, settings, vehicle, precomputed_VD=jig_vd)
    structural_results = Data(FEA(conditions, VLM_results, VLM_results.VD,
                                   aerostructural_analyses.settings, vehicle))

    history = []
    for it in range(max_iter):
        VD_deformed             = deform_vortex_distribution(jig_vd, vehicle, structural_results, ti=0)
        VLM_results_new         = VLM(conditions, settings, vehicle, precomputed_VD=VD_deformed)
        structural_results_new  = Data(FEA(conditions, VLM_results_new, VLM_results_new.VD,
                                            aerostructural_analyses.settings, vehicle))

        delta = 0.0
        for wing_tag in structural_results_new.keys():
            d_old = structural_results[wing_tag].deflection
            d_new = structural_results_new[wing_tag].deflection
            delta = max(delta, float(np.max(np.abs(d_new - d_old))) / span)
        history.append(delta)

        VLM_results, structural_results = VLM_results_new, structural_results_new
        if delta < tol:
            break

    return VLM_results, structural_results, np.array(history)


def call_VLM(full_conditions,settings,vehicle,aerostructural_analyses=None,coupled=False,jig_vd=None,tol=1e-4,max_iter=15):

    # rigid geometry is fixed within this call, so cases at the same Mach share C_mn; coupled cases solve on their own deformed mesh
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

        if coupled and aerostructural_analyses is not None:
            # FEA() also needs these freestream/point-load fields, which the
            # other (rigid-loads-only) callers of call_VLM never populate
            conditions.freestream.density                    = np.atleast_2d(full_conditions.freestream.density[i,:])
            conditions.freestream.dynamic_pressure           = np.atleast_2d(full_conditions.freestream.dynamic_pressure[i,:])
            conditions.freestream.gravitational_acceleration = np.atleast_2d(full_conditions.freestream.gravitational_acceleration[i,:])
            conditions.aerostructures = Data()
            if 'weights' in full_conditions:
                conditions.weights = Data()
                conditions.weights.components = Data()
                conditions.weights.components.mass = Data()
                for tag in full_conditions.weights.components.mass.keys():
                    conditions.weights.components.mass[tag] = np.atleast_2d(
                        full_conditions.weights.components.mass[tag][i,:])
            # Jig-shape VD is pure geometry - independent of (AoA,Mach) - so it is
            # built once, on the first row, and reused unchanged for every row.
            if jig_vd is None:
                jig_vd = generate_vortex_distribution(conditions, settings, vehicle)
            VLM_results, structural_results, history = _converge_aeroelastic(
                conditions, settings, vehicle, aerostructural_analyses, jig_vd, tol, max_iter)
            row_convergence           = np.full(max_iter, np.nan)
            row_convergence[:len(history)] = history
        else:
            # while convergence or until max iterations reached, run VLM and FEA
            VLM_results         = VLM(conditions,settings,vehicle,induced_velocity_cache)
            structural_results  = None
            row_convergence     = None

        if i == 0:
            RES                 = Data()
            RES.CLift           = VLM_results.CLift
            RES.CDrag_induced   = VLM_results.CDrag_induced
            RES.CP              = VLM_results.CP
            RES.CX              = VLM_results.CX
            RES.CY              = VLM_results.CY
            RES.CZ              = VLM_results.CZ
            RES.CL              = VLM_results.CL
            RES.CM              = VLM_results.CM
            RES.CN              = VLM_results.CN
            RES.sectional_CLift = VLM_results.sectional_CLift
            RES.gamma           = VLM_results.gamma
            RES.VD              = jig_vd if coupled else VLM_results.VD   # undeformed mesh shared by every case
            if structural_results is not None:
                RES.structural = Data()
                for wing_tag in structural_results.keys():
                    RES.structural[wing_tag] = Data(
                        structural_node_data = structural_results[wing_tag].structural_node_data,
                        load                  = structural_results[wing_tag].load,
                        deflection            = structural_results[wing_tag].deflection,
                        elastic_twist         = structural_results[wing_tag].elastic_twist,
                        normal_stress         = structural_results[wing_tag].normal_stress,
                        shear_stress          = structural_results[wing_tag].shear_stress,
                        margin_of_safety      = structural_results[wing_tag].margin_of_safety)
                # convergence.delta[row, iteration] = max(|delta-deflection|)/span at
                # that iteration, NaN-padded past the row's own convergence point
                RES.convergence          = Data()
                RES.convergence.delta    = row_convergence[None, :]
        else:
            RES.CLift           = np.vstack((RES.CLift          ,VLM_results.CLift))
            RES.CDrag_induced   = np.vstack((RES.CDrag_induced  ,VLM_results.CDrag_induced))
            RES.CP              = np.vstack((RES.CP             ,VLM_results.CP))
            RES.CX              = np.vstack((RES.CX             ,VLM_results.CX))
            RES.CY              = np.vstack((RES.CY             ,VLM_results.CY))
            RES.CZ              = np.vstack((RES.CZ             ,VLM_results.CZ))
            RES.CL              = np.vstack((RES.CL             ,VLM_results.CL))
            RES.CM              = np.vstack((RES.CM             ,VLM_results.CM))
            RES.CN              = np.vstack((RES.CN             ,VLM_results.CN))
            RES.sectional_CLift = np.vstack((RES.sectional_CLift,VLM_results.sectional_CLift))
            RES.gamma           = np.vstack((RES.gamma          ,VLM_results.gamma))
            if structural_results is not None:
                for wing_tag in structural_results.keys():
                    s = RES.structural[wing_tag]
                    s.load             = np.vstack((s.load            , structural_results[wing_tag].load))
                    s.deflection       = np.vstack((s.deflection      , structural_results[wing_tag].deflection))
                    s.elastic_twist    = np.vstack((s.elastic_twist   , structural_results[wing_tag].elastic_twist))
                    s.normal_stress    = np.vstack((s.normal_stress   , structural_results[wing_tag].normal_stress))
                    s.shear_stress     = np.vstack((s.shear_stress    , structural_results[wing_tag].shear_stress))
                    s.margin_of_safety = np.vstack((s.margin_of_safety, structural_results[wing_tag].margin_of_safety))
                RES.convergence.delta = np.vstack((RES.convergence.delta, row_convergence[None, :]))

    return RES

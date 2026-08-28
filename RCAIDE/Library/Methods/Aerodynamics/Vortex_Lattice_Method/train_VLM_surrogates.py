# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/train_VLM_surrogates.py
#  
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core import  Data
from RCAIDE.Library.Plots import *
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.VLM   import VLM
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.FEA   import FEA
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.control_surface_registry import lookup as cs_lookup, CONTROL_SURFACE_TYPES
from copy import deepcopy

# package imports
import numpy  as np

# ----------------------------------------------------------------------------------------------------------------------
#  Vortex_Lattice
# ---------------------------------------------------------------------------------------------------------------------- 
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

    # only build supersonic surrogates if necessary
    if len(sup_Mach) > 2:
        training.supersonic  =  train_model(aerodynamics, sup_Mach, vehicle, aerostructural_analyses)
        training.transonic   =  train_trasonic_model(aerodynamics, training.subsonic,training.supersonic,sub_Mach, sup_Mach, vehicle)
    else:
        training.supersonic  = None
        training.transonic   = None
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
    settings     = aerodynamics.settings
    settings_str = aerostructural_analyses.settings if aerostructural_analyses is not None else None
    AoA          = aerodynamics.training.angle_of_attack
    Beta         = aerodynamics.training.sideslip_angle
    MAC          = vehicle.reference_chord
    b            = vehicle.reference_span
    training     = Data()
    training.Mach = Mach
    
    # loop through wings to determine what control surfaces are present
    delta_0     = {}
    delta_train = {}
    len_delta   = {}
    for wing in vehicle.wings:
        for control_surface in wing.control_surfaces:
            for letter, name, channel, flag, deflection_attr in cs_lookup(control_surface):
                delta_0[letter]     = getattr(control_surface, deflection_attr)
                delta_train[letter] = getattr(aerodynamics.training, channel + '_deflection')
                len_delta[letter]   = len(delta_train[letter])
                setattr(aerodynamics, flag, True)
            control_surface.deflection           = 0 # set all control surfaces to be 0
            control_surface.secondary_deflection = 0
             
    u              = aerodynamics.training.u
    pitch_rate     = aerodynamics.training.pitch_rate
    roll_rate      = aerodynamics.training.roll_rate
    yaw_rate       = aerodynamics.training.yaw_rate  
    len_Mach       = len(Mach)        
    len_AoA        = len(AoA)  
    len_Beta       = len(Beta)
    len_u          = len(u)
    len_q          = len(pitch_rate)
    len_p          = len(roll_rate) 
    len_r          = len(yaw_rate) 
    
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

    # run VLM for all conditions
    VLM_results = call_VLM(conditions, settings, clean_wing_vehicle)

    # run FEA alongside VLM training if aerostructural_analyses is provided
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

        FEA_results = FEA(conditions, VLM_results, settings.vortex_distribution,
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
    VD_0             = settings.vortex_distribution
    Cdrag_res        = VLM_results.CDrag_induced
    CX_res           = VLM_results.CX
    CY_res           = VLM_results.CY
    CZ_res           = VLM_results.CZ
    CL_res           = VLM_results.CL
    CM_res           = VLM_results.CM
    CN_res           = VLM_results.CN   

    training.Clift_spanwise  = VLM_results.sectional_CLift.reshape(len_Mach, len_AoA, np.shape(VLM_results.sectional_CLift)[1]).transpose(1, 0, 2)
    Clift_alpha              = np.reshape(Clift_res,(len_Mach,len_AoA)).T 
    Cdrag_induced_alpha      = np.reshape(Cdrag_res,(len_Mach,len_AoA)).T 
    CX_alpha                 = np.reshape(CX_res,(len_Mach,len_AoA)).T 
    CY_alpha                 = np.reshape(CY_res,(len_Mach,len_AoA)).T 
    CZ_alpha                 = np.reshape(CZ_res,(len_Mach,len_AoA)).T 
    CL_alpha                 = np.reshape(CL_res,(len_Mach,len_AoA)).T 
    CM_alpha                 = np.reshape(CM_res,(len_Mach,len_AoA)).T 
    CN_alpha                 = np.reshape(CN_res,(len_Mach,len_AoA)).T  
    
    # Angle of Attack at 0 Degrees .
    Clift_alpha_0   =  np.tile(Clift_alpha[2][None,:],(2,1))
    Cdrag_alpha_0   =  np.tile(Cdrag_induced_alpha[2][None,:],(2,1))
    CX_alpha_0      =  np.tile(CX_alpha[2][None,:],(2, 1)) 
    CY_alpha_0      =  0 * np.tile(CY_alpha[2][None,:],(2, 1)) 
    CZ_alpha_0      =  np.tile(CZ_alpha[2][None,:],(2, 1)) 
    CL_alpha_0      =  0 * np.tile(CL_alpha[2][None,:],(2, 1)) 
    CM_alpha_0      =  np.tile(CM_alpha[2][None,:],(2, 1)) 
    CN_alpha_0      =  0 * np.tile(CN_alpha[2][None,:],(2, 1)) 
        
    # --------------------------------------------------------------------------------------------------------------
    # Beta 
    # --------------------------------------------------------------------------------------------------------------
    Betas         = np.atleast_2d(np.tile(Beta,len_Mach).T.flatten()).T 
    Machs         = np.atleast_2d(np.repeat(Mach,len_Beta)).T      
    conditions                                      = RCAIDE.Framework.Mission.Common.Results() 
    conditions.expand_rows(rows= len(Machs))
    conditions.freestream.mach_number               = Machs 
    conditions.freestream.velocity                  = np.zeros_like(Machs)  
    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs) *1E-12
    conditions.aerodynamics.angles.beta             = np.ones_like(Machs)*Betas  
    conditions.static_stability.pitch_rate          = np.zeros_like(Machs)   
    conditions.static_stability.roll_rate           = np.zeros_like(Machs)   
    conditions.static_stability.yaw_rate            = np.zeros_like(Machs)    
    
    VLM_results = call_VLM(conditions,settings,clean_wing_vehicle)
    
    Clift_res   = VLM_results.CLift
    Cdrag_res   = VLM_results.CDrag_induced
    CX_res      = VLM_results.CX
    CY_res      = VLM_results.CY
    CZ_res      = VLM_results.CZ
    CL_res      = VLM_results.CL
    CM_res      = VLM_results.CM
    CN_res      = VLM_results.CN
    
    Clift_beta         =    np.reshape(Clift_res,(len_Mach,len_Beta)).T - Clift_alpha_0
    Cdrag_induced_beta =    np.reshape(Cdrag_res,(len_Mach,len_Beta)).T - Cdrag_alpha_0                                
    CX_beta            =    np.reshape(CX_res,(len_Mach,len_Beta)).T    - CX_alpha_0   
    CY_beta            =    np.reshape(CY_res,(len_Mach,len_Beta)).T    - CY_alpha_0    
    CZ_beta            =    np.reshape(CZ_res,(len_Mach,len_Beta)).T    - CZ_alpha_0   
    CL_beta            =    np.reshape(CL_res,(len_Mach,len_Beta)).T    - CL_alpha_0  
    CM_beta            =    np.reshape(CM_res,(len_Mach,len_Beta)).T    - CM_alpha_0   
    CN_beta            =    np.reshape(CN_res,(len_Mach,len_Beta)).T    - CN_alpha_0  
 
    # -------------------------------------------------------      
    # Velocity u 
    # -------------------------------------------------------
    u_s     = np.atleast_2d(np.tile(u, len_Mach).T.flatten()).T 
    Machs   = np.atleast_2d(np.repeat(Mach,len_u)).T                   
    conditions                                      = RCAIDE.Framework.Mission.Common.Results()  
    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs) *1E-12 
    conditions.aerodynamics.angles.beta             = np.zeros_like(Machs) 
    conditions.freestream.mach_number               = Machs + u_s/343 
    conditions.freestream.velocity                  = np.zeros_like(Machs)   
    conditions.static_stability.pitch_rate          = np.zeros_like(Machs)   
    conditions.static_stability.roll_rate           = np.zeros_like(Machs)   
    conditions.static_stability.yaw_rate            = np.zeros_like(Machs)   
    VLM_results = call_VLM(conditions,settings,clean_wing_vehicle)
    CX_res    = VLM_results.CX
    CZ_res    = VLM_results.CZ
    CM_res    = VLM_results.CM
    CX_u      = np.reshape(VLM_results.CX,(len_Mach,len_u)).T    - CX_alpha_0   
    CZ_u      = np.reshape(VLM_results.CZ,(len_Mach,len_u)).T    - CZ_alpha_0   
    CM_u      = np.reshape(VLM_results.CM,(len_Mach,len_u)).T    - CM_alpha_0  
                    
    # -------------------------------------------------------               
    # Pitch Rate 
    # -------------------------------------------------------
    q_s     = np.atleast_2d(np.tile(pitch_rate, len_Mach).T.flatten()).T 
    Machs   = np.atleast_2d(np.repeat(Mach,len_q)).T 
    conditions                                      = RCAIDE.Framework.Mission.Common.Results()  
    conditions.freestream.mach_number               = Machs 
    conditions.freestream.velocity                  = Machs * 343 # speed of sound  
    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs) *1E-12
    conditions.aerodynamics.angles.beta             = np.zeros_like(Machs) 
    conditions.static_stability.pitch_rate          = np.ones_like(Machs)*q_s      
    conditions.static_stability.roll_rate           = np.zeros_like(Machs)   
    conditions.static_stability.yaw_rate            = np.zeros_like(Machs)    
    
    VLM_results = call_VLM(conditions,settings,clean_wing_vehicle)
    CM_res      = VLM_results.CM  
    CM_q        = np.reshape(CM_res,(len_Mach,len_q)).T   # - CM_alpha_0    
    CZ_q        = np.reshape(CZ_res,(len_Mach,len_q)).T   # - CZ_alpha_0

    # -------------------------------------------------------               
    # Roll  Rate 
    # -------------------------------------------------------    
    p_s           = 1 * np.atleast_2d(np.tile(roll_rate, len_Mach).T.flatten()).T 
    Machs         = np.atleast_2d(np.repeat(Mach,len_p)).T 
    conditions                                      = RCAIDE.Framework.Mission.Common.Results()  
    conditions.freestream.mach_number               = Machs  
    conditions.freestream.velocity                  = Machs * 343 # speed of sound  
    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs) *1E-12 
    conditions.aerodynamics.angles.beta             = np.zeros_like(Machs)     
    conditions.static_stability.pitch_rate          = np.zeros_like(Machs) 
    conditions.static_stability.roll_rate           = np.ones_like(Machs)*p_s     
    conditions.static_stability.yaw_rate            = np.zeros_like(Machs)         
    VLM_results =  call_VLM(conditions,settings,clean_wing_vehicle)
    CL_res      =  VLM_results.CL
    CN_res      =  VLM_results.CN
    CY_res      =  VLM_results.CY
    CL_p        =  np.reshape(CL_res,(len_Mach,len_p)).T    - CL_alpha_0    
    CN_p        = -(np.reshape(CN_res,(len_Mach,len_p)).T    - CN_alpha_0)    
    CY_p        =  np.reshape(CY_res,(len_Mach,len_p)).T    - CY_alpha_0    

    # -------------------------------------------------------               
    # Yaw Rate 
    # -------------------------------------------------------        
    r_s     = np.atleast_2d(np.tile(yaw_rate, len_Mach).T.flatten()).T 
    Machs   = np.atleast_2d(np.repeat(Mach,len_r)).T

    conditions                                      = RCAIDE.Framework.Mission.Common.Results()  
    conditions.freestream.mach_number               = Machs 
    conditions.freestream.velocity                  = Machs * 343   
    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs)*1E-2 
    conditions.aerodynamics.angles.beta             = np.zeros_like(Machs) 
    conditions.static_stability.pitch_rate          = np.zeros_like(Machs)   
    conditions.static_stability.roll_rate           = np.zeros_like(Machs)  
    conditions.static_stability.yaw_rate            = np.ones_like(Machs)*r_s  
    
    VLM_results = call_VLM(conditions,settings,clean_wing_vehicle)
    CL_res      = VLM_results.CL
    CN_res      = VLM_results.CN
    CY_res      = VLM_results.CY
    CL_r        = np.reshape(CL_res,(len_Mach,len_r)).T    - CL_alpha_0   
    CN_r        = np.reshape(CN_res,(len_Mach,len_r)).T    - CN_alpha_0   
    CY_r        = np.reshape(CY_res,(len_Mach,len_r)).T    - CY_alpha_0
        
    # STABILITY COEFFICIENTS  
    training.Clift_alpha               = Clift_alpha  
    training.Cdrag_induced_alpha       = Cdrag_induced_alpha   
    training.CX_alpha                  = CX_alpha
    training.CY_alpha                  = CY_alpha 
    training.CZ_alpha                  = CZ_alpha  
    training.CL_alpha                  = CL_alpha   
    training.CM_alpha                  = CM_alpha 
    training.CN_alpha                  = CN_alpha    
    training.CM_0                      = CM_alpha_0[0]  
            
    training.Clift_beta                = Clift_beta 
    training.Cdrag_induced_beta        = Cdrag_induced_beta  
    training.CX_beta                   = CX_beta
    training.CY_beta                   = CY_beta 
    training.CZ_beta                   = CZ_beta
    training.CL_beta                   = CL_beta  
    training.CM_beta                   = CM_beta
    training.CN_beta                   = CN_beta 
        
    training.CX_u                      = CX_u
    training.CZ_u                      = CZ_u
    training.CM_u                      = CM_u
        
    training.CM_q                      = CM_q
    training.CZ_q                      = CZ_q
        
    training.CL_p                      = CL_p
    training.CN_p                      = CN_p
    training.CY_p                      = CY_p
        
    training.CL_r                      = CL_r
    training.CN_r                      = CN_r
    training.CY_r                      = CY_r 
       
    # STABILITY DERIVATIVES 
    V = np.reshape(conditions.freestream.velocity ,(len_Mach,len_r)).T
    
    training.dClift_dalpha = (Clift_alpha[0,:] - Clift_alpha[1,:]) / (AoA[0] - AoA[1])       
    training.dCX_dalpha = (CX_alpha[0,:] - CX_alpha[1,:]) / (AoA[0] - AoA[1])       
    training.dCX_du     = (CX_u[0,:] - CX_u[1,:]) / (u[0] - u[1])                                     

    training.dCY_dbeta  = (CY_beta[0,:] - CY_beta[1,:]) / (Beta[0] - Beta[1])
    training.dCY_dr     = (CY_r[0,:] - CY_r[1,:]) / ((yaw_rate[0]-yaw_rate[1])* b / (2 *V[0,:]))   

    training.dCZ_dalpha = (CZ_alpha[0,:] - CZ_alpha[1,:]) / (AoA[0] - AoA[1])             
    training.dCZ_du     = (CZ_u[0,:] - CZ_u[1,:]) / (u[0] - u[1])    
    training.dCZ_dq     = (CZ_q[0,:] - CZ_q[1,:]) / ((pitch_rate[0]-pitch_rate[1])* MAC / (2 *V[0,:]))    
    
    training.dCL_dbeta  = (CL_beta[0,:] - CL_beta[1,:]) / (Beta[0] - Beta[1])  
    training.dCL_dp     = (CL_p[0,:] - CL_p[1,:]) / ((roll_rate[0]-roll_rate[1])* b / (2 *V[0,:]))  
    training.dCL_dr     = (CL_r[0,:] - CL_r[1,:]) / ((yaw_rate[0]-yaw_rate[1])* b / (2 *V[0,:]))   

    training.dCM_dalpha = (CM_alpha[0,:] - CM_alpha[1,:]) / (AoA[0] - AoA[1])          
    training.dCM_du     = (CM_u[0,:] - CM_u[1,:]) / (u[0] - u[1])                                               
    training.dCM_dq     = (CM_q[0,:] - CM_q[1,:]) / ((pitch_rate[0]-pitch_rate[1])* MAC / (2 *V[0,:]))      
            
    training.dCN_dbeta  = (CN_beta[0,:] - CN_beta[1,:]) / (Beta[0] - Beta[1]) 
    training.dCN_dp     = (CN_p[0,:] - CN_p[1,:]) / ((roll_rate[0]-roll_rate[1])* b / (2 *V[0,:]))  
    training.dCN_dr     = (CN_r[0,:] - CN_r[1,:]) / ((yaw_rate[0]-yaw_rate[1])* b / (2 *V[0,:]))  

    # for control surfaces, subtract influence WITHOUT control surface deflected from coefficients WITH control
    # surface deflected; see control_surface_registry.py for why every type is treated identically here.
    Machs = np.atleast_2d(np.repeat(Mach,1)).T
    V_cs  = Machs * a0   # real velocity for this Mach row, needed for FEA's dynamic pressure below
    for wing in vehicle.wings:
        for control_surface in wing.control_surfaces:
            for letter, name, channel, flag, deflection_attr in cs_lookup(control_surface):
                delta = delta_train[letter]
                len_d = len_delta[letter]

                Clift_d = np.zeros((len_d,len_Mach))
                Cdrag_d = np.zeros((len_d,len_Mach))
                CX_d    = np.zeros((len_d,len_Mach))
                CY_d    = np.zeros((len_d,len_Mach))
                CZ_d    = np.zeros((len_d,len_Mach))
                CL_d    = np.zeros((len_d,len_Mach))
                CM_d    = np.zeros((len_d,len_Mach))
                CN_d    = np.zeros((len_d,len_Mach))
                deflection_u_d  = Data()
                deflection_v_d  = Data()
                deflection_w_d  = Data()
                elastic_twist_d = Data()
                for d_i in range(len_d):
                    conditions                            = RCAIDE.Framework.Mission.Common.Results()
                    conditions.expand_rows(len(Mach),override=False)
                    conditions.aerodynamics.angles.alpha  = np.ones_like(Machs) *1E-12
                    conditions.aerodynamics.angles.beta   = np.zeros_like(Machs)
                    conditions.freestream.mach_number     = Machs
                    conditions.freestream.velocity        = np.zeros_like(Machs)
                    conditions.static_stability.pitch_rate= np.zeros_like(Machs)
                    conditions.static_stability.roll_rate = np.zeros_like(Machs)
                    conditions.static_stability.yaw_rate  = np.zeros_like(Machs)

                    # Structural derivative w.r.t. this control surface; reuses the
                    # VLM call below since its coefficients don't depend on velocity.
                    if aerostructural_analyses is not None:
                        conditions.freestream.velocity                    = V_cs
                        conditions.freestream.density                     = rho0 * np.ones_like(Machs)
                        conditions.freestream.dynamic_pressure            = 0.5 * rho0 * V_cs**2
                        conditions.freestream.gravitational_acceleration  = 9.81 * np.ones_like(Machs)
                        conditions.aerostructures                         = Data()
                        conditions.weights                                = Data()
                        conditions.weights.components                     = Data()
                        conditions.weights.components.mass                = Data()
                        n_pts_cs = len(Machs)
                        for network in vehicle.networks:
                            for source in network.sources:
                                if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank):
                                    conditions.weights.components.mass[source.fuel.tag] = (
                                        source.mass_properties.mass * np.ones((n_pts_cs, 1)))

                    setattr(control_surface, deflection_attr, delta[d_i])
                    VLM_results   = call_VLM(conditions,settings,vehicle)
                    Clift_d[d_i,:] = VLM_results.CLift[:,0]         - Clift_alpha_0[0,:]
                    Cdrag_d[d_i,:] = VLM_results.CDrag_induced[:,0] - Cdrag_alpha_0[0,:]
                    CX_d[d_i,:]    = VLM_results.CX[:,0]            - CX_alpha_0[0,:]
                    CY_d[d_i,:]    = VLM_results.CY[:,0]            - CY_alpha_0[0,:]
                    CZ_d[d_i,:]    = VLM_results.CZ[:,0]            - CZ_alpha_0[0,:]
                    CL_d[d_i,:]    = VLM_results.CL[:,0]            - CL_alpha_0[0,:]
                    CM_d[d_i,:]    = VLM_results.CM[:,0]            - CM_alpha_0[0,:]
                    CN_d[d_i,:]    = VLM_results.CN[:,0]            - CN_alpha_0[0,:]

                    if aerostructural_analyses is not None:
                        q_dyn_cs       = 0.5 * rho0 * V_cs**2
                        FEA_results_d  = FEA(conditions, VLM_results, settings.vortex_distribution,
                                             settings_str, vehicle)
                        for wing2 in vehicle.wings:
                            n_nodes = FEA_results_d[wing2.tag].deflection.shape[1]
                            if d_i == 0:
                                deflection_u_d[wing2.tag]  = np.zeros((len_d, len_Mach, n_nodes))
                                deflection_v_d[wing2.tag]  = np.zeros((len_d, len_Mach, n_nodes))
                                deflection_w_d[wing2.tag]  = np.zeros((len_d, len_Mach, n_nodes))
                                elastic_twist_d[wing2.tag] = np.zeros((len_d, len_Mach, n_nodes))
                            deflection_u_d[wing2.tag][d_i]  = FEA_results_d[wing2.tag].deflection[:, :, 0] / q_dyn_cs
                            deflection_v_d[wing2.tag][d_i]  = FEA_results_d[wing2.tag].deflection[:, :, 1] / q_dyn_cs
                            deflection_w_d[wing2.tag][d_i]  = FEA_results_d[wing2.tag].deflection[:, :, 2] / q_dyn_cs
                            elastic_twist_d[wing2.tag][d_i] = FEA_results_d[wing2.tag].elastic_twist[:, :, 0] / q_dyn_cs

                training['dClift_ddelta_' + letter] = (Clift_d[0,:] - Clift_d[1,:]) / (delta[0] - delta[1])
                training['dCdrag_ddelta_' + letter] = (Cdrag_d[0,:] - Cdrag_d[1,:]) / (delta[0] - delta[1])
                training['dCX_ddelta_'    + letter] = (CX_d[0,:]    - CX_d[1,:]   ) / (delta[0] - delta[1])
                training['dCY_ddelta_'    + letter] = (CY_d[0,:]    - CY_d[1,:]   ) / (delta[0] - delta[1])
                training['dCZ_ddelta_'    + letter] = (CZ_d[0,:]    - CZ_d[1,:]   ) / (delta[0] - delta[1])
                training['dCL_ddelta_'    + letter] = (CL_d[0,:]    - CL_d[1,:]   ) / (delta[0] - delta[1])
                training['dCM_ddelta_'    + letter] = (CM_d[0,:]    - CM_d[1,:]   ) / (delta[0] - delta[1])
                training['dCN_ddelta_'    + letter] = (CN_d[0,:]    - CN_d[1,:]   ) / (delta[0] - delta[1])

                if aerostructural_analyses is not None:
                    training['ddeflection_u_ddelta_'  + letter] = Data()
                    training['ddeflection_v_ddelta_'  + letter] = Data()
                    training['ddeflection_w_ddelta_'  + letter] = Data()
                    training['delastic_twist_ddelta_' + letter] = Data()
                    for wing2 in vehicle.wings:
                        training['ddeflection_u_ddelta_'  + letter][wing2.tag] = (deflection_u_d[wing2.tag][0]  - deflection_u_d[wing2.tag][1] ) / (delta[0] - delta[1])
                        training['ddeflection_v_ddelta_'  + letter][wing2.tag] = (deflection_v_d[wing2.tag][0]  - deflection_v_d[wing2.tag][1] ) / (delta[0] - delta[1])
                        training['ddeflection_w_ddelta_'  + letter][wing2.tag] = (deflection_w_d[wing2.tag][0]  - deflection_w_d[wing2.tag][1] ) / (delta[0] - delta[1])
                        training['delastic_twist_ddelta_' + letter][wing2.tag] = (elastic_twist_d[wing2.tag][0] - elastic_twist_d[wing2.tag][1]) / (delta[0] - delta[1])

                # reset to 0, not the real deflection, so later surfaces train in isolation
                setattr(control_surface, deflection_attr, 0)

    # restore real deflections now that training is done
    for wing in vehicle.wings:
        for control_surface in wing.control_surfaces:
            for letter, name, channel, flag, deflection_attr in cs_lookup(control_surface):
                setattr(control_surface, deflection_attr, delta_0[letter])

    # reset vortex distribution after training
    settings.vortex_distribution = VD_0
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
    AoA            = aerodynamics.training.angle_of_attack                  
    Beta           = aerodynamics.training.sideslip_angle
    training       = Data() 
    training.Mach  = np.array([sub_Mach[-1], sup_Mach[0]])
    u              = aerodynamics.training.u 
    pitch_rate     = aerodynamics.training.pitch_rate
    roll_rate      = aerodynamics.training.roll_rate
    yaw_rate       = aerodynamics.training.yaw_rate  
    
    # --------------------------------------------------------------------------------------------------------------
    # Alpha
    # --------------------------------------------------------------------------------------------------------------  
    Clift_alpha           =  np.concatenate((training_subsonic.Clift_alpha[:,-1][:,None] , training_supersonic.Clift_alpha[:,0][:,None] ), axis = 1)
    Clift_spanwise        =np.concatenate((training_subsonic.Clift_spanwise[:,-1][:,None] , training_supersonic.Clift_spanwise[:,0][:,None] ), axis = 1)
    Cdrag_induced_alpha   =  np.concatenate((training_subsonic.Cdrag_induced_alpha[:,-1][:,None]  , training_supersonic.Cdrag_induced_alpha[:,0][:,None] ), axis = 1) 
    CX_alpha              =  np.concatenate((training_subsonic.CX_alpha[:,-1][:,None]    , training_supersonic.CX_alpha[:,0][:,None] ), axis = 1)   
    CY_alpha              =  np.concatenate((training_subsonic.CY_alpha[:,-1][:,None]    , training_supersonic.CY_alpha[:,0][:,None] ), axis = 1)   
    CZ_alpha              =  np.concatenate((training_subsonic.CZ_alpha[:,-1][:,None]    , training_supersonic.CZ_alpha[:,0][:,None] ), axis = 1)   
    CL_alpha              =  np.concatenate((training_subsonic.CL_alpha[:,-1][:,None]    , training_supersonic.CL_alpha[:,0][:,None] ), axis = 1)   
    CM_alpha              =  np.concatenate((training_subsonic.CM_alpha[:,-1][:,None]    , training_supersonic.CM_alpha[:,0][:,None] ), axis = 1)   
    CN_alpha              =  np.concatenate((training_subsonic.CN_alpha[:,-1][:,None]    , training_supersonic.CN_alpha[:,0][:,None] ), axis = 1) 
    CM_0                  =  np.concatenate((training_subsonic.CM_0[:][-1,None]    , training_supersonic.CM_0[:][0,None] ))      
 
    # --------------------------------------------------------------------------------------------------------------
    # Beta 
    # -------------------------------------------------------------------------------------------------------------- 
    
    Clift_beta =  np.concatenate((training_subsonic.Clift_beta[:,-1][:,None] , training_supersonic.Clift_beta[:,0][:,None] ), axis = 1)      
    Cdrag_induced_beta =  np.concatenate((training_subsonic.Cdrag_induced_beta[:,-1][:,None] , training_supersonic.Cdrag_induced_beta[:,0][:,None] ), axis = 1)             
    CX_beta    =  np.concatenate((training_subsonic.CX_beta[:,-1][:,None]    , training_supersonic.CX_beta[:,0][:,None] ), axis = 1)        
    CY_beta    =  np.concatenate((training_subsonic.CY_beta[:,-1][:,None]    , training_supersonic.CY_beta[:,0][:,None] ), axis = 1)        
    CZ_beta    =  np.concatenate((training_subsonic.CZ_beta[:,-1][:,None]    , training_supersonic.CZ_beta[:,0][:,None] ), axis = 1)        
    CL_beta    =  np.concatenate((training_subsonic.CL_beta[:,-1][:,None]    , training_supersonic.CL_beta[:,0][:,None] ), axis = 1)        
    CM_beta    =  np.concatenate((training_subsonic.CM_beta[:,-1][:,None]    , training_supersonic.CM_beta[:,0][:,None] ), axis = 1)        
    CN_beta    =  np.concatenate((training_subsonic.CN_beta[:,-1][:,None]    , training_supersonic.CN_beta[:,0][:,None] ), axis = 1)        
 
    # -------------------------------------------------------      
    # Velocity u 
    # -------------------------------------------------------      
    CX_u        =   np.concatenate((training_subsonic.CX_u[:,-1][:,None]    , training_supersonic.CX_u[:,0][:,None] ), axis = 1)         
    CZ_u        =   np.concatenate((training_subsonic.CZ_u[:,-1][:,None]    , training_supersonic.CZ_u[:,0][:,None] ), axis = 1)          
    CM_u        =   np.concatenate((training_subsonic.CM_u[:,-1][:,None]    , training_supersonic.CM_u[:,0][:,None] ), axis = 1)         
             
                    
    # -------------------------------------------------------               
    # Pitch Rate 
    # -------------------------------------------------------        
    CZ_q        =  np.concatenate((training_subsonic.CZ_q[:,-1][:,None]    , training_supersonic.CZ_q[:,0][:,None] ), axis = 1)          
    CM_q        =  np.concatenate((training_subsonic.CM_q[:,-1][:,None]    , training_supersonic.CM_q[:,0][:,None] ), axis = 1)          
  
    # -------------------------------------------------------               
    # Roll  Rate 
    # -------------------------------------------------------                     
    CL_p        =  np.concatenate((training_subsonic.CL_p[:,-1][:,None]    , training_supersonic.CL_p[:,0][:,None] ), axis = 1)         
    CN_p        =  np.concatenate((training_subsonic.CN_p[:,-1][:,None]    , training_supersonic.CN_p[:,0][:,None] ), axis = 1)       


    # -------------------------------------------------------               
    # Yaw Rate 
    # -------------------------------------------------------                  
    CY_r        =  np.concatenate((training_subsonic.CY_r[:,-1][:,None]    , training_supersonic.CY_r[:,0][:,None] ), axis = 1)          
    CL_r        =  np.concatenate((training_subsonic.CL_r[:,-1][:,None]    , training_supersonic.CL_r[:,0][:,None] ), axis = 1)          
    CN_r        =  np.concatenate((training_subsonic.CN_r[:,-1][:,None]    , training_supersonic.CN_r[:,0][:,None] ), axis = 1)         
 
    # STABILITY COEFFICIENTS 
    training.Clift_alpha               = Clift_alpha 
    training.Cdrag_induced_alpha       = Cdrag_induced_alpha   
    training.CX_alpha                  = CX_alpha   
    training.CY_alpha                  = CY_alpha
    training.CZ_alpha                  = CZ_alpha  
    training.CL_alpha                  = CL_alpha
    training.CM_alpha                  = CM_alpha  
    training.CN_alpha                  = CN_alpha
    training.CM_0                      = CM_0  
    training.Clift_spanwise            = Clift_spanwise
    
    
    training.Clift_beta                = Clift_beta
    training.Cdrag_induced_beta        = Cdrag_induced_beta
    training.CX_beta                   = CX_beta
    training.CY_beta                   = CY_beta  
    training.CZ_beta                   = CZ_beta
    training.CL_beta                   = CL_beta
    training.CM_beta                   = CM_beta
    training.CN_beta                   = CN_beta   

    # STABILITY DERIVATIVES 
    training.dClift_dalpha = (Clift_alpha[0,:] - Clift_alpha[1,:]) / (AoA[0] - AoA[1])          
    training.dCX_dalpha    = (CX_alpha[0,:] - CX_alpha[1,:]) / (AoA[0] - AoA[1])            
    training.dCX_du        = (CX_u[0,:] - CX_u[1,:]) / (u[0] - u[1])                                 
         
    training.dCY_dbeta  = (CY_beta[0,:] - CY_beta[1,:]) / (Beta[0] - Beta[1])    
    training.dCY_dr     = (CY_r[0,:] - CY_r[1,:]) / (yaw_rate[0]-yaw_rate[1])                     
    
    training.dCZ_dalpha = (CZ_alpha[0,:] - CZ_alpha[1,:]) / (AoA[0] - AoA[1])             
    training.dCZ_du     = (CZ_u[0,:] - CZ_u[1,:]) / (u[0] - u[1])                                              
    training.dCZ_dq     = (CZ_q[0,:] - CZ_q[1,:]) / (pitch_rate[0]-pitch_rate[1])    

    training.dCL_dbeta  = (CL_beta[0,:] - CL_beta[1,:]) / (Beta[0] - Beta[1])                                                    
    training.dCL_dp     = (CL_p[0,:] - CL_p[1,:]) / (roll_rate[0]-roll_rate[1])                
    training.dCL_dr     = (CL_r[0,:] - CL_r[1,:]) / (yaw_rate[0]-yaw_rate[1])                    
    
    training.dCM_dalpha = (CM_alpha[0,:] - CM_alpha[1,:]) / (AoA[0] - AoA[1])          
    training.dCM_du     = (CM_u[0,:] - CM_u[1,:]) / (u[0] - u[1])                                               
    training.dCM_dq     = (CM_q[0,:] - CM_q[1,:]) / (pitch_rate[0]-pitch_rate[1])             
    
    training.dCN_dbeta  = (CN_beta[0,:] - CN_beta[1,:]) / (Beta[0] - Beta[1])                
    training.dCN_dp =  (CN_p[0,:] - CN_p[1,:]) / (roll_rate[0]-roll_rate[1])                 
    training.dCN_dr =  (CN_r[0,:] - CN_r[1,:]) / (yaw_rate[0]-yaw_rate[1])


    # for control surfaces, subtract influence WITHOUT control surface deflected from coefficients WITH control
    # surface deflected; see control_surface_registry.py for why every type is treated identically here.
    for cls, letter, name, channel, flag, deflection_attr in CONTROL_SURFACE_TYPES:
        if getattr(aerodynamics, flag):
            for coeff in ('Clift', 'Cdrag', 'CX', 'CY', 'CZ', 'CL', 'CM', 'CN'):
                key = 'd' + coeff + '_ddelta_' + letter
                training[key] = np.array([training_subsonic[key][-1], training_subsonic[key][0]])
            for field in ('ddeflection_u_ddelta_', 'ddeflection_v_ddelta_', 'ddeflection_w_ddelta_', 'delastic_twist_ddelta_'):
                key = field + letter
                if key in training_subsonic:
                    training[key] = Data()
                    for wing_tag in training_subsonic[key].keys():
                        training[key][wing_tag] = np.array([training_subsonic[key][wing_tag][-1], training_subsonic[key][wing_tag][0]])

    return training


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

        VLM_results         = VLM(conditions,settings,vehicle)         
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
            settings.vortex_distribution  = settings.vortex_distribution    
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
    
    return RES

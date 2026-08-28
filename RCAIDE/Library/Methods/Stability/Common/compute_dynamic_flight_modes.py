# RCAIDE/Library/Methods/Stability/compute_dynamic_flight_modes.py
# 
# 
# Created:  Apr 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
import RCAIDE 

# python imports 
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  compute_dynamic_flight_modes
# ----------------------------------------------------------------------------------------------------------------------
def compute_dynamic_flight_modes(state,settings,vehicle): 
    """This function follows the stability axis EOM derivation in Blakelock
    to return the vehicle's dynamic modes and state space 
    
    Assumptions:
       Linerarized Equations are used following the reference below

    Source:
      Automatic Control of Aircraft and Missiles by J. Blakelock Pg 23 and 117
      Dynanmics of Flight Stability and Control by Bernard Edkin and Llyod Reid Chapter 5, 

    Inputs:
       conditions.aerodynamics  
       conditions.static_stability

    Outputs: 
       conditions.dynamic_stability.LatModes
       conditions.dynamic_stability.LongModes 

    Properties Used:
       N/A
     """

    conditions = state.conditions 
    AoA        = conditions.aerodynamics.angles.alpha  
    n_cpts     = state.numerics.number_of_control_points 
    S_ref      = vehicle.reference_area  
    c_ref      = vehicle.reference_chord
    b_ref      = vehicle.reference_span
 
    if (np.count_nonzero(vehicle.mass_properties.moments_of_inertia.tensor) > 0) and (np.all(np.isnan(AoA)) !=  True):
        method = settings.dynamic_stability_method
        if method != "linearized_derivatives":
            raise NotImplementedError(
                "dynamic_stability_method '" + str(method) + "' is not implemented. "
                "Only 'linearized_derivatives' (the classical small-perturbation "
                "stability-derivative EOM) is currently available."
            )

        g      = conditions.freestream.gravity
        rho    = conditions.freestream.density
        u0     = conditions.freestream.velocity
        qDyn0  = conditions.freestream.dynamic_pressure

        # VLM-based stability derivatives are undefined at zero airspeed (hover);
        # skip rather than let u0/qDyn0 divisions silently produce inf/nan.
        near_zero_airspeed_threshold = 1.0 # m/s
        if np.any(np.abs(u0) < near_zero_airspeed_threshold):
            import warnings
            warnings.warn(
                "compute_dynamic_flight_modes: airspeed below "
                + str(near_zero_airspeed_threshold) + " m/s (hover or "
                "near-hover) at one or more control points. This method's "
                "VLM-based stability derivatives are undefined at zero "
                "airspeed; skipping dynamic-mode computation for this "
                "segment rather than returning silently invalid results."
            )
            return
        theta0 = np.arctan(conditions.frames.inertial.velocity_vector[:,2]/conditions.frames.inertial.velocity_vector[:,0])[:,None] 
        SS     = conditions.static_stability
        SSD    = SS.derivatives 
        DS     = conditions.dynamic_stability 
        Ixx    = conditions.weights.vehicle.moments_of_inertia_Ixx  
        Ixz    = conditions.weights.vehicle.moments_of_inertia_Ixz  
        Iyy    = conditions.weights.vehicle.moments_of_inertia_Iyy  
        Izx    = conditions.weights.vehicle.moments_of_inertia_Izx  
        Izz    = conditions.weights.vehicle.moments_of_inertia_Izz  
        m      = conditions.weights.vehicle.mass
        
        if np.all(conditions.static_stability.spiral_criteria) == 0: 
            conditions.static_stability.spiral_criteria = SSD.CL_beta*SSD.CN_r / (SSD.CL_r*SSD.CN_beta) 
          
        ## Build longitudinal EOM A Matrix (stability axis)
        ALon = np.zeros((n_cpts,4,4))
        BLon = np.zeros((n_cpts,4,1))  
        

        # Locate the main wing and, if present, an aft tail by geometry.
        vertical_types    = (RCAIDE.Library.Components.Wings.Vertical_Tail,
                              RCAIDE.Library.Components.Wings.Vertical_Tail_All_Moving)
        lifting_surfaces  = [w for w in vehicle.wings if not isinstance(w, vertical_types)]
        main_wing         = max(lifting_surfaces, key=lambda w: w.areas.reference) if lifting_surfaces else None

        horizontal_tail = None
        if main_wing is not None:
            main_AC          = main_wing.origin[0][0] + main_wing.aerodynamic_center[0]
            tail_size_margin = 0.5 # candidate tail must be under half the main wing's area
            tail_candidates  = [w for w in lifting_surfaces if w is not main_wing
                                 and w.areas.reference < tail_size_margin * main_wing.areas.reference
                                 and (w.origin[0][0] + w.aerodynamic_center[0]) > main_AC]
            if len(tail_candidates) > 1:
                import warnings
                warnings.warn(str(len(tail_candidates)) + " candidate aft tail surfaces found; using the largest.")
            horizontal_tail = max(tail_candidates, key=lambda w: w.areas.reference) if tail_candidates else None

        a_t               = 2 * np.pi # dCL_t_dalphat
        has_separate_tail = (horizontal_tail is not None) and (main_wing is not None)
        if has_separate_tail:
            l_t              = (horizontal_tail.origin[0][0] +horizontal_tail.aerodynamic_center[0]) - vehicle.mass_properties.center_of_gravity[0][0] # disstance from CG to tail AC
            S_t              = horizontal_tail.areas.reference # tail area
            S                = main_wing.areas.reference # wing area
            l_bar_t          = (horizontal_tail.origin[0][0] +  horizontal_tail.aerodynamic_center[0]) -(main_wing.origin[0][0] +  main_wing.aerodynamic_center[0])  # distance from AC of main wing to tail AC
            V_H              = ( l_bar_t *S_t ) /(c_ref *S)  # tail volume
            dEpsilon_dalpha  =  0.3
            SSD.CZ_alpha_dot =  a_t * dEpsilon_dalpha *  (l_t /u0) *  (S_t / S)
            SSD.CM_alpha_dot =  -a_t * V_H * dEpsilon_dalpha*  (l_t /u0)
        else:
            import warnings
            warnings.warn(
                "No aft tail-like surface found (single lifting surface e.g. "
                "flying-wing/BWB, a forward/canard surface, or a second "
                "surface too close in size to count as a tail e.g. tandem/"
                "box-wing); the tail-downwash-lag alpha-dot damping terms "
                "(CZ_alpha_dot, CM_alpha_dot) are not modeled for this "
                "configuration and are set to zero."
            )
            SSD.CZ_alpha_dot = 0.0 * u0
            SSD.CM_alpha_dot = 0.0 * u0
        Cw               = m * g / (qDyn0 * S_ref)
        Xu               = rho * u0 * S_ref * Cw * np.sin(theta0) + 0.5 * rho * u0 * S_ref * SSD.CX_u  
        Xw               = 0.5 * rho * u0 * S_ref * SSD.CX_alpha     
        Zu               = -rho * u0 * S_ref * Cw * np.cos(theta0) + 0.5 * rho * u0 * S_ref * SSD.CZ_u 
        Zw               = 0.5 * rho * u0 * S_ref * SSD.CZ_alpha   
        Zq               = 0.25 * rho * u0 * c_ref * S_ref *  SSD.CZ_q    
        Mu               =  0.5 * rho * u0 * c_ref * S_ref * SSD.CM_u     
        Mw               = 0.5 * rho * u0 * c_ref * S_ref * SSD.CM_alpha  
        Mq               = 0.25 * rho * u0 * c_ref * c_ref * S_ref * SSD.CM_q     
        ZwDot            = 0.25 * rho * c_ref * S_ref * SSD.CZ_alpha_dot  
        MwDot            = 0.25 * rho * c_ref * S_ref * SSD.CM_alpha_dot  
        
        
        ALon[:,0,0] = (Xu / m).T[0]
        ALon[:,0,1] = (Xw / m).T[0] 
        ALon[:,0,3] = (-g * np.cos(theta0)).T[0]
        ALon[:,1,0] = (Zu / (m - ZwDot)).T[0]
        ALon[:,1,1] = (Zw / (m - ZwDot)).T[0]
        ALon[:,1,2] = ((Zq + (m * u0)) / (m - ZwDot) ).T[0]
        ALon[:,1,3] = (-m * g * np.sin(theta0) / (m - ZwDot)).T[0]
        ALon[:,2,0] = ((Mu + MwDot * Zu / (m - ZwDot)) / Iyy).T[0]  
        ALon[:,2,1] = ((Mw + MwDot * Zw / (m - ZwDot)) / Iyy).T[0] 
        ALon[:,2,2] = ((Mq + MwDot * (Zq + m * u0) / (m - ZwDot)) / Iyy ).T[0] 
        ALon[:,2,3] = (-MwDot * m * g * np.sin(theta0) / (Iyy * (m - ZwDot))).T[0] 
        ALon[:,3,0] = 0
        ALon[:,3,1] = 0
        ALon[:,3,2] = 1
        ALon[:,3,3] = 0 
 
        Control_Surfaces      = RCAIDE.Library.Components.Wings.Control_Surfaces
        all_control_surfaces  = [cs for wing in vehicle.wings for cs in wing.control_surfaces]
        pitch_letters         = [letter for letter, cls in (('e', Control_Surfaces.Elevator),
                                                              ('pe', Control_Surfaces.Elevon),
                                                              ('pr', Control_Surfaces.Ruddervator))
                                  if any(isinstance(cs, cls) for cs in all_control_surfaces)]

        Xe = 0 # Neglect
        if pitch_letters:
            Clift_de = sum(getattr(SSD, 'Clift_delta_' + letter, 0.0) for letter in pitch_letters)
            CM_de    = sum(getattr(SSD, 'CM_delta_'    + letter, 0.0) for letter in pitch_letters)
            Ze       = 0.5 * rho * u0 * u0 * S_ref * Clift_de
            Me       = 0.5 * rho * u0 * u0 * S_ref * c_ref * CM_de
        else:
            import warnings
            warnings.warn(
                "No elevator, elevon, or ruddervator found in vehicle.wings[*].control_surfaces; "
                "setting pitch-control effectiveness terms (Ze, Me) to zero."
            )
            Ze = Me = 0.0 * u0

        BLon[:,0,0] = Xe / m[:, 0]
        BLon[:,1,0] = (Ze / (m - ZwDot)).T[0]
        BLon[:,2,0] = (Me / Iyy + MwDot / Iyy * Ze / (m - ZwDot)).T[0]
        BLon[:,3,0] = 0        

        # Look at eigenvalues and eigenvectors
        LonModes                  = np.zeros((n_cpts,4), dtype = complex)
        VLon                      = np.zeros((n_cpts,4,4), dtype = complex)
        phugoidFreqHz             = np.zeros((n_cpts,1))
        phugoidDamping            = np.zeros((n_cpts,1))
        phugoidTimeDoubleHalf     = np.zeros((n_cpts,1))
        shortPeriodFreqHz         = np.zeros((n_cpts,1))
        shortPeriodDamping        = np.zeros((n_cpts,1))
        shortPeriodTimeDoubleHalf = np.zeros((n_cpts,1)) 
        try: 
            LonModes  , VLon = np.linalg.eig(ALon)
            phugoidInd                    = np.argmax(LonModes,axis=1)
            Ind                           = np.arange(n_cpts)
            phugoidFreqHz                 = np.atleast_2d(abs(LonModes[Ind, phugoidInd]) / (2 * np.pi)).T
            phugoidDamping                = np.atleast_2d(np.sqrt(1/ (1 + ( LonModes[Ind,phugoidInd].imag/ LonModes[Ind,phugoidInd].real )**2 ))).T 
            phugoidTimeDoubleHalf         = np.log(2) / abs(2 * np.pi * phugoidFreqHz * phugoidDamping)
            
            # Find short period
            shortPeriodInd               = np.argmin(LonModes,axis=1)
            shortPeriodFreqHz            = np.atleast_2d(abs(LonModes[Ind, shortPeriodInd]) / (2 * np.pi)).T
            shortPeriodDamping           = np.atleast_2d(np.sqrt(1/ (1 + (LonModes[Ind, shortPeriodInd].imag/LonModes[Ind,shortPeriodInd].real)**2 ))).T
            shortPeriodTimeDoubleHalf    = np.log(2) / abs(2 * np.pi * shortPeriodFreqHz * shortPeriodDamping)
        except:
            pass
        
        ## Build lateral EOM A Matrix (stability axis)
        ALat = np.zeros((n_cpts,4,4))
        BLat = np.zeros((n_cpts,4,1))
        
        # Rotate inertias into the stability axis (function of angle of attack)
        R    = np.zeros((n_cpts,2,2))
        modI = np.zeros((n_cpts,2,2)) 

        R[:,0, 0]  = np.cos(AoA[:, 0])
        R[:,0, 1]  = - np.sin(AoA[:, 0])
        R[:,1, 0]  = np.sin( AoA[:, 0])
        R[:,1, 1]  = np.cos(AoA[:, 0])
         
        modI[:,0, 0]    = Ixx[:, 0]
        modI[:,0, 1]    = Ixz[:, 0]
        modI[:,1, 0]    = Izx[:, 0]
        modI[:,1, 1]    = Izz[:, 0]
                               
        INew      = R * modI * np.transpose(R, axes = (0,2,1))
        IxxStab   =  INew[:,0,0]
        IxzStab   = -INew[:,0,1]
        IzzStab   =  INew[:,1,1]
        Ixp       = np.atleast_2d((IxxStab * IzzStab - IxzStab**2) / IzzStab).T
        Izp       = np.atleast_2d((IxxStab * IzzStab - IxzStab**2) / IxxStab).T
        Ixzp      = np.atleast_2d(IxzStab / (IxxStab * IzzStab - IxzStab**2)).T 
            
        Yv = 0.5 * rho * u0 * S_ref * SSD.CY_beta
        Yp = 0.25 * rho * u0 * b_ref * S_ref * SSD.CY_p
        Yr = 0.25 * rho * u0 * b_ref * S_ref * SSD.CY_r
        Lv = 0.5 * rho * u0 * b_ref * S_ref * SSD.CL_beta
        Lp = 0.25 * rho * u0 * b_ref**2 * S_ref * SSD.CL_p
        Lr = 0.25 * rho * u0 * b_ref**2 * S_ref * SSD.CL_r
        Nv = 0.5 * rho * u0 * b_ref * S_ref * SSD.CN_beta
        Np = 0.25 * rho * u0 * b_ref**2 * S_ref * SSD.CN_p
        Nr = 0.25 * rho * u0 * b_ref**2 * S_ref * SSD.CN_r
        
        roll_letters = [letter for letter, cls in (('a', Control_Surfaces.Aileron),
                                                     ('se', Control_Surfaces.Elevon),
                                                     ('sf', Control_Surfaces.Flaperon))
                         if any(isinstance(cs, cls) for cs in all_control_surfaces)]

        if roll_letters:
            CY_da = sum(getattr(SSD, 'CY_delta_' + letter, 0.0) for letter in roll_letters)
            CL_da = sum(getattr(SSD, 'CL_delta_' + letter, 0.0) for letter in roll_letters)
            CN_da = sum(getattr(SSD, 'CN_delta_' + letter, 0.0) for letter in roll_letters)
            Ya    = 0.5 * rho * u0 * u0 * S_ref * CY_da
            La    = 0.5 * rho * u0 * u0 * S_ref * b_ref * CL_da
            Na    = 0.5 * rho * u0 * u0 * S_ref * b_ref * CN_da
        else:
            import warnings
            warnings.warn(
                "No aileron, elevon, or flaperon found in vehicle.wings[*].control_surfaces; "
                "setting roll-control effectiveness terms (Ya, La, Na) to zero."
            )
            Ya = La = Na = 0.0 * u0

        BLat[:,0,0] = (Ya / m).T[0]
        BLat[:,1,0] = (La / Ixp + Ixzp * Na).T[0]
        BLat[:,2,0] = (Ixzp * La + Na / Izp).T[0]
        BLat[:,3,0] = 0
     
        ALat[:,0,0] = (Yv / m).T[0]
        ALat[:,0,1] = (Yp / m).T[0]
        ALat[:,0,2] = (Yr/m - u0).T[0]
        ALat[:,0,3] = (g * np.cos(theta0)).T[0]
        
        ALat[:,1,0] = (Lv / Ixp + Ixzp * Nv).T[0] 
        ALat[:,1,1] = (Lp / Ixp + Ixzp * Np).T[0] 
        ALat[:,1,2] = (Lr / Ixp + Ixzp * Nr).T[0] 
        ALat[:,1,3] = 0
        
        ALat[:,2,0] = (Ixzp * Lv + Nv / Izp).T[0] 
        ALat[:,2,1] = (Ixzp * Lp + Np / Izp).T[0] 
        ALat[:,2,2] = (Ixzp * Lr + Nr / Izp).T[0] 
        ALat[:,2,3] = 0
        
        ALat[:,3,0] = 0
        ALat[:,3,1] = 1
        ALat[:,3,2] = (np.tan(theta0)).T[0] 
        ALat[:,3,3] = 0
                                    
        LatModes                    = np.zeros((n_cpts,4),dtype=complex)
        VLat                        = np.zeros((n_cpts,4,4),dtype=complex)
        dutchRollFreqHz             = np.zeros((n_cpts,1))
        dutchRollDamping            = np.zeros((n_cpts,1))
        dutchRollTimeDoubleHalf     = np.zeros((n_cpts,1))
        rollSubsistenceFreqHz       = np.zeros((n_cpts,1))
        rollSubsistenceTimeConstant = np.zeros((n_cpts,1))
        rollSubsistenceDamping      = np.zeros((n_cpts,1))
        spiralFreqHz                = np.zeros((n_cpts,1))
        spiralTimeDoubleHalf        = np.zeros((n_cpts,1))
        spiralDamping               = np.zeros((n_cpts,1))
        dutchRoll_mode_real         = np.zeros((n_cpts,1))
         
        try: 
            LatModes  , VLat = np.linalg.eig(ALat) # State order: beta, p, r, phi

            real_parts = LatModes.real
            unique_elements, counts = np.unique(real_parts, return_counts=True, axis=1)
            idx = np.where(counts==2)[0]
    
            dutchRollFreqHz         = abs(LatModes[:,idx]) / (2 * np.pi)
            dutchRollDamping        = np.sqrt(1/ (1 + ( LatModes[:,idx].imag/ LatModes[:,idx].real )**2 ))  
            dutchRollTimeDoubleHalf = np.log(2) / abs(2 * np.pi * dutchRollFreqHz * dutchRollDamping)
            dutchRoll_mode_real     = LatModes[:,idx].real / (2 * np.pi)
             
            dutch_roll_idx                = np.where( unique_elements[:, idx] != LatModes.real ) 
            remaining_modes               = LatModes[dutch_roll_idx].reshape(n_cpts,2)
            rollInd                       = np.argmin(remaining_modes,axis=1)
            rollSubsistenceFreqHz         = np.atleast_2d(abs(remaining_modes[Ind,rollInd]) / 2 / np.pi).T
            rollSubsistenceDamping        =  np.atleast_2d(- np.sign(remaining_modes[Ind,rollInd].real)).T
            rollSubsistenceTimeConstant   = 1 / (2 * np.pi * rollSubsistenceFreqHz  * rollSubsistenceDamping)
            
            # Find spiral mode 
            spiralInd                   = np.argmax(remaining_modes,axis=1)           
            spiralFreqHz                = np.atleast_2d(abs(remaining_modes[Ind,spiralInd]) / 2 / np.pi).T
            spiralDamping               = np.atleast_2d(- np.sign(remaining_modes[Ind,spiralInd].real)).T
            spiralTimeDoubleHalf        = np.log(2) / abs(2 * np.pi * spiralFreqHz  * spiralDamping)
        except:
            pass 
        
        # Inertial coupling susceptibility
        # See Etkin & Reid pg. 118 
        DS.pMax = min(min(np.sqrt(-Mw * u0 / (Izz - Ixx))), min(np.sqrt(-Nv * u0 / (Iyy - Ixx)))) 
        
        # -----------------------------------------------------------------------------------------------------------------------  
        # Store Results
        # ------------------------------------------------------------------------------------------------------------------------  
        DS.LongModes.A                            = ALon
        DS.LongModes.eigenvectors                 = VLon
        DS.LongModes.LongModes                    = LonModes
        DS.LongModes.phugoidFreqHz                = phugoidFreqHz
        DS.LongModes.phugoidDamping               = phugoidDamping
        DS.LongModes.phugoidTimeDoubleHalf        = phugoidTimeDoubleHalf
        DS.LongModes.shortPeriodFreqHz            = shortPeriodFreqHz
        DS.LongModes.shortPeriodDamping           = shortPeriodDamping
        DS.LongModes.shortPeriodTimeDoubleHalf    = shortPeriodTimeDoubleHalf
                                                                        
        DS.LatModes.A                              = ALat
        DS.LatModes.eigenvectors                   = VLat
        DS.LatModes.LatModes                      = LatModes
        DS.LatModes.dutchRollFreqHz               = dutchRollFreqHz
        DS.LatModes.dutchRollDamping              = dutchRollDamping
        DS.LatModes.dutchRollTimeDoubleHalf       = dutchRollTimeDoubleHalf
        DS.LatModes.dutchRoll_mode_real           = dutchRoll_mode_real 
        DS.LatModes.rollSubsistenceFreqHz         = rollSubsistenceFreqHz
        DS.LatModes.rollSubsistenceTimeConstant   = rollSubsistenceTimeConstant
        DS.LatModes.rollSubsistenceDamping        = rollSubsistenceDamping
        DS.LatModes.spiralFreqHz                  = spiralFreqHz
        DS.LatModes.spiralTimeDoubleHalf          = spiralTimeDoubleHalf 
        DS.LatModes.spiralDamping                 = spiralDamping
    
    return 

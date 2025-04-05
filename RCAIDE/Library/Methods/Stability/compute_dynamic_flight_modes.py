# RCAIDE/Library/Methods/Stability/Dynamic_Stability/compute_dynamic_flight_modes.py
# 
# 
# Created:  Apr 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
import RCAIDE 
from RCAIDE.Library.Components.Wings.Control_Surfaces import Aileron , Elevator  

# python imports 
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  compute_dynamic_flight_modes
# ----------------------------------------------------------------------------------------------------------------------
def compute_dynamic_flight_modes(state,settings,aircraft): 
    """This function follows the stability axis EOM derivation in Blakelock
    to return the aircraft's dynamic modes and state space 
    
    Assumptions:
       Linerarized Equations are used following the reference below

    Source:
      Automatic Control of Aircraft and Missiles by J. Blakelock Pg 23 and 117
      Dynanmics of Flight Stability and Control by Bernard Edkin and Llyod Reid Chapter 5, 

    Inputs:
       conditions.aerodynamics  
       conditions.static_stability  
       conditions.stability.dynamic 

    Outputs: 
       conditions.dynamic_stability.LatModes
       conditions.dynamic_stability.LongModes 

    Properties Used:
       N/A
     """

    conditions = state.conditions 
    AoA        = conditions.aerodynamics.angles.alpha
    
    vertical_fligth_flag = False
    if isinstance(state,RCAIDE.Framework.Mission.Segments.Vertical_Flight.Climb) or \
       isinstance(state,RCAIDE.Framework.Mission.Segments.Vertical_Flight.Hover) or \
       isinstance(state,RCAIDE.Framework.Mission.Segments.Vertical_Flight.Descent):
        vertical_fligth_flag = True
    
    if (np.count_nonzero(aircraft.mass_properties.moments_of_inertia.tensor) > 0) and  (vertical_fligth_flag !=  True) and (np.all(np.isnan(AoA)) !=  True):
        g          = conditions.freestream.gravity  
        rho        = conditions.freestream.density
        u0         = conditions.freestream.velocity
        qDyn0      = conditions.freestream.dynamic_pressure  
        theta0     = np.arctan(conditions.frames.inertial.velocity_vector[:,2]/conditions.frames.inertial.velocity_vector[:,0])[:,None] 
        SS         = conditions.static_stability
        SSD        = SS.derivatives 
        DS         = conditions.dynamic_stability
                 
        num_cases  = len(AoA)
         
        b_ref  = conditions.b_ref
        c_ref  = conditions.c_ref
        S_ref  = conditions.S_ref  
        moments_of_inertia = aircraft.mass_properties.moments_of_inertia.tensor
        Ixx    = moments_of_inertia[0][0]
        Iyy    = moments_of_inertia[1][1]
        Izz    = moments_of_inertia[2][2]    
        if aircraft.mass_properties.mass == 0:
            m  = aircraft.mass_properties.max_takeoff
        elif aircraft.mass_properties.max_takeoff == 0:
            m = aircraft.mass_properties.mass
        else:
            raise AttributeError("Specify Vehicle Mass") 
        
        if np.all(conditions.static_stability.spiral_criteria) == 0: 
            conditions.static_stability.spiral_criteria = SSD.CL_beta*SSD.CN_r / (SSD.CL_r*SSD.CN_beta) 
          
        ## Build longitudinal EOM A Matrix (stability axis)
        ALon = np.zeros((num_cases,4,4))
        BLon = np.zeros((num_cases,4,1)) 
        CLon = np.zeros((num_cases,4,4))
        

        # Elevator effectiveness
        ht_tag         =  None
        main_wing_tag  = None
        for wing in aircraft.wings:
            if isinstance(wing,RCAIDE.Library.Components.Wings.Horizontal_Tail):
                ht_tag  = wing.tag
            if isinstance(wing,RCAIDE.Library.Components.Wings.Main_Wing):
                main_wing_tag = wing.tag
                
        if main_wing_tag != None and  ht_tag !=None: 
            main_wing       = aircraft.wings[main_wing_tag]     
            horizontal_tail = aircraft.wings[ht_tag] 
            
            # unpack unit conversions 
            V_t_prime       =  u0
            a_t             = 2 * np.pi # dCL_t_dalphat 
            l_t             = (horizontal_tail.origin[0][0] +horizontal_tail.aerodynamic_center[0]) - aircraft.mass_properties.center_of_gravity[0][0] # disstance from CG to tail AC
            S_t             = horizontal_tail.areas.reference # tail area
            S               = main_wing.areas.reference # wing area
            l_bar_t         = (horizontal_tail.origin[0][0] +  horizontal_tail.aerodynamic_center[0]) -(main_wing.origin[0][0] +  main_wing.aerodynamic_center[0])  # distance from AC of main wing to tail AC
            V_H             = ( l_bar_t *S_t ) /(c_ref *S)  # tail volume
            dEpsilon_dalpha =  0.3
            
            SSD.CZ_alpha_dot =  a_t * dEpsilon_dalpha *  (l_t /u0) *  (S_t / S) 
            SSD.CM_alpha_dot =  -a_t * V_H * dEpsilon_dalpha*  (l_t /u0)        
                        
        for i in range(num_cases): 
            CLon[i,:,:] = np.eye(4)        
        Cw         = m * g / (qDyn0 * S_ref)  
        Xu         = rho * u0 * S_ref * Cw * np.sin(theta0) + 0.5 * rho * u0 * S_ref * SSD.CX_u  
        Xw         = 0.5 * rho * u0 * S_ref * SSD.CX_alpha     
        Zu         = -rho * u0 * S_ref * Cw * np.cos(theta0) + 0.5 * rho * u0 * S_ref * SSD.CZ_u 
        Zw         = 0.5 * rho * u0 * S_ref * SSD.CZ_alpha   
        Zq         = 0.25 * rho * u0 * c_ref * S_ref *  SSD.CZ_q    
        Mu         =  0.5 * rho * u0 * c_ref * S_ref * SSD.CM_u     
        Mw         = 0.5 * rho * u0 * c_ref * S_ref * SSD.CM_alpha  
        Mq         = 0.25 * rho * u0 * c_ref * c_ref * S_ref * SSD.CM_q     
        ZwDot      = 0.25 * rho * c_ref * S_ref * SSD.CZ_alpha_dot  
        MwDot      = 0.25 * rho * c_ref * S_ref * SSD.CM_alpha_dot  
        
        
        ALon[:,0,0] = (Xu / m).T[0]
        ALon[:,0,1] = (Xw / m).T[0] 
        ALon[:,0,3] = (-g * np.cos(theta0)).T[0]
        ALon[:,1,0] = (Zu / (m - ZwDot)).T[0]
        ALon[:,1,1] = (Zw / (m - ZwDot)).T[0] 
        ALon[:,1,2] = ((Zq + (m * u0)) / (m - ZwDot) ).T[0]
        ALon[:,2,0] = ((Mu + MwDot * Zu / (m - ZwDot)) / Iyy).T[0]  
        ALon[:,2,1] = ((Mw + MwDot * Zw / (m - ZwDot)) / Iyy).T[0] 
        ALon[:,2,2] = ((Mq + MwDot * (Zq + m * u0) / (m - ZwDot)) / Iyy ).T[0] 
        ALon[:,2,3] = (-MwDot * m * g * np.sin(theta0) / (Iyy * (m - ZwDot))).T[0] 
        ALon[:,3,0] = 0
        ALon[:,3,1] = 0
        ALon[:,3,2] = 1
        ALon[:,3,3] = 0 

        for wing in aircraft.wings: 
            if wing.control_surfaces :
                for ctrl_surf in wing.control_surfaces: 
                    if (type(ctrl_surf) ==  Elevator):
                        ele = conditions.control_surfaces.elevator.static_stability.coefficients 
                        Xe  = 0 # Neglect
                        Ze  = 0.5 * rho * u0 * u0 * S_ref * ele.lift
                        Me  = 0.5 * rho * u0 * u0 * S_ref * c_ref * ele.M
                        
                        BLon[:,0,0] = Xe / m
                        BLon[:,1,0] = (Ze / (m - ZwDot)).T[0]
                        BLon[:,2,0] = (Me / Iyy + MwDot / Iyy * Ze / (m - ZwDot)).T[0]
                        BLon[:,3,0] = 0        
         
        # Look at eigenvalues and eigenvectors
        LonModes                  = np.zeros((num_cases,4), dtype = complex)
        phugoidFreqHz             = np.zeros((num_cases,1))
        phugoidDamping            = np.zeros((num_cases,1))
        phugoidTimeDoubleHalf     = np.zeros((num_cases,1))
        shortPeriodFreqHz         = np.zeros((num_cases,1))
        shortPeriodDamping        = np.zeros((num_cases,1))
        shortPeriodTimeDoubleHalf = np.zeros((num_cases,1))
        
        if np.any(np.isnan(ALon)):
            pass
        else:
            for i_long in range(num_cases):
                D  , V = np.linalg.eig(ALon[i_long,:,:]) # State order: u, w, q, theta
                LonModes[i_long,:] = D
                
                # Find phugoid
                phugoidInd                    = np.argmax(D)  
                phugoidFreqHz[i_long]         = abs(D[phugoidInd]) / (2 * np.pi)
                phugoidDamping[i_long]        = np.sqrt(1/ (1 + ( D[phugoidInd].imag/ D[phugoidInd].real )**2 ))  
                phugoidTimeDoubleHalf[i_long] = np.log(2) / abs(2 * np.pi * phugoidFreqHz[i_long] * phugoidDamping[i_long])
                
                # Find short period
                shortPeriodInd                    = np.argmin(D)  
                shortPeriodFreqHz[i_long]         = abs(D[shortPeriodInd]) / (2 * np.pi)
                shortPeriodDamping[i_long]        = np.sqrt(1/ (1 + (D[shortPeriodInd].imag/D[shortPeriodInd].real)**2 ))  
                shortPeriodTimeDoubleHalf[i_long] = np.log(2) / abs(2 * np.pi * shortPeriodFreqHz[i_long] * shortPeriodDamping[i_long]) 
        
        ## Build lateral EOM A Matrix (stability axis)
        ALat = np.zeros((num_cases,4,4))
        BLat = np.zeros((num_cases,4,1))
        CLat = np.zeros((num_cases,4,4))
        for c_i1 in range(num_cases): 
            CLat[c_i1,:,:] = np.eye(4) 
        DLat = np.zeros((num_cases,4,1))
        
        # Need to compute Ixx, Izz, and Ixz as a function of alpha
        Ixp  = np.zeros((num_cases,1))
        Izp  = np.zeros((num_cases,1))
        Ixzp = np.zeros((num_cases,1))
        
        for c_i2 in range(num_cases): 
            R       = np.array( [[np.cos(AoA[c_i2][0]) ,  - np.sin(AoA[c_i2][0]) ], [ np.sin( AoA[c_i2][0]) , np.cos(AoA[i][0])]])
            modI    = np.array([[moments_of_inertia[0][0],moments_of_inertia[0][2]],[moments_of_inertia[2][0],moments_of_inertia[2][2]]] ) 
            INew    = R * modI  * np.transpose(R)
            IxxStab =  INew[0,0]
            IxzStab = -INew[0,1]
            IzzStab =  INew[1,1]
            Ixp[c_i2]  = (IxxStab * IzzStab - IxzStab**2) / IzzStab
            Izp[c_i2]  = (IxxStab * IzzStab - IxzStab**2) / IxxStab
            Ixzp[c_i2] = IxzStab / (IxxStab * IzzStab - IxzStab**2) 
            
        Yv = 0.5 * rho * u0 * S_ref * SSD.CY_beta 
        Yr = 0.25 * rho * u0 * b_ref * S_ref * SSD.CY_r
        Lv = 0.5 * rho * u0 * b_ref * S_ref * SSD.CL_beta
        Lp = 0.25 * rho * u0 * b_ref**2 * S_ref * SSD.CL_p
        Lr = 0.25 * rho * u0 * b_ref**2 * S_ref * SSD.CL_r
        Nv = 0.5 * rho * u0 * b_ref * S_ref * SSD.CN_beta
        Np = 0.25 * rho * u0 * b_ref**2 * S_ref * SSD.CN_p
        Nr = 0.25 * rho * u0 * b_ref**2 * S_ref * SSD.CN_r
        
        # Aileron effectiveness 
        for wing in aircraft.wings:
            if wing.control_surfaces :
                for ctrl_surf in wing.control_surfaces:
                    if (type(ctrl_surf) ==  Aileron): 
                        ail = conditions.control_surfaces.aileron.static_stability.coefficients                  
                        Ya = 0.5 * rho * u0 * u0 * S_ref * ail.Y 
                        La = 0.5 * rho * u0 * u0 * S_ref * b_ref * ail.L 
                        Na = 0.5 * rho * u0 * u0 * S_ref * b_ref * ail.N 
                        
                        BLat[:,0,0] = (Ya / m).T[0]
                        BLat[:,1,0] = (La / Ixp + Ixzp * Na).T[0]
                        BLat[:,2,0] = (Ixzp * La + Na / Izp).T[0]
                        BLat[:,3,0] = 0
     
        ALat[:,0,0] = (Yv / m).T[0]  
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
                                    
        LatModes                    = np.zeros((num_cases,4),dtype=complex)
        dutchRollFreqHz             = np.zeros((num_cases,1))
        dutchRollDamping            = np.zeros((num_cases,1))
        dutchRollTimeDoubleHalf     = np.zeros((num_cases,1))
        rollSubsistenceFreqHz       = np.zeros((num_cases,1))
        rollSubsistenceTimeConstant = np.zeros((num_cases,1))
        rollSubsistenceDamping      = np.zeros((num_cases,1))
        spiralFreqHz                = np.zeros((num_cases,1))
        spiralTimeDoubleHalf        = np.zeros((num_cases,1))
        spiralDamping               = np.zeros((num_cases,1))
        dutchRoll_mode_real         = np.zeros((num_cases,1))
        
        for i_lat in range(num_cases):
            try: 
                D  , V = np.linalg.eig(ALat[i_lat,:,:]) # State order: u, w, q, theta
                LatModes[i_lat,:] = D  
                
                real_parts = LatModes[i_lat,:].real
                unique_elements, counts = np.unique(real_parts, return_counts=True)
                idx = np.where(counts==2)[0]
    
                dutchRollFreqHz[i_lat]         = abs(D[idx]) / (2 * np.pi)
                dutchRollDamping[i_lat]        = np.sqrt(1/ (1 + ( D[idx].imag/ D[idx].real )**2 ))  
                dutchRollTimeDoubleHalf[i_lat] = np.log(2) / abs(2 * np.pi * dutchRollFreqHz[i_lat] * dutchRollDamping[i_lat])
                dutchRoll_mode_real[i_lat]     = D[idx].real / (2 * np.pi)
                
                dutch_roll_idx                 = np.where( unique_elements[idx] == D.real )[0]
                
                remaining_modes                    = np.delete(D, dutch_roll_idx)
                rollInd                            = np.argmin(remaining_modes) 
                rollSubsistenceFreqHz[i_lat]       = abs(remaining_modes[rollInd]) / 2 / np.pi
                rollSubsistenceDamping[i_lat]      = - np.sign(remaining_modes[rollInd].real)
                rollSubsistenceTimeConstant[i_lat] = 1 / (2 * np.pi * rollSubsistenceFreqHz[i_lat] * rollSubsistenceDamping[i_lat])
                
                # Find spiral mode 
                sprial_mode                 = np.delete(remaining_modes,rollInd)             
                spiralFreqHz[i_lat]         = abs(sprial_mode) / 2 / np.pi
                spiralDamping[i_lat]        = - np.sign(sprial_mode.real)
                spiralTimeDoubleHalf[i_lat] = np.log(2) / abs(2 * np.pi * spiralFreqHz[i_lat] * spiralDamping[i_lat])
            except:
                pass 
        
        # Inertial coupling susceptibility
        # See Etkin & Reid pg. 118 
        DS.pMax = min(min(np.sqrt(-Mw * u0 / (Izz - Ixx))), min(np.sqrt(-Nv * u0 / (Iyy - Ixx)))) 
        
        # -----------------------------------------------------------------------------------------------------------------------  
        # Store Results
        # ------------------------------------------------------------------------------------------------------------------------  
        DS.LongModes.LongModes                    = LonModes
        #DS.LongModes.LongSys                      = LonSys    
        DS.LongModes.phugoidFreqHz                = phugoidFreqHz
        DS.LongModes.phugoidDamping               = phugoidDamping
        DS.LongModes.phugoidTimeDoubleHalf        = phugoidTimeDoubleHalf
        DS.LongModes.shortPeriodFreqHz            = shortPeriodFreqHz
        DS.LongModes.shortPeriodDamping           = shortPeriodDamping
        DS.LongModes.shortPeriodTimeDoubleHalf    = shortPeriodTimeDoubleHalf
                                                                        
        DS.LatModes.LatModes                      = LatModes  
        #DS.LatModes.Latsys                        = LatSys   
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

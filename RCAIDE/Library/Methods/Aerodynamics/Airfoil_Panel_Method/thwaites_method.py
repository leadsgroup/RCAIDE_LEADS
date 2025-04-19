# RCAIDE/Methods/Aerodynamics/Airfoil_Panel_Method/thwaites_method_new.py
# 
# 
# Created:  Apr 2024, Niranjan Nanjappa

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
from RCAIDE.Framework.Core import Data 

# pacakge imports  
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Thwaites Method
# ----------------------------------------------------------------------------------------------------------------------
def thwaites_method(npanel,ncases,ncpts,NU,L,RE_L,X_I,VE_I, DVE_I,tol,THETA_0):
    """ Computes the boundary layer characteristics in laminar 
    flow pressure gradients
    
    Source:
    Thwaites, Bryan. "Approximate calculation of the laminar boundary layer." 
    Aeronautical Quarterly 1.3 (1949): 245-280.
    
    Assumptions:
    None  

    Inputs:  
    npanel         - number of points on surface                                                 [unitless]
    ncases         - number of cases                                                             [unitless]
    ncpts          - number of control points                                                    [unitless]
    batch_analysis - flag for batch analysis                                                     [boolean]
    THETA_0        - initial momentum thickness                                                  [m]
    L              - normalized length of surface                                                [unitless]
    RE_L           - Reynolds number                                                             [unitless]
    X_I            - x coordinate on surface of airfoil                                          [unitless]
    VE_I           - boundary layer velocity at transition location                              [m/s] 
    DVE_I          - initial derivative value of boundary layer velocity at transition location  [m/s-m] 
    tol            - boundary layer error correction tolerance                                   [unitless]

    Outputs: 
    RESULTS.
      X_T          - reshaped distance along airfoil surface             [unitless]
      THETA_T      - momentum thickness                                  [m]
      DELTA_STAR_T - displacement thickness                              [m] 
      H_T          - shape factor                                        [unitless]
      CF_T         - friction coefficient                                [unitless]
      RE_THETA_T   - Reynolds number as a function of momentum thickness [unitless]
      RE_X_T       - Reynolds number as a function of distance           [unitless]
      DELTA_T      - boundary layer thickness                            [m]

    Properties Used:
    N/A
    """ 
    # Initialize vectors
    X_T          = np.zeros((npanel,ncases,ncpts))
    THETA_T      = np.zeros_like(X_T)
    DELTA_STAR_T = np.zeros_like(X_T)
    H_T          = np.zeros_like(X_T)
    CF_T         = np.zeros_like(X_T)
    RE_THETA_T   = np.zeros_like(X_T)
    RE_X_T       = np.zeros_like(X_T)
    DELTA_T      = np.zeros_like(X_T)  
      
    for case in range(ncases):
        for cpt in range(ncpts):
            
            def dy_by_dx(index, X, Y):
                return 0.45*nu*Ve_i[index]**5            
            
            l              = L[case,cpt]
            theta_0        = THETA_0 
            Re_L           = RE_L[case,cpt]
            x_i            = X_I.data[:,case,cpt][X_I.mask[:,case,cpt] ==False]
            Ve_i           = VE_I.data[:,case,cpt][VE_I.mask[:,case,cpt] ==False]
            dVe_i          = DVE_I.data[:,case,cpt][DVE_I.mask[:,case,cpt] ==False]
            nu             = NU[case,cpt]
            n              = len(x_i)
            dx_i           = np.diff(x_i)
            theta2_Ve6     = np.zeros(n)
            theta2_Ve6[0]  = (theta_0**2)*Ve_i[0]**6
            
            # determine (Theta**2)*(Ve**6)
            for i in range(1,n):
                theta2_Ve6[i] = RK4(i-1, dx_i, x_i, theta2_Ve6, dy_by_dx)
            
            # Compute momentum thickness
            theta       = np.sqrt(theta2_Ve6/Ve_i**6)
            
            # find theta values that do not converge and replace them with neighbor
            idx1        = np.where(abs((theta[1:] - theta[:-1])/theta[:-1]) > tol)[0] 
            if len(idx1)> 1:  
                np.put(theta,idx1 + 1, theta[idx1])
                
            # Thwaites separation criteria 
            lambda_val  = theta**2*dVe_i/nu 
            
            # Compute H 
            H           = getH(lambda_val)
            H[H<0]      = 1E-6   # H cannot be negative 
            # find H values that do not converge and replace them with neighbor
            idx1        = np.where(abs((H[1:] - H[:-1])/H[:-1]) > tol)[0]
            if len(idx1)> 1: 
                np.put(H,idx1 + 1, H[idx1]) 
            
            # Compute Reynolds numbers based on momentum thickness  
            Re_theta    = Ve_i*theta/nu
            
            # Compute Reynolds numbers based on distance along airfoil
            Re_x        = Ve_i*x_i/nu
            
            # Compute skin friction 
            cf          = abs(getcf(lambda_val, Re_theta)) 
            
            # Compute displacement thickness
            del_star    = H*theta   
            
            # Compute boundary layer thickness 
            delta       = 5.2*x_i/np.sqrt(Re_x)
            delta[0]    = 0   
            
            # Reynolds number at x=0 cannot be negative 
            Re_x[0]     = 1E-5
            
            # Find where matrices are not masked 
            indices = np.where(X_I.mask[:,case,cpt] == False)
            
            # Store results 
            np.put(X_T[:,case,cpt],indices,x_i)
            np.put(THETA_T[:,case,cpt],indices,theta)
            np.put(DELTA_STAR_T[:,case,cpt],indices,del_star)
            np.put(H_T[:,case,cpt],indices,H)
            np.put(CF_T[:,case,cpt],indices ,cf)
            np.put(RE_THETA_T[:,case,cpt],indices,Re_theta)
            np.put(RE_X_T[:,case,cpt],indices,Re_x)
            np.put(DELTA_T[:,case,cpt],indices,delta)
    
    RESULTS = Data(
        X_T          = X_T,      
        THETA_T      = THETA_T,   
        DELTA_STAR_T = DELTA_STAR_T,
        H_T          = H_T,       
        CF_T         = CF_T,      
        RE_THETA_T   = RE_THETA_T,   
        RE_X_T       = RE_X_T,    
        DELTA_T      = DELTA_T,  
    )    
    
    return RESULTS
            



def getH(lambda_val ): 
    """ Computes the shape factor, H

    Assumptions:
    None

    Source:
    None

    Inputs: 
    lamdda_val  - thwaites separation criteria [unitless]

    Outputs:  
    H           - shape factor [unitless]

    Properties Used:
    N/A
    """       
    H       = 0.0731/(0.14 + lambda_val ) + 2.088 
    idx1    = (lambda_val>0.0)  
    H[idx1] = 2.61 - 3.75*lambda_val[idx1]  + 5.24*lambda_val[idx1]**2   
    return H


def getcf(lambda_val , Re_theta):
    """ Computes the skin friction coefficient, cf

    Assumptions:
    None

    Source:
    None

    Inputs: 
    lambda_val - thwaites separation criteria                        [unitless]
    Re_theta   - Reynolds Number as a function of momentum thickness [unitless]

    Outputs:  
    cf         - skin friction coefficient [unitless]

    Properties Used:
    N/A 
    """        
    l       = 0.22 + 1.402*lambda_val  + (0.018*lambda_val)/(0.107 + lambda_val ) 
    idx1    = (lambda_val>0.0)   
    l[idx1] = 0.22 + 1.57*lambda_val[idx1] - 1.8*lambda_val[idx1]**2 
    cf      = 2*l/Re_theta  
    return cf


def RK4(ind, dx, x, Var, Slope):
    m1 = Slope(ind,  x[ind],  Var[ind])
    m2 = Slope(ind,  x[ind] + dx[ind]/2,  Var[ind] + m1*dx[ind]/2)
    m3 = Slope(ind,  x[ind] + dx[ind]/2,  Var[ind] + m2*dx[ind]/2)
    m4 = Slope(ind,  x[ind] + dx[ind],  Var[ind] + m3*dx[ind])
    
    change = (dx[ind]/6)*(m1 + 2*m2 + 2*m3 + m4)
    return Var[ind] + change            
            

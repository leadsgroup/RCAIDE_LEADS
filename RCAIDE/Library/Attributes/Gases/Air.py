# RCAIDE/Library/Attributes/Gases/Air.py
# 
#
# Created:  Mar 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------  
#  Imports
# ----------------------------------------------------------------------------------------------------------------------  
 
from .Gas import Gas 
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------  
# Air Class
# ----------------------------------------------------------------------------------------------------------------------   
class Air(Gas):
    """Generic class of air gas. 
    """

    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None
        
        Source:
            None
        """          
        self.tag                    = 'air'
        self.molecular_mass         = 28.96442        # kg/kmol
        self.gas_specific_constant  = 287.0528742     # m^2/s^2-K, specific gas constant  
        self.specific_heat_capacity = 1006            # J/kgK         
        self.composition.O2         = 0.20946
        self.composition.Ar         = 0.00934
        self.composition.CO2        = 0.00036
        self.composition.N2         = 0.78084
        self.composition.other      = 0.00

    def compute_density(self,T=300.,p=101325.):
        """Computes air density given temperature and pressure
        
        Assumptions:
            Ideal gas
            
        Source:
            None
    
        Args:
            self       : air                   [unitless]
            T (float)  : temperature           [K]
            P (float)  : pressure              [Pa]
            
        Returns:
            rho (float): density               [kg/m^3]       
        """        
        return p/(self.gas_specific_constant*T)

    def compute_speed_of_sound(self,T=300.,p=101325.,variable_gamma=False):
        """Computes speed of sound given temperature and pressure 
 
        Assumptions:
            Ideal gas with gamma = 1.4 if variable gamma is False

        Source:
            None 

        Args:
            self                     : air           [unitless]
            T (float)                : temperature   [K]    
            p (float)                : Pressure      [Pa]      
            variable_gamma (boolean) :               [unitless]

        Returns:
            a (float)                : speed of sound [m/s] 
        """                  

        if variable_gamma:
            g = self.compute_gamma(T,p)
        else:
            g = 1.4*np.ones_like(T)
            
        return np.sqrt(g*self.gas_specific_constant*T)

    def compute_cp(self,T=300.,p=101325.):
        """Computes Cp by 3rd-order polynomial data fit:
        cp(T) = c1*T^3 + c2*T^2 + c3*T + c4

        Coefficients (with 95% confidence bounds):
        c1 = -7.357e-007  (-9.947e-007, -4.766e-007)
        c2 =    0.001307  (0.0009967, 0.001617)
        c3 =     -0.5558  (-0.6688, -0.4429)
        c4 =        1074  (1061, 1086) 
             
        Assumptions:
            123 K < T < 673 K 
            
        Source:
            Unknown, possibly Combustion Technologies for a Clean Environment 
            (Energy, Combustion and the Environment), Jun 15, 1995, Carvalhoc
    
        Args:
            self      : air               [unitless]
            T (float) : temperature       [K]
            P (float) : pressure          [Pa]
            
        Returns:
            cp (float): specfic heat capacity [J/(kg K)]         
        """   

        c = [-7.357e-007, 0.001307, -0.5558, 1074.0]
        cp = c[0]*T*T*T + c[1]*T*T + c[2]*T + c[3]

        return cp

    def compute_gamma(self,T=300.,p=101325.):
        """Computes Cp by 3rd-order polynomial data fit:
        gamma(T) = c1*T^3 + c2*T^2 + c3*T + c4

        Coefficients (with 95% confidence bounds):
        c1 =  1.629e-010  (1.486e-010, 1.773e-010)
        c2 = -3.588e-007  (-3.901e-007, -3.274e-007)
        c3 =   0.0001418  (0.0001221, 0.0001614)
        c4 =       1.386  (1.382, 1.389) 
 
        Assumptions:
             233 K < T < 1273 K 
            
        Source:
            None
    
        Args:
            self      : air           [unitless]
            T (float) : temperature   [K]
            P (float) : pressure      [Pa]
            
        Returns: 
            g  (float): gamma         [unitless] 
        """     

        c = [1.629e-010, -3.588e-007, 0.0001418, 1.386]
        g = c[0]*T*T*T + c[1]*T*T + c[2]*T + c[3]

        return g

    def compute_absolute_viscosity(self,T=300.,p=101325.):
        """Compute the absolute (dynamic) viscosity 
        
        Assumptions:
            Ideal gas
            
        Source:
            https://www.cfd-online.com/Wiki/Sutherland's_law
    
        Args:
            self      : air                   [unitless]
            T (float) : temperature           [K]
            P (float) : pressure              [Pa]
            
        Returns:
            mu (float): absolute viscosity    [kg/(m-s)]       
        """ 

        S = 110.4                   # constant in deg K (Sutherland's Formula)
        C1 = 1.458e-6               # kg/m-s-sqrt(K), constant (Sutherland's Formula)

        return C1*(T**(1.5))/(T + S)
    
    def compute_thermal_conductivity(self,T=300.,p=101325.):
        """Compute the thermal conductivity of air 
 
        Assumptions:
            Properties computed at 1 bar (14.5 psia)
            
        Source:
            https://www.engineeringtoolbox.com/air-properties-viscosity-conductivity-heat-capacity-d_1509.html 
    
        Args:
            self      : air                   [unitless]
            T (float) : temperature           [K]
            P (float) : pressure              [Pa]
            
        Returns:
            k  (float): thermal conductivity  [W/(m-K)]   
        """ 
        return 3.99E-4 + 9.89E-5*(T) -4.57E-8*(T**2) + 1.4E-11*(T**3)
    
    
    def compute_prandtl_number(self,T=300.):
        """Compute the prandtl number 
             
        Assumptions:
            None
            
        Source:
            None
    
        Args:
            self      : air                   [unitless]
            T (float) : temperature           [K] 
            
        Returns:
            Pr  (float): Prandtl Number       [unitless]
        """ 
        
        Cp = self.specific_heat_capacity 
        mu = self.compute_absolute_viscosity(T)
        K  = self.compute_thermal_conductivity(T)
        return  mu*Cp/K      
    
    def compute_R(self,T=300.,p=101325.):
        """Compute the prandtl number 
             
        Assumptions:
            None
            
        Source:
            None
    
        Args:
            self      : air                   [unitless]
            T (float) : temperature           [K] 
            
        Returns:
            Pr  (float): Prandtl Number       [unitless]
        """ 
        
        gamma = self.compute_gamma(T,p)
        cp = self.compute_cp(T,p)
        R  = ((gamma - 1)/gamma)*cp
        return  R          
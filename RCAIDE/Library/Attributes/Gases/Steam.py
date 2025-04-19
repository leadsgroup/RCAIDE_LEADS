# RCAIDE/Library/Attributes/Gases/CO2.py
# 
#
# Created:  Mar 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------  
#  Imports
# ----------------------------------------------------------------------------------------------------------------------  
 
from .Gas import Gas  

# ----------------------------------------------------------------------------------------------------------------------  
# Steam Class
# ----------------------------------------------------------------------------------------------------------------------   
class Steam(Gas):
    """Holds constants and functions that compute gas properties for steam. 
    """

    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None
        
        Source:
            None
        """ 
        self.tag                   ='steam'
        self.molecular_mass        = 18.           # kg/kmol
        self.gas_specific_constant = 461.889       # m^2/s^2-K, specific gas constant
        self.composition.H2O       = 1.0

    def compute_density(self,T=300,p=101325):
        """Computes steam density given temperature and pressure 
        
        Assumptions:
            Ideal gas
            
        Source:
            None
    
        Args:
            self       : steam                 [unitless]
            T (float)  : temperature           [K]
            P (float)  : pressure              [Pa]
            
        Returns:
            rho (float): density               [kg/m^3]       
        """        
        return p/(self.gas_specific_constant*T)

    def compute_speed_of_sound(self,T=300,p=101325,variable_gamma=False):
        """Computes speed of sound given temperature and pressure

        Assumptions:
            Ideal gas with gamma = 1.33 if variable gamma is False

        Source:
            None 

        Args:
            self                     : steam         [unitless]
            T (float)                : temperature   [K]    
            p (float)                : Pressure      [Pa]      
            variable_gamma (boolean) :               [unitless]

        Returns:
            a (float)                : speed of sound [m/s] 
        """        
        if variable_gamma:
            g = self.compute_gamma(T,p)
        else:
            g = 1.33

        return (g*self.gas_specific_constant*T)**0.5 

    def compute_cp(self,T=300,p=101325):
        """Computes Cp by 3rd-order polynomial data fit: cp(T) = c1*T^3 + c2*T^2 + c3*T + c4
             
        Assumptions:
            300 K < T < 1500 K
            
        Source:
            Unknown, possibly Combustion Technologies for a Clean Environment 
            (Energy, Combustion and the Environment), Jun 15, 1995, Carvalhoc
    
        Args:
            self      : steam             [unitless]
            T (float) : temperature       [K]
            P (float) : pressure          [Pa]
            
        Returns:
            cp (float): specfic heat capacity [J/(kg K)]         
        """   
        c = [5E-9, -.0001,  .9202, 1524.7]
        cp = c[0]*T**3 + c[1]*T**2 + c[2]*T + c[3]

        return cp

    def compute_gamma(self,T=300,p=101325):
        """Gives constant gamma of 1.33 
        
        Assumptions:
             233 K < T < 1273 K 
            
        Source:
            None
    
        Args:
            self      : steam         [unitless]
            T (float) : temperature   [K]
            P (float) : pressure      [Pa]
            
        Returns: 
            g  (float): gamma         [unitless] 
        """           
        g = 1.33   
        return g

    def compute_absolute_viscosity(self,T=300,p=101325):
        """Returns a constant absolute viscosity of 1e-6 
        
        Assumptions:
            None
            
        Source:
            None
    
        Args:
            self      : steam                 [unitless]
            T (float) : temperature           [K]
            P (float) : pressure              [Pa]
            
        Returns:
            mu (float): absolute viscosity    [kg/(m-s)]       
        """ 
        mu =1E-6 
        return mu 
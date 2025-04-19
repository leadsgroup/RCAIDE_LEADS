# RCAIDE/Library/Attributes/Coolants/Glycol_Water.py
# 
#
# Created:  Mar 2024, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 
from .Coolant import Coolant

# ---------------------------------------------------------------------------------------------------------------------- 
#  Glycol_Water
# ----------------------------------------------------------------------------------------------------------------------  
class Glycol_Water(Coolant):
    """Generic class of ethelyne glycol-water mixture coolant.
    """

    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            Mixture is 50% water-50% ethylene-glycol
        
        Source: 
            Engineering Toolbox: https://www.engineeringtoolbox.com/ethylene-glycol-d_146.html
            University of Waterloo: http://www.mhtl.uwaterloo.ca/old/onlinetools/airprop/airprop.html 
        """ 
        self.tag                       = 'Glycol_Water'
        self.percent_glycol            = 0.5 
        self.density                   = 1075                       # kg/m^3
        self.specific_heat_capacity    = 3300                       # J/kg.K
        self.thermal_conductivity      = 0.387                      # W/m.K
        self.dynamic_viscosity         = 0.0019                     # Pa.s
        self.Prandtl_number            = self.specific_heat_capacity * self.dynamic_viscosity / self.thermal_conductivity
        self.kinematic_viscosity       = self.dynamic_viscosity / self.density

    def compute_cp(self,T=300):
        """ Computes the specfic heat capacity of water-glycol at a specfic temperature  
        
        Assumptions:
            Mixture is 50% water-50% glycol
            
        Source:
            Engineering Toolbox: https://www.engineeringtoolbox.com/ethylene-glycol-d_146.html
    
        Args:
            self      : glycol-water coolant  [unitless]
            T (float) : temperature           [K]
            
        Returns:
            cp (float): specfic heat capacity [J/(kg K)]         
        """ 
        return self.specific_heat_capacity
    
    def compute_absolute_viscosity(self,T=300.,p=101325.):
        """ Computes the absolute viscosity of water-glycol at a specfic temperature and pressure 
        
        Assumptions:
            Mixture is 50% water-50% glycol
            
        Source:
            Engineering Toolbox: https://www.engineeringtoolbox.com/ethylene-glycol-d_146.html
    
        Args:
            self      : glycol-water coolant  [unitless]
            T (float) : temperature           [K]
            P (float) : pressure              [Pa]
            
        Returns:
            mu (float): absolute viscosity    [kg/(m-s)] 
        """ 
        return  self.dynamic_viscosity
    
    def compute_density(self,T=300.,p=101325.):  
        """ Computes the density of water-glycol at a specfic temperature and pressure 
        
        Assumptions:
            Mixture is 50% water-50% glycol
            
        Source:
            Engineering Toolbox: https://www.engineeringtoolbox.com/ethylene-glycol-d_146.html
    
        Args:
            self       : glycol-water coolant  [unitless]
            T (float)  : temperature           [K]
            P (float)  : pressure              [Pa]
            
        Returns:
            rho (float): density               [kg/m^3]         
        """         
        return self.density  
    
    def compute_thermal_conductivity(self,T=300.,p=101325.): 
        """ Computes the density of water-glycol at a specfic temperature and pressure 
        
        Assumptions:
            Mixture is 50% water-50% glycol
            
        Source:
            University of Waterrloo: http://www.mhtl.uwaterloo.ca/old/onlinetools/airprop/airprop.html
    
        Args:
            self       : glycol-water coolant  [unitless]
            T (float)  : temperature           [K]
            P (float)  : pressure              [Pa]
            
        Returns:
            k (float)  : thermal conductivity  [W/m·K]         
        """          
        return self.thermal_conductivity
    
    
    def compute_prandtl_number(self,T=300.): 
        """ Computes the density of water-glycol at a specfic temperature and pressure 
        
        Assumptions:
            Mixture is 50% water-50% glycol
            
        Source:
            University of Waterrloo: http://www.mhtl.uwaterloo.ca/old/onlinetools/airprop/airprop.html
    
        Args:
            self       : glycol-water coolant  [unitless]
            T (float)  : temperature           [K] 
            
        Returns:
            Pr (float) : Prandtl number        [unitless]         
        """          
        
        Cp = self.compute_cp(T)
        mu = self.compute_absolute_viscosity(T)
        K  = self.compute_thermal_conductivity(T)
        return  mu*Cp/K      
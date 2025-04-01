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
    """
    A class representing air and its thermodynamic properties. Provides methods for computing various 
    gas properties including density, speed of sound, specific heat, and transport properties.

    Attributes
    ----------
    tag : str
        Identifier for the gas type ('air')
    molecular_mass : float
        Molecular mass of air in kg/kmol
    gas_specific_constant : float
        Specific gas constant in m²/s²-K
    specific_heat_capacity : float
        Specific heat capacity in J/kg·K
    composition : Data
        Chemical composition of air
            - O2 : float
                Mass fraction of oxygen (0.20946)
            - Ar : float
                Mass fraction of argon (0.00934)
            - CO2 : float
                Mass fraction of carbon dioxide (0.00036)
            - N2 : float
                Mass fraction of nitrogen (0.78084)
            - other : float
                Mass fraction of other components (0.00)

    Notes
    -----
    This class implements standard atmospheric air properties and various methods
    for computing thermodynamic and transport properties.
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
        """
        Computes air density using the ideal gas law.

        Parameters
        ----------
        T : float
            Temperature in Kelvin
        p : float
            Pressure in Pascal

        Returns
        -------
        rho : float
            Density in kg/m³

        Notes
        -----
        **Major Assumptions**
            * Air behaves as an ideal gas    
        """        
        return p/(self.gas_specific_constant*T)

    def compute_speed_of_sound(self,T=300.,p=101325.,variable_gamma=False):
        """
        Computes speed of sound in air.

        Parameters
        ----------
        T : float
            Temperature in Kelvin
        p : float
            Pressure in Pascal
        variable_gamma : bool
            If True, uses temperature-dependent specific heat ratio

        Returns
        -------
        a : float
            Speed of sound in m/s

        Notes
        -----
        **Major Assumptions**
            * If variable_gamma is False, assumes γ = 1.4
            * Air behaves as an ideal gas
        """                  

        if variable_gamma:
            g = self.compute_gamma(T,p)
        else:
            g = 1.4*np.ones_like(T)
            
        return np.sqrt(g*self.gas_specific_constant*T)

    def compute_cp(self,T=300.,p=101325.):
        """
        Computes specific heat capacity at constant pressure using a 3rd-order polynomial fit.

        Parameters
        ----------
        T : float
            Temperature in Kelvin
        p : float
            Pressure in Pascal

        Returns
        -------
        cp : float
            Specific heat capacity in J/(kg·K)

        Notes
        -----
        **Major Assumptions**
            * Valid for temperature range: 123 K < T < 673 K

        **Theory**
        .. math::
            c_p(T) = c_1T^3 + c_2T^2 + c_3T + c_4

        References
        ----------
        [1] Ekin, J. (2006). Experimental techniques for low-temperature measurements: Cryostat design, material properties and superconductor critical-current testing. Oxford University Press.         
        """   

        c = [-7.357e-007, 0.001307, -0.5558, 1074.0]
        cp = c[0]*T*T*T + c[1]*T*T + c[2]*T + c[3]

        return cp

    def compute_gamma(self,T=300.,p=101325.):
        """
        Computes specific heat ratio using a 3rd-order polynomial fit.

        Parameters
        ----------
        T : float
            Temperature in Kelvin
        p : float
            Pressure in Pascal

        Returns
        -------
        g : float
            Specific heat ratio (gamma) [unitless]

        Notes
        -----
        **Major Assumptions**
        * Valid for temperature range: 233 K < T < 1273 K
        """     

        c = [1.629e-010, -3.588e-007, 0.0001418, 1.386]
        g = c[0]*T*T*T + c[1]*T*T + c[2]*T + c[3]

        return g

    def compute_absolute_viscosity(self,T=300.,p=101325.):
        """
        Computes absolute (dynamic) viscosity using Sutherland's law.

        Parameters
        ----------
        T : float
            Temperature in Kelvin
        p : float
            Pressure in Pascal

        Returns
        -------
        mu : float
            Absolute viscosity in kg/(m·s)

        Notes
        -----
        **Theory**
        Uses Sutherland's formula with S = 110.4K and C1 = 1.458e-6 kg/m-s-sqrt(K)

        References
        ----------
        [1] Sutherland's law   
        """ 

        S = 110.4                   # constant in deg K (Sutherland's Formula)
        C1 = 1.458e-6               # kg/m-s-sqrt(K), constant (Sutherland's Formula)

        return C1*(T**(1.5))/(T + S)
    
    def compute_thermal_conductivity(self,T=300.,p=101325.):
        """
        Computes thermal conductivity of air.

        Parameters
        ----------
        T : float
            Temperature in Kelvin
        p : float
            Pressure in Pascal

        Returns
        -------
        k : float
            Thermal conductivity in W/(m·K)

        Notes
        -----
        **Major Assumptions**
            * Properties computed at 1 bar (14.5 psia)

        References
        ----------
        [1] The Engineering ToolBox (2009). Air - Thermal Conductivity vs. Temperature and Pressure. [online] Available at: https://www.engineeringtoolbox.com/air-properties-viscosity-conductivity-heat-capacity-d_1509.html [Accessed 8 January 2025].    
        """ 
        return 3.99E-4 + 9.89E-5*(T) -4.57E-8*(T**2) + 1.4E-11*(T**3)
    
    
    def compute_prandtl_number(self,T=300.):
        """
        Computes Prandtl number.

        Parameters
        ----------
        T : float
            Temperature in Kelvin

        Returns
        -------
        Pr : float
            Prandtl number [unitless]

        Notes
        -----
        **Theory**
        .. math::
            Pr = \\frac{\\mu C_p}{k}
        """ 
        
        Cp = self.specific_heat_capacity 
        mu = self.compute_absolute_viscosity(T)
        K  = self.compute_thermal_conductivity(T)
        return  mu*Cp/K      
    
    def compute_R(self,T=300.,p=101325.):
        """
        Computes specific gas constant.

        Parameters
        ----------
        T : float
            Temperature in Kelvin
        p : float
            Pressure in Pascal

        Returns
        -------
        R : float
            Specific gas constant in J/(kg·K)

        Notes
        -----
        **Theory**
        .. math::
            R = \\frac{\\gamma - 1}{\\gamma}c_p
        """ 
        
        gamma = self.compute_gamma(T,p)
        cp = self.compute_cp(T,p)
        R  = ((gamma - 1)/gamma)*cp
        return  R          
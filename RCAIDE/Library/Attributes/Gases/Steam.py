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
    """
    A class representing water vapor (steam) and its thermodynamic properties. Provides methods for computing 
    various gas properties including density, speed of sound, and specific heat.

    Attributes
    ----------
    tag : str
        Identifier for the gas type ('steam')
    
    molecular_mass : float
        Molecular mass of H2O in kg/kmol (18.0)
    
    gas_specific_constant : float
        Specific gas constant in m²/s²-K (461.889)
    
    composition : Data
        Chemical composition of steam
        
        - H2O : float
            Mass fraction of water vapor (1.0)

    Notes
    -----
    This class implements steam properties assuming ideal gas behavior for most calculations.
    All properties are for pure water vapor.
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
        """
        Computes steam density using the ideal gas law.

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
        * Steam behaves as an ideal gas   
        """        
        return p/(self.gas_specific_constant*T)

    def compute_speed_of_sound(self,T=300,p=101325,variable_gamma=False):
        """
        Computes speed of sound in steam.

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
            * If variable_gamma is False, assumes γ = 1.33
            * Steam behaves as an ideal gas
        """        
        if variable_gamma:
            g = self.compute_gamma(T,p)
        else:
            g = 1.33

        return (g*self.gas_specific_constant*T)**0.5 

    def compute_cp(self,T=300,p=101325):
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
            * Valid for temperature range: 300 K < T < 1500 K

        **Theory**
        .. math::
            c_p(T) = c_1T^3 + c_2T^2 + c_3T + c_4

        References
        ----------
        [1] Carvalho, M. G., Lockwood, F. C., Fiveland, W. A., & Papadopoulos, C. (2022). Combustion Technologies for a clean environment: Selected papers from the proceedings of the first international conference, Vilamoura, Portugal, September 3-6, 1991. CRC Press.       
        """   
        c = [5E-9, -.0001,  .9202, 1524.7]
        cp = c[0]*T**3 + c[1]*T**2 + c[2]*T + c[3]

        return cp

    def compute_gamma(self,T=300,p=101325):
        """
        Returns the specific heat ratio for steam.

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
            * Uses constant value of 1.33
        """           
        g = 1.33   
        return g

    def compute_absolute_viscosity(self,T=300,p=101325):
        """
        Returns the absolute (dynamic) viscosity of steam.

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
        **Major Assumptions**
            * Uses constant value of 1e-6 kg/(m·s)     
        """ 
        mu =1E-6 
        return mu 
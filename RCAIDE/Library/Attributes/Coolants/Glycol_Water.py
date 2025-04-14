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
    """
    Implementation of a 50-50 ethylene glycol-water mixture coolant properties.

    This class provides thermophysical properties and calculation methods for a standard
    ethylene glycol-water mixture commonly used in cooling systems.

    Attributes
    ----------
    tag : str
        Identifier set to 'Glycol_Water'   
    percent_glycol : float
        Mass fraction of glycol in the mixture (default: 0.5)
    density : float
        Mass per unit volume [kg/m³]
    specific_heat_capacity : float
        Heat capacity at constant pressure [J/kg·K]
    thermal_conductivity : float
        Heat conduction coefficient [W/m·K]
    dynamic_viscosity : float
        Absolute viscosity [Pa·s]
    Prandtl_number : float
        Dimensionless number for heat transfer characteristics
    kinematic_viscosity : float
        Ratio of dynamic viscosity to density [m²/s]

    Notes
    -----
    All properties are currently implemented as constant values, though the method
    structure allows for future implementation of temperature and pressure dependence.

    **Major Assumptions**
        * Mixture is exactly 50% water and 50% ethylene glycol by mass
        * Standard atmospheric pressure conditions
        * Single-phase liquid mixture

    **Extra modules required**
    None

    References
    ----------
    [1] The Engineering ToolBox (2003). Ethylene Glycol Heat-Transfer Fluid Properties. [online] Available at: https://www.engineeringtoolbox.com/ethylene-glycol-d_146.html [Accessed 8 January 2025].
    [2] Microelectronics Heat Transfer Laboratory. (1997). Fluid Properties Calculator. Fluid properties calculator. http://www.mhtl.uwaterloo.ca/old/onlinetools/airprop/airprop.html 

    See Also
    --------
    RCAIDE.Library.Components.Thermal_Management : Thermal management system components
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
        """         
        Calculate specific heat capacity of glycol-water mixture at given temperature.

        Parameters
        ----------
        T : float, optional
            Temperature [K] (default: 300)

        Returns
        -------
        cp : float
            Specific heat capacity [J/(kg·K)]

        Notes
        -----
        Currently returns constant value of 3300 J/(kg·K), based on the default value, regardless of temperature.

        **Major Assumptions**
            * Temperature independence
            * 50-50 water-glycol mixture

        References
        ----------
        [1] The Engineering ToolBox (2003). Ethylene Glycol Heat-Transfer Fluid Properties. [online] Available at: https://www.engineeringtoolbox.com/ethylene-glycol-d_146.html [Accessed 8 January 2025].
  
        """ 
        return self.specific_heat_capacity
    
    def compute_absolute_viscosity(self,T=300.,p=101325.):
        """ 
        Calculate dynamic viscosity of glycol-water mixture at given conditions.

        Parameters
        ----------
        T : float, optional
            Temperature [K] (default: 300)
        p : float, optional
            Pressure [Pa] (default: 101325)

        Returns
        -------
        mu : float
            Dynamic viscosity [kg/(m·s)]

        Notes
        -----
        Currently returns constant value of 0.0019 Pa·s, based on the default value, regardless of conditions.

        **Major Assumptions**
            * Temperature and pressure independence
            * 50-50 water-glycol mixture

        References
        ----------
        [1] The Engineering ToolBox (2003). Ethylene Glycol Heat-Transfer Fluid Properties. [online] Available at: https://www.engineeringtoolbox.com/ethylene-glycol-d_146.html [Accessed 8 January 2025].

        """ 
        return  self.dynamic_viscosity
    
    def compute_density(self,T=300.,p=101325.):  
        """ 
        Calculate density of glycol-water mixture at given conditions.

        Parameters
        ----------
        T : float, optional
            Temperature [K] (default: 300)
        p : float, optional
            Pressure [Pa] (default: 101325)

        Returns
        -------
        rho : float
            Density [kg/m³]

        Notes
        -----
        Currently returns constant value of 1075 kg/m³ based on the default value.

        **Major Assumptions**
            * Temperature and pressure independence
            * 50-50 water-glycol mixture

        References
        ----------
        [1] The Engineering ToolBox (2003). Ethylene Glycol Heat-Transfer Fluid Properties. [online] Available at: https://www.engineeringtoolbox.com/ethylene-glycol-d_146.html [Accessed 8 January 2025].
   
        """         
        return self.density  
    
    def compute_thermal_conductivity(self,T=300.,p=101325.): 
        """
        Calculate thermal conductivity of glycol-water mixture at given conditions.

        Parameters
        ----------
        T : float, optional
            Temperature [K] (default: 300)
        p : float, optional
            Pressure [Pa] (default: 101325)

        Returns
        -------
        k : float
            Thermal conductivity [W/(m·K)]

        Notes
        -----
        Currently returns constant value of 0.387 W/(m·K), based on the default values, 
        regardless of conditions.

        **Major Assumptions**
            * Temperature and pressure independence
            * 50-50 water-glycol mixture

        References
        ----------
        [1] Microelectronics Heat Transfer Laboratory. (1997). Fluid Properties Calculator. Fluid properties calculator. http://www.mhtl.uwaterloo.ca/old/onlinetools/airprop/airprop.html 
       
        """          
        return self.thermal_conductivity
    
    
    def compute_prandtl_number(self,T=300.): 
        """
        Calculate Prandtl number of glycol-water mixture at given temperature.

        Parameters
        ----------
        T : float, optional
            Temperature [K] (default: 300)

        Returns
        -------
        Pr : float
            Prandtl number [dimensionless]

        Notes
        -----
        Calculated using the relationship:
        Pr = (μ·Cp)/k

        where:
            - μ is dynamic viscosity
            - Cp is specific heat capacity
            - k is thermal conductivity

        **Major Assumptions**
            * Temperature independence of constituent properties
            * 50-50 water-glycol mixture

        References
        ----------
        [1] Microelectronics Heat Transfer Laboratory. (1997). Fluid Properties Calculator. Fluid properties calculator. http://www.mhtl.uwaterloo.ca/old/onlinetools/airprop/airprop.html 
       
        """          
        
        Cp = self.compute_cp(T)
        mu = self.compute_absolute_viscosity(T)
        K  = self.compute_thermal_conductivity(T)
        return  mu*Cp/K      
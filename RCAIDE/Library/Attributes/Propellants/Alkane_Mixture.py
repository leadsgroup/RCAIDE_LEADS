# RCAIDE/Library/Attributes/Propellants/Alkane_Mixture.py
#  
# Created:  Mar 2024, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE
from .Propellant import Propellant 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Propanol Propellant Class
# ----------------------------------------------------------------------------------------------------------------------   
class Alkane_Mixture(Propellant):
    """
    A class representing a binary mixture of alkane fuels with customizable composition.

    Attributes
    ----------
    tag : str
        Identifier for the propellant (defaults to 'Alkane_Mixture')
    reactant : str
        Oxidizer used for combustion (defaults to 'O2')
    propellant_1 : Propellant
        First alkane component (default: Ethane)
    propellant_1_mass_fraction : float
        Mass fraction of first component (defaults to 0.5)
    propellant_2 : Propellant
        Second alkane component (default: Propane)
    propellant_2_mass_fraction : float
        Mass fraction of second component (defaults to 0.5)
    density : float
        Mixture density in kg/m³, computed from components
    specific_energy : float
        Mixture specific energy in J/kg, computed from components
    energy_density : float
        Mixture energy density in J/m³, computed from components
    lower_heating_value : float
        Mixture lower heating value in J/kg, computed from components

    Notes
    -----
    This class implements properties for binary alkane fuel mixtures. All mixture 
    properties are computed using mass fraction weighted averages of the component 
    properties, except density which uses a volume-based mixing rule.

    **Theory**
    
    Density mixing rule:
    .. math::
        \\rho_{mix} = \\left(\\frac{X_1}{\\rho_1} + \\frac{X_2}{\\rho_2}\\right)^{-1}
    
    Property mixing rule:
    .. math::
        P_{mix} = X_1P_1 + X_2P_2
    
    where:
        - X₁, X₂ are mass fractions
        - ρ₁, ρ₂ are component densities
        - P₁, P₂ are component properties

    **Definitions**
    
    'Mass Fraction'
        The ratio of component mass to total mixture mass
    
    'Lower Heating Value'
        Heat of combustion excluding latent heat of water vapor
    """

    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            Density at 20C 1 atm
        
        Source: 
    
        """    
        self.tag                        = 'Alkane_Mixture'
        self.reactant                   = 'O2'
        self.propellant_1               = RCAIDE.Library.Attributes.Propellants.Ethane() 
        self.propellant_1_mass_fraction = 0.5
        self.propellant_2               = RCAIDE.Library.Attributes.Propellants.Propane()
        self.propellant_2_mass_fraction = 0.5
        self.density                    = self.compute_mixture_density()                                 # kg/m^3 (15 C, 1 atm)
        self.specific_energy            = self.compute_mixture_specific_energy()                         # J/kg
        self.energy_density             = self.compute_mixture_energy_density()                          # J/m^3
        self.lower_heating_value        = self.compute_mixture_lower_heating_value()                     # J/kg
        
    def compute_mixture_density(self):
        """
        Compute the mixture density using volume-based mixing rule.

        Returns
        -------
        rho_mix : float
            Mixture density in kg/m³

        Notes
        -----
        Uses mixing rule for proper volume-based averaging of alkane components.
        """
        p1    = self.propellant_1 
        p2    = self.propellant_2
        rho1  = p1.density
        rho2  = p2.density
        X1    = self.propellant_1_mass_fraction
        X2    = self.propellant_2_mass_fraction

        rho_mix = ((X1/rho1) + (X2/rho2) ) ** (-1)
        return rho_mix 
    
    def compute_mixture_specific_energy(self):
        """
        Compute the mixture specific energy using mass fraction weighted average.

        Returns
        -------
        e_mix : float
            Mixture specific energy in J/kg

        Notes
        -----
        Linear combination of component specific energies weighted by mass fractions.
        """
        p1    = self.propellant_1 
        p2    = self.propellant_2
        e1    = p1.specific_energy
        e2    = p2.specific_energy
        X1    = self.propellant_1_mass_fraction
        X2    = self.propellant_2_mass_fraction
        
        e_mix =  X1 * e1 +  X2 * e2
        
        return e_mix
    
    def compute_mixture_energy_density(self): 
        """
        Compute the mixture energy density from specific energy and density.

        Returns
        -------
        U_mix : float
            Mixture energy density in J/m³

        Notes
        -----
        Product of mixture specific energy and mixture density.
        """
        e_mix   =  self.compute_mixture_specific_energy()
        rho_mix =  self.compute_mixture_density()
        
        U_mix = e_mix * rho_mix 
        return U_mix
    
    def compute_mixture_lower_heating_value(self):
        """
        Compute the mixture lower heating value using mass fraction weighted average.

        Returns
        -------
        LHV_mix : float
            Mixture lower heating value in J/kg

        Notes
        -----
        Linear combination of component lower heating values weighted by mass fractions.
        """
        p1    = self.propellant_1 
        p2    = self.propellant_2
        LHV1  = p1.lower_heating_value
        LHV2  = p2.lower_heating_value
        X1    = self.propellant_1_mass_fraction
        X2    = self.propellant_2_mass_fraction
        
        LHV_mix =  X1 * LHV1 +  X2 * LHV2
        return LHV_mix
    
    def compute_all(self):
        """
        Update all mixture properties based on current composition.

        Notes
        -----
        Recalculates all mixture properties:
            - Density
            - Specific energy
            - Energy density
            - Lower heating value
        
        Should be called after changing mixture composition.
        """
        self.density                    = self.compute_mixture_density()                                 # kg/m^3 (15 C, 1 atm)
        self.specific_energy            = self.compute_mixture_specific_energy()                         # J/kg
        self.energy_density             = self.compute_mixture_energy_density()                          # J/m^3
        self.lower_heating_value        = self.compute_mixture_lower_heating_value() 

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
    """Alkane_Mixture class propellant  
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
        p1    = self.propellant_1 
        p2    = self.propellant_2
        rho1  = p1.density
        rho2  = p2.density
        X1    = self.propellant_1_mass_fraction
        X2    = self.propellant_2_mass_fraction

        rho_mix = ((X1/rho1) + (X2/rho2) ) ** (-1)
        return rho_mix 
    
    def compute_mixture_specific_energy(self):
        p1    = self.propellant_1 
        p2    = self.propellant_2
        e1    = p1.specific_energy
        e2    = p2.specific_energy
        X1    = self.propellant_1_mass_fraction
        X2    = self.propellant_2_mass_fraction
        
        e_mix =  X1 * e1 +  X2 * e2
        
        return e_mix
    
    def compute_mixture_energy_density(self): 
        e_mix   =  self.compute_mixture_specific_energy()
        rho_mix =  self.compute_mixture_density()
        
        U_mix = e_mix * rho_mix 
        return U_mix
    
    def compute_mixture_lower_heating_value(self):
        p1    = self.propellant_1 
        p2    = self.propellant_2
        LHV1  = p1.lower_heating_value
        LHV2  = p2.lower_heating_value
        X1    = self.propellant_1_mass_fraction
        X2    = self.propellant_2_mass_fraction
        
        LHV_mix =  X1 * LHV1 +  X2 * LHV2
        return LHV_mix
    
    def compute_all(self):
        self.density                    = self.compute_mixture_density()                                 # kg/m^3 (15 C, 1 atm)
        self.specific_energy            = self.compute_mixture_specific_energy()                         # J/kg
        self.energy_density             = self.compute_mixture_energy_density()                          # J/m^3
        self.lower_heating_value        = self.compute_mixture_lower_heating_value() 

# RCAIDE/Library/Attributes/Cryogens/Liquid_Hydrogen.py
# 
#
# Created:  Mar 2024, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 

from .Cryogen import Cryogen

# ---------------------------------------------------------------------------------------------------------------------- 
#  Liquid Hydrogen Cryogen
# ----------------------------------------------------------------------------------------------------------------------  
class Liquid_Hydrogen(Cryogen):
    """
    A class representing liquid hydrogen cryogenic fuel and its thermodynamic properties.

    Attributes
    ----------
    tag : str
        Identifier for the cryogen type ('Liquid_Hydrogen')
    density : float
        Density of liquid hydrogen in kg/m³
    specific_energy : float
        Specific energy content in J/kg
    energy_density : float
        Energy density in J/m³
    temperatures : Container
        Temperature thresholds for the cryogen
            - freeze : float
                Freezing point temperature in Kelvin
            - boiling : float
                Boiling point temperature in Kelvin
    vaporization_enthalpy : float
        Enthalpy of vaporization in kJ/kg
    specific_heat : float
        Specific heat capacity in kJ/kg·K
    liquid_cp_coefficients : Data
        Coefficients for liquid specific heat capacity polynomial
            - LCP_C0 : float
                Constant term coefficient
            - LCP_C1 : float
                Linear term coefficient
            - LCP_C2 : float
                Quadratic term coefficient
            - LCP_C3 : float
                Cubic term coefficient
            - LCP_minT : float
                Minimum valid temperature in Kelvin
            - LCP_maxT : float
                Maximum valid temperature in Kelvin
    
    gas_cp_coefficients : Data
        Coefficients for gas specific heat capacity polynomial
            - GCP_C0 : float
                Constant term coefficient
            - GCP_C1 : float
                Linear term coefficient
            - GCP_C2 : float
                Quadratic term coefficient
            - GCP_C3 : float
                Cubic term coefficient
            - GCP_minT : float
                Minimum valid temperature in Kelvin
            - GCP_maxT : float
                Maximum valid temperature in Kelvin
    
    antoine_coefficients : Data
        Antoine equation coefficients for vapor pressure calculation
            - antoine_A : float
                A coefficient
            - antoine_B : float
                B coefficient
            - antoine_C : float
                C coefficient
            - antoine_minT : float
                Minimum valid temperature in Kelvin
            - antoine_maxT : float
                Maximum valid temperature in Kelvin
    
    vaporization_coefficients : Data
        Coefficients for vaporization enthalpy polynomial
            - H_C0 : float
                Constant term coefficient
            - H_C1 : float
                Linear term coefficient
            - H_C2 : float
                Quadratic term coefficient
            - H_C3 : float
                Cubic term coefficient
            - H_minP : float
                Minimum valid pressure in Pascal
            - H_maxP : float
                Maximum valid pressure in Pascal

    Notes
    -----
    The class implements various thermodynamic properties using polynomial fits and the Antoine equation.
    Specific heat capacity calculations are provided for both liquid and gas phases.
    Valid temperature and pressure ranges are specified for each correlation.
    
    **Definitions**
    
    'Antoine Equation'
        An equation relating vapor pressure to temperature: log10(P) = A - (B/(T+C))
    
    'Specific Heat Capacity'
        The amount of heat required to raise the temperature of 1 kg of the substance by 1 Kelvin
    
    References
    ----------
    [1] Ekin, J. (2006). Experimental techniques for low-temperature measurements: Cryostat design, material properties and superconductor critical-current testing. Oxford University Press. 
    [2] National Institute of Standards and Technology. (2023). NIST Chemistry Webbook, SRD 69. Thermophysical Properties of Fluid Systems. https://webbook.nist.gov/chemistry/fluid/ 
    """

    def __defaults__(self):
        """This sets the default values.
        Assumptions:
            None
        
        Source:
            Ekin - Experimental Techniques for Low Temperature Measurements, ISBN 0-19-857054-6
            NIST Chemistry Webbook
       """ 
        
        self.tag                        = 'Liquid_Hydrogen'
        self.density                    =    59.9            # [kg/m^3] 
        self.specific_energy            =   141.86e6         # [J/kg] 
        self.energy_density             =  8491.0e6          # [J/m^3]
        self.temperatures.freeze        =    13.99           # [K]
        self.temperatures.boiling       =    20.271          # [K]
        self.vaporization_enthalpy      =   461.             # [kJ/kg]
        self.specific_heat              =    10.67           # [kJ/kgK]

        # Coefficiencts for polynomial fit of Liquid Specific Heat Capacity (C_P) curve.
        # C_P = CP_C3*T^3 + CP_C2*T^2 + CP_C1*T^1 + CP_C0*T^0 where C_P is Specific Heat Capacity (J/gK) T is temperature (kelvin).
        # Data from NIST Chemistry Webbook. Pressure is 1.295MPa.
        self.LCP_C0                     =   -31.2
        self.LCP_C1                     =     5.56
        self.LCP_C2                     =    -0.272
        self.LCP_C3                     =     4.76E-03
        
        # Range for which this polynomial fit holds
        self.LCP_minT                   =    15.0              # [K]
        self.LCP_maxT                   =    30.0              # [K]

        # Coefficiencts for polynomial fit of Gas Specific Heat Capacity (C_P) curve.
        # C_P = CP_C3*T^3 + CP_C2*T^2 + CP_C1*T^1 + CP_C0*T^0 where C_P is Specific Heat Capacity (J/gK) T is temperature (kelvin).
        # Data from NIST Chemistry Webbook. Pressure is 0.01 MPa
        self.GCP_C0                     =   10.3
        self.GCP_C1                     =   -7.39E-03
        self.GCP_C2                     =    0.221E-03
        self.GCP_C3                     =   -0.516E-06
        
        # Range for which this polynomial fit holds
        self.GCP_minT                   =   20.0              # [K]
        self.GCP_maxT                   =  300.0              # [K]

        # Antoine Equation Coefficients for calculatating the evaporation temperature.
        # log10(P) = A - (B/(T+C)) where P is vapour pressure (Pa) and T temperature (kelvin).
        # Data from NIST Chemistry Webbook, coefficients converted so as to use pressure in Pa.
        self.antoine_A                  =    8.54314
        self.antoine_B                  =   99.395
        self.antoine_C                  =    7.726
        
        # Range for which Antoine Equation is referenced
        self.antoine_minT               =   21.01             # [K]
        self.antoine_maxT               =   32.27             # [K]

        # Coefficiencts for polynomial fit of vapourisation enthalpy
        # ΔH = H_C3*P^3 + H_C2*P^2 + H_C1*P^1 + H_C0*P^0 where ΔH is vapourisation enthalpy (kJ/kg), P is pressure (Pa). 
        self.H_C0                       =   464.
        self.H_C1                       =    -0.000176
        self.H_C2                       =    52.3E-12
        self.H_C3                       =  -100.00E-18
        
        # Range for which this polynomial fit holds
        self.H_minP                     =     0.02E6         # [Pa]
        self.H_maxP                     =     1.20E6         # [Pa]


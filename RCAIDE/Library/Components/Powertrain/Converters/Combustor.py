# RCAIDE/Library/Components/Propulsors/Converters/Combustor.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Feb 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports   
from .Converter  import Converter
from RCAIDE.Library.Methods.Powertrain.Converters.Combustor.append_combustor_conditions import  append_combustor_conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  Combustor
# ---------------------------------------------------------------------------------------------------------------------- 
class Combustor(Converter):
    """
    A combustor component model for gas turbine engines that simulates the combustion process.

    Attributes
    ----------
    tag : str
        Identifier for the combustor. Default is 'Combustor'.
        
    alphac : float
        Combustor entrance angle [rad]. Default is 0.0.
        
    turbine_inlet_temperature : float
        Temperature at turbine inlet [K]. Default is 1500.
        
    area_ratio : float
        Ratio of combustor exit to inlet area. Default is 1.0.
        
    axial_fuel_velocity_ratio : float
        Ratio of axial fuel velocity to inlet velocity. Default is 0.0.
        
    fuel_velocity_ratio : float
        Ratio of fuel velocity to inlet velocity. Default is 0.0.
        
    burner_drag_coefficient : float
        Drag coefficient of the burner. Default is 0.0.
        
    absolute_sensible_enthalpy : float
        Absolute sensible enthalpy [J/kg]. Default is 0.0.
        
    diameter : float
        Combustor diameter [m]. Default is 0.2.
        
    length : float
        Combustor length [m]. Default is 0.3.
        
    fuel_equivalency_ratio : float
        Fuel-to-air equivalency ratio. Default is 0.3.
        
    number_of_combustors : int
        Number of combustor cans. Default is 30.
        
    f_air_PZ : float
        Fraction of total air entering Primary Zone. Default is 0.18.
        
    FAR_st : float
        Stoichiometric Fuel to Air ratio. Default is 0.068.
        
    N_comb : int
        Number of can-annular combustors. Default is 10.
        
    N_PZ : int
        Number of PSR in the Primary Zone. Default is 8.
        
    A_PZ : float
        Primary Zone cross-sectional area [m²]. Default is 0.15.
        
    L_PZ : float
        Primary Zone length [m]. Default is 0.0153.
        
    N_SZ : int
        Number of dilution air inlets in the Secondary Zone. Default is 3.
        
    A_SZ : float
        Secondary Zone cross-sectional area [m²]. Default is 0.15.
        
    L_SZ : float
        Secondary Zone length [m]. Default is 0.075.
        
    phi_SZ : float
        Equivalence Ratio in the Secondary Zone. Default is 0.2.
        
    S_PZ : float
        Mixing parameter in the Primary Zone. Default is 0.6.
        
    F_SC : float
        Fuel scaler. Default is 0.425.
        
    number_of_assigned_PSR_1st_mixers : int
        Number of assigned PSRs to first row mixers. Default is 2.
        
    number_of_assigned_PSR_2nd_mixers : int
        Number of assigned mixers to second row mixers. Default is 2.

    Notes
    -----
    The Combustor class models the combustion process in gas turbine engines,
    splitting the combustor into primary and secondary zones. It uses a Chemical
    Reactor Network (CRN) approach with Perfectly Stirred Reactors (PSR) and Plug Flow Reactors (PFR) for
    modeling the combustion process.

    **Definitions**

    'PSR'
        Perfectly Stirred Reactor - A reactor model assuming perfect mixing
    
    'PZ'
        Primary Zone - Initial combustion region
        
    'SZ'
        Secondary Zone - Dilution region where the combustion is completed

    'FAR'
        Fuel-to-Air Ratio
    
    'PFR'
        Plug Flow Reactor - A reactor model assuming no mixing in axial direction

    See Also
    --------
    RCAIDE.Library.Methods.Emissions.Chemical_Reaction_Network.evaluate_cantera
    """
    
    def __defaults__(self):
        """This sets the default values for the component to function.

        Assumptions:
        None

        Source:
        None 
        """         
        
        self.tag                               = 'Combustor' 
        self.alphac                            = 0.0
        self.turbine_inlet_temperature         = 1500
        self.area_ratio                        = 1.0
        self.axial_fuel_velocity_ratio         = 0.0
        self.fuel_velocity_ratio               = 0.0
        self.burner_drag_coefficient           = 0.0
        self.absolute_sensible_enthalpy        = 0.0 
        self.diameter                          = 0.2
        self.length                            = 0.3
        self.fuel_equivalency_ratio            = 0.3 
        self.number_of_combustors              = 30 
                                               
        self.f_air_PZ                          = 0.18                                                  # [-]       Fraction of total air present in the combustor that enters the Primary Zone         
        self.FAR_st                            = 0.068                                                 # [-]       Stoichiometric Fuel to Air ratio
        self.N_comb                            = 10                                                    # [-]       Number of can-annular combustors
        self.N_PZ                              = 8                                                     # [-]       Number of PSR (EVEN, must match the number of PSR below)
        self.A_PZ                              = 0.15                                                  # [m**2]    Primary Zone cross-sectional area     
        self.L_PZ                              = 0.0153                                                # [m]       Primary Zone length  
        self.N_SZ                              = 3                                                     # [-]       Number of dilution air inlets        
        self.A_SZ                              = 0.15                                                  # [m**2]    Secondary Zone cross-sectional area
        self.L_SZ                              = 0.075                                                 # [m]       Secondary Zone length  
        self.phi_SZ                            = 0.2                                                   # [-]       Equivalence Ratio for PFR    phi_PZ_des              = 0.6                                                   # [-]       Primary Zone Design Equivalence Ratio
        self.S_PZ                              = 0.6                                                   # [-]       Mixing parameter, used to define the Equivalence Ratio standard deviation  
        self.F_SC                              = 0.425                                                 # [-]       Fuel scaler
        self.number_of_assigned_PSR_1st_mixers = 2                                                     # [-]       Number of assigned PSRs to each mixer in the first row of mixers (CRN network model)
        self.number_of_assigned_PSR_2nd_mixers = 2                                                     # [-]       Number of assigned mixers to each mixer in the second row of mixers (CRN network model)
    
    def append_operating_conditions(self,segment,propulsor):
        """
        Appends operating conditions to the combustor.
        """
        propulsor_conditions =  segment.state.conditions.energy[propulsor.tag]
        append_combustor_conditions(self,segment,propulsor_conditions)
        return
# RCAIDE/Library/Components/Propulsors/Converters/Combustor.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Feb 2024, M. Clarke
# Updated:  Mar 2025, M. Guidotti

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

        self.diameter                          = 0.437          # [m] Combustor Diameter
        self.length                            = 0.0903         # [m] Combustor Length
        self.number_of_combustors              = 1              # [-] Number of Combustors for one engine
        self.F_SC                              = 1              # [-] Fuel scale factor
        self.N_PZ                              = 21             # [-] Number of PSR in the Primary Zone
        self.L_PZ                              = 0.0153         # [m] Primary Zone length  
        self.S_PZ                              = 0.39           # [-] Mixing parameter in the Primary Zone  
        self.design_equivalence_ratio_PZ       = 1.77           # [-] Design Equivalence Ratio in Primary Zone at Maximum Throttle  
        self.N_SZ                              = 500            # [-] Number of discritizations in the Secondary Zone
        self.f_SM                              = 0.5            # [-] Slow mode fraction
        self.l_SA_SM                           = 0.55           # [-] Secondary air length fraction (of L_SZ) in slow mode
        self.l_SA_FM                           = 0.055          # [-] Secondary air length fraction (of L_SZ) in fast mode
        self.l_DA_start                        = 0.95           # [-] Dilution air start length fraction (of L_SZ)
        self.l_DA_end                          = 1.0            # [-] Dilution air end length fraction (of L_SZ)
        self.joint_mixing_fraction             = 0.6            # [-] Joint mixing fraction
        self.design_equivalence_ratio_SZ       = 0.7            # [-] Design Equivalence Ratio in Secondary Zone at Maximum Throttle
  
    def append_operating_conditions(self,segment,propulsor):
        """
        Appends operating conditions to the combustor.
        """
        propulsor_conditions =  segment.state.conditions.energy[propulsor.tag]
        append_combustor_conditions(self,segment,propulsor_conditions)
        return
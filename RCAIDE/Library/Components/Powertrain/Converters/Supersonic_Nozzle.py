# RCAIDE/Library/Components/Propulsors/Converters/Compressor.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Feb 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports   
from .Converter  import Converter
from RCAIDE.Library.Methods.Powertrain.Converters.Supersonic_Nozzle.append_supersonic_nozzle_conditions import append_supersonic_nozzle_conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  Compressor 
# ----------------------------------------------------------------------------------------------------------------------  
class Supersonic_Nozzle(Converter):
    """
    A supersonic nozzle component model for propulsion systems capable of supersonic outflow.

    Attributes
    ----------
    tag : str
        Identifier for the nozzle. Default is 'Supersonic_Nozzle'.
        
    polytropic_efficiency : float
        Efficiency of the expansion process accounting for losses. Default is 1.0.
        
    pressure_ratio : float
        Ratio of outlet to inlet pressure. Default is 1.0.
        
    pressure_recovery : float
        Ratio of recovered to ideal total pressure. Default is 1.0.
        
    max_area_ratio : float
        Maximum allowable exit to throat area ratio. Default is 1000.0.
        
    min_area_ratio : float
        Minimum allowable exit to throat area ratio. Default is 0.0.

    Notes
    -----
    The Supersonic_Nozzle class models a convergent-divergent nozzle that can 
    achieve supersonic exit flow. The model includes:
    
    * Isentropic flow relations
    * Shock wave effects
    * Variable area ratio capabilities
    * Real gas effects through efficiency terms
    * Pressure recovery modeling

    **Major Assumptions**

    * Pressure ratio and efficiency do not change with varying conditions
    * One-dimensional flow
    * Perfect gas behavior
    * Adiabatic process
    * No boundary layer separation
    * Steady flow conditions

    **Definitions**

    'Polytropic Efficiency'
        Measure of expansion process efficiency accounting for losses
    'Pressure Ratio'
        Ratio of exit static pressure to inlet total pressure
    'Pressure Recovery'
        Measure of total pressure preservation through the nozzle
    'Area Ratio'
        Ratio of exit area to throat area

    References
    ----------
    [1] Mattingly, J. D., & Boyer, K. M. (2016). Elements of propulsion: Gas 
        turbines and rockets, second edition Jack D. Mattingly, Keith M. Boyer. 
        American Institute of Aeronautics and Astronautics.

    See Also
    --------
    RCAIDE.Library.Components.Component
    RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle
    """
    
    def __defaults__(self):
        """ This sets the default values for the component to function.
        
        Assumptions:
        None
    
        Source:
        N/A
    
        Inputs:
        None
    
        Outputs:
        None
    
        Properties Used:
        None
        """        
        
        #set the defaults
        self.tag                             = 'Supersonic_Nozzle'
        self.polytropic_efficiency           = 1.0
        self.pressure_ratio                  = 1.0
        self.pressure_recovery               = 1.0 
        self.max_area_ratio                  = 1000.
        self.min_area_ratio                  = 0.

    def append_operating_conditions(self,segment,propulsor): 
        propulsor_conditions =  segment.state.conditions.energy[propulsor.tag]
        append_supersonic_nozzle_conditions(self,segment,propulsor_conditions)
        return 
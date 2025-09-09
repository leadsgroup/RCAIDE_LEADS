# RCAIDE/Library/Components/Powertrain/Converters/Auxiliary_Power_Unit.py
# 
#  
# Created:  Sep. 2025, M. Guidotti 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
## RCAIDE imports
from RCAIDE.Framework.Core                  import Data 
from .Turboelectric_Generator               import Turboelectric_Generator
 
# ----------------------------------------------------------------------
#  Auxiliary_Power_Unit
# ----------------------------------------------------------------------
class Auxiliary_Power_Unit(Turboelectric_Generator):
    """
    A Auxiliary_Power_Unit propulsion system model that simulates the performance of a Auxiliary_Power_Unit engine.
   

    Attributes
    ----------
    tag : str
        Identifier for the shaft engine. Default is 'turboshaft'. 
        
    turboshaft : Component
        Turboshaft component. Default is the Turboshaft Class.
        
    generator : Component
        Generator component. Default is DC_Generator Class.
        
    gearbox : Component
        Gearbox data structure. Default is None. 
        
    inverse_calculation : Component
        Flag that determines the how calculations are performed. Default is False    

    Notes
    -----
    The Auxiliary_Power_Unit class inherits from the Turboshaft class and implements
    methods for computing Auxiliary_Power_Unit engine performance. Unlike other gas turbine
    engines that produce thrust, a Auxiliary_Power_Unit engine's primary output is
    power. 

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Propulsors.Turboshaft 
    """ 
    def __defaults__(self):
        # setting the default values
        self.tag                       = 'Auxiliary_Power_Unit'
        self.gearbox                   = Data()
        self.gearbox.gear_ratio        = None  
        self.inverse_calculation       = False

    def append_operating_conditions(self,segment,energy_conditions,noise_conditions=None): 
        """
        Appends operating conditions to the segment.
        """  
        Turboelectric_Generator.append_turboelectric_generator_conditions(self,segment,energy_conditions) 
        return
 
    def compute_performance(self,state,fuel_line = None,bus = None):
        """
        Computes Turboelectric_Generator performance including power.
        """
        P_mech,P_elec,stored_results_flag,stored_propulsor_tag =  Turboelectric_Generator.compute_turboelectric_generator_performance(self,state,fuel_line, bus)
        return P_mech,P_elec,stored_results_flag,stored_propulsor_tag
    
    def reuse_stored_data(turboelectric_generator,state,stored_propulsor_tag):
        power  = Turboelectric_Generator.reuse_stored_turboelectric_generator_data(turboelectric_generator,state,stored_propulsor_tag)
        return power 


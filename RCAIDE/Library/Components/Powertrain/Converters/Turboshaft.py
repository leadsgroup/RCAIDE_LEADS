# RCAIDE/Library/Components/Converters/Turboshaft.py
# 
#  
# Created:  Mar 2024, M. Clarke
# Modified: Jun 2024, M. Guidotti  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
## RCAIDE imports
import RCAIDE
from .Converter                             import Converter
from RCAIDE.Library.Methods.Powertrain.Converters.Turboshaft.append_turboshaft_conditions     import append_turboshaft_conditions 
from RCAIDE.Library.Methods.Powertrain.Converters.Turboshaft.compute_turboshaft_performance   import compute_turboshaft_performance, reuse_stored_turboshaft_data
 
# ----------------------------------------------------------------------
#  Turboshaft
# ----------------------------------------------------------------------
class Turboshaft(Converter):
    """
    MATTEO There are defaults that are defined that might not be called anywhere so need to remove them
    """ 
    def __defaults__(self):
        # setting the default values
        self.tag                                              = 'turboshaft'
        self.fuel_type                                        = RCAIDE.Library.Attributes.Propellants.Jet_A1() 
        self.ram                                              = None 
        self.inlet_nozzle                                     = None 
        self.compressor                                       = None 
        self.low_pressure_turbine                             = None 
        self.high_pressure_turbine                            = None 
        self.combustor                                        = None 
        self.core_nozzle                                      = None
        self.active                                           = True
        self.length                                           = 0.0
        self.design_isa_deviation                             = 0.0
        self.design_altitude                                  = 0.0
        self.SFC_adjustment                                   = 0.0  
        self.reference_temperature                            = 288.15
        self.reference_pressure                               = 1.01325*10**5 
        self.design_power                                     = 0.0
        self.design_mass_flow_rate                            = 0.0 
        self.conversion_efficiency                            = 0.5
        self.compressor_nondimensional_massflow               = 0.0
        self.design_angular_velocity                          = 0.0

    def append_operating_conditions(self,segment,converter): 
        """
        Appends operating conditions to the segment.
        """  
        append_turboshaft_conditions(self,segment,converter) 
        return

    def unpack_propulsor_unknowns(self,segment):   
        return 

    def pack_propulsor_residuals(self,segment): 
        return      

    def append_propulsor_unknowns_and_residuals(self,segment): 
        return
    
    def compute_performance(self,state,converter = None,fuel_line = None,bus = None):
        """
        Computes turboshaft performance including thrust, moment, and power.
        """
        power,stored_results_flag,stored_propulsor_tag =  compute_turboshaft_performance(self,state,converter,fuel_line,bus)
        return  power,stored_results_flag,stored_propulsor_tag
    
    def reuse_stored_data(turboshaft,state,network,fuel_line = None, bus = None, stored_propulsor_tag = None,center_of_gravity = [[0, 0, 0]]):
        power  = reuse_stored_turboshaft_data(turboshaft,state,network,fuel_line,bus,stored_propulsor_tag,center_of_gravity)
        return power 

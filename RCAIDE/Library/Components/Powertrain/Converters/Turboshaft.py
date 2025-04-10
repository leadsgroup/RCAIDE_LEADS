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
    A turboshaft system model that simulates the performance of a turboshaft engine

    Attributes
    ----------
    tag : str
        Identifier for the shaft engine. Default is 'turboshaft'. 
        
    ram : Component
        Ram inlet component. Default is None.
        
    inlet_nozzle : Component
        Inlet nozzle component. Default is None.
        
    compressor : Component
        Low pressure compressor component. Default is None. 
        
    low_pressure_turbine : Component
        Low pressure turbine component. Default is None.
        
    high_pressure_turbine : Component
        High pressure turbine component. Default is None.
        
    combustor : Component
        Combustor component. Default is None.
        
    core_nozzle : Component
        Core exhaust nozzle component. Default is None. 
        
    active_crypgenic_tanks_tanks : None or list
        Collection of active cryogenoc tanks. Default is None.
        
    diameter : float
        Diameter of the engine [m]. Default is 0.0.
        
    length : float
        Length of the engine [m]. Default is 0.0.
        
    height : float
        Engine centerline height above the ground plane [m]. Default is 0.5. 
        Engine bypass ratio. Default is 0.0.
        
    design_isa_deviation : float
        ISA temperature deviation at design point [K]. Default is 0.0.
        
    design_altitude : float
        Design altitude of the engine [m]. Default is 0.0.
         
    specific_fuel_consumption_reduction_factor : float
        Specific fuel consumption adjustment factor (Less than 1 is a reduction). Default is 0.0.
        
    compressor_nondimensional_massflow : float
        Non-dimensional mass flow through the compressor. Default is 0.0.
        
    reference_temperature : float
        Reference temperature for calculations [K]. Default is 288.15.
        
    reference_pressure : float
        Reference pressure for calculations [Pa]. Default is 101325.0.
        
    design_mass_flow_rate : float
        Design mass flow rate of turboshaft [kg/s]. Default is 0.0
        
    conversion_efficiency : float
        Conversion efficiency of turboshaft [-]. Default is 0.5
        
    design_power : float
        Design power of the engine [W]. Default is 0.0. 

    **Definitions**

    'ISA'
        International Standard Atmosphere - standard atmospheric model

    'SFC'
        Specific Fuel Consumption - fuel efficiency metric 

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters.Turboelectric_Generator
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
        self.diamter                                          = 0.0
        self.design_isa_deviation                             = 0.0
        self.design_altitude                                  = 0.0
        self.specific_fuel_consumption_reduction_factor       = 0.0  
        self.reference_temperature                            = 288.15
        self.reference_pressure                               = 1.01325*10**5 
        self.design_power                                     = 0.0
        self.design_mass_flow_rate                            = 0.0 
        self.conversion_efficiency                            = 0.5
        self.compressor_nondimensional_massflow               = 0.0
        self.design_angular_velocity                          = 0.0
        self.inverse_calculation                              = False

    def append_operating_conditions(self,segment,energy_conditions,noise_conditions=None): 
        """
        Appends operating conditions to the segment.
        """  
        append_turboshaft_conditions(self,segment,energy_conditions,noise_conditions) 
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
        power,stored_results_flag,stored_propulsor_tag =  compute_turboshaft_performance(self,state,fuel_line=fuel_line,bus=bus)
        return  power,stored_results_flag,stored_propulsor_tag
    
    def reuse_stored_data(turboshaft,state,network,stored_propulsor_tag = None):
        power  = reuse_stored_turboshaft_data(turboshaft,state,network,stored_propulsor_tag)
        return power 

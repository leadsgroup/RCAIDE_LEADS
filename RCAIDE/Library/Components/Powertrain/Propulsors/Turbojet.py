# RCAIDE/Library/Components/Propulsors/Turbojet.py  
#
#
# Created:  Mar 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
## RCAIDE imports   
from RCAIDE.Framework.Core      import Data
from .                          import Propulsor
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet_Propulsor.append_turbojet_conditions     import append_turbojet_conditions 
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet_Propulsor.compute_turbojet_performance   import compute_turbojet_performance, reuse_stored_turbojet_data
 
 
# ----------------------------------------------------------------------
#  Turbojet Propulsor
# ---------------------------------------------------------------------- 
class Turbojet(Propulsor):
    """
    A turbojet propulsion system model that simulates the performance of a turbojet engine.

    Attributes
    ----------
    tag : str
        Identifier for the turbojet engine. Default is 'Turbojet'. 
        
    nacelle : Component
        Nacelle component of the engine. Default is None.
        
    ram : Component
        Ram inlet component. Default is None.
        
    inlet_nozzle : Component
        Inlet nozzle component. Default is None.
        
    low_pressure_compressor : Component
        Low pressure compressor component. Default is None.
        
    high_pressure_compressor : Component
        High pressure compressor component. Default is None.
        
    low_pressure_turbine : Component
        Low pressure turbine component. Default is None.
        
    high_pressure_turbine : Component
        High pressure turbine component. Default is None.
        
    combustor : Component
        Combustor component. Default is None.
        
    afterburner : Component
        Afterburner component. Default is None.
        
    core_nozzle : Component
        Core exhaust nozzle component. Default is None.
        
    engine_length : float
        Length of the engine [m]. Default is 0.0.
        
    bypass_ratio : float
        Engine bypass ratio. Default is 0.0.
        
    design_isa_deviation : float
        ISA temperature deviation at design point [K]. Default is 0.0.
        
    design_altitude : float
        Design altitude of the engine [m]. Default is 0.0.
        
    afterburner_active : bool
        Flag indicating if afterburner is in use. Default is False.
        
    SFC_adjustment : float
        Specific fuel consumption adjustment factor (Less than 1 is a reduction). Default is 0.0.
        
    compressor_nondimensional_massflow : float
        Non-dimensional mass flow through the compressor. Default is 0.0.
        
    reference_temperature : float
        Reference temperature for calculations [K]. Default is 288.15.
        
    reference_pressure : float
        Reference pressure for calculations [Pa]. Default is 101325.0.
        
    design_thrust : float
        Design thrust of the engine [N]. Default is 0.0.
        
    mass_flow_rate_design : float
        Design mass flow rate [kg/s]. Default is 0.0.
        
    OpenVSP_flow_through : bool
        Flag for OpenVSP flow-through analysis. Default is False.
        
    areas : Data
        Collection of engine areas

        - wetted : float
            Wetted area [m²]. Default is 0.0.

        - maximum : float
            Maximum cross-sectional area [m²]. Default is 0.0.
            
        - exit : float
            Exit area [m²]. Default is 0.0.

        - inflow : float
            Inflow area [m²]. Default is 0.0.

    Notes
    -----
    The Turbojet class inherits from the Propulsor base class and implements
    methods for computing turbojet engine performance. Unlike a turbofan engine,
    a turbojet does not have a bypass flow and all air goes through the core.

    **Definitions**
    
    'ISA'
        International Standard Atmosphere - standard atmospheric model

    'SFC'
        Specific Fuel Consumption - fuel efficiency metric
    
    'OpenVSP'
        Open Vehicle Sketch Pad - open-source parametric aircraft geometry tool

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Propulsors.Propulsor
    RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan
    """ 
    def __defaults__(self):
        # setting the default values
        self.tag                                      = 'Turbojet'  
        self.nacelle                                  = None  
        self.ram                                      = None 
        self.inlet_nozzle                             = None 
        self.low_pressure_compressor                  = None 
        self.high_pressure_compressor                 = None 
        self.low_pressure_turbine                     = None 
        self.high_pressure_turbine                    = None 
        self.combustor                                = None 
        self.afterburner                              = None
        self.core_nozzle                              = None      
        
        self.engine_length                            = 0.0
        self.bypass_ratio                             = 0.0 
        self.design_isa_deviation                     = 0.0
        self.design_altitude                          = 0.0
        self.afterburner_active                       = False
        self.SFC_adjustment                           = 0.0  
        self.compressor_nondimensional_massflow       = 0.0
        self.reference_temperature                    = 288.15
        self.reference_pressure                       = 1.01325*10**5 
        self.design_thrust                            = 0.0
        self.mass_flow_rate_design                    = 0.0 
        self.OpenVSP_flow_through                     = False

        #areas needed for drag; not in there yet
        self.areas                                    = Data()
        self.areas.wetted                             = 0.0
        self.areas.maximum                            = 0.0
        self.areas.exit                               = 0.0
        self.areas.inflow                             = 0.0 


    def append_operating_conditions(self,segment):
        append_turbojet_conditions(self,segment)
        return

    def unpack_propulsor_unknowns(self,segment):   
        return 

    def pack_propulsor_residuals(self,segment): 
        return        

    def append_propulsor_unknowns_and_residuals(self,segment): 
        return
    
    def compute_performance(self,state,center_of_gravity = [[0, 0, 0]]):
        thrust,moment,power,stored_results_flag,stored_propulsor_tag =  compute_turbojet_performance(self,state,center_of_gravity)
        return thrust,moment,power,stored_results_flag,stored_propulsor_tag
    
    def reuse_stored_data(turbojet,state,network,stored_propulsor_tag,center_of_gravity = [[0, 0, 0]]):
        thrust,moment,power  = reuse_stored_turbojet_data(turbojet,state,network,stored_propulsor_tag,center_of_gravity)
        return thrust,moment,power 
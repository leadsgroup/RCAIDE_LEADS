# RCAIDE/Library/Components/Propulsors/Turbofan.py 
#
#
# Created:  Mar 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports  
from RCAIDE.Framework.Core      import Container
from .                          import Propulsor
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan_Propulsor.append_turbofan_conditions     import append_turbofan_conditions 
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan_Propulsor.compute_turbofan_performance   import compute_turbofan_performance, reuse_stored_turbofan_data
 
# ---------------------------------------------------------------------------------------------------------------------- 
#  Fan Component
# ---------------------------------------------------------------------------------------------------------------------- 
class Turbofan(Propulsor):
    """
    A turbofan propulsion system model that simulates the performance of a turbofan engine.

    Attributes
    ----------
    tag : str
        Identifier for the turbofan engine. Default is 'Turbofan'.
    
    nacelle : Component
        Nacelle component of the engine. Default is None.
        
    fan : Component
        Fan component of the engine. Default is None.
        
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
        
    core_nozzle : Component
        Core exhaust nozzle component. Default is None.
        
    fan_nozzle : Component
        Fan exhaust nozzle component. Default is None.
        
    active_crypgenic_tanks_tanks : None or list
        Collection of active cryogenoc tanks. Default is None.
        
    engine_diameter : float
        Diameter of the engine [m]. Default is 0.0.
        
    engine_length : float
        Length of the engine [m]. Default is 0.0.
        
    engine_height : float
        Engine centerline height above the ground plane [m]. Default is 0.5.
        
    exa : float
        Distance from fan face to fan exit normalized by fan diameter. Default is 1.0.
        
    plug_diameter : float
        Diameter of the engine plug [m]. Default is 0.1.
        
    geometry_xe : float
        Installation effects geometry parameter. Default is 1.0.
        
    geometry_ye : float
        Installation effects geometry parameter. Default is 1.0.
        
    geometry_Ce : float
        Installation effects geometry parameter. Default is 2.0.
        
    bypass_ratio : float
        Engine bypass ratio. Default is 0.0.
        
    design_isa_deviation : float
        ISA temperature deviation at design point [K]. Default is 0.0.
        
    design_altitude : float
        Design altitude of the engine [m]. Default is 0.0.
        
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

    Notes
    -----
    The Turbofan class inherits from the Propulsor base class and implements
    methods for computing turbofan engine performance. It includes functionality
    for handling different operating conditions and performance calculations.

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
    RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet
    RCAIDE.Library.Components.Powertrain.Propulsors.Turboprop
    RCAIDE.Library.Components.Powertrain.Propulsors.Turboshaft
    """
    def __defaults__(self):    
        # setting the default values
        self.tag                                      = 'Turbofan'  
        self.nacelle                                  = None 
        self.fan                                      = None 
        self.ram                                      = None 
        self.inlet_nozzle                             = None 
        self.low_pressure_compressor                  = None 
        self.high_pressure_compressor                 = None 
        self.low_pressure_turbine                     = None 
        self.high_pressure_turbine                    = None 
        self.combustor                                = None 
        self.core_nozzle                              = None 
        self.fan_nozzle                               = None       
        self.engine_diameter                          = 0.0      
        self.engine_length                            = 0.0
        self.engine_height                            = 0.5     # Engine centerline heigh above the ground plane
        self.exa                                      = 1       # distance from fan face to fan exit/ fan diameter)
        self.plug_diameter                            = 0.1     # dimater of the engine plug
        self.geometry_xe                              = 1.      # Geometry information for the installation effects function
        self.geometry_ye                              = 1.      # Geometry information for the installation effects function
        self.geometry_Ce                              = 2.      # Geometry information for the installation effects function
        self.bypass_ratio                             = 0.0 
        self.design_isa_deviation                     = 0.0
        self.design_altitude                          = 0.0
        self.SFC_adjustment                           = 0.0 # Less than 1 is a reduction
        self.compressor_nondimensional_massflow       = 0.0
        self.reference_temperature                    = 288.15
        self.reference_pressure                       = 1.01325*10**5 
        self.design_thrust                            = 0.0
        self.mass_flow_rate_design                    = 0.0
        self.OpenVSP_flow_through                     = False
    
    def append_operating_conditions(self,segment):
        """
        Appends operating conditions to the segment.
        """
        append_turbofan_conditions(self,segment)
        return

    def unpack_propulsor_unknowns(self,segment):   
        return 

    def pack_propulsor_residuals(self,segment): 
        return

    def append_propulsor_unknowns_and_residuals(self,segment): 
        return
    
    def compute_performance(self,state,center_of_gravity = [[0, 0, 0]]):
        """
        Computes turbofan performance including thrust, moment, and power.
        """
        thrust,moment,power,stored_results_flag,stored_propulsor_tag =  compute_turbofan_performance(self,state,center_of_gravity)
        return thrust,moment,power,stored_results_flag,stored_propulsor_tag
    
    def reuse_stored_data(turbofan,state,network,stored_propulsor_tag,center_of_gravity = [[0, 0, 0]]):
        """
        Reuses stored turbofan data for performance calculations.
        """
        thrust,moment,power  = reuse_stored_turbofan_data(turbofan,state,network,stored_propulsor_tag,center_of_gravity)
        return thrust,moment,power 
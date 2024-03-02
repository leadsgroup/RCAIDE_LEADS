## @ingroup Energy-Propulsors-Converters
# RCAIDE/Energy/Propulsors/Converters/Fan.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
## RCAIDE imports   
from RCAIDE.Core      import Data, Units 
from .                import Propulsor

import numpy as np 

# ----------------------------------------------------------------------
#  Fan Component
# ----------------------------------------------------------------------
## @ingroup Energy-Propulsors-Converters
class Turbojet(Propulsor):
    """This is a  turbojet propulsor

    Assumptions:
    None

    Source:
    None
    """ 
    def __defaults__(self):
        # setting the default values
        self.tag                                      = 'Turbojet' 
        self.active_fuel_tanks                        = None
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

        self.inputs                                   = Data()
        self.outputs                                  = Data()

        self.inputs.fuel_to_air_ratio                 = 0.0
        self.outputs.thrust                           = 0.0 
        self.outputs.thrust_specific_fuel_consumption = 0.0
        self.outputs.specific_impulse                 = 0.0
        self.outputs.non_dimensional_thrust           = 0.0
        self.outputs.core_mass_flow_rate              = 0.0
        self.outputs.fuel_flow_rate                   = 0.0
        self.outputs.fuel_mass                        = 0.0
        self.outputs.power                            = 0.0 

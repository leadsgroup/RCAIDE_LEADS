# Created:  Jul 2023, M. Clarke
# Modified: Jun 2024, M. Guidotti & D.J. Lee

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core                                             import Units    
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboshaft_Propulsor import design_turboshaft 
from RCAIDE.Framework.Mission.Common                                   import Conditions
from RCAIDE.Library.Plots                                              import *     

# python imports 
import numpy   as np      

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():  
    altitude           = 0.01*Units.feet 
    mach               = 0.1  
    P , eta , PSFC = turboshaft_engine_Boeing_502_14(altitude,mach)
     
    P_truth    = 147999.99999999997
    eta_truth  = 0.505404846037369
    PSFC_truth = 1.3566178140154075e-07
    

    P_error = np.abs((P  - P_truth)/P_truth)
    print('Power Error: ' + str(P_error)) 
    print(np.abs((P  - P_truth)/P_truth))
    assert P_error < 1e-6 

    PSFC_error = np.abs((PSFC  - PSFC_truth)/PSFC_truth)
    print('Power Error: ' + str(PSFC_error)) 
    print(np.abs((PSFC  - PSFC_truth)/PSFC_truth))
    assert PSFC_error < 1e-6 

    eta_error = np.abs((eta  - eta_truth)/eta_truth)
    print('Power Error: ' + str(eta_error)) 
    print(np.abs((eta  - eta_truth)/eta_truth))
    assert eta_error < 1e-6         
     
    
    return     
  
def turboshaft_engine_Boeing_502_14(altitude,mach):   

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------         
    turboshaft                                     = RCAIDE.Library.Components.Powertrain.Propulsors.Turboshaft() 
    turboshaft.tag                                 = 'Turboshaft_propulsor'
    turboshaft.origin                              = [[13.72, 4.86,-1.1]] 
    turboshaft.engine_length                       = 0.945     
    turboshaft.bypass_ratio                        = 0    
    turboshaft.design_altitude                     = 0.01*Units.ft
    turboshaft.design_mach_number                  = 0.1   
    turboshaft.design_power                        = 148000.0*Units.W 
    turboshaft.mass_flow_rate_design               = 1.9 #[kg/s]
    
    # working fluid                                    
    turboshaft.working_fluid                       = RCAIDE.Library.Attributes.Gases.Air() 
    ram                                            = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                        = 'ram' 
    turboshaft.ram                                 = ram 
                                                   
    # inlet nozzle                                 
    inlet_nozzle                                   = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                               = 'inlet nozzle'
    inlet_nozzle.polytropic_efficiency             = 0.98
    inlet_nozzle.pressure_ratio                    = 0.98 
    turboshaft.inlet_nozzle                        = inlet_nozzle 
                                                   
    # compressor                                   
    compressor                                     = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    compressor.tag                                 = 'compressor'
    compressor.polytropic_efficiency               = 0.91
    compressor.pressure_ratio                      = 4.35  
    compressor.mass_flow_rate                      = 1.9 
    turboshaft.compressor                          = compressor

    # low pressure turbine  
    low_pressure_turbine                           = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    low_pressure_turbine.tag                       ='lpt'
    low_pressure_turbine.mechanical_efficiency     = 0.99
    low_pressure_turbine.polytropic_efficiency     = 0.93 
    turboshaft.low_pressure_turbine                = low_pressure_turbine
   
    # high pressure turbine     
    high_pressure_turbine                          = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    high_pressure_turbine.tag                      ='hpt'
    high_pressure_turbine.mechanical_efficiency    = 0.99
    high_pressure_turbine.polytropic_efficiency    = 0.93 
    turboshaft.high_pressure_turbine               = high_pressure_turbine 

    # combustor  
    combustor                                      = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                                  = 'Comb'
    combustor.efficiency                           = 0.99 
    combustor.alphac                               = 1.0     
    combustor.turbine_inlet_temperature            = 889
    combustor.pressure_ratio                       = 0.95
    combustor.fuel_data                            = RCAIDE.Library.Attributes.Propellants.Jet_A()  
    turboshaft.combustor                           = combustor

    # core nozzle
    core_nozzle                                    = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                = 'core nozzle'
    core_nozzle.polytropic_efficiency              = 0.95
    core_nozzle.pressure_ratio                     = 0.99  
    turboshaft.core_nozzle                         = core_nozzle

    # design turboshaft
    design_turboshaft(turboshaft)     
    
    # ------------------------------------------------------------------------------------------------------------------------
    # Run engine 
    # ------------------------------------------------------------------------------------------------------------------------   
                                                                                  
    # define atmospheric properties                                           
    atmosphere                                                          = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data                                                           = atmosphere.compute_values(altitude,0.0)
    planet                                                              = RCAIDE.Library.Attributes.Planets.Earth() 
    p                                                                   = atmo_data.pressure          
    T                                                                   = atmo_data.temperature       
    rho                                                                 = atmo_data.density          
    a                                                                   = atmo_data.speed_of_sound    
    mu                                                                  = atmo_data.dynamic_viscosity      
                                                                              
    conditions = RCAIDE.Framework.Mission.Common.Results()
    
    # setting conditions for current simulation                               
    conditions.freestream.altitude                                      = np.atleast_2d(altitude)
    conditions.freestream.mach_number                                   = np.atleast_2d(mach)
    conditions.freestream.pressure                                      = np.atleast_2d(p)
    conditions.freestream.temperature                                   = np.atleast_2d(T)
    conditions.freestream.density                                       = np.atleast_2d(rho)
    conditions.freestream.dynamic_viscosity                             = np.atleast_2d(mu)
    conditions.freestream.gravity                                       = np.atleast_2d(planet.sea_level_gravity)
    conditions.freestream.isentropic_expansion_factor                   = np.atleast_2d(turboshaft.working_fluid.compute_gamma(T,p))
    conditions.freestream.Cp                                            = np.atleast_2d(turboshaft.working_fluid.compute_cp(T,p))
    conditions.freestream.R                                             = np.atleast_2d(turboshaft.working_fluid.gas_specific_constant)
    conditions.freestream.speed_of_sound                                = np.atleast_2d(a)
    conditions.freestream.velocity                                      = np.atleast_2d(a*mach)   
 

    # setup conditions   
    segment                  = RCAIDE.Framework.Mission.Segments.Segment()  
    segment.state.conditions = conditions      
    turboshaft.append_operating_conditions(segment) 
    for tag, item in  turboshaft.items(): 
        if issubclass(type(item), RCAIDE.Library.Components.Component):
            item.append_operating_conditions(segment,turboshaft) 
    
    # set throttle
    segment.state.conditions.energy[turboshaft.tag].throttle[:,0] = 1.0  
    thrust,moment,power,stored_results_flag,stored_propulsor_tag = turboshaft.compute_performance(segment.state)  
    
    power                = power[0][0]
    thermal_efficiency   = conditions.energy[turboshaft.tag].thermal_efficiency[0][0]
    PSFC                 = conditions.energy[turboshaft.tag].power_specific_fuel_consumption[0][0]

    return power, thermal_efficiency, PSFC

if __name__ == '__main__': 
    main()
    
    
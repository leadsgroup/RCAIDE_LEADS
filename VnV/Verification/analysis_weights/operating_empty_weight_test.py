# weights.py
import  RCAIDE
from RCAIDE.Framework.Core import Data, Units 
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan   import design_turbofan  
from RCAIDE.Library.Plots import * 
from RCAIDE.load import load as load_results
from RCAIDE.save import save as save_results 
from RCAIDE.Library.Methods.Geometry.LOPA import compute_layout_of_passenger_accommodations
from RCAIDE.Library.Methods.Geometry.Planform import wing_planform,bwb_wing_planform,update_blended_wing_body_planform
import numpy as  np 
import sys
import os

sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
# the analysis functions

from Boeing_737             import vehicle_setup as transport_setup
from Cessna_172             import vehicle_setup as general_aviation_setup
from BWB                    import vehicle_setup as bwb_setup
from Stopped_Rotor_EVTOL    import vehicle_setup as evtol_setup
from Boeing_787             import vehicle_setup as hydrogen_transport_setup

def main():
    update_regression_values = False  # should be false unless code functionally changes
    show_figure              = False # leave false for regression

    Transport_Aircraft_Test(update_regression_values,show_figure)
    BWB_Aircraft_Test(update_regression_values,show_figure)
    General_Aviation_Test(update_regression_values,show_figure)
    EVTOL_Aircraft_Test(update_regression_values,show_figure)
    Transport_Hydrogen_Test(update_regression_values,show_figure)
    return


def Transport_Hydrogen_Test(update_regression_values, show_figure):
    method_types = ['Semi_Empirical']

    vehicle = hydrogen_transport_setup()
    for propulsor in vehicle.networks.fuel.propulsors:
        propulsor.combustor.fuel_data =  RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen() 
    for fuel_line in vehicle.networks.fuel.fuel_lines:
        for fuel_tank in fuel_line.fuel_tanks:
            fuel_tank.fuel                                   = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()   
            fuel_tank.fuel.gravimetric_efficiency            = 0.5

    for method_type in method_types:
        print(f'Testing Transport Aircraft Method: {method_type} | Method: {"Complex"}')        
        weight_analysis = RCAIDE.Framework.Analyses.Weights.Hydrogen_Transport()
        weight_analysis.vehicle = vehicle
        for wing in weight_analysis.vehicle.wings: 
            wing_planform(wing,overwrite_reference =  True) 
            if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing):
                weight_analysis.vehicle.reference_area = wing.areas.reference
        weight_analysis.method = method_type 


        weight = weight_analysis.evaluate()
        plot_weight_breakdown(weight_analysis.vehicle, show_figure=show_figure)

        save_path = os.path.join(os.path.dirname(__file__), f'hydrogen_weights_transport_{method_type}.res')

        if update_regression_values:
            save_results(weight, save_path)
        old_weight = load_results(save_path)

        check_list = [
            'payload.total', 'payload.passengers', 'payload.baggage',
            'empty.structural.wings', 'empty.structural.fuselage',
            'empty.propulsion.total', 'empty.structural.landing_gear',
            'empty.systems.total', 'empty.total'
        ]

        for k in check_list:
            old_val = old_weight.deep_get(k)
            new_val = weight.deep_get(k)
            err = (new_val - old_val) / old_val
            print(f'{k} Error: {err:.6e}')
            assert np.abs(err) < 1e-6, f'Check Failed: {k}'
        print('')

def Transport_Aircraft_Test(update_regression_values, show_figure):
    method_types = ['FLOPS', 'Raymer']
    FLOPS_number = 0 
    for advanced_composites in [True, False]:  

        for method_type in method_types:
            print(f'Testing Transport Aircraft Method: {method_type} | Advanced Composites: {advanced_composites} | Method: {"Simple" if FLOPS_number == 0 else "Complex"}')        
            weight_analysis = RCAIDE.Framework.Analyses.Weights.Conventional_Transport()
            weight_analysis.vehicle = transport_setup()
            for wing in weight_analysis.vehicle.wings: 
                wing_planform(wing,overwrite_reference =  True) 
                if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing):
                    weight_analysis.vehicle.reference_area = wing.areas.reference
            weight_analysis.method = method_type
            weight_analysis.settings.advanced_composites = advanced_composites 

            if method_type == 'FLOPS':
                save_filename = f'FLOPS_{"Simple" if FLOPS_number == 0 else "Complex"}'
                weight_analysis.settings.FLOPS.complexity = 'Simple' if FLOPS_number == 0 else 'Complex'
                FLOPS_number += 1
            else:
                save_filename = 'Raymer'

            if advanced_composites:
                save_filename += '_Advanced_Composite'

            weight = weight_analysis.evaluate()
            plot_weight_breakdown(weight_analysis.vehicle, show_figure=show_figure)

            save_path = os.path.join(os.path.dirname(__file__), f'weights_transport_{save_filename}.res')

            if update_regression_values:
                save_results(weight, save_path)
            old_weight = load_results(save_path)

            check_list = [
                'payload.total', 'payload.passengers', 'payload.baggage',
                'empty.structural.wings', 'empty.structural.fuselage',
                'empty.propulsion.total', 'empty.structural.landing_gear',
                'empty.systems.total', 'empty.total'
            ]

            for k in check_list:
                old_val = old_weight.deep_get(k)
                new_val = weight.deep_get(k)
                err = (new_val - old_val) / old_val
                print(f'{k} Error: {err:.6e}')
                assert np.abs(err) < 1e-6, f'Check Failed: {k}'
            print('')


def General_Aviation_Test(update_regression_values, show_figure):
    method_types = ['FLOPS', 'FLOPS','Raymer']

    for advanced_composite in [True, False]:  
        FLOPS_number = 0  
        for method_type in method_types:
            print(f'Testing General Aviation Method: {method_type} | Advanced Composites: {advanced_composite} | Method: {("Simple" if FLOPS_number == 0 else "Complex") if method_type == "FLOPS" else "Raymer"}')


            weight_analysis = RCAIDE.Framework.Analyses.Weights.Conventional_General_Aviation()
            weight_analysis.vehicle = general_aviation_setup()
            for wing in weight_analysis.vehicle.wings: 
                wing_planform(wing,overwrite_reference =  True) 
                if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing):
                    weight_analysis.vehicle.reference_area = wing.areas.reference
            weight_analysis.method = method_type 
            weight_analysis.settings.advanced_composites = advanced_composite

            if method_type == 'FLOPS':
                save_filename = f'FLOPS_{"Simple" if FLOPS_number == 0 else "Complex"}'
                weight_analysis.settings.FLOPS.complexity = 'Simple' if FLOPS_number == 0 else 'Complex'
                FLOPS_number += 1
            else:
                save_filename = 'Raymer'

            if advanced_composite:
                save_filename += '_Advanced_Composite'

            weight = weight_analysis.evaluate()
            plot_weight_breakdown(weight_analysis.vehicle, show_figure=show_figure)

            save_path = os.path.join(os.path.dirname(__file__), f'weights_general_aviation_{save_filename}.res')

            if update_regression_values:
                save_results(weight, save_path)
            old_weight = load_results(save_path)

            check_list = [
                'empty.total', 'empty.structural.wings', 'empty.structural.fuselage',
                'empty.structural.total', 'empty.propulsion.total', 'empty.systems.total'
            ]

            for k in check_list:
                old_val = old_weight.deep_get(k)
                new_val = weight.deep_get(k)
                err = (new_val - old_val) / old_val
                print(f'{k} Error: {err:.6e}')
                assert np.abs(err) < 1e-6, f'Check Failed: {k}'
            print('')

    FLOPS_number = 0
    for method_type in method_types:
        # ---------
        # Jet Cessna 172 Variant Testing
        print(f'Testing Jet General Aviation Method: {method_type} | Advanced Composites: {advanced_composite} | Method: {("Simple" if FLOPS_number == 0 else "Complex") if method_type == "FLOPS" else "Raymer"}')
        weight_analysis = RCAIDE.Framework.Analyses.Weights.Conventional_General_Aviation()
        jet_cessna_172 = general_aviation_setup()
        jet_cessna_172.networks.fuel.propulsors.pop('ice_propeller')
        turbine = Jet_engine()
        jet_cessna_172.networks.fuel.propulsors.append(turbine)
        jet_cessna_172.networks.fuel.fuel_lines['fuel_line'].assigned_propulsors = [[turbine.tag]]
        weight_analysis.vehicle = jet_cessna_172
        weight_analysis.method = method_type 
        weight_analysis.settings.advanced_composites = False

        if method_type == 'FLOPS' and FLOPS_number == 0:
            save_filename = f'FLOPS_{"Simple"}_Jet'
            weight_analysis.settings.FLOPS.complexity = 'Simple'
            FLOPS_number += 1
        elif method_type == 'FLOPS' :
            save_filename = f'FLOPS_{"Complex"}_Jet'
            weight_analysis.settings.FLOPS.complexity = 'Complex'
            FLOPS_number += 1
        else:
            save_filename = 'Raymer_Jet'

        weight = weight_analysis.evaluate()
        plot_weight_breakdown(weight_analysis.vehicle, show_figure=show_figure)

        save_path = os.path.join(os.path.dirname(__file__), f'weights_general_aviation_{save_filename}.res')

        if update_regression_values:
            save_results(weight, save_path)
        old_weight = load_results(save_path)

        check_list = [
            'empty.total', 'empty.structural.wings', 'empty.structural.fuselage',
                'empty.structural.total', 'empty.propulsion.total', 'empty.systems.total'
        ]
        for k in check_list:
            old_val = old_weight.deep_get(k)
            new_val = weight.deep_get(k)
            err = (new_val - old_val) / old_val
            print(f'{k} Error: {err:.6e}')
            assert np.abs(err) < 1e-6, f'Check Failed: {k}'
        print('')

def BWB_Aircraft_Test(update_regression_values,show_figure):
    cabin_types = ['Non-PERSUS','PERSUS']
    for cabin_type in cabin_types:
        for FLOPS_number in [0,1]:
            print(f'Testing Transport Aircraft Method: BWB| Composites: {cabin_type} | Method: {"Simple" if FLOPS_number == 0 else "Complex"}') 
            weight_analysis          = RCAIDE.Framework.Analyses.Weights.Conventional_BWB()
            weight_analysis.vehicle  = bwb_setup()
            if cabin_type == 'PERSUS':
                weight_analysis.settings.PRSEUS = True
            elif cabin_type == 'Non-PERSUS':
                weight_analysis.settings.PRSEUS = False
            for wing in weight_analysis.vehicle.wings: 
                if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
                    compute_layout_of_passenger_accommodations(wing)  
                    update_blended_wing_body_planform(wing)
                    bwb_wing_planform(wing,overwrite_reference = True)
                    weight_analysis.vehicle.reference_area = wing.areas.reference 
            weight_analysis.settings.FLOPS.complexity = 'Simple' if FLOPS_number == 0 else 'Complex'
            weight                   = weight_analysis.evaluate()
            plot_weight_breakdown(weight_analysis.vehicle, show_figure = show_figure) 

            if update_regression_values:
                save_results(weight, os.path.join(os.path.dirname(__file__), f"{cabin_type}_{'Simple' if FLOPS_number == 0 else 'Complex'}_weights_BWB.res"))
            old_weight = load_results(os.path.join(os.path.dirname(__file__), f"{cabin_type}_{'Simple' if FLOPS_number == 0 else 'Complex'}_weights_BWB.res"))

            check_list = [
                'empty.total',
                'empty.structural.wings', 
                'empty.structural.total',
                'empty.propulsion.total',   
                'empty.systems.total',  
            ]

            # do the check
            for k in check_list:
                print(k)

                old_val = old_weight.deep_get(k)
                new_val = weight.deep_get(k)
                err = (new_val-old_val)/old_val
                print('Error:' , err)
                assert np.abs(err) < 1e-6 , 'Check Failed : %s' % k     

                print('')

    return

def EVTOL_Aircraft_Test(update_regression_values,show_figure): 
    weight_analysis          = RCAIDE.Framework.Analyses.Weights.Electric_VTOL()
    weight_analysis.vehicle  = evtol_setup(update_regression_values) 
    for wing in weight_analysis.vehicle.wings: 
        wing_planform(wing,overwrite_reference =  True) 
        if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing):
            weight_analysis.vehicle.reference_area = wing.areas.reference 
    weight_analysis.method   = 'Physics_Based'
    weight_analysis.settings.safety_factor = 1.5    # CHECK THIS VALUE
    weight_analysis.settings.miscelleneous_weight_factor = 1.1 # CHECK THIS VALUE
    weight_analysis.settings.disk_area_factor = 1.15
    weight_analysis.settings.max_thrust_to_weight_ratio = 1.1
    weight_analysis.settings.max_g_load = 3.8
    weight                   = weight_analysis.evaluate()
    plot_weight_breakdown(weight_analysis.vehicle, show_figure = show_figure) 

    if update_regression_values:
        save_results(weight, os.path.join(os.path.dirname(__file__), 'weights_EVTOL.res'))
    old_weight = load_results(os.path.join(os.path.dirname(__file__), 'weights_EVTOL.res'))

    check_list = [
        'empty.total', 
        'empty.structural.total',
        'empty.propulsion.total',   
        'empty.systems.total',  
    ]

    # do the check
    for k in check_list:
        print(k)

        old_val = old_weight.deep_get(k)
        new_val = weight.deep_get(k)
        err = (new_val-old_val)/old_val
        print('Error:' , err)
        assert np.abs(err) < 1e-6 , 'Check Failed : %s' % k     

        print('')

    return

def Jet_engine():
    turbofan                                    = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan() 
    turbofan.tag                                = 'starboard_propulsor' 
    turbofan.origin                             = [[13.72, 4.86,-1.1]] 
    turbofan.length                             = 2.71     
    turbofan.bypass_ratio                       = 5.4    
    turbofan.design_altitude                    = 35000.0*Units.ft
    turbofan.design_mach_number                 = 0.78   
    turbofan.design_thrust                      = 35000.0* Units.N 

    # fan                
    fan                                         = RCAIDE.Library.Components.Powertrain.Converters.Fan()   
    fan.tag                                     = 'fan'
    fan.polytropic_efficiency                   = 0.93
    fan.pressure_ratio                          = 1.7   
    turbofan.fan                                = fan        

    # working fluid                   
    turbofan.working_fluid                      = RCAIDE.Library.Attributes.Gases.Air() 
    ram                                         = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                     = 'ram' 
    turbofan.ram                                = ram 

    # inlet nozzle          
    inlet_nozzle                                = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                            = 'inlet nozzle'
    inlet_nozzle.polytropic_efficiency          = 0.98
    inlet_nozzle.pressure_ratio                 = 0.98 
    turbofan.inlet_nozzle                       = inlet_nozzle 

    # low pressure compressor    
    low_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    low_pressure_compressor.tag                   = 'lpc'
    low_pressure_compressor.polytropic_efficiency = 0.91
    low_pressure_compressor.pressure_ratio        = 1.9   
    turbofan.low_pressure_compressor              = low_pressure_compressor

    # high pressure compressor  
    high_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    high_pressure_compressor.tag                   = 'hpc'
    high_pressure_compressor.polytropic_efficiency = 0.91
    high_pressure_compressor.pressure_ratio        = 10.0    
    turbofan.high_pressure_compressor              = high_pressure_compressor

    # low pressure turbine  
    low_pressure_turbine                           = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    low_pressure_turbine.tag                       ='lpt'
    low_pressure_turbine.mechanical_efficiency     = 0.99
    low_pressure_turbine.polytropic_efficiency     = 0.93 
    turbofan.low_pressure_turbine                  = low_pressure_turbine

    # high pressure turbine     
    high_pressure_turbine                          = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    high_pressure_turbine.tag                      ='hpt'
    high_pressure_turbine.mechanical_efficiency    = 0.99
    high_pressure_turbine.polytropic_efficiency    = 0.93 
    turbofan.high_pressure_turbine                 = high_pressure_turbine 

    # combustor  
    combustor                                      = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                                  = 'Comb'
    combustor.efficiency                           = 0.99 
    combustor.alphac                               = 1.0     
    combustor.turbine_inlet_temperature            = 1500
    combustor.pressure_ratio                       = 0.95
    combustor.fuel_data                            = RCAIDE.Library.Attributes.Propellants.Jet_A1()  
    turbofan.combustor                             = combustor

    # core nozzle
    core_nozzle                                    = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                = 'core nozzle'
    core_nozzle.polytropic_efficiency              = 0.95
    core_nozzle.pressure_ratio                     = 0.99  
    turbofan.core_nozzle                           = core_nozzle

    # fan nozzle             
    fan_nozzle                                     = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    fan_nozzle.tag                                 = 'fan nozzle'
    fan_nozzle.polytropic_efficiency               = 0.95
    fan_nozzle.pressure_ratio                      = 0.99 
    turbofan.fan_nozzle                            = fan_nozzle 

    # design turbofan
    design_turbofan(turbofan)  
    # append propulsor to distribution line  

    # Nacelle 
    nacelle                                     = RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle()
    nacelle.diameter                            = 2.05
    nacelle.length                              = 2.71
    nacelle.tag                                 = 'nacelle_1'
    nacelle.inlet_diameter                      = 2.0
    nacelle.origin                              = [[13.5,4.38,-1.5]] 
    nacelle.areas.wetted                        = 1.1*np.pi*nacelle.diameter*nacelle.length 
    nacelle_airfoil                             = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    nacelle_airfoil.NACA_4_Series_code          = '2410'
    nacelle.append_airfoil(nacelle_airfoil)  
    turbofan.nacelle                            = nacelle

    return(turbofan)

if __name__ == '__main__':
    main()

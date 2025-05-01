# test_automatic_regression.py
import pytest
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')

import sys, os, traceback, time

# Adjust this path as needed so Python can see "Tests" and "Vehicles" directories:
sys.path.append(os.path.join(sys.path[0], 'Vehicles'))
sys.path.append(os.path.join(sys.path[0], 'Vehicles', 'Rotors'))

modules = [ 
    # 'Verification/analysis_aerodynamics/airfoil_panel_method_test.py',    
    # 'Verification/analysis_aerodynamics/airfoil_panel_method_convergence.py',
    # 'Verification/analysis_aerodynamics/VLM_control_surface_test.py',    
    # 'Verification/analysis_aerodynamics/VLM_moving_surface_test.py',   
    # 'Verification/analysis_aerodynamics/AVL_test.py',     
    # 'Verification/atmosphere/atmosphere.py',
    # 'Verification/atmosphere/constant_temperature.py',
    # 'Verification/analysis_emissions/emissions_test.py',   
    # 'Verification/analysis_noise/digital_elevation_test.py',  
    # 'Verification/analysis_noise/frequency_domain_test.py', 
    # 'Verification/analysis_noise/empirical_jet_noise_test.py',    
    # 'Verification/analysis_stability/trimmed_flight_test.py', 
    # 'Verification/analysis_stability/untrimmed_flight_test.py', 
    'Verification/analysis_weights/operating_empty_weight_test.py',
    'Verification/analysis_weights/cg_and_moi_test.py',
    # 'Verification/energy_sources/battery_cell.py',
    # 'Verification/energy_sources/fuel_cell.py',
    # 'Verification/geometry/airfoil_import_test.py', 
    # 'Verification/geometry/airfoil_interpolation_test.py',    
    # 'Verification/geometry/fuselage_planform_compute.py',  
    # 'Verification/future_capability_coverage/coverage_test.py',    
    # 'Verification/mission_segments/transition_segment_test.py', 
    # 'Verification/network_electric/battery_electric_aircraft_test.py',
    'Verification/network_electric/electric_ducted_fan_aircraft_test.py',
    # 'Verification/network_fuel_cell/hydrogen_fuel_cell_aircraft_test.py', 
    # 'Verification/network_hybrid/hybrid_network_test.py', 
    #'Verification/network_turbofan/turbofan_network_test.py',
    'Verification/network_turbojet/turbojet_network_test.py',
    # 'Verification/network_turboprop/turboprop_network_test.py',
    # 'Verification/network_turboshaft/turboshaft_network_test.py',
    # 'Verification/network_internal_combustion_engine/ICE_test.py',
    #'Verification/network_internal_combustion_engine/ICE_constant_speed_test.py',
    # 'Verification/optimization/optimization_packages.py',
    # 'Verification/optimization/multifidelity_optimization.py',
    'Verification/performance/landing_field_length_test.py',
    'Verification/performance/payload_range_test.py',
    'Verification/performance/take_off_field_length_test.py',
    'Verification/performance/take_off_weight_from_tofl_test.py',
    # 'Verification/performance/aircraft_aerodynamics_test.py', 
    'Verification/performance/noise_certification_test.py', 
    'Verification/performance/V_n_diagram_test.py', 
    # 'Verification/propulsion/rotor_performance_test.py',  
    # 'Verification/propulsion/propeller_non_uniform_inflow.py',    
    # 'Verification/propulsion/propeller_wing_interaction_test.py', 
    # 'Verification/propulsion/generator_test.py',
    # 'Verification/propulsion/motor_test.py',
    # 'Verification/propulsion/reformer_test.py', 
    # 'Validation/converters/test_dc_motor_validation.py',
    # 'Validation/converters/test_pmsm_motor_validation.py',
    # 'Validation/converters/test_rotor_validation.py',
    # 'Validation/propulsors/test_turbofan_validation.py',
    #'Validation/aircraft_performance/test_b787_payload_rage.py'

]

def run_module_test(module_path):
    """
    Helper function to import the module, call module.main(), and
    return True (passed) or False (failed).
    """
    original_dir = os.getcwd()
    passed = False
    start_time = time.time()

    try:
        regression_dir = os.path.dirname(os.path.abspath(__file__))
        sys.path.append(regression_dir)

        full_module_path = os.path.join(regression_dir, module_path)
        test_dir = os.path.dirname(full_module_path)
        module_name = os.path.basename(module_path)

        print(f'# ---------------------------------------------------------------------')
        print(f'# Start Test: {full_module_path}')
        sys.stdout.flush()

        if not os.path.exists(full_module_path):
            raise FileNotFoundError(f'File {full_module_path} does not exist')

        # Change directory so the test can run as expected
        if test_dir:
            sys.path.append(test_dir)
            os.chdir(test_dir)

        name = os.path.splitext(module_name)[0]
        module = __import__(name)
        module.main()  # The test module must define a main()

        passed = True

    except Exception:
        sys.stderr.write('Test Failed:\n')
        sys.stderr.write(traceback.format_exc())
        sys.stderr.write('\n')
        sys.stderr.flush()

    finally:
        plt.close('all')
        os.chdir(original_dir)
        elapsed = (time.time() - start_time) / 60
        if passed:
            print(f'# Passed: {module_name}')
        else:
            print(f'# FAILED: {module_name}')
        print(f'# Test Duration: {elapsed:.4f} min\n')
        sys.stdout.flush()
        sys.stderr.flush()

    return passed

@pytest.mark.parametrize("module_path", modules)
def test_each_module(module_path):
    """
    This creates one pytest test for each item in 'modules'.
    We'll simply assert that 'run_module_test()' returns True.
    """
    result = run_module_test(module_path)
    assert result, f"Module {module_path} failed!"

# If you also want the old "all-or-nothing" approach, keep it here but remove sys.exit
def regressions():
    """
    (Optional) A single function to run all modules in one go, if you want it.
    But remove 'sys.exit(...)' so it doesn't kill the whole pytest run.
    """
    results = {}
    print('# ---------------------------------------------------------------------')
    print('#   RCAIDE-UIUC Automatic Verification and Validation')
    print('#   {}'.format(time.strftime("%B %d, %Y - %H:%M:%S", time.gmtime())))
    print('# ---------------------------------------------------------------------\n')

    all_pass = True
    for module_path in modules:
        passed = run_module_test(module_path)
        results[module_path] = passed
        if not passed:
            all_pass = False
    
    # Final report
    print('# ---------------------------------------------------------------------')
    print('Final Results')
    for m, passed in results.items():
        print('Passed - ' + m if passed else 'FAILED - ' + m)

    return all_pass

if __name__ == '__main__':
    # If you run directly: use your single function approach (no sys.exit).
    all_passed = regressions()
    if all_passed:
        sys.exit(0)
    else:
        sys.exit(1)

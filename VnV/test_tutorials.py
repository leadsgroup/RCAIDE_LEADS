# VnV/test_tutorials.py

import os
os.environ['PYVISTA_OFF_SCREEN'] = 'true'
os.environ['RCAIDE_TUTORIAL_TEST_MODE'] = '1'

import pytest
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pyvista as pv
import RCAIDE.Library.Plots as rcaide_plots
pv.OFF_SCREEN = True
pv.Plotter.show = lambda self, *args, **kwargs: None


def _noop_plot(*args, **kwargs):
    return None


rcaide_plots.plot_pressure_coefficient_distribution = _noop_plot
rcaide_plots.plot_3d_vehicle_vlm_panelization = _noop_plot

import sys, traceback, time

# Each path is relative to this file's directory (VnV/).
# All Tutorial files define a main() function.
modules = [
    '../Tutorials/Analysis_Aerodynamic_Polars/aircraft_aerodynamics_analysis.py',
    '../Tutorials/Analysis_Approach_Sideline_Takeoff_Noise/approach_landing_takeoff_noise_certification.py',
    '../Tutorials/Analysis_Landing_Field_Length/landing_field_length.py',
    #'../Tutorials/Analysis_Loading_and_Trim_Diagram/aircraft_loading_and_trim_diagram.py',#- DO NOT RUN ON GITHUB, TAKES TOO LONG
    #'../Tutorials/Analysis_Payload_Range/aircraft_payload_range_diagram.py',#-DO NOT RUN ON GITHUB, TAKES TOO LONG
    '../Tutorials/Analysis_Take_Off_Field_Length/takeoff_field_length_estimation.py',
    '../Tutorials/Analysis_Take_Off_Field_Length/takeoff_weight_estimation_from_target_TOFL.py',
    '../Tutorials/Analysis_V_n_Diagram/Part_23_V_n_diagram.py',
    '../Tutorials/Analysis_V_n_Diagram/Part_35_V_n_diagram.py',
    #'../Tutorials/Optimization_Wing_Planform_Fuel_Burn/Optimize.py',# DO NOT RUN ON GITHUB, TAKES TOO LONG
    '../Tutorials/Simulation_Turbofan_Transonic_Aircraft_5000nmi_Mission/Boeing_737_800.py',
    '../Tutorials/Simulation_Turbojet_Supersonic_Aircraft_4500nmi_Mission/Concorde.py',
    '../Tutorials/Simulation_Turboprop_Aircraft_1000nmi_Mission/ATR_72.py',
]


def run_module_test(module_path):
    original_dir = os.getcwd()
    passed = False
    start_time = time.time()

    try:
        regression_dir = os.path.dirname(os.path.abspath(__file__))
        full_module_path = os.path.normpath(os.path.join(regression_dir, module_path))
        test_dir = os.path.dirname(full_module_path)
        module_name = os.path.basename(module_path)

        print(f'# ---------------------------------------------------------------------')
        print(f'# Start Test: {full_module_path}')
        sys.stdout.flush()

        if not os.path.exists(full_module_path):
            raise FileNotFoundError(f'File not found: {full_module_path}')

        if test_dir not in sys.path:
            sys.path.insert(0, test_dir)
        os.chdir(test_dir)

        name = os.path.splitext(module_name)[0]
        if name in sys.modules:
            del sys.modules[name]

        module = __import__(name)
        module.main()

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
        status = 'Passed' if passed else 'FAILED'
        print(f'# {status}: {module_name}')
        print(f'# Test Duration: {elapsed:.4f} min\n')
        sys.stdout.flush()

    return passed


@pytest.mark.parametrize("module_path", modules)
def test_each_tutorial(module_path):
    assert run_module_test(module_path), f"Tutorial {module_path} failed!"


if __name__ == '__main__':
    results = {}
    all_pass = True

    print('# ---------------------------------------------------------------------')
    print('#   RCAIDE Tutorial Regression')
    print('#   {}'.format(time.strftime("%B %d, %Y - %H:%M:%S", time.gmtime())))
    print('# ---------------------------------------------------------------------\n')

    for module_path in modules:
        passed = run_module_test(module_path)
        results[module_path] = passed
        if not passed:
            all_pass = False

    print('# ---------------------------------------------------------------------')
    print('Final Results')
    for m, passed in results.items():
        print(('Passed - ' if passed else 'FAILED - ') + m)

    sys.exit(0 if all_pass else 1)

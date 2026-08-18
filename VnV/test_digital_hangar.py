# VnV/test_digital_hangar.py

import os
os.environ['PYVISTA_OFF_SCREEN'] = 'true'

import pytest
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pyvista as pv
pv.OFF_SCREEN = True
pv.Plotter.show = lambda self, *args, **kwargs: None

import sys, traceback, time

# Each path is relative to this file's directory (VnV/).
# All Digital Hangar files define a main() function.
modules = [
    '../Digital_Hangar/ATR_72/ATR_72.py',
    '../Digital_Hangar/ATR_72_All_Electric/ATR_72_all_electric.py',
    '../Digital_Hangar/ATR_72_Parallel_Hybrid/ATR_72_cryo_parallel_hybrid_electric.py',
    '../Digital_Hangar/Airbus_A220/Airbus_A220_100.py',
    '../Digital_Hangar/Airbus_A320/Airbus_A320.py',
    '../Digital_Hangar/Airbus_Vahana/Vahana.py',
    '../Digital_Hangar/Blended_Wing_Body/BWB.py',
    '../Digital_Hangar/Blended_Wing_Body_Hydrogen/BWB_Hydrogen.py',
    '../Digital_Hangar/Boeing_737_800/Boeing_737_800.py',
    '../Digital_Hangar/Boeing_747_100/Boeing_747_100.py',
    '../Digital_Hangar/Boeing_777/Boeing_777_200er.py',
    '../Digital_Hangar/Boeing_787/Boeing_787.py',
    '../Digital_Hangar/Canadair_Regional_Jet_CRJ_700/CRJ_700.py',
    '../Digital_Hangar/Cessna_172/Cessna_172.py',
    '../Digital_Hangar/Concorde/Concorde.py',
    '../Digital_Hangar/De_Havilland_Canada_DHC_6/Twin_Otter.py',
    '../Digital_Hangar/De_Havilland_Canada_DHC_6_Electric/Electric_Twin_Otter.py',
    '../Digital_Hangar/De_Havilland_Canada_DHC_8/DHC8_100.py',
    '../Digital_Hangar/Elysian_E9X/Elysian_E9X_Model.py',
    '../Digital_Hangar/Embraer_E190/Embraer_190.py',
    '../Digital_Hangar/Lockheed_C5a/Lockheed_C5a.py',
    '../Digital_Hangar/Lockheed_F22/Lockheed_F22.py',
    '../Digital_Hangar/Lockheed_F35C/Lockheed_F35C.py',
    '../Digital_Hangar/NASA_X_57_Modification_2/X57_Maxwell_M2.py',
    '../Digital_Hangar/Ryan_Navion/Navion.py',
    '../Digital_Hangar/Stopped_Rotor_EVTOL/Stopped_Rotor.py',
    '../Digital_Hangar/Tecnam_P2012/Tecnam_P2012.py',
    '../Digital_Hangar/Tilt_Stopped_Rotor_V_tail_EVTOL/Tilt_Stopped_Rotor_V_Tail.py',
    '../Digital_Hangar/Tiltrotor_EVTOL/Tiltrotor.py',
    '../Digital_Hangar/Tiltwing_EVTOL/Tiltwing.py',
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
def test_each_hangar_model(module_path):
    assert run_module_test(module_path), f"Digital Hangar model {module_path} failed!"


if __name__ == '__main__':
    results = {}
    all_pass = True

    print('# ---------------------------------------------------------------------')
    print('#   RCAIDE Digital Hangar Regression')
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

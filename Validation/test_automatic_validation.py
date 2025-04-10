# test_automatic_regression.py
import pytest
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')

import sys, os, traceback, time


modules = [ 
   
    'Validation_tests/Propulsors/turbofan_validation.py',
    'Validation_tests/Converters/PMSM_motor_validation.py',
    'Validation_tests/Converters/DC_motor_validation.py',
    'Validation_tests/Converters/CRN_combustor_validation.py',
    'Validation_tests/Converters/rotor_validation.py',
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
    print('#   RCAIDE-UIUC Automatic Validation')
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

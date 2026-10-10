# VnV/Verification/analysis_aerodynamics/test_vortex_lattice_method.py
#
# Created: Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import importlib
import pickle

import RCAIDE

# the package __init__ shadows the module name with the class, so fetch the module itself for monkeypatching
vlm_module = importlib.import_module('RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method')

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def test_supersonic_training_is_stored(monkeypatch, tmp_path):
    monkeypatch.setattr(vlm_module, 'train_VLM_supersonic_surrogates', lambda aerodynamics, vehicle: setattr(aerodynamics.training, 'Mach', [1.2]))
    monkeypatch.setattr(vlm_module, 'build_VLM_surrogates', lambda aerodynamics, vehicle: None)

    aerodynamics                              = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.settings.store_training_data = True
    aerodynamics.filename                     = str(tmp_path / 'training.pkl')
    aerodynamics.training_vehicle             = RCAIDE.Vehicle()
    aerodynamics.train_supersonic_surrogates()

    with open(aerodynamics.filename, 'rb') as file:
        assert pickle.load(file).Mach == [1.2]

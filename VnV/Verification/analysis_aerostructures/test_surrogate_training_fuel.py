# VnV/Verification/analysis_aerostructures/test_surrogate_training_fuel.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from copy import deepcopy
import numpy as np
import sys
import os

from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.train_VLM_surrogates import full_fuel_masses, control_surface_structural_conditions

base_dir      = os.path.dirname(os.path.abspath(__file__))
vehicles_path = os.path.abspath(os.path.join(base_dir, "..", "..", "Vehicles"))
if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)

from Boeing_737 import vehicle_setup

# fuel and empty tank masses [kg] given to the two wing tanks; different so a mix-up shows
FUEL_MASSES       = {'integral_tank': 5000.0, 'integral_tank2': 3000.0}
EMPTY_TANK_MASSES = {'integral_tank':  300.0, 'integral_tank2':  200.0}

# ----------------------------------------------------------------------------------------------------------------------
#  Main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    check_training_uses_fuel_mass()
    check_control_surface_training_uses_fuel_mass()
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def build_vehicle():
    """Boeing 737 whose two wing tanks each carry their own fuel, with known fuel and empty tank masses."""
    vehicle = vehicle_setup()
    for network in vehicle.networks:
        for source in network.sources:
            if source.tag in FUEL_MASSES:
                # give each tank its own fuel object and tag, as the training keys fuel by tag
                source.fuel                           = deepcopy(source.fuel)
                source.fuel.tag                       = source.tag + '_fuel'
                source.fuel.mass_properties.mass      = FUEL_MASSES[source.tag]
                source.mass_properties.mass           = EMPTY_TANK_MASSES[source.tag]
    return vehicle

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def check_training_uses_fuel_mass():
    # the training conditions must carry the fuel, as the mission does, not the empty tank
    vehicle     = build_vehicle()
    fuel_masses = full_fuel_masses(vehicle, 4)
    for tank_tag, fuel_mass in FUEL_MASSES.items():
        values = fuel_masses[tank_tag + '_fuel']
        assert values.shape == (4, 1), values.shape
        assert np.all(values == fuel_mass), (tank_tag, values[0, 0], fuel_mass)
    return


def check_control_surface_training_uses_fuel_mass():
    vehicle    = build_vehicle()
    Mach       = np.array([0.3, 0.5, 0.7])
    conditions = control_surface_structural_conditions(Mach, 340.0, 1.225, vehicle)
    for tank_tag, fuel_mass in FUEL_MASSES.items():
        values = conditions.weights.components.mass[tank_tag + '_fuel']
        assert values.shape == (3, 1), values.shape
        assert np.all(values == fuel_mass), (tank_tag, values[0, 0], fuel_mass)
    return


if __name__ == '__main__':
    main()
    print('test_surrogate_training_fuel passed')

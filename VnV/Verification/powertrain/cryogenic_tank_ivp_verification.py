"""Standalone entry point for cryogenic tank IVP verification.

Tank trajectories are now integrated inside the production performance model,
not exposed as mission solver unknowns. The maintained performance verification
checks mass conservation and compares the production solve against a tighter,
independent Radau integration.
"""

import RCAIDE

from cryogenic_tank_performance_test import run_case


def main():
    liquid_hydrogen = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    run_case(
        liquid_hydrogen,
        design_inlet_temperature=20.0,
        chemical_power_demand=1.0e3,
        label="LH2 IVP verification",
        diameter=2.6,
        length=4.5,
        duration_hr=1.0,
        check_truth=True,
    )


if __name__ == "__main__":
    main()

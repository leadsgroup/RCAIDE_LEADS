# RCAIDE/Library/Methods/Powertrain/Sources/Fuel_Tanks/Cryogenic_Tank/append_cryogenic_tank_unknown_and_residual.py
#
# Created: Jun 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ----------------------------------------------------------------------------------------------------------------------
def append_cryogenic_tank_unknown_and_residual(tank, segment):
    """
    No-op. The tank's 6-state boil-off trajectory (ullage/liquid mass,
    temperature, volume) is no longer solved as coupled mission-level Newton
    unknowns/residuals -- it's solved as its own internal, decoupled IVP
    (adaptive Radau, warm-started the same way the isolated validation
    scripts already did) inside ``compute_cryogenic_tank_performance``,
    driven by the mission's own current-iterate inputs and re-solved fresh
    on every outer network iterate. That removes the 24 extra coupled
    unknowns (6 states x 4 tanks on the Hydrogen_BWB) that were preventing
    the full aircraft-level mission from converging as a single simultaneous
    Newton solve, even though every individual tank's physics checked out
    cleanly in isolation (VnV/Verification/powertrain/
    cryogenic_tank_performance_test.py, RESEARCH/.../
    cryogenic_tank_boil_off_validation.py both passed).
    """
    return

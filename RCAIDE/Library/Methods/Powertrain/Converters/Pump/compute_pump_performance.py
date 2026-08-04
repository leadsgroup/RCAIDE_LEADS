# RCAIDE/Library/Methods/Powertrain/Converters/Pump/compute_pump_performance.py
#
#
# Created:  Sep. 2025, M. Guidotti

def compute_pump_performance(pump, state, network=None):
    """
    Generic electrically-driven pump performance is not implemented.

    The base `Pump` class only provides shared attributes (e.g.
    `casting_and_mount_factor`) and condition bookkeeping
    (`append_pump_conditions`) for its subclasses. For an actual working
    performance model, use a concrete subclass instead -- e.g.
    `RCAIDE.Library.Components.Powertrain.Converters.Cryogenic_Pump`, whose
    `compute_performance` reads the required shaft power directly from its
    fuel line's already-computed distribution losses (see
    RCAIDE.Library.Methods.Powertrain.Converters.Cryogenic_Pump.
    compute_cryogenic_pump_performance).
    """
    raise NotImplementedError(
        "Pump.compute_performance() has no generic implementation. "
        "Use a concrete subclass such as Cryogenic_Pump, or implement "
        "compute_pump_performance() for this pump's use case."
    )

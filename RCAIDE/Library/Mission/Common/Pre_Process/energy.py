# RCAIDE/Library/Missions/Common/Pre_Process/energy.py
#
#
# Created:  Jul 2023, M. Clarke
# Modified: Jun 2026, M. Clarke
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Data

# ----------------------------------------------------------------------------------------------------------------------
#  energy
# ----------------------------------------------------------------------------------------------------------------------
def energy(mission):
    """Pre-processes the energy network for each mission segment.

    This function runs once before any simulation begins. It performs two roles:

    1. **Topology analysis and hybridization resolution** — inspects the network
       to classify every component by energy domain (chemical, electrical, etc.),
       maps the provider/consumer relationships on each distributor, and determines
       the power split ratios (phi and psi) that control how propulsive power is
       sourced.

       If the user has set ``segment.hybrid_power_split_ratio`` (phi) or
       ``segment.battery_fuel_cell_power_split_ratio`` (psi) on the segment, those
       values are used directly. Otherwise the function auto-derives them:

       - Pure fuel network (no electrical path)     →  phi = 0.0
       - All-electric network (no chemical path)    →  phi = 1.0
       - Hybrid with user-specified phi             →  honored as-is
       - Only batteries as electrical sources             →  psi = 1.0
       - Only fuel cells/generators as electrical sources →  psi = 0.0
       - Multiple same-type sources                 →  power split evenly
       - Multiple different-type electrical sources →  user must specify psi

    2. **Component initialization** — iterates every component container
       (propulsors, converters, modulators, sources, systems, distributors) and
       calls ``initialize`` / ``append_operating_conditions`` so that each
       component's unknowns, residuals, and operating-condition arrays are
       registered on the segment before the solver begins.

    Parameters
    ----------
    mission : Mission
        The mission containing all segments to pre-process.

    Notes
    -----
    The resolved phi and psi values are written into
    ``segment.state.conditions.energy.hybrid_power_split_ratio`` and
    ``segment.state.conditions.energy.battery_fuel_cell_power_split_ratio``
    as arrays (one value per discretization point).

    Notes
    -----
    Components are initialized sources-before-distributors, since
    ``Electrical_Bus.initialize()`` reads a source's ``voltage``/
    ``maximum_power`` (e.g. ``Battery_Pack``) to size itself. Operating
    conditions are appended distributors-before-sources instead, since a
    source's own conditions can reach into a distributor's conditions while
    being built (e.g. a battery module's coolant-line conditions).

    See Also
    --------
    RCAIDE.Framework.Mission.Segments.Evaluate
        Where phi and psi are set by the user per segment.
    RCAIDE.Framework.Networks.Network
        Network class whose topology is analyzed here.
    RCAIDE.Library.Mission.Common.Initialize.energy
        Runs after pre-processing to allocate result data structures.
    """

    for seg_i ,segment in enumerate(mission.segments):
        verbose = segment.analyses.energy.verbose
        for network in segment.analyses.vehicle.networks: 

            # ----------------------------------------------------------------
            # Analyze network topology and resolve hybridization
            # ----------------------------------------------------------------
            topology = analyze_topology(network,seg_i,verbose=verbose)
            phi, psi_by_distributor = resolve_hybridization(segment, topology, seg_i,verbose=verbose)

            psi_conditions = Data()
            for dist_tag, psi_value in psi_by_distributor.items():
                psi_conditions[dist_tag] = psi_value * segment.state.ones_row(1)

            segment.state.conditions.energy.hybrid_power_split_ratio            = phi * segment.state.ones_row(1)
            segment.state.conditions.energy.battery_fuel_cell_power_split_ratio = psi_conditions
            segment.state.conditions.energy.topology                            = topology

            # ----------------------------------------------------------------
            # Initialize components
            # ----------------------------------------------------------------
            # Sources before distributors: Electrical_Bus.initialize() reads
            # source.voltage/maximum_power (e.g. Battery_Pack) to size itself, so
            # those must already be computed by the time a distributor initializes.
            for propulsor in network.propulsors:
                propulsor.initialize(network)

            for converter in network.converters:
                converter.initialize(network)

            for modulator in network.modulators:
                modulator.initialize(network)

            for source in network.sources:
                source.initialize(network)

            for distributor in network.distributors:
                distributor.initialize(network)

            for system in network.systems:
                system.initialize(network)

            # ----------------------------------------------------------------
            # Append operating conditions
            # ----------------------------------------------------------------
            # Distributors before sources here: a source's own conditions can
            # reach into a distributor's conditions while being built (e.g. a
            # battery module's coolant-line heat-acquisition-system conditions),
            # so the distributor side must already exist.
            for propulsor in network.propulsors:
                propulsor.append_operating_conditions(segment)

            for converter in network.converters:
                converter.append_operating_conditions(segment)

            for modulator in network.modulators:
                modulator.append_operating_conditions(segment)

            for distributor in network.distributors:
                distributor.append_operating_conditions(segment)

            for source in network.sources:
                source.append_operating_conditions(segment)

            for system in network.systems:
                system.append_operating_conditions(segment)

    return


# ==============================================================================
#  Topology Analysis
# ==============================================================================

def analyze_topology(network, seg_i,  verbose=False):
    """Classify all network components by energy domain and map distributor connections.

    Walks the network in five passes:

    1. **Distributors** — reads each distributor's ``domain`` attribute
       (``'electrical'``, ``'chemical'``, etc.) to build a domain lookup table.
    2. **Propulsors** — classified by their self-declared ``domain`` (chemical
       or electrical). A plain propulsor is an energy *consumer* on its
       assigned distributors; one with an integrated drive generator is also
       a *provider* on the distributor that generator feeds.
    3. **Sources** — classified as electrical (Battery_Pack) or chemical
       (Fuel_Tank, by self-declared ``domain``). Sources are energy
       *providers* on their assigned distributors.
    4. **Converters** — classified by their self-declared ``provides_domain``:
       generators and fuel-cell stacks are electrical *providers*; motors
       (DC_Motor, PMSM_Motor) are electrical *consumers*. Classification also
       picks up converters that are sub-components of propulsors (e.g. an
       integrated drive generator) since those are not in ``network.converters``.
    5. **Systems** — all systems (avionics, environmental controls, etc.) are
       energy *consumers* on their assigned distributors.

    Parameters
    ----------
    network : Network
        The energy network to analyze.

    Returns
    -------
    Data
        Topology data structure containing:

        - ``distributor_domains``     : dict  — {distributor_tag: domain_string}
        - ``has_chemical_path``       : bool  — True if any chemical distributor exists
        - ``has_electrical_path``     : bool  — True if any electrical distributor exists
        - ``chemical_propulsors``     : list  — propulsors on chemical distributors
        - ``electrical_propulsors``   : list  — propulsors on electrical distributors
        - ``batteries``               : list  — Battery_Pack sources
        - ``fuel_tanks``              : list  — Fuel_Tank sources
        - ``fuel_cells``              : list  — fuel-cell converters (Generic_Fuel_Cell_Stack and subclasses)
        - ``generators``              : list  — Generator / Turboelectric_Generator converters
        - ``electrical_consumers``    : list  — motors and systems on electrical distributors
        - ``distributor_connections`` : dict  — {distributor_tag: {providers: [...], consumers: [...]}}

    Notes
    -----
    Propulsors and sources self-declare their ``domain`` (see
    ``Propulsor.__defaults__`` / ``Source.__defaults__``); providing
    converters self-declare ``provides_domain`` (see
    ``Converter.__defaults__``). This lets a new subclass work without
    updating this function. Two classifications stay ``isinstance``-based on
    purpose rather than domain: ``batteries`` (battery vs. other electrical
    provider is psi's own definition, not a narrowing proxy) and
    ``electrical_motors`` (specifically "motor providing propulsive assist"
    for phi, narrower than "any non-providing converter" -- a motor driving
    a pump should not count as electrical propulsion). A propulsor with an
    integrated drive generator (e.g. a turbofan's IDG) is a *provider* on
    the distributor its IDG feeds and a *consumer* on any other (e.g. the
    fuel line it draws from) -- unlike a plain propulsor, which is always a
    pure consumer.
    """

    topology = Data()

    # ------------------------------------------------------------------
    # 1. Build distributor domain map
    # ------------------------------------------------------------------
    distributor_domains = {}
    for distributor in network.distributors:
        distributor_domains[distributor.tag] = getattr(distributor, 'domain', 'unknown')

    topology.distributor_domains = distributor_domains
    topology.has_chemical_path   = any(d == 'chemical'   for d in distributor_domains.values())
    topology.has_electrical_path = any(d == 'electrical'  for d in distributor_domains.values())

    # ------------------------------------------------------------------
    # 2. Classify propulsors
    # ------------------------------------------------------------------
    chemical_propulsors   = [p for p in network.propulsors if p.domain == 'chemical']
    electrical_propulsors = [p for p in network.propulsors if p.domain == 'electrical']

    topology.chemical_propulsors   = chemical_propulsors
    topology.electrical_propulsors = electrical_propulsors

    # ------------------------------------------------------------------
    # 3. Classify sources
    # ------------------------------------------------------------------
    batteries  = [s for s in network.sources if isinstance(s, RCAIDE.Library.Components.Powertrain.Sources.Batteries.Battery_Pack)]
    fuel_tanks = [s for s in network.sources if s.domain == 'chemical']

    topology.batteries  = batteries
    topology.fuel_tanks = fuel_tanks

    # ------------------------------------------------------------------
    # 4. Classify converters (fuel cells, generators, motors), plus
    #    propulsor sub-components (integrated drive generators and
    #    motors), since those are not in network.converters.
    # ------------------------------------------------------------------
    fuel_cells          = []
    generators          = []
    electrical_motors   = []

    for converter in network.converters:
        if converter.provides_domain == 'electrical':
            if isinstance(converter, RCAIDE.Library.Components.Powertrain.Converters.Generic_Fuel_Cell_Stack):
                fuel_cells.append(converter)
            else:
                generators.append(converter)
        elif isinstance(converter, (RCAIDE.Library.Components.Powertrain.Converters.DC_Motor,
                                    RCAIDE.Library.Components.Powertrain.Converters.PMSM_Motor)):
            electrical_motors.append(converter)

    for propulsor in network.propulsors:
        idg = getattr(propulsor, 'integrated_drive_generator', None)
        if idg is not None:
            generators.append(idg)
        idm = getattr(propulsor, 'integrated_drive_motor', None)
        if idm is not None:
            electrical_motors.append(idm)

    topology.fuel_cells        = fuel_cells
    topology.generators        = generators
    topology.electrical_motors = electrical_motors

    # ------------------------------------------------------------------
    # 5. Build per-distributor provider / consumer map
    # ------------------------------------------------------------------
    connections = {}
    for dist_tag in distributor_domains:
        connections[dist_tag] = Data(providers=[], consumers=[])

    def register(component, role):
        if component.assigned_distributors is not None:
            for dist_tag in component.assigned_distributors[0]:
                if dist_tag in connections:
                    if role == 'provider':
                        connections[dist_tag].providers.append(component)
                    else:
                        connections[dist_tag].consumers.append(component)

    def register_by_domain(component, provider_domain):
        """Provider on distributors matching provider_domain, consumer on the rest."""
        if component.assigned_distributors is not None:
            for dist_tag in component.assigned_distributors[0]:
                if dist_tag in connections:
                    if distributor_domains[dist_tag] == provider_domain:
                        connections[dist_tag].providers.append(component)
                    else:
                        connections[dist_tag].consumers.append(component)

    def register_consumer_by_domain(component, consumer_domain):
        """Consumer only on distributors matching consumer_domain, ignored elsewhere --
        e.g. a pump is physically placed along a fuel line (for sizing/lookup purposes)
        but only electrically consumes from the bus that powers it."""
        if component.assigned_distributors is not None:
            for dist_tag in component.assigned_distributors[0]:
                if dist_tag in connections and distributor_domains[dist_tag] == consumer_domain:
                    connections[dist_tag].consumers.append(component)

    for source in network.sources:
        register(source, 'provider')

    for converter in fuel_cells + generators:
        register_by_domain(converter, converter.provides_domain)

    for propulsor in network.propulsors:
        idg = getattr(propulsor, 'integrated_drive_generator', None)
        if idg is not None:
            register_by_domain(propulsor, idg.provides_domain)
        else:
            register(propulsor, 'consumer')

    for converter in electrical_motors:
        register(converter, 'consumer')

    # Any other converter (e.g. a fuel-line transfer pump) that isn't a power
    # provider draws electrical power to operate without providing anything
    # itself -- register it as an electrical consumer. Using
    # register_consumer_by_domain (not register) so a pump's fuel-line
    # membership -- present purely for physical/sizing purposes -- doesn't
    # get misread as it being a chemical consumer of that fuel line.
    other_converters = [c for c in network.converters
                         if c not in fuel_cells and c not in generators and c not in electrical_motors]
    for converter in other_converters:
        register_consumer_by_domain(converter, 'electrical')

    for system in network.systems:
        register(system, 'consumer')

    topology.distributor_connections = connections

    # ------------------------------------------------------------------
    # Verbose output
    # ------------------------------------------------------------------
    if verbose and seg_i == 0:  # only print for first segment to avoid clutter
        print('\n  Energy Network Topology Analysis')
        print('  ' + '-' * 50)
        print(f'  Chemical propulsors:   {[p.tag for p in chemical_propulsors]}')
        print(f'  Electrical propulsors: {[p.tag for p in electrical_propulsors]}')
        print(f'  Fuel tanks:            {[s.tag for s in fuel_tanks]}')
        print(f'  Batteries:             {[s.tag for s in batteries]}')
        print(f'  Fuel cells:            {[c.tag for c in fuel_cells]}')
        print(f'  Generators:            {[c.tag for c in generators]}')
        print(f'  Electrical motors:     {[c.tag for c in electrical_motors]}')
        print(f'  Has chemical path:     {topology.has_chemical_path}')
        print(f'  Has electrical path:   {topology.has_electrical_path}')
        for dist_tag, conn in connections.items():
            domain = distributor_domains[dist_tag]
            providers = [p.tag for p in conn.providers]
            consumers = [c.tag for c in conn.consumers]
            has_flow  = len(providers) > 0 and len(consumers) > 0
            print(f'  Distributor: {dist_tag} ({domain})')
            print(f'    Providers: {providers}')
            print(f'    Consumers: {consumers}')
            if not has_flow:
                if len(providers) == 0 and len(consumers) > 0:
                    print(f'    WARNING: consumers but no providers on this distributor')
                elif len(consumers) == 0 and len(providers) > 0:
                    print(f'    WARNING: providers but no consumers on this distributor')

        # add space between next print out 
        print('\n \n') 

    return topology


# ==============================================================================
#  Hybridization Resolution
# ==============================================================================

def resolve_hybridization(segment, topology,seg_i, verbose=False):
    """Determine phi and psi from user input or network topology.

    Resolution logic:

    **Phi** (``hybrid_power_split_ratio`` — fuel vs electrical split):

    - If the user set phi on the segment, use it directly.
    - If only chemical distributors exist (no electrical path), phi = 0.0.
    - If only electrical distributors exist (no chemical path), phi = 1.0.
    - If both paths exist (hybrid), the user must specify phi on the segment.
      A default of 0.0 is used with a warning if omitted.
    - Electrical propulsion includes both dedicated propulsors
      (Electric_Rotor, Electric_Ducted_Fan) and a motor assisting a
      chemical propulsor's own shaft (e.g. Turbofan.integrated_drive_motor).

    **Psi** (``battery_fuel_cell_power_split_ratio`` — battery vs other-source split):

    - Resolved per electrical distributor from that distributor's own
      providers (``topology.distributor_connections``), not vehicle-wide
      existence counts -- a single vehicle-wide value cannot correctly
      represent two buses with opposite compositions (e.g. a generator-only
      bus needing psi=0 and an isolated battery-only bus needing psi=1).
    - If the user set psi on the segment, it applies uniformly to every
      electrical distributor.
    - "Other source" means a fuel cell or generator (e.g. Turboelectric_Generator) —
      anything that dispatches off ``(1 - psi)`` of demand, same as a fuel cell.
    - If only batteries provide on a distributor, psi = 1.0.
    - If only fuel cells/generators provide on a distributor, psi = 0.0.
    - If both provide on the same distributor, the user must specify psi.
      A default of 1.0 is used with a warning if omitted.

    **Source power splitting**: when multiple sources of the same type feed the
    same distributor, each source's ``power_split_ratio`` is set to 1/N so power
    is divided evenly.

    Parameters
    ----------
    segment : Segment
        The mission segment (carries user-specified phi/psi or None).
    topology : Data
        Output of ``analyze_topology``.

    Returns
    -------
    phi : float
        Resolved hybrid power split ratio.
    psi_by_distributor : dict
        Resolved battery / fuel-cell power split ratio, keyed by electrical
        distributor tag -- a single vehicle-wide value cannot correctly
        represent multiple electrically-isolated buses with different
        provider compositions.
    """

    # ------------------------------------------------------------------
    # Resolve phi
    # ------------------------------------------------------------------
    phi = segment.hybrid_power_split_ratio

    has_chemical_propulsion = len(topology.chemical_propulsors) > 0
    has_electrical_propulsion = (len(topology.electrical_propulsors) > 0 or
                                  len(topology.electrical_motors)    > 0)

    if phi is None:
        if has_chemical_propulsion and not has_electrical_propulsion:
            phi = 0.0
        elif has_electrical_propulsion and not has_chemical_propulsion:
            phi = 1.0
        elif has_chemical_propulsion and has_electrical_propulsion:
            has_electrical_source = (len(topology.batteries)  > 0 or
                                    len(topology.fuel_cells)  > 0 or
                                    len(topology.generators)  > 0)
            if has_electrical_source:
                import warnings
                if verbose and seg_i == 0:
                    warnings.warn(
                        "Hybrid network detected (both chemical and electrical paths) "
                        "but segment.hybrid_power_split_ratio (phi) is not set. "
                        "Defaulting to phi = 0.0 (all fuel). Set phi on the segment "
                        "or it will be registered as an optimization variable.",
                        stacklevel=4)
            phi = 0.0
        else:
            phi = 0.0

    # ------------------------------------------------------------------
    # Resolve psi -- per electrical distributor (see docstring).
    # ------------------------------------------------------------------
    user_psi            = segment.battery_fuel_cell_power_split_ratio
    psi_by_distributor  = {}

    for dist_tag, domain in topology.distributor_domains.items():
        if domain != 'electrical':
            continue

        if user_psi is not None:
            psi_by_distributor[dist_tag] = user_psi
            continue

        providers         = topology.distributor_connections[dist_tag].providers
        battery_providers = [p for p in providers if isinstance(p, RCAIDE.Library.Components.Powertrain.Sources.Batteries.Battery_Pack)]
        other_providers   = [p for p in providers if p not in battery_providers]
        has_battery       = len(battery_providers) > 0
        has_other_source  = len(other_providers)   > 0

        if has_battery and not has_other_source:
            psi_by_distributor[dist_tag] = 1.0
        elif has_other_source and not has_battery:
            psi_by_distributor[dist_tag] = 0.0
        elif has_battery and has_other_source:
            import warnings
            if verbose and seg_i == 0:
                warnings.warn(
                    f"Distributor '{dist_tag}' has both batteries and other electrical "
                    f"providers (fuel cells and/or generators) but "
                    f"segment.battery_fuel_cell_power_split_ratio (psi) is not set. "
                    f"Defaulting to psi = 1.0 (all battery) for this distributor. Set psi "
                    f"on the segment or it will be registered as an optimization variable.",
                    stacklevel=4)
            psi_by_distributor[dist_tag] = 1.0
        else:
            psi_by_distributor[dist_tag] = 1.0

    # ------------------------------------------------------------------
    # Auto-set power_split_ratio for same-type sources on a distributor
    # ------------------------------------------------------------------
    for dist_tag, conn in topology.distributor_connections.items():
        providers = conn.providers
        if len(providers) > 1:
            source_types = {}
            for provider in providers:
                key = type(provider).__name__
                source_types.setdefault(key, []).append(provider)

            for type_name, sources in source_types.items():
                if len(sources) > 1:
                    for source in sources:
                        source.power_split_ratio = 1.0 / len(sources)

    if verbose and seg_i == 0:
        # print segment name
        print(f'  Segment {seg_i + 1}: {segment.tag}')
        print(f'  Resolved phi = {phi}  (fuel/electric split)')
        for dist_tag, psi_val in psi_by_distributor.items():
            print(f'  Resolved psi = {psi_val}  (battery/fuel-cell split, distributor: {dist_tag})')
        print('  ' + '-' * 50)

    return phi, psi_by_distributor

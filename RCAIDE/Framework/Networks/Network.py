# RCAIDE/Framework/Networks/Network.py 
#
# Created:  Mar 2025, M.Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  Imports
# ---------------------------------------------------------------------------------------------------------------------
# RCAIDE Imports
import  RCAIDE 
from RCAIDE.Framework.Mission.Common                      import Residuals 
from RCAIDE.Library.Mission.Common.Unpack_Unknowns.energy import unknowns
from RCAIDE.Library.Methods.Powertrain.Systems.compute_systems_power_draw                 import compute_systems_power_draw
from RCAIDE.Library.Methods.Powertrain.Converters.Motor.compute_motor_performance         import *
from RCAIDE.Library.Methods.Powertrain.Converters.Generator.compute_generator_performance import * 
from RCAIDE.Library.Components import Component

# python imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Network
# ---------------------------------------------------------------------------------------------------------------------- 
class Network(Component):  
    """ 
    
    Generalized Energy Network (powertrain) Class capable of creating all derivatives of conventional
    and unconventional powertrains, including hybrid-electric powertrains and the all-electric network.
    
                                            GENERIC NETWORK
          .........................................:..........................................                          
          :                        :                                :                        :
    .-------------.         .-------------.                 .-------------.           .-------------.                     
    | propulsor 1 |         | propulsor 2 |                 | propulsor 2 |           | propulsor 3 | 
    '-------------'         '-------------'                 '-------------'           '-------------'            
          ||                       ||                              ||                        ||                                  
          ||   .-------------.     ||                              ||  .-------------.       ||
          ||== | converter 1 |====== electric bus / fuel line =========| converter 2 |=======|| 
               '-------------'                                         '-------------'  
                           
    Attributes
    ----------
    tag : str
        Identifier for the network   
    
    Notes
    -----
    The evaluate function is broken into three sections: Section 1 computes all the forces and moments
    from propulsors regardless of if they are powered by fuel or an electrochemical energy storage system;
    Section 2 computees the perfomrance of any converters on the distrution lines, for example,
    turboshafts, motors, pumps etc; and Section 3 computes the thermal mangement of the system as
    well as energy consumtion of the powertrain. The state of storage devices such as covnentional fuel tanks,
    batteries are also updates. Propulsor groups can be "active" or "inactive" to simulate
    engine out conditions. Energy consumtion from avionics is also modeled 
    
    **Definitions** 
    'Propulsor Group'
        Any single or group of Components that work together to provide thrust.

    
    
    See Also
    --------
    RCAIDE.Library.Framework.Networks.Fuel
        Fuel network class 
    RCAIDE.Library.Framework.Networks.Fuel_Cell
        Fuel_Cell network class 
    RCAIDE.Library.Framework.Networks.Electric
        All-Electric network class  
    """      
    
    def __defaults__(self):
        """ This sets the default values for the network to function.
        """        
        self.tag                          = 'network'
        self.propulsors                   = Container() 
        self.converters                   = Container()
        self.non_propulsive_converters    = []
        self.nacelles                     = Container()
        self.modulators                   = Container()
        self.distributors                 = Container()
        self.sources                      = Container()
        self.systems                      = Container() 

    # linking the different network components
    def evaluate(network,state,center_of_gravity):
        """ Computes the performance of the network.
        
            This routine evaluates propulsors, converters, modulators, distributors, sources, and systems,
            and assembles forces, moments, and power balances.
        
            Energetic domains (Effort–Flow pairs) used in the model follow the power-conjugate convention:
        
            +------------+--------------------------+--------------------+----------------+----------------+
            | Domain     | Effort                   | Flow               | Power Relation | Units          |
            +============+==========================+====================+================+================+
            | Propulsive | Force (F)                | Velocity (v)       | P = F · v      | N, m/s → W     |
            +------------+--------------------------+--------------------+----------------+----------------+
            | Mechanical | Torque (τ)               | Angular speed (ω)  | P = τ · ω      | N·m, rad/s → W |
            +------------+--------------------------+--------------------+----------------+----------------+
            | Electrical | Voltage (V)              | Current (I)        | P = V · I      | V, A → W       |
            +------------+--------------------------+--------------------+----------------+----------------+
            | Chemical   | Lower Heating Value (LHV)| Mass flow (ṁ)      | P = ṁ·LHV      | J/kg, kg/s → W |
            +------------+--------------------------+--------------------+----------------+----------------+
            | Pneumatic  | Pressure (p)             | Vol. flow rate (Ṽ) | P = p · Ṽ      | Pa, m³/s → W   |
            +------------+--------------------------+--------------------+----------------+----------------+
            | Hydraulic  | Pressure (p)             | Vol. flow rate (Ṽ) | P = p · Ṽ      | Pa, m³/s → W   |
            +------------+--------------------------+--------------------+----------------+----------------+
            | Thermal    | Temperature (T)          | Entropy flow (Ṡ)   | P = T · Ṡ      | K, W/K → W     |
            +------------+--------------------------+--------------------+----------------+----------------+

            Notes
            -----
            * Electrical bus power balance uses the sign convention: 
            negative = leaving the distributor, positive = feeding the distributor.
        """ 

        # unpack   
        conditions              = state.conditions 
        propulsors              = network.propulsors  
        converters              = network.converters  
        distributors            = network.distributors
        modulators              = network.modulators
        sources                 = network.sources     
        systems                 = network.systems
        
        total_thrust            = 0. * state.ones_row(3) 
        total_moment            = 0. * state.ones_row(3)  
        total_mdot              = 0. * state.ones_row(1) 
        total_propulsive_power  = 0. * state.ones_row(1)

        # Accumulator for engine chemical power draw (negative when drawing from fuel line)
        ncp = state.numerics.number_of_control_points
        engine_chem_draw = np.zeros((ncp, 1))

        # ----------------------------------------------------------       
        # Propulsors
        # ----------------------------------------------------------

        for propulsor in propulsors:

            stored_results_flag  = False

            if propulsor.active:   
                if propulsor.identical_propulsors == False or stored_results_flag == False:
                    Thrust, Moment, Power, stored_results_flag, stored_propulsor_tag = propulsor.compute_performance(state, network, center_of_gravity= center_of_gravity)
                else:             
                    Thrust, Moment, Power = propulsor.reuse_stored_data(state,network,stored_propulsor_tag=stored_propulsor_tag,center_of_gravity= center_of_gravity)

                if propulsor.reverse_thrust  ==  True:
                    total_thrust =  total_thrust * -1    
                    total_moment =  total_moment * -1 

                total_thrust             += Thrust   
                total_moment             += Moment  
                total_propulsive_power   += Power.propulsive
                
                if isinstance(propulsor,RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan) or \
                    isinstance(propulsor,RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet) or \
                    isinstance(propulsor,RCAIDE.Library.Components.Powertrain.Propulsors.Turboprop) or \
                    isinstance(propulsor,RCAIDE.Library.Components.Powertrain.Propulsors.Internal_Combustion_Engine) or \
                    isinstance(propulsor,RCAIDE.Library.Components.Powertrain.Propulsors.Constant_Speed_Internal_Combustion_Engine):
                    total_mdot               += - Power.chemical/network.sources.fuel_tank.fuel.lower_heating_value

        for system in systems:
            Power = system.compute_performance(state)

        # --- Ensure the fuel tank supplies the *sum* of engine chemical draws to the fuel_line
        try:
            tank_rec = conditions.energy.sources['fuel_tank']
        except Exception:
            tank_rec = None

        if tank_rec is not None:
            # Ensure flat and per-link structures exist
            if not hasattr(tank_rec, 'power'):
                tank_rec.power = RCAIDE.Framework.Mission.Common.Conditions()
            if getattr(tank_rec.power, 'chemical', None) is None:
                tank_rec.power.chemical = np.zeros((ncp, 1))
            if not hasattr(tank_rec, 'power_by_distributor'):
                tank_rec.power_by_distributor = {}
            if 'fuel_line' not in tank_rec.power_by_distributor:
                tank_rec.power_by_distributor['fuel_line'] = np.zeros((ncp, 1))

            # Tank supplies positive chemical power into the fuel_line equal to -engine_chem_draw
            # engine_chem_draw is negative; we want a positive supply of the same magnitude
            tank_supply = -engine_chem_draw[:, 0]
            tank_rec.power_by_distributor['fuel_line'][:, 0] = tank_supply

            # Optionally keep the flat chemical field consistent with the net tank output
            tank_rec.power.chemical[:, 0] = tank_supply

        # ----------------------------------------------------------
        # Solve distributor power balances (simple, explicit version)
        # ----------------------------------------------------------
        # Sign convention (link seen from TARGET distributor):
        #   +P  = component supplies distributor (source on distributor)
        #   -P  = component draws from distributor (load to distributor)
        #
        # Per distributor i and domain d:  sum_over_links_into_(i,d) P_link = 0

        # 0) collect ordered lists and tags (keep it visibly simple)

        print(
            f"# dist:{len(network.distributors)} "
            f"# prop:{len(network.propulsors)} "
            f"# conv:{len(network.non_propulsive_converters)} "
            f"# mod:{len(network.modulators)} "
            f"# sys:{len(network.systems)} "
            f"# src:{len(network.sources)}"
        )

        # 1) distributor->domain map (what domain to use when a link targets this distributor)
        domain = {}
        for distributor in network.distributors:
            if isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus):
                domain[distributor.tag] = 'electrical'
            elif isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line):
                domain[distributor.tag] = 'chemical'
            elif isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Coolant_Line):
                domain[distributor.tag] = 'thermal'
            else:
                domain[distributor.tag] = None
        print(f"domain (by distributor): {domain}")

        # 2) build a flat list of LINKS: (group, comp_tag, dist_tag, domain)
        links = []

        print("Building links from components → distributors")
        for group_name in ['propulsors', 'converters', 'modulators', 'systems', 'sources']:
            group = getattr(network, group_name)
            for comp in group:
                ads = getattr(comp, 'assigned_distributors', [])
                # flatten up to two nesting levels without helpers
                for ad in ads:
                    lvl1 = ad if isinstance(ad, (list, tuple, set)) else [ad]
                    for tag1 in lvl1:
                        lvl2 = tag1 if isinstance(tag1, (list, tuple, set)) else [tag1]
                        for dist_tag in lvl2:
                            if isinstance(dist_tag, str):
                                if dist_tag in domain and domain[dist_tag] is not None:
                                    dom = domain[dist_tag]
                                    links.append((group_name, comp.tag, dist_tag, dom))
                                    print(f"  + link: ({group_name}, {comp.tag}, {dist_tag}, {dom})")
                                else:
                                    print(f"  - skip: tag '{dist_tag}' not in domain or domain is None")
                            else:
                                # this is the case that caused "unhashable type: 'list'"
                                print(f"  - skip non-string tag under {comp.tag}: {dist_tag} (type {type(dist_tag)})")

        print("Building links from distributors → distributors")
        for src_d in network.distributors:
            ads = getattr(src_d, 'assigned_distributors', [])
            for ad in ads:
                lvl1 = ad if isinstance(ad, (list, tuple, set)) else [ad]
                for tag1 in lvl1:
                    lvl2 = tag1 if isinstance(tag1, (list, tuple, set)) else [tag1]
                    for dist_tag in lvl2:
                        if isinstance(dist_tag, str):
                            if dist_tag in domain and domain[dist_tag] is not None:
                                dom = domain[dist_tag]
                                links.append(('distributors', src_d.tag, dist_tag, dom))
                                print(f"  + link: (distributors, {src_d.tag}, {dist_tag}, {dom})")
                            else:
                                print(f"  - skip: tag '{dist_tag}' not in domain or domain is None")
                        else:
                            print(f"  - skip non-string dist_tag under distributor {src_d.tag}: {dist_tag} (type {type(dist_tag)})")

        print(f"Total links found: {len(links)}")

        # ensure a per-link storage exists (power as seen by each target distributor)
        ncp = state.numerics.number_of_control_points
        def ensure_pbd(rec, dist_tag):
            if not hasattr(rec, "power_by_distributor"):
                rec.power_by_distributor = {}
            if dist_tag not in rec.power_by_distributor:
                rec.power_by_distributor[dist_tag] = np.zeros((ncp, 1))

        for grp, comp_tag, dist_tag, dom in links:
            if grp == 'propulsors':
                rec = conditions.energy.propulsors[comp_tag]
            elif grp == 'converters':
                rec = conditions.energy.converters[comp_tag]
            elif grp == 'modulators':
                rec = conditions.energy.modulators[comp_tag]
            elif grp == 'systems':
                rec = conditions.energy.systems[comp_tag]
            elif grp == 'sources':
                rec = conditions.energy.sources[comp_tag]
            else:
                rec = conditions.energy.distributors[comp_tag]
            ensure_pbd(rec, dist_tag)
                
        # 3) solve A x = b at each control point
        for t_idx in range(state.numerics.number_of_control_points):
            tiny = 1e-12

            # 3a) decide which links are KNOWN (non-zero) vs UNKNOWN (zero) at this t_idx
            unknown_cols = {}
            col_count = 0

            # First pass: detect which TRUs need a shared unknown (if ANY of their links is unknown)
            tru_needs_unknown = set()

            for grp, comp_tag, dist_tag, dom in links:
                if grp == 'propulsors':
                    rec = conditions.energy.propulsors[comp_tag]
                elif grp == 'converters':
                    rec = conditions.energy.converters[comp_tag]
                elif grp == 'modulators':
                    rec = conditions.energy.modulators[comp_tag]
                elif grp == 'systems':
                    rec = conditions.energy.systems[comp_tag]
                elif grp == 'sources':
                    rec = conditions.energy.sources[comp_tag]
                else:
                    rec = conditions.energy.distributors[comp_tag]

                # Prefer per-link value for this distributor if present and non-zero; otherwise fall back to flat field
                vflat = float(getattr(rec.power, dom)[t_idx, 0]) if hasattr(rec, 'power') and getattr(rec.power, dom, None) is not None else 0.0
                vlink = 0.0
                pbd   = getattr(rec, 'power_by_distributor', None)
                if isinstance(pbd, dict) and (dist_tag in pbd) and (pbd[dist_tag] is not None):
                    try:
                        vlink = float(pbd[dist_tag][t_idx, 0])
                    except Exception:
                        vlink = 0.0
                val = vlink if abs(vlink) >= tiny else vflat

                if grp == 'modulators':
                    # Is this component a TRU?
                    is_tru = False
                    for m in network.modulators:
                        if m.tag == comp_tag:
                            is_tru = isinstance(m, RCAIDE.Library.Components.Powertrain.Modulators.Transformer_Rectifier_Unit)
                            break
                    if is_tru and abs(val) < 1e-12:
                        tru_needs_unknown.add(comp_tag)

            # Second pass: add unknowns
            for grp, comp_tag, dist_tag, dom in links:
                if grp == 'propulsors':
                    rec = conditions.energy.propulsors[comp_tag]
                elif grp == 'converters':
                    rec = conditions.energy.converters[comp_tag]
                elif grp == 'modulators':
                    rec = conditions.energy.modulators[comp_tag]
                elif grp == 'systems':
                    rec = conditions.energy.systems[comp_tag]
                elif grp == 'sources':
                    rec = conditions.energy.sources[comp_tag]
                else:
                    rec = conditions.energy.distributors[comp_tag]

                vflat = float(getattr(rec.power, dom)[t_idx, 0]) if hasattr(rec, 'power') and getattr(rec.power, dom, None) is not None else 0.0
                vlink = 0.0
                pbd   = getattr(rec, 'power_by_distributor', None)
                if isinstance(pbd, dict) and (dist_tag in pbd) and (pbd[dist_tag] is not None):
                    try:
                        vlink = float(pbd[dist_tag][t_idx, 0])
                    except Exception:
                        vlink = 0.0
                val = vlink if abs(vlink) >= tiny else vflat

                is_tru = False
                if grp == 'modulators':
                    for m in network.modulators:
                        if m.tag == comp_tag:
                            is_tru = isinstance(m, RCAIDE.Library.Components.Powertrain.Modulators.Transformer_Rectifier_Unit)
                            break

                if is_tru:
                    # If this TRU needs an unknown, allocate exactly ONE shared column
                    if comp_tag in tru_needs_unknown:
                        key = ('modulators', comp_tag, 'TRU_SHARED', 'electrical')
                        if key not in unknown_cols:
                            unknown_cols[key] = col_count
                            col_count += 1
                    # Do not allocate per-link unknowns for TRU (they share one)
                    continue

                # Non-TRU path: add per-link unknown if still zero
                if abs(val) < 1e-12:
                    key = (grp, comp_tag, dist_tag, dom)
                    if key not in unknown_cols:
                        unknown_cols[key] = col_count
                        col_count += 1

            n_row = len(distributors)
            n_col = col_count

            print(f"t={t_idx}] unknowns: {n_col}  (rows={n_row})")

            A = np.zeros((n_row, n_col))
            b = np.zeros((n_row, 1))

            # 3b) fill A and b : each link contributes to the ROW of its TARGET distributor

            dist_index = {d.tag: i for i, d in enumerate(network.distributors)}

            for grp, comp_tag, dist_tag, dom in links:
                row = dist_index[dist_tag]

                if grp == 'propulsors':
                    rec2 = conditions.energy.propulsors[comp_tag]
                elif grp == 'converters':
                    rec2 = conditions.energy.converters[comp_tag]
                elif grp == 'modulators':
                    rec2 = conditions.energy.modulators[comp_tag]
                elif grp == 'systems':
                    rec2 = conditions.energy.systems[comp_tag]
                elif grp == 'sources':
                    rec2 = conditions.energy.sources[comp_tag]
                else:
                    rec2 = conditions.energy.distributors[comp_tag]

                # known value for this link at t_idx: prefer per-link if non-zero, else flat
                vflat2 = float(getattr(rec2.power, dom)[t_idx, 0]) if hasattr(rec2, 'power') and getattr(rec2.power, dom, None) is not None else 0.0
                vlink2 = 0.0
                pbd2   = getattr(rec2, 'power_by_distributor', None)
                if isinstance(pbd2, dict) and (dist_tag in pbd2) and (pbd2[dist_tag] is not None):
                    try:
                        vlink2 = float(pbd2[dist_tag][t_idx, 0])
                    except Exception:
                        vlink2 = 0.0
                # Prefer per-link if non-zero; otherwise fall back to flat value
                pow = vlink2 if abs(vlink2) >= tiny else vflat2

                is_tru = False
                tru_eta = 1.0
                if grp == 'modulators':
                    for m in network.modulators:
                        if m.tag == comp_tag:
                            if isinstance(m, RCAIDE.Library.Components.Powertrain.Modulators.Transformer_Rectifier_Unit):
                                is_tru = True
                                tru_eta = getattr(m, 'electrical_efficiency', 1.0) or 1.0
                            break

                if not is_tru:
                    if abs(pow) >= tiny:
                        b[row, 0] -= pow
                        # Mirror the known value into per-link storage for later accumulation
                        if grp == 'propulsors':
                            rec = conditions.energy.propulsors[comp_tag]
                        elif grp == 'converters':
                            rec = conditions.energy.converters[comp_tag]
                        elif grp == 'modulators':
                            rec = conditions.energy.modulators[comp_tag]
                        elif grp == 'systems':
                            rec = conditions.energy.systems[comp_tag]
                        elif grp == 'sources':
                            rec = conditions.energy.sources[comp_tag]
                        else:
                            rec = conditions.energy.distributors[comp_tag]
                        if not hasattr(rec, "power_by_distributor"):
                            rec.power_by_distributor = {}
                        if dist_tag not in rec.power_by_distributor:
                            rec.power_by_distributor[dist_tag] = np.zeros((ncp, 1))
                        rec.power_by_distributor[dist_tag][t_idx, 0] = pow
                    else:
                        col = unknown_cols[(grp, comp_tag, dist_tag, dom)]
                        A[row, col] += 1.0
                    continue

                # --- TRU: one shared unknown column
                shared_key = ('modulators', comp_tag, 'TRU_SHARED', 'electrical')
                if shared_key not in unknown_cols:
                    # If we get here, ensure the shared column exists to avoid KeyError
                    unknown_cols[shared_key] = n_col
                    # expand A to include the new column
                    A = np.hstack((A, np.zeros((A.shape[0], 1))))
                    n_col += 1
                col = unknown_cols[shared_key]

                if abs(pow) >= tiny:
                    # If you ever pre-seed a TRU link as known, keep sign convention:
                    b[row, 0] -= pow
                    # Mirror the known value into per-link storage for later accumulation
                    if not hasattr(conditions.energy.modulators[comp_tag], "power_by_distributor"):
                        conditions.energy.modulators[comp_tag].power_by_distributor = {}
                    if dist_tag not in conditions.energy.modulators[comp_tag].power_by_distributor:
                        conditions.energy.modulators[comp_tag].power_by_distributor[dist_tag] = np.zeros((ncp, 1))
                    conditions.energy.modulators[comp_tag].power_by_distributor[dist_tag][t_idx, 0] = pow
                else:
                    # Determine which side this row is: AC or DC bus
                    bus = None
                    for d in network.distributors:
                        if d.tag == dist_tag:
                            bus = d
                            break
                    if hasattr(bus, 'bus_type') and bus.bus_type == 'DC':
                        # DC bus gets +Pdc
                        A[row, col] += 1.0
                    else:
                        # AC bus gets -Pdc/eta
                        A[row, col] += -1.0 / tru_eta

            # 3c) solve with least-squares (handles square / over / under)
            print(A)
            print(b)
            x, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
            print(f"t={t_idx}] rank={rank}  ||res||^2={residuals.sum() if residuals.size else 0.0}")

            # 3d) write solved unknowns back
            for key, col in unknown_cols.items():
                # TRU shared column
                if key[0] == 'modulators' and key[2] == 'TRU_SHARED':
                    comp_tag = key[1]
                    Pdc = float(x[col, 0])

                    # find the TRU object and eta once
                    tru_eta = 1.0
                    for m in network.modulators:
                        if m.tag == comp_tag and isinstance(m, RCAIDE.Library.Components.Powertrain.Modulators.Transformer_Rectifier_Unit):
                            tru_eta = getattr(m, 'electrical_efficiency', 1.0) or 1.0
                            break

                    # apply to BOTH links (per distributor)
                    for (_grp, _ctag, dist_tag, dom) in links:
                        if _grp != 'modulators' or _ctag != comp_tag or dom != 'electrical':
                            continue
                        # which bus type is dist_tag?
                        bus = None
                        for d in network.distributors:
                            if d.tag == dist_tag:
                                bus = d
                                break
                        if hasattr(bus, 'bus_type') and bus.bus_type == 'DC':
                            # DC bus sees +Pdc
                            conditions.energy.modulators[comp_tag].power_by_distributor[dist_tag][t_idx, 0] = +Pdc
                        else:
                            # AC bus sees -Pdc/eta
                            conditions.energy.modulators[comp_tag].power_by_distributor[dist_tag][t_idx, 0] = -Pdc / tru_eta
                    continue

                # default path for non-TRU links: write *this link’s* value
                grp, comp_tag, dist_tag, dom = key
                val = float(x[col, 0])

                if grp == 'propulsors':
                    rec = conditions.energy.propulsors[comp_tag]
                elif grp == 'converters':
                    rec = conditions.energy.converters[comp_tag]
                elif grp == 'modulators':
                    rec = conditions.energy.modulators[comp_tag]
                elif grp == 'systems':
                    rec = conditions.energy.systems[comp_tag]
                elif grp == 'sources':
                    rec = conditions.energy.sources[comp_tag]
                else:
                    rec = conditions.energy.distributors[comp_tag]

                # store into the per-link slot (as seen by dist_tag)
                rec.power_by_distributor[dist_tag][t_idx, 0] = val

                # keep your old “flat” field too if you like (optional)
                try:
                    rec.power[dom][t_idx, 0] = val
                except Exception:
                    pass

            # ----------------------------------------------------------
            #  accumulate net power per distributor (once, after solving)
            # ----------------------------------------------------------
            # reset distributor nets for this time index
            for d in network.distributors:
                p = conditions.energy.distributors[d.tag].power
                if hasattr(p, 'electrical'): p.electrical[t_idx, 0] = 0.0
                if hasattr(p, 'chemical'):   p.chemical[t_idx, 0]   = 0.0
                if hasattr(p, 'thermal'):    p.thermal[t_idx, 0]    = 0.0

            # accumulate using per-link values
            for (grp, comp_tag, dist_tag, dom) in links:
                # get the component record
                if grp == 'propulsors':
                    rec = conditions.energy.propulsors[comp_tag]
                elif grp == 'converters':
                    rec = conditions.energy.converters[comp_tag]
                elif grp == 'modulators':
                    rec = conditions.energy.modulators[comp_tag]
                elif grp == 'systems':
                    rec = conditions.energy.systems[comp_tag]
                elif grp == 'sources':
                    rec = conditions.energy.sources[comp_tag]
                else:
                    rec = conditions.energy.distributors[comp_tag]

                # prefer per-link value; fall back to flat if needed, and if present but zero, fallback to flat
                if hasattr(rec, 'power_by_distributor') and dist_tag in rec.power_by_distributor:
                    pv = float(rec.power_by_distributor[dist_tag][t_idx, 0])
                    if abs(pv) < tiny:
                        pv = float(getattr(rec.power, dom)[t_idx, 0])
                else:
                    pv = float(getattr(rec.power, dom)[t_idx, 0])
                val = pv

                drec = conditions.energy.distributors[dist_tag].power
                if dom == 'electrical' and hasattr(drec, 'electrical'):
                    drec.electrical[t_idx, 0] += val
                elif dom == 'chemical' and hasattr(drec, 'chemical'):
                    drec.chemical[t_idx, 0] += val
                elif dom == 'thermal' and hasattr(drec, 'thermal'):
                    drec.thermal[t_idx, 0] += val
                    
            dist_tags = [d.tag for d in network.distributors]

            # print a tiny summary line
            if t_idx < 3:  # don't spam too much
                for dtag in dist_tags:
                    de = float(conditions.energy.distributors[dtag].power.electrical[t_idx, 0])
                    dc = float(conditions.energy.distributors[dtag].power.chemical[t_idx, 0])
                    dt = float(conditions.energy.distributors[dtag].power.thermal[t_idx, 0])
                    print(f"[PBAL t={t_idx}] net at {dtag}: elec={de:.3f} W, chem={dc:.3f} W, therm={dt:.3f} W")   

            # ----------------------------------------------------------
            # Fuel-line chemical power consistency check (diagnostics)
            # ----------------------------------------------------------
            try:
                LHV = float(network.sources.fuel_tank.fuel.lower_heating_value)
            except Exception:
                LHV = None

            # Identify all chemical links that terminate at the fuel_line distributor
            fuel_line_tag = 'fuel_line'
            tank_key      = ('sources', 'fuel_tank', fuel_line_tag, 'chemical')

            sum_draws_chem = 0.0  # total chemical draws into fuel_line (negative values expected for engines)
            tank_chem      = None

            for (grp, comp_tag, dist_tag, dom) in links:
                if dist_tag != fuel_line_tag or dom != 'chemical':
                    continue

                # get per-link value if present; fallback to flat power.chemical
                if grp == 'propulsors':
                    rec = conditions.energy.propulsors[comp_tag]
                elif grp == 'converters':
                    rec = conditions.energy.converters[comp_tag]
                elif grp == 'modulators':
                    rec = conditions.energy.modulators[comp_tag]
                elif grp == 'systems':
                    rec = conditions.energy.systems[comp_tag]
                elif grp == 'sources':
                    rec = conditions.energy.sources[comp_tag]
                else:
                    rec = conditions.energy.distributors[comp_tag]

                if hasattr(rec, 'power_by_distributor') and fuel_line_tag in rec.power_by_distributor:
                    val = float(rec.power_by_distributor[fuel_line_tag][t_idx, 0])
                else:
                    val = float(rec.power.chemical[t_idx, 0])

                if (grp, comp_tag, dist_tag, dom) == tank_key:
                    tank_chem = val
                else:
                    sum_draws_chem += val

            # Only report if we successfully read the tank value
            if tank_chem is not None:
                # In your sign convention: tank supplies (+), engines draw (-)
                # so tank_chem + sum_draws_chem should be ~ 0
                imbalance = tank_chem + sum_draws_chem
                if abs(imbalance) > 1e-6:
                    print(f"[check t={t_idx}] fuel-line chem power mismatch: tank={tank_chem:+.3f} W, draws={sum_draws_chem:+.3f} W, sum={imbalance:+.3f} W (expect ~0)")

                # mdot diagnostic from the *tank* power if LHV is available
                if LHV is not None and LHV > 0.0:
                    mdot_from_tank = tank_chem / LHV  # positive when consuming fuel
                    if t_idx < 3:
                        print(f"[check t={t_idx}] mdot_from_tank = {mdot_from_tank:.6f} kg/s  (LHV={LHV:.3f} J/kg)")

        conditions.energy.thrust_force_vector  = total_thrust
        conditions.energy.thrust_moment_vector = total_moment 
        conditions.energy.net_power            = total_propulsive_power
        conditions.weights.vehicle_mass_rate   = total_mdot  

        return
    
    def unpack_unknowns(self,segment):
        """Unpacks the unknowns set in the mission to be available for the mission.
    
        Assumptions:
        N/A
        
        Source:
        N/A
        
        Inputs: 
            segment   - data structure of mission segment [-]
        
        Outputs: 
        
        Properties Used:
        N/A
        """            
         
        unknowns(segment)  
        for network in segment.analyses.energy.vehicle.networks:
            for propulsor in network.propulsors:
                propulsor.unpack_propulsor_unknowns(segment) 
        return    
     
    def residuals(self,segment):
        """ This packs the residuals to be sent to the mission solver.
    
           Assumptions:
           None
    
           Source:
           N/A
    
           Inputs:
           state.conditions.energy:
               motor(s).torque                      [N-m]
               rotor(s).torque                      [N-m] 
           residuals soecific to the battery cell   
           
           Outputs:
           residuals specific to battery cell and network
    
           Properties Used: 
           N/A
       """         
        for network in segment.analyses.energy.vehicle.networks:
            for propulsor_i, propulsor in enumerate(network.propulsors):    
                if propulsor.active:
                    propulsor =  network.propulsors[propulsor.tag]
                    propulsor.pack_propulsor_residuals(segment) 
        return      
    
    def add_unknowns_and_residuals_to_segment(self, segment):
        """ This function sets up the information that the mission needs to run a mission segment using this network 
         
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            segment
            eestimated_throttles           [-]
            estimated_propulsor_group_rpms [-]  
            
            Outputs:
            segment
    
            Properties Used:
            N/A
        """                   
        segment.state.residuals.network = Residuals()
        
        for network in segment.analyses.energy.vehicle.networks:
            
            for propulsor in network.propulsors: 
                propulsor.append_operating_conditions(segment, network)  
                propulsor.append_propulsor_unknowns_and_residuals(segment)   
    
            for converter in network.converters: 
                converter.append_operating_conditions(segment)  

            for modulator in network.modulators: 
                modulator.append_operating_conditions(segment)  

            for source in  network.sources: 
                source.append_operating_conditions(segment)  

            for system in network.systems:
                system.append_operating_conditions(segment)             
    
            for distributor_i, distributor in enumerate(network.distributors):
                distributor.append_operating_conditions(segment)              
                
                # # Assign network-specific  residuals, unknowns and results data structures 
                # if distributor.active:
                #     for propulsor_group in  distributor.assigned_propulsors:
                #         propulsor =  network.propulsors[propulsor_group[0]]
                #         propulsor.append_propulsor_unknowns_and_residuals(segment)

                #     for converter_group in  distributor.assigned_converters:
                #         converter =  network.converters[converter_group[0]]
                        
                # if isinstance(distributor,RCAIDE.Library.Components.Powertrain.Distributors.Coolant_Line):                                                    
                #     # ------------------------------------------------------------------------------------------------------            
                #     # Create coolant_lines results data structure  
                #     # ------------------------------------------------------------------------------------------------------
                #     segment.state.conditions.energy.distributors[distributor.tag] = RCAIDE.Framework.Mission.Common.Conditions()        
                    
                #     # ------------------------------------------------------------------------------------------------------
                #     # Assign network-specific  residuals, unknowns and results data structures
                #     # ------------------------------------------------------------------------------------------------------       
                #     for battery_module in distributor.assigned_sources: 
                #         for btms in battery_module:
                #             btms.append_operating_conditions(segment,distributor)
                            
                #     for heat_exchanger in distributor.heat_exchangers: 
                #         heat_exchanger.append_operating_conditions(segment, distributor)
                            
                #     for reservoir in distributor.reservoirs: 
                #         reservoir.append_operating_conditions(segment, distributor)                           
    
        # Ensure the mission knows how to pack and unpack the unknowns and residuals
        segment.process.iterate.unknowns.network            = self.unpack_unknowns
        segment.process.iterate.residuals.network           = self.residuals   
        
        return segment

# ----------------------------------------------------------------------
#  Component Container
# ---------------------------------------------------------------------- 
class Container(Component.Container):
    """ The Network container class 
    """
    def evaluate(self,state,center_of_gravity):
        """ This is used to evaluate the thrust and moments produced by the network.

            Assumptions:  
                If multiple networks are attached their performances will be summed

            Source:
                None 
        """ 
        for net in self.values():             
            net.evaluate(state,center_of_gravity)  
        return   

# ----------------------------------------------------------------------
#  Handle Linking
# ----------------------------------------------------------------------
Network.Container = Container
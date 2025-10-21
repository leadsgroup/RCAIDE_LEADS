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

        print("[PBAL] building links from components → distributors")
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

        print("[PBAL] building links from distributors → distributors")
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

        print(f"[PBAL] total links found: {len(links)}")
        
        # 3) solve A x = b at each control point
        for t_idx in range(state.numerics.number_of_control_points):

            # 3a) decide which links are KNOWN (non-zero) vs UNKNOWN (zero) at this t_idx
            unknown_cols = {}
            col_count = 0

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

                arr = getattr(rec.power, dom)  # (ncp,1)
                val = float(arr[t_idx, 0])

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
            dist_list  = [d for d in network.distributors]

            for grp, comp_tag, dist_tag, dom in links:
                row = dist_index[dist_tag]

                if grp == 'propulsors':
                    pow = conditions.energy.propulsors[comp_tag].power[dom][t_idx, 0]
                elif grp == 'converters':
                    pow = conditions.energy.converters[comp_tag].power[dom][t_idx, 0]
                elif grp == 'modulators':
                    pow = conditions.energy.modulators[comp_tag].power[dom][t_idx, 0]
                elif grp == 'systems':
                    pow = conditions.energy.systems[comp_tag].power[dom][t_idx, 0]
                elif grp == 'sources':
                    pow = conditions.energy.sources[comp_tag].power[dom][t_idx, 0]
                else:
                    pow = conditions.energy.distributors[comp_tag].power[dom][t_idx, 0]

                if abs(pow) >= 1e-12:
                    # known → move to RHS:  sum(P_links)=0  ⇒  b[row] -= known
                    b[row, 0] -= pow
                else:
                    col = unknown_cols[(grp, comp_tag, dist_tag, dom)]
                    A[row, col] += 1.0

            # 3c) solve with least-squares (handles square / over / under)
            print(A)
            print(b)
            x, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
            print(f"t={t_idx}] rank={rank}  ||res||^2={residuals.sum() if residuals.size else 0.0}")

            # 3d) write solved unknowns back to the proper arrays
            for key, col in unknown_cols.items():
                grp, comp_tag, dist_tag, dom = key
                if grp == 'propulsors':
                    conditions.energy.propulsors[comp_tag].power[dom][t_idx, 0] = float(x[col, 0])
                elif grp == 'converters':
                    conditions.energy.converters[comp_tag].power[dom][t_idx, 0] = float(x[col, 0])
                elif grp == 'modulators':
                    conditions.energy.modulators[comp_tag].power[dom][t_idx, 0] = float(x[col, 0])
                elif grp == 'systems':
                    conditions.energy.systems[comp_tag].power[dom][t_idx, 0] = float(x[col, 0])
                elif grp == 'sources':
                    conditions.energy.sources[comp_tag].power[dom][t_idx, 0] = float(x[col, 0])
                else:
                    conditions.energy.distributors[comp_tag].power[dom][t_idx, 0] = float(x[col, 0])

            for d in network.distributors:
                if hasattr(conditions.energy.distributors[d.tag].power, 'electrical'):
                    conditions.energy.distributors[d.tag].power.electrical[t_idx, 0] = 0.0
                if hasattr(conditions.energy.distributors[d.tag].power, 'chemical'):
                    conditions.energy.distributors[d.tag].power.chemical[t_idx, 0]   = 0.0
                if hasattr(conditions.energy.distributors[d.tag].power, 'thermal'):
                    conditions.energy.distributors[d.tag].power.thermal[t_idx, 0]    = 0.0

            # ----------------------------------------------------------
            #  accumulate net power per distributor (once, after solving)
            # ----------------------------------------------------------
            for dist in network.distributors:
                tag = dist.tag
                pe = pc = pt = 0.0
                for (grp, comp_tag, dist_tag, dom) in links:
                    if dist_tag != tag:
                        continue
                    # get power value from correct component group
                    rec_group = getattr(conditions.energy, grp)
                    val = float(getattr(rec_group[comp_tag].power, dom)[t_idx, 0])
                    if dom == "electrical": pe += val
                    elif dom == "chemical": pc += val
                    elif dom == "thermal":  pt += val
                # store the net sums
                p = conditions.energy.distributors[tag].power
                if hasattr(p, "electrical"): p.electrical[t_idx, 0] = pe
                if hasattr(p, "chemical"):   p.chemical[t_idx, 0]   = pc
                if hasattr(p, "thermal"):    p.thermal[t_idx, 0]    = pt
                # warn if nonzero
                if abs(pe) > 1e-6: print(f"[warn t={t_idx}] {tag}: net elec {pe:+.3f} W (should be ~0)")
                if abs(pc) > 1e-6: print(f"[warn t={t_idx}] {tag}: net chem {pc:+.3f} W (should be ~0)")
                if abs(pt) > 1e-6: print(f"[warn t={t_idx}] {tag}: net therm {pt:+.3f} W (should be ~0)")

            # print a tiny summary line
            if t_idx < 3:  # don't spam too much
                for dtag in dist_tags:
                    de = float(conditions.energy.distributors[dtag].power.electrical[t_idx, 0])
                    dc = float(conditions.energy.distributors[dtag].power.chemical[t_idx, 0])
                    dt = float(conditions.energy.distributors[dtag].power.thermal[t_idx, 0])
                    print(f"[PBAL t={t_idx}] net at {dtag}: elec={de:.3f} W, chem={dc:.3f} W, therm={dt:.3f} W")   

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
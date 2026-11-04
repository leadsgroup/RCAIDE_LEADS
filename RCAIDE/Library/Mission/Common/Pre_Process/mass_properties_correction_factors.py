# RCAIDE/Library/Methods/Mission/Common/Pre_Process/mass_properties_correction_factors.py
# 
# 
# Created: Apr 2026, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  Imports 
# ----------------------------------------------------------------------------------------------------------------------  
import RCAIDE  

# ----------------------------------------------------------------------------------------------------------------------
#  Helper Functions for Mass Properties Analysis
# ----------------------------------------------------------------------------------------------------------------------   
def apply_correction_factors(analyses): 
    weights_analysis = analyses.weights
    # Apply correction factors  
    for tag, item in weights_analysis.settings.weight_correction_factors.items():
        if tag == 'empty':
            for subtag, subitem in weights_analysis.settings.weight_correction_factors[tag].items():
                for subsubtag, subsubitem in weights_analysis.settings.weight_correction_factors[tag][subtag].items():
                    analyses.vehicle.mass_properties.weight_breakdown[tag].total  -= analyses.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
                    analyses.vehicle.mass_properties.weight_breakdown[tag][subtag].total  -= analyses.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
                    analyses.vehicle.mass_properties.operating_empty -= analyses.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
                    analyses.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag] *= subsubitem
                    analyses.vehicle.mass_properties.weight_breakdown[tag][subtag].total  += analyses.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
                    analyses.vehicle.mass_properties.weight_breakdown[tag].total  += analyses.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
                    analyses.vehicle.mass_properties.operating_empty += analyses.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
        elif tag == 'operational_items':
            for subtag, subitem in weights_analysis.settings.weight_correction_factors[tag].items():
                analyses.vehicle.mass_properties.weight_breakdown[tag].total  -= subitem
                analyses.vehicle.mass_properties.operating_empty -= subitem
                analyses.vehicle.mass_properties.weight_breakdown[tag][subtag] *= subitem
                analyses.vehicle.mass_properties.weight_breakdown[tag].total  += analyses.vehicle.mass_properties.weight_breakdown[tag][subtag]
                analyses.vehicle.mass_properties.operating_empty += analyses.vehicle.mass_properties.weight_breakdown[tag][subtag]

    for tag, _ in weights_analysis.settings.weight_correction_additions.items():
        if tag == 'empty':
            for subtag, subitem in weights_analysis.settings.weight_correction_additions[tag].items():
                for subsubtag, subsubitem in weights_analysis.settings.weight_correction_additions[tag][subtag].items():
                    analyses.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag] = subsubitem
                    analyses.vehicle.mass_properties.weight_breakdown[tag][subtag].total += subsubitem
                    analyses.vehicle.mass_properties.weight_breakdown[tag].total  += subsubitem
                    analyses.vehicle.mass_properties.operating_empty += subsubitem
        elif tag == 'operational_items':
            for subtag, subitem in weights_analysis.settings.weight_correction_additions[tag].items():
                analyses.vehicle.mass_properties.weight_breakdown[tag][subtag] = subitem
                analyses.vehicle.mass_properties.weight_breakdown[tag].total  += subitem
                analyses.vehicle.mass_properties.operating_empty += subitem
    return

def apply_component_weights(analyses):
    weight_correction_factors = analyses.weights.settings.weight_correction_factors
    for key in analyses.vehicle.keys():
        structural = weight_correction_factors.empty.structural
        propulsion = weight_correction_factors.empty.propulsion
        systems    = weight_correction_factors.empty.systems
        if key == 'wings':
            for wing in analyses.vehicle.wings:
                if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing):
                    wing.mass_properties.mass *= structural.get('wing', 1.0)
                if isinstance(wing, RCAIDE.Library.Components.Wings.Horizontal_Tail):
                    wing.mass_properties.mass *= structural.get('empennage', 1.0)
                if isinstance(wing, RCAIDE.Library.Components.Wings.Vertical_Tail):
                    wing.mass_properties.mass *= structural.get('empennage', 1.0)
        elif key == 'fuselages':
            for fuselage in analyses.vehicle.fuselages:
                if isinstance(fuselage, RCAIDE.Library.Components.Fuselages.Fuselage):
                    fuselage.mass_properties.mass *= structural.get('fuselage', 1.0)
        elif key == 'networks':
            for network in analyses.vehicle.networks:
                for propulsor in network.propulsors:
                    if propulsor.nacelle is not None:
                        propulsor.nacelle.mass_properties.mass *= structural.get('nacelle', 1.0)
                    propulsor.mass_properties.mass *= propulsion.get('engines', 1.0)
        elif key == 'landing_gears':
            for landing_gear in analyses.vehicle.landing_gears:
                landing_gear.mass_properties.mass *= structural.get('landing_gear', 1.0)
        elif key == 'booms':
            for boom in analyses.vehicle.booms:
                boom.mass_properties.mass *= structural.get('boom', 1.0)
        elif key == 'systems':
            Systems      = RCAIDE.Library.Components.Powertrain.Systems
            breakdown    = analyses.vehicle.mass_properties.weight_breakdown
            system_types = {Systems.Avionics: 'avionics', Systems.Flight_Controls: 'control_systems', Systems.Auxiliary_Power_Unit: 'apu',
                            Systems.Electrical: 'electrical', Systems.Hydraulics: 'hydraulics', Systems.Environmental_Controls: 'air_conditioner',
                            Systems.Instruments: 'instruments', Systems.Furnishings: 'furnishings'}
            all_systems  = [system for network in analyses.vehicle.networks for system in network.systems]
            for system_type, name in system_types.items():
                components = [system for system in all_systems if type(system) == system_type and system.mass_properties.mass != 0]
                if len(components) == 0:
                    continue
                for system in components:
                    if system.mass_properties.calculated_flag:
                        system.mass_properties.mass *= systems.get(name, 1.0)

                # breakdown entry is the sum of its components so the OEW and the CG use the same mass
                delta = sum(system.mass_properties.mass for system in components) - breakdown.empty.systems.get(name, 0.0)
                breakdown.empty.systems[name]                     = breakdown.empty.systems.get(name, 0.0) + delta
                breakdown.empty.systems.total                    += delta
                breakdown.empty.total                            += delta
                analyses.vehicle.mass_properties.operating_empty += delta
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

    for tag, _ in weights_analysis.settings.weight_correction_additions.items():
        if tag == 'empty':
            for subtag, subitem in weights_analysis.settings.weight_correction_additions[tag].items():
                for subsubtag, subsubitem in weights_analysis.settings.weight_correction_additions[tag][subtag].items():
                    analyses.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag] = subsubitem
                    analyses.vehicle.mass_properties.weight_breakdown[tag][subtag].total += subsubitem
                    analyses.vehicle.mass_properties.weight_breakdown[tag].total  += subsubitem
                    analyses.vehicle.mass_properties.operating_empty += subsubitem
    return

def apply_component_weights(analyses):
    weight_correction_factors = analyses.weights.settings.weight_correction_factors
    for key in analyses.vehicle.keys():
        if key =='wings':
            for wing in analyses.vehicle.wings:
                if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing):
                    wing.mass_properties.mass *= weight_correction_factors.empty.structural.wing
                if isinstance(wing, RCAIDE.Library.Components.Wings.Horizontal_Tail):
                    wing.mass_properties.mass *= weight_correction_factors.empty.structural.empennage
                if isinstance(wing, RCAIDE.Library.Components.Wings.Vertical_Tail):
                    wing.mass_properties.mass *= weight_correction_factors.empty.structural.empennage
        elif key == 'fuselages':
            for fuselage in analyses.vehicle.fuselages:
                if isinstance(fuselage, RCAIDE.Library.Components.Fuselages.Fuselage):
                    fuselage.mass_properties.mass *= weight_correction_factors.empty.structural.fuselage
        elif key == 'networks':
            for network in analyses.vehicle.networks:
                for propulsor in network.propulsors:
                    propulsor.nacelle.mass_properties.mass *= weight_correction_factors.empty.structural.nacelle
                    propulsor.mass_properties.mass         *= weight_correction_factors.empty.propulsion.engines
        elif key == 'landing_gears':
            for landing_gear in analyses.vehicle.landing_gears:
                landing_gear.mass_properties.mass *= weight_correction_factors.empty.structural.landing_gear
        elif key == 'booms':
            for boom in analyses.vehicle.booms:
                boom.mass_properties.mass *= weight_correction_factors.empty.structural.boom
        elif key == 'systems':
            for system in analyses.vehicle.systems:
                if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Avionics:
                    if system.mass_properties.calculated_flag:
                        system.mass_properties.mass *= weight_correction_factors.empty.systems.avionics
                    else:
                        analyses.vehicle.mass_properties.weight_breakdown.empty.systems.avionics = system.mass_properties.mass
                if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Flight_Controls:
                    if system.mass_properties.calculated_flag:
                        system.mass_properties.mass *= weight_correction_factors.empty.systems.control_systems
                    else:
                        analyses.vehicle.mass_properties.weight_breakdown.empty.systems.control_systems = system.mass_properties.mass
                if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Auxiliary_Power_Unit:
                    if system.mass_properties.calculated_flag:
                        system.mass_properties.mass *= weight_correction_factors.empty.systems.apu
                    else:
                        analyses.vehicle.mass_properties.weight_breakdown.empty.systems.apu = system.mass_properties.mass
                if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Electrical:
                    if system.mass_properties.calculated_flag:
                        system.mass_properties.mass *= weight_correction_factors.empty.systems.electrical
                    else:
                        analyses.vehicle.mass_properties.weight_breakdown.empty.systems.electrical = system.mass_properties.mass
                if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Hydraulics:
                    if system.mass_properties.calculated_flag:
                        system.mass_properties.mass *= weight_correction_factors.empty.systems.hydraulics
                    else:
                        analyses.vehicle.mass_properties.weight_breakdown.empty.systems.hydraulics = system.mass_properties.mass
                if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Environmental_Controls:
                    if system.mass_properties.calculated_flag:
                        system.mass_properties.mass *= weight_correction_factors.empty.systems.air_conditioner
                    else:
                        analyses.vehicle.mass_properties.weight_breakdown.empty.systems.air_conditioner = system.mass_properties.mass
                if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Instruments:
                    if system.mass_properties.calculated_flag:
                        system.mass_properties.mass *= weight_correction_factors.empty.systems.instruments
                    else:
                        analyses.vehicle.mass_properties.weight_breakdown.empty.systems.instruments = system.mass_properties.mass  
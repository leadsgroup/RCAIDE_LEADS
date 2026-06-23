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
        if key =='wings':
            for wing in analyses.vehicle.wings:
                if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing):
                    if hasattr(weight_correction_factors.empty.structural, 'wing'):
                        wing.mass_properties.mass *= weight_correction_factors.empty.structural.wing
                if isinstance(wing, RCAIDE.Library.Components.Wings.Horizontal_Tail):
                    if hasattr(weight_correction_factors.empty.structural, 'empennage'):
                        wing.mass_properties.mass *= weight_correction_factors.empty.structural.empennage
                if isinstance(wing, RCAIDE.Library.Components.Wings.Vertical_Tail):
                    if hasattr(weight_correction_factors.empty.structural, 'empennage'):
                        wing.mass_properties.mass *= weight_correction_factors.empty.structural.empennage
        elif key == 'fuselages':
            for fuselage in analyses.vehicle.fuselages:
                if isinstance(fuselage, RCAIDE.Library.Components.Fuselages.Fuselage):
                    if hasattr(weight_correction_factors.empty.structural, 'fuselage'):
                        fuselage.mass_properties.mass *= weight_correction_factors.empty.structural.fuselage
        elif key == 'networks':
            for network in analyses.vehicle.networks:
                for propulsor in network.propulsors: 
                    if hasattr(weight_correction_factors.empty.structural, 'nacelle'):
                        propulsor.nacelle.mass_properties.mass *= weight_correction_factors.empty.structural.nacelle
                    if hasattr(weight_correction_factors.empty.propulsion, 'engines'):
                        propulsor.mass_properties.mass *= weight_correction_factors.empty.propulsion.engines 
        elif key == 'landing_gears':
            for landing_gear in analyses.vehicle.landing_gears:
                if hasattr(weight_correction_factors.empty.structural, 'landing_gear'):
                    landing_gear.mass_properties.mass *= weight_correction_factors.empty.structural.landing_gear
        elif key == 'booms':
            for boom in analyses.vehicle.booms:
                if hasattr(weight_correction_factors.empty.structural, 'boom'):
                    boom.mass_properties.mass *= weight_correction_factors.empty.structural.boom    
        elif key == 'systems': # If you have factor and a defined system weight, multiply those two in a calculator. 
            for system in analyses.vehicle.systems:
                if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Avionics:
                    if hasattr(weight_correction_factors.empty.systems, 'avionics') and system.mass_properties.calculated_flag:
                        system.mass_properties.mass *= weight_correction_factors.empty.systems.avionics
                    elif hasattr(weight_correction_factors.empty.systems, 'avionics') and system.mass_properties.calculated_flag == False:
                        analyses.vehicle.mass_properties.weight_breakdown.empty.systems.avionics = system.mass_properties.mass
                if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Flight_Controls:
                    if hasattr(weight_correction_factors.empty.systems, 'control_systems') and system.mass_properties.calculated_flag:
                        system.mass_properties.mass *= weight_correction_factors.empty.systems.control_systems 
                    elif hasattr(weight_correction_factors.empty.systems, 'control_systems') and system.mass_properties.calculated_flag == False:
                        analyses.vehicle.mass_properties.weight_breakdown.empty.systems.control_systems = system.mass_properties.mass
                if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Auxiliary_Power_Unit: 
                    if hasattr(weight_correction_factors.empty.systems, 'apu') and system.mass_properties.calculated_flag:
                        system.mass_properties.mass *= weight_correction_factors.empty.systems.apu  
                    elif hasattr(weight_correction_factors.empty.systems, 'apu') and system.mass_properties.calculated_flag == False:
                        analyses.vehicle.mass_properties.weight_breakdown.empty.systems.apu = system.mass_properties.mass
                if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Electrical: 
                    if hasattr(weight_correction_factors.empty.systems, 'electrical') and system.mass_properties.calculated_flag:
                        system.mass_properties.mass *= weight_correction_factors.empty.systems.electrical  
                    elif hasattr(weight_correction_factors.empty.systems, 'electrical') and system.mass_properties.calculated_flag == False:
                        analyses.vehicle.mass_properties.weight_breakdown.empty.systems.electrical = system.mass_properties.mass
                if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Hydraulics: 
                    if hasattr(weight_correction_factors.empty.systems, 'hydraulics') and system.mass_properties.calculated_flag:
                        system.mass_properties.mass *= weight_correction_factors.empty.systems.hydraulics 
                    elif hasattr(weight_correction_factors.empty.systems, 'hydraulics') and system.mass_properties.calculated_flag == False:
                        analyses.vehicle.mass_properties.weight_breakdown.empty.systems.hydraulics = system.mass_properties.mass
                if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Environmental_Controls: 
                    if hasattr(weight_correction_factors.empty.systems, 'air_conditioner') and system.mass_properties.calculated_flag:
                        system.mass_properties.mass *= weight_correction_factors.empty.systems.air_conditioner  
                    elif hasattr(weight_correction_factors.empty.systems, 'air_conditioner') and system.mass_properties.calculated_flag == False:
                        analyses.vehicle.mass_properties.weight_breakdown.empty.systems.air_conditioner = system.mass_properties.mass  
                if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Instruments:
                    if hasattr(weight_correction_factors.empty.systems, 'instruments') and system.mass_properties.calculated_flag:
                        system.mass_properties.mass *= weight_correction_factors.empty.systems.instruments  
                    elif hasattr(weight_correction_factors.empty.systems, 'instruments') and system.mass_properties.calculated_flag == False:
                        analyses.vehicle.mass_properties.weight_breakdown.empty.systems.instruments = system.mass_properties.mass  
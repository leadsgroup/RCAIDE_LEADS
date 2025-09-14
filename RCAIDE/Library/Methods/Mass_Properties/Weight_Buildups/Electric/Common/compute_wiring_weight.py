# RCAIDE/Library/Methods/Weights/Buildups/Common/compute_boom_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
 
# package imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
# Compute wiring weight
# ----------------------------------------------------------------------------------------------------------------------
def compute_wiring_weight(config):
    """ Calculates mass of wiring required for a wing, including DC power
        cables and communication cables, assuming power cables run an average of
        half the fuselage length and height in addition to reaching the motor
        location on the wingspan, and that communication and sesor  wires run an
        additional length based on the fuselage and wing dimensions. 
        
        Sources:
        Project Vahana Conceptual Trade Study

        Inputs:

            config                      RCAIDE Config Data Structure 
            max_power_draw              Maximum DC Power Draw           [W]

        Outputs:

            weight:                     Wiring Mass                     [kg]

    """
    total_mass = 0.0
    for network in config.networks:

        for battery_module in network.battery_modules:
            electrical_line       = network.Electrical_Line()
            electrical_line.to    = battery_module.tag
            electrical_line.from_ = network.tag
            electrical_line.length = manhattan_distance(electrical_line)
            electrical_line.mass = cable_weight(electrical_line)
            network.electrical_lines.append(electrical_line) 
            total_mass += electrical_line.mass
 
        for fuel_cell_stack in network.fuel_cell_stacks:
            electrical_line       = network.Electrical_Line()
            electrical_line.to    = fuel_cell_stack.tag
            electrical_line.from_ = network.tag
            electrical_line.length = manhattan_distance(electrical_line)
            electrical_line.mass = cable_weight(electrical_line)
            network.electrical_lines.append(electrical_line) 
            total_mass += electrical_line.mass

        for propulsor_tag in network.assigned_propulsors:
            electrical_line       = network.Electrical_Line()
            electrical_line.to    = propulsor_tag
            electrical_line.from_ = network.tag
            electrical_line.length = manhattan_distance(electrical_line)
            electrical_line.mass = cable_weight(electrical_line)
            network.electrical_lines.append(electrical_line)
            total_mass += electrical_line.mass 

        for converter_tag in network.assigned_converters:
            electrical_line       = network.Electrical_Line()
            electrical_line.to    = converter_tag
            electrical_line.from_ = network.tag
            electrical_line.length = manhattan_distance(electrical_line)
            electrical_line.mass = cable_weight(electrical_line)
            network.electrical_lines.append(electrical_line) 
            total_mass += electrical_line.mass
     
    # Determine mass of sensor/communication wires
    
    # fLength = 0
    # for fus in config.fuselages:
    #     fLength  += fus.lengths.total 
    
    # wiresPerBundle  = 6
    # wireDensity     = 460e-5
    # wireLength      = cableLength + (10 * fLength) +  4*network.wing.spans.projected
    # massWires       = wireDensity * wiresPerBundle * wireLength
     
    # Sum Total 
    # weight += massCables + massWires
    
    return total_mass

def manhattan_distance(electrical_line):
    """Calculate the Manhattan distance between two points in 3D space.

    Parameters:
        point1 (tuple or list): Coordinates of the first point (x1, y1, z1).
        point2 (tuple or list): Coordinates of the second point (x2, y2, z2).

    Returns:
        float: The Manhattan distance between the two points.
    """
    return sum(abs(a - b) for a, b in zip(electrical_line.from_.origin, electrical_line.to.origin))

def cable_weight(electrical_line):

    radius_conductor = 0.5 * electrical_line.diameter_conductor
    radius_insulator = 0.5 * electrical_line.diameter_insulator

    area_conductor = np.pi * radius_conductor ** 2 
    area_insulator = np.pi * (radius_insulator ** 2 - radius_conductor ** 2) 

    volume_conductor = area_conductor * electrical_line.length
    volume_insulator  = area_insulator * electrical_line.length

    m_conductor = electrical_line.conductor_material.density * volume_conductor
    m_insulator = electrical_line.insulator_material.density * volume_insulator 
    m_tot = m_conductor + m_insulator

    return m_tot
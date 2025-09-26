# RCAIDE/Methods/Powertrain/Sources/Fuel_Tanks/compute_integral_tank_volume.py
# 
# 
# Created:  Jul 2023, M. Clarke
# Modified: Aug 2025, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import  RCAIDE 
from RCAIDE.Library.Methods.Geometry.Airfoil import import_airfoil_geometry,  compute_naca_4series 

# Python Imports 
import numpy as np
from scipy.interpolate import interp1d

# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ----------------------------------------------------------------------------------------------------------------------  
def compute_fuselage_integral_tank_fuel_volume(fuel_tank,fuselage):
    """
    Computes the fuel volume for an integral fuel tank within a fuselage structure.

    This function calculates the volume of fuel that can be stored in an integral tank
    located between two fuselage segments. The calculation assumes a truncated cone
    geometry between the inner and outer segments.

    Parameters
    ----------
    fuel_tank : Fuel_Tank
        The fuel tank object containing fuel properties and mass characteristics
    fuselage : Fuselage
        The fuselage object containing segment geometry and positioning data

    Returns
    -------
    volume : float
        The calculated fuel volume in cubic meters

    Notes
    -----
    The function iterates through fuselage segments to find adjacent segments where
    the inner segment has a fuel tank. The volume calculation uses the truncated
    cone formula for the space between two elliptical cross-sections.

    **Major Assumptions**
        * Fuel tank spans exactly between two adjacent fuselage segments
        * Fuselage cross-sections are elliptical
        * Fuel density is uniform throughout the tank

    **Theory**

    The volume of a truncated cone is calculated using:

    :math:`V = \\frac{1}{3} \\left( A_1 + A_2 + \\sqrt{A_1 A_2} \\right) h`

    where:
        - :math:`A_1` is the area of the inner segment cross-section
        - :math:`A_2` is the area of the outer segment cross-section  
        - :math:`h` is the height (length) between segments

    **Definitions**

    'Integral Tank'
        A fuel tank that is built into the structure of the aircraft rather than being a separate container

    'Truncated Cone'
        A cone with the top cut off by a plane parallel to the base
    """
    total_fuel_mass  = 0
    tank_volume_o    = 0
    tank_volume_i    = 0
    origin_x         = 100
    origin_y         = 0
    origin_z         = 0

    if len(fuselage.segments) > 1:
        segment_tank_moment = np.array([0.0, 0.0, 0.0])
        seg_tags = list(fuselage.segments.keys())
        for i in range(len(seg_tags)-1):
            inner_segment = fuselage.segments[seg_tags[i]]
            outer_segment = fuselage.segments[seg_tags[i+1]]
            if inner_segment.has_fuel_tank == True: 
                h        = fuselage.lengths.total * (outer_segment.percent_x_location  - inner_segment.percent_x_location) 
                
                # volume of truncated cylinder 
                A_1_o    = np.pi * inner_segment.height /2  *  inner_segment.width/2
                A_2_o    = np.pi * outer_segment.height/2   *  outer_segment.width/2
                volume_o = (1 /3) * ( A_1_o + A_2_o + np.sqrt(A_2_o*A_2_o)) *h

                A_1_i    = np.pi * inner_segment.height /2  *  inner_segment.width/2
                A_2_i    = np.pi * outer_segment.height/2   *  outer_segment.width/2 
                volume_i = (1 /3) * ( A_1_i + A_2_i + np.sqrt(A_1_i*A_2_i)) *h
                  
                total_fuel_mass        += volume_i * fuel_tank.fuel.density  
                segment_cg             = np.array([[fuselage.lengths.total * (inner_segment.percent_x_location  + outer_segment.percent_x_location)/2 ,0,  (inner_segment.height  + outer_segment.height)/2]])
                segment_tank_moment    += segment_cg[0] * volume_i * fuel_tank.fuel.density  
                tank_volume_i          += volume_i
                tank_volume_o          += volume_o

                if fuselage.lengths.total * inner_segment.percent_x_location < origin_x: 
                    origin_x = fuselage.lengths.total * inner_segment.percent_x_location 
                    origin_y = inner_segment.percent_y_location *fuselage.lengths.total    
                    origin_z = inner_segment.percent_z_location *fuselage.lengths.total                    
            
        fuel_tank.fuel.mass_properties.center_of_gravity  = list(segment_tank_moment / total_fuel_mass)  
        fuel_tank.volume_properties.net_volume       = tank_volume_i
        fuel_tank.volume_properties.gross_volume     = tank_volume_o
    
        if fuel_tank.fuel.mass_properties.mass != 0:
            actual_fuel_volume = fuel_tank.fuel.mass_properties.mass /  fuel_tank.fuel.density  
            if actual_fuel_volume > fuel_tank.volume_properties.net_volume :
                raise AttributeError('Specified fuel mass greater than mass of fuel capable of being stored in fuel tank') 
        else:
            fuel_tank.fuel.mass_properties.mass = float(tank_volume_i *  fuel_tank.fuel.density)    
            fuel_tank.fuel.volume_properties.gross_volume = tank_volume_i
            
    # update orign of tank 
    fuel_tank.origin      = [[origin_x, origin_y, origin_z]]             
    fuel_tank.fuel.origin = [[origin_x, origin_y, origin_z]]  
    return 

def compute_wing_integral_tank_volume(fuel_tank,wing):
    """
    Computes the fuel volume for an integral fuel tank within a wing structure.

    This function calculates the volume of fuel that can be stored in an integral tank
    within the wing. It handles both single-segment and multi-segment wing configurations,
    updating the fuel tank's mass properties and center of gravity accordingly.

    Parameters
    ----------
    fuel_tank : Fuel_Tank
        The fuel tank object containing fuel properties and mass characteristics
    wing : Wing
        The wing object containing segment geometry, airfoil data, and fuel tank specifications

    Returns
    -------
    volume : float
        The calculated fuel volume in cubic units

    Notes
    -----
    The function determines the fuel tank origin from the wing origin and calculates
    volume based on whether the wing has multiple segments or is a single segment.
    For multi-segment wings, it iterates through adjacent segments to find fuel tank
    locations and accumulates moments of inertia.

    **Major Assumptions**
        * Fuel tank geometry follows the wing's airfoil profile
        * Fuel density is uniform throughout the tank
        * Wing segments are properly connected and oriented

    **Theory**

    For multi-segment wings, the volume is calculated segment by segment using
    truncated prism geometry. The center of gravity is computed as a weighted
    average of segment centers.

    **Definitions**

    'Integral Wing Tank'
        A fuel tank built into the wing structure, typically within the wing box

    'Wing Box'
        The structural box formed by the front and rear spars of the wing

    See Also
    --------
    compute_wing_integral_tank_fuel_volume : Calculates volume for single-segment wings
    compute_segmented_wing_integral_tank_fuel_volume : Calculates volume for wing segments
    """ 
    total_fuel_mass    = 0
    total_fuel_volume  = 0

    # get orgin of fuel tank     
    fuel_tank.origin      = wing.origin 
    fuel_tank.fuel.origin = wing.origin 
    
    if len(wing.segments) > 1:
        segment_tank_moment = np.array([0.0, 0.0, 0.0])
        seg_tags = list(wing.segments.keys()) 
        for i in range(len(seg_tags)-1):
            inner_segment = wing.segments[seg_tags[i]]
            outer_segment = wing.segments[seg_tags[i+1]]
            if inner_segment.has_fuel_tank == True:

                # compute volume of fuel in wing
                volume = compute_segmented_wing_integral_tank_fuel_volume(wing,inner_segment,outer_segment)
     
                total_fuel_mass      += volume * fuel_tank.fuel.density  
                segment_tank_moment  += np.array(inner_segment.mass_properties.center_of_gravity)[0] * volume * fuel_tank.fuel.density  
                total_fuel_volume    += volume
                 
        fuel_tank.fuel.mass_properties.center_of_gravity  = list(segment_tank_moment / total_fuel_mass)
        fuel_tank.volume_properties.net_volume            = total_fuel_volume
        fuel_tank.volume_properties.gross_volume          = total_fuel_volume
         
        if fuel_tank.fuel.mass_properties.mass != 0:
            actual_fuel_volume = fuel_tank.fuel.mass_properties.mass /  fuel_tank.fuel.density  
            if actual_fuel_volume > fuel_tank.volume_properties.net_volume :
                raise AttributeError('Specified fuel mass greater than mass of fuel capable of being stored in fuel tank') 
        else:
            fuel_tank.fuel.mass_properties.mass = float(total_fuel_volume *  fuel_tank.fuel.density)  
            
    else:  
        # assume whole wing has fuel 
        total_fuel_volume                                 = compute_wing_integral_tank_fuel_volume(wing) 
        total_fuel_mass                                   = total_fuel_volume  * fuel_tank.fuel.density
        
        fuel_tank.volume_properties.internal_volume       = total_fuel_volume
        fuel_tank.volume_properties.external_volume       = total_fuel_volume 
        fuel_tank.volume_properties.net_volume            = total_fuel_volume
        fuel_tank.fuel.mass_properties.center_of_gravity  = wing.mass_properties.center_of_gravity

    if fuel_tank.fuel.mass_properties.mass != 0:
        actual_fuel_volume = fuel_tank.fuel.mass_properties.mass /  fuel_tank.fuel.density  
        if actual_fuel_volume > fuel_tank.volume_properties.net_volume :
            raise AttributeError('Specified fuel mass greater than mass of fuel capable of being stored in fuel tank') 
        fuel_tank.fuel.volume_properties.net_volume = actual_fuel_volume
    else:
        fuel_tank.fuel.mass_properties.mass = float(total_fuel_volume *  fuel_tank.fuel.density)  
        fuel_tank.fuel.volume_properties.net_volume = total_fuel_volume    
    return 

def compute_wing_integral_tank_fuel_volume(wing):     
    """
    Computes the fuel volume for an integral fuel tank in a single-segment wing.

    This function calculates the volume of fuel that can be stored in an integral tank
    spanning the entire wing span. It uses the wing's root and tip chord dimensions
    along with the fuel tank's chord-wise location specifications.

    Parameters
    ---------- 
    wing : Wing
        The wing object containing chord dimensions, span, and fuel tank specifications

    Returns
    -------
    volume : float
        The calculated fuel volume in cubic meters

    Notes
    -----
    The function calculates the wing box dimensions at both root and tip locations
    and uses the truncated prism formula to determine the total volume. The wing box
    is defined by the fuel tank's chord-wise start and end locations. Assumes a NACA 0012
    airfoil if no airfoil is provided.

    **Major Assumptions**
        * Fuel tank spans the entire wing from root to tip
        * Wing box geometry follows the airfoil profile
        * Linear variation of chord dimensions from root to tip

    **Theory**

    The volume is calculated using a truncated prism formula:

    :math:`V = \\frac{1}{3} \\left( A_1 + A_2 + \\sqrt{A_1 A_2} \\right) h`

    where:
        - :math:`A_1` is the wing box area at the root
        - :math:`A_2` is the wing box area at the tip
        - :math:`h` is the wing span

    **Definitions**

    'Wing Box'
        The structural box formed by the front and rear spars, containing the fuel tank

    'Chord Location'
        The position along the wing chord where the fuel tank begins and ends
    """

    inner_front_rib_yu,inner_rear_rib_yu,inner_front_rib_yl,inner_rear_rib_yl = compute_non_dimensional_rib_coordinates(wing) 
    inner_front_rib_length  = wing.chords.root * (abs(inner_front_rib_yu) + abs(inner_front_rib_yl))
    inner_rear_rib_length   = wing.chords.root * (abs(inner_rear_rib_yu) + abs(inner_rear_rib_yl))
    inner_wingbox_length    = wing.chords.root * (wing.fuel_tank.percent_chord_end_location -wing.fuel_tank.percent_chord_start_location)  

    outer_front_rib_yu,outer_rear_rib_yu,outer_front_rib_yl,outer_rear_rib_yl = compute_non_dimensional_rib_coordinates(wing) 
    outer_front_rib_length  = wing.chords.tip * (abs(outer_front_rib_yu) + abs(outer_front_rib_yl))
    outer_rear_rib_length   = wing.chords.tip * (abs(outer_rear_rib_yu) + abs(outer_rear_rib_yl))
    outer_wingbox_length    = wing.chords.tip * (wing.fuel_tank.percent_chord_end_location -wing.fuel_tank.percent_chord_start_location)   

    # volume of truncated prism
    A_1 = inner_wingbox_length * (inner_front_rib_length + inner_rear_rib_length) / 2 
    A_2 = outer_wingbox_length * (outer_front_rib_length + outer_rear_rib_length) / 2
    h =  wing.spans.projected
    volume  = (1 /3) * ( A_1 + A_2 + np.sqrt(A_1*A_2)) *h   

    return volume


def compute_segmented_wing_integral_tank_fuel_volume(wing,inner_segment,outer_segment):   
    """
    Computes the fuel volume for an integral fuel tank between two wing segments in a multi-segment wing.

    This function calculates the volume of fuel that can be stored in an integral tank
    located between two adjacent wing segments. It accounts for the varying chord
    dimensions and fuel tank specifications of each segment.

    Parameters
    ---------- 
    wing : Wing
        The wing object containing overall geometry and symmetry properties
    inner_segment : Wing_Segment
        The inner wing segment containing chord percentage and fuel tank specifications
    outer_segment : Wing_Segment
        The outer wing segment containing chord percentage and fuel tank specifications

    Returns
    -------
    volume : float
        The calculated fuel volume in cubic units

    Notes
    -----
    The function calculates wing box dimensions for both segments and uses the
    truncated prism formula. For symmetric wings, the volume is doubled to account
    for both left and right sides. Assumes a NACA 0012 airfoil if no airfoil is provided.

    **Major Assumptions**
        * Fuel tank spans exactly between the two specified segments
        * Linear variation of chord dimensions between segments
        * Wing box geometry follows the airfoil profile

    **Theory**

    The volume is calculated using a truncated prism formula:

    :math:`V = \\frac{1}{3} \\left( A_1 + A_2 + \\sqrt{A_1 A_2} \\right) h`

    where:
        - :math:`A_1` is the wing box area at the inner segment
        - :math:`A_2` is the wing box area at the outer segment
        - :math:`h` is the span distance between segments

    **Definitions**

    'Wing Segment'
        A portion of the wing with defined chord percentage and fuel tank specifications

    'Span Location'
        The position along the wing span where the segment is located
    """

    inner_front_rib_yu,inner_rear_rib_yu,inner_front_rib_yl,inner_rear_rib_yl = compute_non_dimensional_rib_coordinates(inner_segment)
    inner_segment_chord     = wing.chords.root * inner_segment.root_chord_percent
    inner_front_rib_length  = inner_segment_chord * (abs(inner_front_rib_yu) + abs(inner_front_rib_yl))
    inner_rear_rib_length   = inner_segment_chord * (abs(inner_rear_rib_yu) + abs(inner_rear_rib_yl) )
    inner_wingbox_length    = inner_segment_chord * (inner_segment.fuel_tank.percent_chord_end_location -inner_segment.fuel_tank.percent_chord_start_location)  

    outer_front_rib_yu,outer_rear_rib_yu,outer_front_rib_yl,outer_rear_rib_yl = compute_non_dimensional_rib_coordinates(outer_segment)
    outer_segment_chord     = wing.chords.root * outer_segment.root_chord_percent
    outer_front_rib_length  = outer_segment_chord * (abs(outer_front_rib_yu) + abs(outer_front_rib_yl) )
    outer_rear_rib_length   = outer_segment_chord * (abs(outer_rear_rib_yu) + abs(outer_rear_rib_yl) )
    outer_wingbox_length    = outer_segment_chord * (outer_segment.fuel_tank.percent_chord_end_location -outer_segment.fuel_tank.percent_chord_start_location)  

    # volume of truncated prism
    A_1 = inner_wingbox_length * (inner_front_rib_length + inner_rear_rib_length) / 2 
    A_2 = outer_wingbox_length * (outer_front_rib_length + outer_rear_rib_length) / 2
    h   =  (outer_segment.percent_span_location -  inner_segment.percent_span_location) *  wing.spans.projected /2    # assumes wing is symmetric
    volume = (1 /3) * ( A_1 + A_2 + np.sqrt(A_1*A_2)) *h

    if wing.xz_plane_symmetric:
        volume *= 2    
 
    return volume

def compute_non_dimensional_rib_coordinates(compoment): 
    """
    Computes the non-dimensional rib coordinates for fuel tank volume calculations.

    This function determines the upper and lower surface coordinates at the front
    and rear rib locations of a fuel tank, accounting for airfoil geometry and
    clearance requirements.

    Parameters
    ----------
    compoment : {Wing, Wing_Segment}
        The wing or wing segment component containing airfoil data and fuel tank specifications

    Returns
    -------
    front_rib_nondim_y_upper : float
        Non-dimensional y-coordinate of upper surface at front rib location
    rear_rib_nondim_y_upper : float
        Non-dimensional y-coordinate of upper surface at rear rib location
    front_rib_nondim_y_lower : float
        Non-dimensional y-coordinate of lower surface at front rib location
    rear_rib_nondim_y_lower : float
        Non-dimensional y-coordinate of lower surface at rear rib location

    Notes
    -----
    The function interpolates airfoil geometry to find the surface coordinates at
    the fuel tank's chord-wise start and end locations. A clearance is applied to
    ensure the fuel tank doesn't interfere with the airfoil surface. Defaults to using a
    NACA 0012 airfoil if no airfoil is provided.

    **Major Assumptions**
        * Airfoil geometry is available and properly defined
        * Fuel tank chord locations are within the airfoil bounds
        * Linear interpolation is sufficient for coordinate determination

    **Definitions**

    'Non-dimensional Coordinates'
        Coordinates normalized by the chord length (x/c, y/c)

    'Rib Location'
        The position along the chord where the fuel tank front and rear boundaries are located

    'Clearance'
        The minimum distance between the fuel tank boundary and the airfoil surface

    See Also
    --------
    RCAIDE.Library.Methods.Geometry.Airfoil.import_airfoil_geometry : For importing airfoil coordinate files
    RCAIDE.Library.Methods.Geometry.Airfoil.compute_naca_4series : For generating NACA 4-series airfoil geometry
    """
    if compoment.airfoil != None: 
        if type(compoment.airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
            geometry = compute_naca_4series(compoment.airfoil.NACA_4_Series_code)
        elif type(compoment.airfoil) == RCAIDE.Library.Components.Airfoils.Airfoil: 
            geometry = import_airfoil_geometry(compoment.airfoil.coordinate_file)
    else:
        geometry = compute_naca_4series('0012')

    clearance = 1.5E-2
    front_rib_nondim_x = compoment.fuel_tank.percent_chord_start_location   
    rear_rib_nondim_x  = compoment.fuel_tank.percent_chord_end_location 
    f_upper            = interp1d(geometry.x_upper_surface  ,geometry.y_upper_surface, kind='linear')
    f_lower            = interp1d(geometry.x_lower_surface  , geometry.y_lower_surface, kind='linear')

    # non-wing box dimension coordinates 
    front_rib_nondim_y_upper = f_upper([front_rib_nondim_x])[0] - clearance
    rear_rib_nondim_y_upper  = f_upper([rear_rib_nondim_x])[0]  - clearance   
    front_rib_nondim_y_lower = f_lower([front_rib_nondim_x])[0] + clearance   
    rear_rib_nondim_y_lower  = f_lower([rear_rib_nondim_x])[0]  + clearance   

    return front_rib_nondim_y_upper,rear_rib_nondim_y_upper, front_rib_nondim_y_lower, rear_rib_nondim_y_lower 
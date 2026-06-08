# RCAIDE/Library/Methods/Mass_Properties/Moment_of_Inertia/compute_bwb_center_of_gravity.py 
# 
# Created:  January 2026, S. Shekar, A. Molloy M. Clarke,  
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE

# package imports 
import numpy as np  
from   copy import deepcopy
import shapely.geometry as geom
from shapely import Polygon, box 
import trimesh

# ----------------------------------------------------------------------------------------------------------------------
#  Compute Blended Wing Body Center of Gravity
# ----------------------------------------------------------------------------------------------------------------------  
def compute_bwb_center_of_gravity(bwb_wing, vehicle): 
    ''' computes the moment of inertia tensor for a blended wing body about a given center of gravity.
    Includes the ability to model a  wing fuel tank as a condensed wing 
    
    Inputs:
    - Wing 
    - Vehicle 

    Outputs:
    - wing center of gravity 

    Properties Used:
    N/A
    '''

    center_body_segs = []
    wing_segs        = []
    for segment in bwb_wing.segments: 
        if isinstance(segment, RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment): 
            center_body_segs.append(segment.tag)
        else:
            if segment.percent_span_location != 1.0:
                wing_segs.append(segment.tag)
    wing_segs.insert(0, center_body_segs[-1])
            
    # compute cabin moment of inertia 
    compute_center_body_center_of_gravity(bwb_wing,center_body_segs)
    
    # compute aft cabin moment of inertia 
    compute_aft_center_body_center_of_gravity(bwb_wing,center_body_segs)

    # compute wing moment of intertia 
    compute_bwb_wing_center_of_gravity(bwb_wing,wing_segs) 

    return bwb_wing.mass_properties.center_of_gravity 

def compute_bwb_wing_center_of_gravity(bwb_wing,seg_keys):

    mass = bwb_wing.mass_properties.mass

    #populate the wing segment properties and other things 
    segment_meshes = [] 
    for i in range(len(seg_keys)-1):
        # compute volume and assume unit density to get mass
        inner_segment = bwb_wing.segments[seg_keys[i]]
        outer_segment = bwb_wing.segments[seg_keys[i+1]]


        x_in = np.array(inner_segment.airfoil.geometry.x_coordinates)[:-1] * bwb_wing.chords.root *inner_segment.root_chord_percent + inner_segment.origin[0][0]
        y_in = np.array(inner_segment.airfoil.geometry.y_coordinates)[:-1] * bwb_wing.chords.root *inner_segment.root_chord_percent + inner_segment.origin[0][2]
        x_out = np.array(outer_segment.airfoil.geometry.x_coordinates)[:-1] * bwb_wing.chords.root *outer_segment.root_chord_percent+ outer_segment.origin[0][0]
        y_out = np.array(outer_segment.airfoil.geometry.y_coordinates)[:-1] * bwb_wing.chords.root *outer_segment.root_chord_percent+ outer_segment.origin[0][2]


        points_out = list(zip(x_out, y_out))
        poly_out = Polygon(points_out)

        points_in = list(zip(x_in, y_in))
        poly_in = Polygon(points_in) 

        # Compute segment span length
        L = (outer_segment.percent_span_location - inner_segment.percent_span_location) * bwb_wing.spans.projected/2
        # STEP 1: Build 3D point clouds for both sections
        x1, y1 = poly_in.exterior.xy
        x2, y2 = poly_out.exterior.xy

        pts1 = np.column_stack((x1[:-1], y1[:-1], np.zeros(len(x1)-1)))   # z = 0
        pts2 = np.column_stack((x2[:-1], y2[:-1], np.full(len(x2)-1, L))) # z = L

        # STEP 2: Combine all points
        all_pts = np.vstack([pts1, pts2])

        # STEP 3: Convex hull → watertight volume mesh
        solid_segment = trimesh.convex.convex_hull(all_pts)

        # Apply spanwise translation AFTER orientation fix
        T = np.eye(4)
        T[0, 3] = 0.0
        T[1, 3] = 0.0
        T[2, 3] = inner_segment.percent_span_location * bwb_wing.spans.projected/2
        solid_segment.apply_transform(T)
        R = trimesh.transformations.rotation_matrix(np.deg2rad(90), [1, 0, 0], [0, 0, 0])
        solid_segment.apply_transform(R)

        segment_meshes.append(solid_segment)
    
    combinde_mesh = trimesh.util.concatenate(segment_meshes)
    # Reflect across the YZ plane (mirror X)
    Ry = np.diag([1, -1, 1])   # reflection matrix

    # 1. copy the mesh
    combined_mesh_sym = deepcopy(combinde_mesh)

    # 2. apply the mirror transform
    combined_mesh_sym.vertices = (Ry @ combined_mesh_sym.vertices.T).T

    # 3. fix face orientation (reverse winding)
    combined_mesh_sym.faces = combined_mesh_sym.faces[:, ::-1]

    # 4. concatenate original + mirrored
    combined_mesh_full         = trimesh.util.concatenate([combinde_mesh, combined_mesh_sym])
    combined_mesh_full.density = mass / combined_mesh_full.volume
    I           = combined_mesh_full.moment_inertia
    centroid    = np.array(combined_mesh_full.centroid)
    centroid[1] = 0 
      
    # store values 
    bwb_wing.mass_properties.center_of_gravity         =  [centroid.tolist()]
    bwb_wing.mass_properties.moments_of_inertia.tensor =  I
    return   

def compute_aft_center_body_center_of_gravity(bwb_wing,seg_keys):
    mass          = bwb_wing.aft_center_body.mass_properties.mass 
    LOPA          = bwb_wing.layout_of_passenger_accommodations 
    cabin_length  =  max(LOPA.object_coordinates[:, 2]) + LOPA.origin[0][0] 

    segment_meshes = [] 
    for i in range(len(seg_keys)-1):
        # compute volume and assume unit density to get mass
        inner_segment = bwb_wing.segments[seg_keys[i]]
        outer_segment = bwb_wing.segments[seg_keys[i+1]]


        x_in = np.array(inner_segment.airfoil.geometry.x_coordinates)[:-1] * bwb_wing.chords.root *inner_segment.root_chord_percent + inner_segment.origin[0][0]
        y_in = np.array(inner_segment.airfoil.geometry.y_coordinates)[:-1] * bwb_wing.chords.root *inner_segment.root_chord_percent + inner_segment.origin[0][2]
        x_out = np.array(outer_segment.airfoil.geometry.x_coordinates)[:-1] * bwb_wing.chords.root *outer_segment.root_chord_percent+ outer_segment.origin[0][0]
        y_out = np.array(outer_segment.airfoil.geometry.y_coordinates)[:-1] * bwb_wing.chords.root *outer_segment.root_chord_percent+ outer_segment.origin[0][2]

        cabin_seperation = box(cabin_length, -1e9, 1e9, 1e9)
        points_out = list(zip(x_out, y_out))
        poly_out = Polygon(points_out)
        poly_out = poly_out.intersection(cabin_seperation)

        points_in = list(zip(x_in, y_in))
        poly_in = Polygon(points_in)
        poly_in = poly_in.intersection(cabin_seperation) 

        # Compute segment span length
        L = (outer_segment.percent_span_location - inner_segment.percent_span_location) * bwb_wing.spans.projected/2
        # STEP 1: Build 3D point clouds for both sections
        x1, y1 = poly_in.exterior.xy
        x2, y2 = poly_out.exterior.xy

        pts1 = np.column_stack((x1[:-1], y1[:-1], np.zeros(len(x1)-1)))   # z = 0
        pts2 = np.column_stack((x2[:-1], y2[:-1], np.full(len(x2)-1, L))) # z = L

        # STEP 2: Combine all points
        all_pts = np.vstack([pts1, pts2])

        # STEP 3: Convex hull → watertight volume mesh
        solid_segment = trimesh.convex.convex_hull(all_pts)

        # Apply spanwise translation AFTER orientation fix
        T = np.eye(4)
        T[0, 3] = 0.0
        T[1, 3] = 0.0
        T[2, 3] = inner_segment.percent_span_location * bwb_wing.spans.projected/2
        solid_segment.apply_transform(T)
        R = trimesh.transformations.rotation_matrix(np.deg2rad(90), [1, 0, 0], [0, 0, 0])
        solid_segment.apply_transform(R)

        segment_meshes.append(solid_segment)
    
    combinde_mesh = trimesh.util.concatenate(segment_meshes)
    # Reflect across the YZ plane (mirror X)
    Ry = np.diag([1, -1, 1])   # reflection matrix

    # 1. copy the mesh
    combined_mesh_sym = deepcopy(combinde_mesh)

    # 2. apply the mirror transform
    combined_mesh_sym.vertices = (Ry @ combined_mesh_sym.vertices.T).T

    # 3. fix face orientation (reverse winding)
    combined_mesh_sym.faces = combined_mesh_sym.faces[:, ::-1]

    # 4. concatenate original + mirrored
    combined_mesh_full         = trimesh.util.concatenate([combinde_mesh, combined_mesh_sym])
    combined_mesh_full.density = mass / combined_mesh_full.volume
    I                          = combined_mesh_full.moment_inertia
    centroid                   = np.array(combined_mesh_full.centroid)
    centroid[1] = 0

    # store values 
    bwb_wing.aft_center_body.mass_properties.center_of_gravity         =  [centroid.tolist()]
    bwb_wing.aft_center_body.mass_properties.moments_of_inertia.tensor =  I
    
    return  

def compute_center_body_center_of_gravity(bwb_wing,seg_keys): 
    mass          = bwb_wing.center_body.mass_properties.mass
    LOPA          = bwb_wing.layout_of_passenger_accommodations 
    cabin_length  = max(LOPA.object_coordinates[:, 2]) + LOPA.origin[0][0]

    segment_meshes = [] 
    for i in range(len(seg_keys)-1):
        # compute volume and assume unit density to get mass
        inner_segment = bwb_wing.segments[seg_keys[i]]
        outer_segment = bwb_wing.segments[seg_keys[i+1]]

        x_in = np.array(inner_segment.airfoil.geometry.x_coordinates)[:-1] * bwb_wing.chords.root *inner_segment.root_chord_percent + inner_segment.origin[0][0]
        y_in = np.array(inner_segment.airfoil.geometry.y_coordinates)[:-1] * bwb_wing.chords.root *inner_segment.root_chord_percent + inner_segment.origin[0][2]
        x_out = np.array(outer_segment.airfoil.geometry.x_coordinates)[:-1] * bwb_wing.chords.root *outer_segment.root_chord_percent+ outer_segment.origin[0][0]
        y_out = np.array(outer_segment.airfoil.geometry.y_coordinates)[:-1] * bwb_wing.chords.root *outer_segment.root_chord_percent+ outer_segment.origin[0][2]

        cabin_seperation = box(-1e9, -1e9, cabin_length, 1e9)
        points_out = list(zip(x_out, y_out))
        poly_out   = Polygon(points_out)
        poly_out   = poly_out.intersection(cabin_seperation)

        points_in = list(zip(x_in, y_in))
        poly_in = Polygon(points_in)
        poly_in = poly_in.intersection(cabin_seperation)
        
        A_1 = poly_in.area
        A_2 = poly_out.area

        # Compute segment span length
        L = (outer_segment.percent_span_location - inner_segment.percent_span_location) * bwb_wing.spans.projected/2
        # STEP 1: Build 3D point clouds for both sections
        x1, y1 = poly_in.exterior.xy
        x2, y2 = poly_out.exterior.xy

        pts1 = np.column_stack((x1[:-1], y1[:-1], np.zeros(len(x1)-1)))   # z = 0
        pts2 = np.column_stack((x2[:-1], y2[:-1], np.full(len(x2)-1, L))) # z = L

        # STEP 2: Combine all points
        all_pts = np.vstack([pts1, pts2])

        # STEP 3: Convex hull → watertight volume mesh
        solid_segment = trimesh.convex.convex_hull(all_pts)

        # Apply spanwise translation AFTER orientation fix
        T = np.eye(4)
        T[0, 3] = 0.0
        T[1, 3] = 0.0
        T[2, 3] = inner_segment.percent_span_location * bwb_wing.spans.projected/2
        solid_segment.apply_transform(T)
        R = trimesh.transformations.rotation_matrix(np.deg2rad(90), [1, 0, 0], [0, 0, 0])
        solid_segment.apply_transform(R)

        segment_meshes.append(solid_segment)
    
    combinde_mesh = trimesh.util.concatenate(segment_meshes)
    
    # Reflect across the YZ plane (mirror X)
    Ry = np.diag([1, -1, 1])   # reflection matrix

    # 1. copy the mesh
    combined_mesh_sym = deepcopy(combinde_mesh)

    # 2. apply the mirror transform
    combined_mesh_sym.vertices = (Ry @ combined_mesh_sym.vertices.T).T

    # 3. fix face orientation (reverse winding)
    combined_mesh_sym.faces = combined_mesh_sym.faces[:, ::-1]

    # 4. concatenate original + mirrored
    combined_mesh_full         = trimesh.util.concatenate([combinde_mesh, combined_mesh_sym])
    combined_mesh_full.density = mass / combined_mesh_full.volume
    I                          = combined_mesh_full.moment_inertia
    centroid                   = np.array(combined_mesh_full.centroid)
    centroid[1] = 0

    # store values 
    bwb_wing.center_body.mass_properties.center_of_gravity         =  [centroid.tolist()]
    bwb_wing.center_body.mass_properties.moments_of_inertia.tensor =  I
    
    return   
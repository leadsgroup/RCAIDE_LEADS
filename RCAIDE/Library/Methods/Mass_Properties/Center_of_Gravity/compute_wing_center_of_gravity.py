# RCAIDE/Library/Methods/Mass_Properties/Center_of_Gravity/compute_wing_center_of_gravity.py 
# 
# Created:  Dec 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE 
from RCAIDE.Library.Methods.Geometry.Airfoil import import_airfoil_geometry,  compute_naca_4series 
from RCAIDE.Library.Methods.Geometry.Planform import compute_segment_meshes

# Python Imports 
import numpy as np
from scipy.interpolate import interp1d 
import trimesh
from copy import deepcopy

# ----------------------------------------------------------------------------------------------------------------------
#  Compute Wing Center of Gravity
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_wing_center_of_gravity(wing,vehicle,n_points = 101):
    
    # unpack  
    span           = wing.spans.projected  
    symm           = wing.xz_plane_symmetric
    semispan       = span/(1+symm) 
    vertical       = wing.vertical
    mass           = wing.mass_properties.mass
    segment_meshes = []
     
    if len(wing.segments) > 1:  
        seg_keys = list(wing.segments.keys())
         
        for i in range(len(wing.segments)-1):
            # compute volume and assume unit density to get mass
            inner_segment = wing.segments[seg_keys[i]]
            outer_segment = wing.segments[seg_keys[i+1]] 
        
            # Compute segment span length
            L = (outer_segment.percent_span_location - inner_segment.percent_span_location) * wing.spans.projected/(symm + 1)
            spanwise_shift = inner_segment.percent_span_location * wing.spans.projected/2 
        
            airfoil_in = inner_segment.airfoil 
            if  airfoil_in !=  None:                 
                if type(airfoil_in) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
                    geometry_in = compute_naca_4series(airfoil_in.NACA_4_Series_code,n_points)
                elif type(airfoil_in) == RCAIDE.Library.Components.Airfoils.Airfoil: 
                    geometry_in     = import_airfoil_geometry(airfoil_in.coordinate_file,n_points)
            else:
                geometry_in = compute_naca_4series('0012',n_points)
        
            airfoil_out = outer_segment.airfoil 
            if  airfoil_out !=  None:                 
                if type(airfoil_out) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
                    geometry_out = compute_naca_4series(airfoil_out.NACA_4_Series_code,n_points)
                elif type(airfoil_out) == RCAIDE.Library.Components.Airfoils.Airfoil: 
                    geometry_out     = import_airfoil_geometry(airfoil_out.coordinate_file,n_points)
            else:
                geometry_out = compute_naca_4series('0012',n_points)
         
         
            if vertical:
                x_in  = np.array(geometry_in.x_coordinates)[:-1] * wing.chords.root *inner_segment.root_chord_percent + inner_segment.origin[0][0]
                y_in  = np.array(geometry_in.y_coordinates)[:-1] * wing.chords.root *inner_segment.root_chord_percent + inner_segment.origin[0][1]
                x_out = np.array(geometry_out.x_coordinates)[:-1] * wing.chords.root *outer_segment.root_chord_percent + outer_segment.origin[0][0]
                y_out = np.array(geometry_out.y_coordinates)[:-1] * wing.chords.root *outer_segment.root_chord_percent + outer_segment.origin[0][1]
            else:
                x_in  = np.array(geometry_in.x_coordinates)[:-1] * wing.chords.root *inner_segment.root_chord_percent + inner_segment.origin[0][0]
                y_in  = np.array(geometry_in.y_coordinates)[:-1] * wing.chords.root *inner_segment.root_chord_percent + inner_segment.origin[0][2]
                x_out = np.array(geometry_out.x_coordinates)[:-1] * wing.chords.root *outer_segment.root_chord_percent + outer_segment.origin[0][0]
                y_out = np.array(geometry_out.y_coordinates)[:-1] * wing.chords.root *outer_segment.root_chord_percent + outer_segment.origin[0][2]
            
            solid_segment =  compute_segment_meshes(x_in,y_in, x_out, y_out, L, spanwise_shift) 
            segment_meshes.append(solid_segment)
        
        combinde_mesh = trimesh.util.concatenate(segment_meshes)
    
        # Reflect across the YZ plane (mirror X)
        Ry = np.diag([1, -1, 1])    
    
        # Compute centroid
        centroid = combinde_mesh.centroid 
        
        if symm:
            # 1. copy the mesh
            combined_mesh_sym = deepcopy(combinde_mesh)
        
            # 2. apply the mirror transform
            combined_mesh_sym.vertices = (Ry @ combined_mesh_sym.vertices.T).T
        
            # 3. fix face orientation (reverse winding)
            combined_mesh_sym.faces = combined_mesh_sym.faces[:, ::-1]
        
            # 4. concatenate original + mirrored
            combined_mesh_full         = trimesh.util.concatenate([combinde_mesh, combined_mesh_sym])
        else:
            combined_mesh_full = combinde_mesh
    
        centroid = combined_mesh_full.centroid
        cg_x     = centroid[0]
        cg_y     = 0
        cg_z     = centroid[2]  
         
        wing.volume_properties.gross_volume             = combined_mesh_full.volume  
        wing.mass_properties.moments_of_inertia.tensor  = combined_mesh_full.moment_inertia
        wing.mass_properties.center_of_gravity          = [[cg_x, cg_y, cg_z]]
        
    else:  
        # update 
        dihedral   = wing.dihedral 
        le_sweep   = wing.sweeps.leading_edge
    
        # Compute segment span length
        L              =  wing.spans.projected/(symm + 1)
        spanwise_shift = 0
    
        airfoil = wing.airfoil 
        if  airfoil !=  None:                 
            if type(airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
                geometry = compute_naca_4series(airfoil.NACA_4_Series_code,n_points)
            elif type(airfoil) == RCAIDE.Library.Components.Airfoils.Airfoil: 
                geometry     = import_airfoil_geometry(airfoil.coordinate_file,n_points)
        else:
            geometry = compute_naca_4series('0012',n_points) 

        if vertical:
            x_in  = np.array(geometry.x_coordinates)[:-1] * wing.chords.root 
            y_in  = np.array(geometry.y_coordinates)[:-1] * wing.chords.root 
            x_out = np.array(geometry.x_coordinates)[:-1] * wing.chords.tip  + L  
            y_out = np.array(geometry.y_coordinates)[:-1] * wing.chords.tip  + L *np.tan(le_sweep)  
        else: 
            x_in  = np.array(geometry.x_coordinates)[:-1] * wing.chords.root 
            y_in  = np.array(geometry.y_coordinates)[:-1] * wing.chords.root 
            x_out = np.array(geometry.x_coordinates)[:-1] * wing.chords.tip  + L 
            y_out = np.array(geometry.y_coordinates)[:-1] * wing.chords.tip  + L *np.tan(dihedral) 
        
        solid_segment =  compute_segment_meshes(x_in,y_in, x_out, y_out, L, spanwise_shift) 
        segment_meshes.append(solid_segment)
    
        combinde_mesh = trimesh.util.concatenate(segment_meshes)
    
        # Reflect across the YZ plane (mirror X)
        Ry = np.diag([1, -1, 1])    
    
        # Compute centroid
        centroid = combinde_mesh.centroid 
        
        if symm:
            # 1. copy the mesh
            combined_mesh_sym = deepcopy(combinde_mesh)
        
            # 2. apply the mirror transform
            combined_mesh_sym.vertices = (Ry @ combined_mesh_sym.vertices.T).T
        
            # 3. fix face orientation (reverse winding)
            combined_mesh_sym.faces = combined_mesh_sym.faces[:, ::-1]
        
            # 4. concatenate original + mirrored
            combined_mesh_full         = trimesh.util.concatenate([combinde_mesh, combined_mesh_sym])
  
        else:
            combined_mesh_full = combinde_mesh
            
        centroid = combined_mesh_full.centroid
        cg_x     = centroid[0]
        cg_y     = 0
        cg_z     = centroid[2] 
     
        wing.volume_properties.gross_volume             = combined_mesh_full.volume  
        wing.mass_properties.moments_of_inertia.tensor  = combined_mesh_full.moment_inertia
        wing.mass_properties.center_of_gravity          = [[cg_x, cg_y, cg_z]]        
     
    return wing.mass_properties.center_of_gravity
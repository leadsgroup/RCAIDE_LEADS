import numpy as np 
import RCAIDE
from RCAIDE.Library.Methods.Geometry.Planform.convert_sweep import convert_sweep_segments, convert_sweep 
from RCAIDE.Library.Methods.Geometry.Airfoil                import  compute_naca_4series, import_airfoil_geometry 

def generate_cabin_geometry(cabin, component, n_points):  
    if issubclass(type(component), RCAIDE.Library.Components.Wings.Wing):

        wing = component 


        LOPA = wing.layout_of_passenger_accommodations.object_coordinates
        # Step 1: plot cabin bounds  
        # get points at x min 
        x_min_locs   =  np.where( LOPA[:,2] == min(LOPA[:,2]))[0]
        x_min        =  LOPA[x_min_locs[0],2] -  LOPA[x_min_locs[0],5]/2
        x_min_y_max  =  max(LOPA[x_min_locs,3] + LOPA[x_min_locs,6]/2 )
        x_min_y_min  =  min(LOPA[x_min_locs,3] - LOPA[x_min_locs,6]/2 ) 
        x_border_pts = [x_min, x_min] 
        y_border_pts = [x_min_y_min, x_min_y_max] 

        # get points at y max 
        y_max_locs   =  np.where( LOPA[:,3] == max(LOPA[:,3]))[0]
        y_max        =  LOPA[y_max_locs[0],3] + LOPA[y_max_locs[0],6]/2 
        y_max_x_max  =  max(LOPA[y_max_locs,2] + LOPA[y_max_locs[0],5]/2)
        y_max_x_min  =  min(LOPA[y_max_locs,2] - LOPA[y_max_locs[0],5]/2) 
        x_border_pts.append(y_max_x_min)
        x_border_pts.append(y_max_x_max)
        y_border_pts.append(y_max)
        y_border_pts.append(y_max) 

        # get points at x max 
        x_max_locs   =  np.where( LOPA[:,2] == max(LOPA[:,2]))[0]
        x_max        =  LOPA[x_max_locs[0],2] + LOPA[x_max_locs[0],5]/2
        x_max_y_max  =  max(LOPA[x_max_locs,3] + LOPA[x_max_locs,6]/2)
        x_max_y_min  =  min(LOPA[x_max_locs,3] - LOPA[x_max_locs,6]/2)  
        x_border_pts.append(x_max)
        x_border_pts.append(x_max)
        y_border_pts.append(x_max_y_max)
        y_border_pts.append(x_max_y_min)

        # get points at y min  
        y_min_locs   =  np.where( LOPA[:,3] == min(LOPA[:,3]))[0]
        y_min        =  LOPA[y_min_locs[0],3] - LOPA[y_min_locs[0],6]/2 
        y_min_x_max  =  max(LOPA[y_min_locs,2] + LOPA[y_min_locs[0],5]/2)
        y_min_x_min  =  min(LOPA[y_min_locs,2] - LOPA[y_min_locs[0],5]/2)
        x_border_pts.append(y_min_x_max)  
        x_border_pts.append(y_min_x_min)
        y_border_pts.append(y_min)
        y_border_pts.append(y_min)    

        # loop through points and determine if there are duplicates
        y_border_pts = np.array(y_border_pts)
        x_border_pts = np.array(x_border_pts)
        # cut where y is negative
        port_idxs  =  np.where(y_border_pts<0)[0]
        starboard_x_points = np.delete(x_border_pts, port_idxs) 
        starboard_y_points = np.delete(y_border_pts, port_idxs)

        leading_edge_points = np.vstack((starboard_x_points[:2] + cabin.origin[0][0], starboard_y_points[:2]))
        trailing_edge_points = np.vstack((starboard_x_points[2:] + cabin.origin[0][0], starboard_y_points[2:]))

        for wing_segment in wing.segments:
            local_chord = wing.chords.root * wing_segment.root_chord_percent
            y_seg = wing_segment.origin[0][0]
            x_le = np.interp(y_seg, leading_edge_points[1], leading_edge_points[0])
            x_te = np.interp(y_seg, trailing_edge_points[1], trailing_edge_points[0])

            airfoil = wing_segment.airfoil  

            if  airfoil !=  None:                 
                if type(airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
                    geometry = compute_naca_4series(airfoil.NACA_4_Series_code,n_points)
                elif type(airfoil) == RCAIDE.Library.Components.Airfoils.Airfoil: 
                    geometry     = import_airfoil_geometry(airfoil.coordinate_file,n_points)
            else:
                t_c = str(int(wing_segment.thickness_to_chord *  100)).zfill(4)
                geometry = compute_naca_4series(t_c,n_points) 


            x_coords = geometry.x_coordinates * local_chord + wing_segment.origin[0][0]
            x_coords[x_coords < x_le] = x_le
            x_coords[x_coords > x_te] = x_te

            x_coords = (x_coords - wing_segment.origin[0][0]) / (local_chord)
            
            y_coords =  geometry.y_coordinates
            y_coords[y_coords<0] =  0

            section_y = wing_segment.percent_span_location * wing.spans.projected / 2

            if  section_y < cabin.origin[0][1] + cabin.width:
                cabin_segment = RCAIDE.Library.Components.Fuselages.Cabins.Segments.Segment()
                cabin_segment.tag = wing_segment.tag + '_cabin'
                cabin_segment.origin = wing_segment.origin
                cabin_segment.percent_span_location = wing_segment.percent_span_location
                cabin_segment.root_chord_percent = wing_segment.root_chord_percent
                cabin_segment.thickness_to_chord = wing_segment.thickness_to_chord
                cabin_segment.sweeps.leading_edge = wing_segment.sweeps.leading_edge
                cabin_segment.dihedral_outboard = wing_segment.dihedral_outboard
                cabin_segment.twist = wing_segment.twist
                cabin_segment.append_airfoil(wing_segment.airfoil)   
                cabin_segment.airfoil.geometry.x_coordinates = x_coords
                cabin_segment.airfoil.geometry.y_coordinates = y_coords
                
                cabin.append_segment(cabin_segment)



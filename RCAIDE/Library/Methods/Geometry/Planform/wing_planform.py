# RCAIDE/Library/Methods/Geometry/Planform/wing_planform.py
# 
# 
# Created:  Jul 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE 
from RCAIDE.Library.Methods.Geometry.Planform.convert_sweep import convert_sweep_segments, convert_sweep 
from RCAIDE.Library.Methods.Geometry.Airfoil                import  compute_naca_4series, import_airfoil_geometry
from RCAIDE.Library.Methods.Geometry.Planform.compute_segment_centroid import compute_segment_centroid

# package imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Wing Segmented Planform
# ----------------------------------------------------------------------------------------------------------------------    
def wing_planform(wing):
    """Computes standard wing planform values.
    
    Assumptions:
    Multisegmented wing. There is no unexposed wetted area, ie wing area that 
    intersects inside a fuselage. Aerodynamic center is at 25% mean aerodynamic chord.
    
    Source:
    None
    
    Inputs: 
    wing.
      chords.root              [m]
      spans.projected          [m]
      symmetric                <boolean> Determines if wing is symmetric
    
    Outputs:
    wing.
      spans.total                [m]
      chords.tip                 [m]
      chords.mean_aerodynamics   [m]
      wing.chords.mean_geometric [m]
      areas.reference            [m^2]
      taper                      [-]
      sweeps.quarter_chord       [radians]
      aspect_ratio               [-]
      thickness_to_chord         [-]
      dihedral                   [radians]
      
      aerodynamic_center         [m]      x, y, and z location

        
    
    Properties Used:
    N/A
    """ 
    sym  = wing.xz_plane_symmetric
    if len(wing.segments) > 1: 
        # Unpack
        RC       = wing.chords.root
        vertical = wing.vertical
        span     = wing.spans.projected
        
        # Pull all the segment data into array format
        span_locs = []
        twists    = []
        sweeps    = []
        dihedrals = []
        chords    = []
        t_cs      = []

        seg_keys = list(wing.segments.keys())  
        for i in range(len(wing.segments)): 
            seg       = wing.segments[seg_keys[i]]
            span_locs.append(seg.percent_span_location)
            twists.append(seg.twist)
            chords.append(seg.root_chord_percent)
            
            if i ==  (len(wing.segments) - 1):
                sweeps.append(sweeps[-1])
                seg.sweeps.quarter_chord = sweeps[-1]
                seg.sweeps.leading_edge  = sweeps[-1]
            else:
                if seg.sweeps.quarter_chord !=  None: 
                    sweeps.append(seg.sweeps.quarter_chord)                
                
                elif seg.sweeps.quarter_chord == None and  seg.sweeps.leading_edge != None:
                    # covert leading edge to quarter chord
                    if i == len(wing.segments) - 1:
                        sweeps.append(0)
                        seg.sweeps.quarter_chord = 0
                    else: 
                        next_seg  = wing.segments[seg_keys[i+1]]                
                        quarter_chord_sweep =  convert_sweep_segments(seg.sweeps.leading_edge, seg, next_seg, wing, old_ref_chord_fraction=0.0, new_ref_chord_fraction=0.25)
                        sweeps.append(quarter_chord_sweep)
                        seg.sweeps.quarter_chord = quarter_chord_sweep 
                else:
                    raise AssertionError("Quarter chord or leading edge sweep must be defined") 
             
            if seg.airfoil != None: 
                if type(seg.airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil: # check if naca 4 series of airfoil from datafile
                    seg.airfoil.geometry = compute_naca_4series(seg.airfoil.NACA_4_Series_code) 
                    seg.thickness_to_chord = seg.airfoil.geometry.thickness_to_chord 
                else:
                    seg.airfoil.geometry = import_airfoil_geometry(seg.airfoil.coordinate_file) 
                    seg.thickness_to_chord =  seg.airfoil.geometry.thickness_to_chord                 
            
                t_cs.append(seg.thickness_to_chord) 
            else: 
                t_cs.append(seg.thickness_to_chord)
            dihedrals.append(seg.dihedral_outboard)
            
        # Convert to arrays
        chords    = np.array(chords)
        span_locs = np.array(span_locs)
        sweeps    = np.array(sweeps)
        t_cs      = np.array(t_cs)
        
        # Basic calcs:
        semispan     = span/(1+sym)
        lengths_ndim = span_locs[1:]-span_locs[:-1]
        lengths_dim  = lengths_ndim*semispan
        chords_dim   = RC*chords
        tapers       = chords[1:]/chords[:-1]
        
        # Calculate the areas of each segment
        As = (lengths_dim*chords_dim[:-1]-(chords_dim[:-1]-chords_dim[1:])*(lengths_dim/2)) 
        Af = lengths_dim*((chords_dim[:-1]*t_cs[:-1]+chords_dim[1:]*t_cs[1:])/2)

        # Calculate the wing area
        ref_area = np.sum(As)*(1+sym)
        frontal_area = np.sum(Af)*(1+sym)
        
        # Calculate the Aspect Ratio
        AR = (span**2)/ref_area
        
        # Calculate the total span
        lens = lengths_dim/np.cos(dihedrals[:-1])
        total_len = np.sum(np.array(lens))*(1+sym)
        
        # Calculate the mean geometric chord
        mgc = ref_area/span
        
        # Calculate the mean aerodynamic chord
        A = chords_dim[:-1]
        B = (A-chords_dim[1:])/(-lengths_ndim)
        C = span_locs[:-1]
        integral = ((A+B*(span_locs[1:]-C))**3-(A+B*(span_locs[:-1]-C))**3)/(3*B)
        # For the cases when the wing doesn't taper in a spot
        integral[np.isnan(integral)] = (A[np.isnan(integral)]**2)*((lengths_ndim)[np.isnan(integral)])
        MAC = (semispan*(1+sym)/(ref_area))*np.sum(integral)
        
        # Calculate the taper ratio
        lamda = chords[-1]/chords[0]
        
        # the tip chord
        ct = chords_dim[-1]
        
        # Calculate an average t/c weighted by area
        t_c = np.sum(As*t_cs[:-1])/(ref_area/2)
        
        # Calculate the segment leading edge sweeps
        r_offsets = chords_dim[:-1]/4
        t_offsets = chords_dim[1:]/4
        le_sweeps = np.arctan((r_offsets+np.tan(sweeps[:-1])*(lengths_dim)-t_offsets)/(lengths_dim)) 
        le_sweeps = np.append(le_sweeps, 0)
        
        # Calculate the effective sweeps
        c_4_sweep     = np.arctan(np.sum(lengths_ndim*np.tan(sweeps[:-1])))
        le_sweep_total= np.arctan(np.sum(lengths_ndim*np.tan(le_sweeps[:-1])))
    
        # Calculate the aerodynamic center, but first the centroid
        dxs = np.cumsum(np.concatenate([np.array([0]),np.tan(le_sweeps[:-1])*lengths_dim]))
        dys = np.cumsum(np.concatenate([np.array([0]),lengths_dim]))
        dzs = np.cumsum(np.concatenate([np.array([0]),np.tan(dihedrals[:-1])*lengths_dim]))
        
        Cxys = []
        for i in range(len(lengths_dim)):
            Cxys.append(compute_segment_centroid(le_sweeps[i],lengths_dim[i],dxs[i],dys[i],dzs[i], tapers[i], 
                                         dihedrals[i], chords_dim[i], chords_dim[i+1]))
    
        aerodynamic_center = (np.dot(np.transpose(Cxys),As)/(ref_area/(1+sym)))
        
        
        single_side_aerodynamic_center = (np.array(aerodynamic_center)*1.)
        single_side_aerodynamic_center[0] = single_side_aerodynamic_center[0] - MAC*.25    
        if sym== True:
            aerodynamic_center[1] = 0 
            
        aerodynamic_center[0] = single_side_aerodynamic_center[0]
        
        # Total length for supersonics
        total_length = np.tan(le_sweep_total)*semispan + chords[-1]*RC
        
        if vertical: 
            for i in range(len(wing.segments)):
                wing.segments[seg_keys[i]].sweeps.leading_edge = le_sweeps[i]
                wing.segments[seg_keys[i]].origin = [[dxs[i],dzs[i],dys[i]]]
        else:
            for i in range(len(wing.segments)):
                wing.segments[seg_keys[i]].sweeps.leading_edge = le_sweeps[i]
                wing.segments[seg_keys[i]].origin = [[dxs[i],dys[i],dzs[i]]]
        wing.spans.total                     = total_len
        wing.chords.mean_geometric           = mgc
        wing.chords.mean_aerodynamic         = MAC
        wing.chords.tip                      = ct
        wing.taper                           = lamda
        wing.areas.projected                 = ref_area
        wing.areas.reference                 = ref_area
        wing.areas.front_projected           = frontal_area
        wing.sweeps.quarter_chord            = c_4_sweep
        wing.sweeps.leading_edge             = le_sweep_total
        wing.thickness_to_chord              = t_c
        wing.aerodynamic_center              = aerodynamic_center 
        wing.total_length                    = total_length  
        wing.aspect_ratio                    = AR
            
        # update remainder segment properties
        segment_properties(wing) 
        
        # compute trap area 
        seg_keys = list(wing.segments.keys())  
        for tag, segment in enumerate(wing.segments): 
            if segment.chords.reference_area_root:                      
                segment_root_chord       = wing.segments[seg_keys[tag]].root_chord_percent * wing.chords.root 
                segment_tip_chord        = wing.segments[seg_keys[tag+1]].root_chord_percent * wing.chords.root 
                segnent_start_span       = wing.segments[seg_keys[tag]].percent_span_location * wing.spans.projected
                reference_wing_span      = wing.segments[seg_keys[tag+1]].percent_span_location * wing.spans.projected
    
                next_seg = wing.segments[seg_keys[tag+1]]
                trailing_edge_sweep = convert_sweep_segments(segment.sweeps.quarter_chord, segment, next_seg, wing, old_ref_chord_fraction=0.25, new_ref_chord_fraction=1.0) 
                leading_edge_sweep  = convert_sweep_segments(segment.sweeps.quarter_chord, segment, next_seg, wing, old_ref_chord_fraction=0.25, new_ref_chord_fraction=0.0) 
    
                projected_root_chord = segment_root_chord + segnent_start_span * (np.tan(leading_edge_sweep) - np.tan(trailing_edge_sweep))
                wing.areas.reference = (projected_root_chord + segment_tip_chord)/2 * reference_wing_span        
        
    else: 
        # unpack
        sref        = wing.areas.reference
        taper       = wing.taper
        sweep       = wing.sweeps.quarter_chord
        ar          = wing.aspect_ratio
        dihedral    = wing.dihedral 
        vertical    = wing.vertical
        symmetric   = wing.xz_plane_symmetric  
        if wing.airfoil != None: 
            if type(wing.airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil: # check if naca 4 series of airfoil from datafile
                wing.airfoil.geometry = compute_naca_4series(wing.airfoil.NACA_4_Series_code) 
                wing.thickness_to_chord = wing.airfoil.geometry.thickness_to_chord 
            else:
                wing.airfoil.geometry   = import_airfoil_geometry(wing.airfoil.coordinate_file) 
                wing.thickness_to_chord =  wing.airfoil.geometry.thickness_to_chord 
                
        t_c_w  = wing.thickness_to_chord   
        
        # calculate
        span       = (ar*sref)**.5
        semispan   = span/(1+sym)
        span_total = span/np.cos(dihedral)
        chord_root = 2*sref/span/(1+taper)
        chord_tip  = taper * chord_root
        mgc        = (chord_root+chord_tip)/2 
        swet       = 2.*span/2.*(chord_root+chord_tip) *  (1.0 + 0.2*t_c_w) 
        mac        = 2./3.*( chord_root+chord_tip - chord_root*chord_tip/(chord_root+chord_tip) )
        
        # calculate leading edge sweep
        if wing.sweeps.leading_edge == None:
            le_sweep = np.arctan( np.tan(sweep) - (4./ar)*(0.-0.25)*(1.-taper)/(1.+taper) )
            wing.sweeps.leading_edge = le_sweep
        else:
            le_sweep = wing.sweeps.leading_edge
            wing.sweeps.quarter_chord = convert_sweep(wing,old_ref_chord_fraction = 0.0,new_ref_chord_fraction = 0.25)
        
        # estimating aerodynamic center coordinates
        y_coord = span / 6. * (( 1. + 2. * taper ) / (1. + taper))
        x_coord = mac * 0.25 + y_coord * np.tan(le_sweep)
        z_coord = y_coord * np.tan(dihedral)
            
        if vertical:
            temp    = y_coord * 1.
            y_coord = z_coord * 1.
            z_coord = temp
    
        if symmetric:
            y_coord = 0    
        
        # Total length calculation
        total_length = np.tan(le_sweep)*span/2. + chord_tip
            
        # update
        wing.chords.root                = chord_root
        wing.chords.tip                 = chord_tip
        wing.chords.mean_aerodynamic    = mac
        wing.chords.mean_geometric      = mgc
        wing.sweeps.leading_edge        = le_sweep
        wing.areas.wetted               = swet
        wing.areas.projected            = sref
        wing.areas.front_projected      = span*mgc
        wing.spans.projected            = span
        wing.spans.total                = span_total
        wing.aerodynamic_center         = [x_coord , y_coord, z_coord]
        wing.total_length               = total_length 

        # estimate LEMAC
        wing.LEMAC =  wing.origin[0][0] + np.tan(wing.sweeps.leading_edge) * y_coord  

        segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                           = 'root'
        segment.percent_span_location         = 0.0
        segment.root_chord_percent            = 1.0 
        segment.sweeps.leading_edge           = le_sweep
        segment.sweeps.quarter_chord          = wing.sweeps.quarter_chord
        segment.dihedral_outboard             = dihedral
        segment.thickness_to_chord            = t_c_w
        wing.append_segment(segment)  

        segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                           = 'tip'
        segment.percent_span_location         = 1.0
        segment.root_chord_percent            = taper  
        segment.thickness_to_chord            = t_c_w
        
        if vertical:
            dx = np.tan(le_sweep) * semispan
            dy = np.tan(dihedral) * semispan
            dz = semispan
            segment.origin   = [[dx,  dy , semispan ]]
        else:
            dx = np.tan(le_sweep) * semispan 
            dz = np.tan(dihedral) * semispan
            segment.origin   = [[dx, semispan  , dz ]] 
        wing.append_segment(segment)     

        segment_properties(wing) 
      
    # control surface  
    taper = wing.taper 
    Sw    = wing.areas.reference                
    bw    = wing.spans.projected               
    for cs in  wing.control_surfaces:    
        cs_span                 = (cs.span_fraction_end - cs.span_fraction_start) * bw
        chord_root              = 2*Sw/bw/(1+taper) 
        chord_tip               = taper * chord_root
        delta_chord             = chord_tip - chord_root 
        wing_chord_cs_start     = chord_root + delta_chord * cs.span_fraction_start 
        wing_chord_cs_end       = chord_root + delta_chord * cs.span_fraction_end
        cs_chord_start          = wing_chord_cs_start* cs.chord_fraction 
        cs_chord_end            = wing_chord_cs_end* cs.chord_fraction   
        cf                      = (cs_chord_start +cs_chord_end) /2  
        Sf                      = cs_span * cf
        cs.area                 = Sf 
        cs.span                 = cs_span
        cs.root_chord           = cs_chord_start
        cs.tip_chord            = cs_chord_end 

    seg_keys = list(wing.segments.keys())  
    for tag, segment in enumerate(wing.segments): 
        if segment.chords.reference_area_root:                      
            segment_root_chord       = wing.segments[seg_keys[tag]].root_chord_percent * wing.chords.root 
            segment_tip_chord        = wing.segments[seg_keys[tag+1]].root_chord_percent * wing.chords.root 
            segnent_start_span       = wing.segments[seg_keys[tag]].percent_span_location * wing.spans.projected
            reference_wing_span      = wing.segments[seg_keys[tag+1]].percent_span_location * wing.spans.projected

            next_seg = wing.segments[seg_keys[tag+1]]
            trailing_edge_sweep = convert_sweep_segments(segment.sweeps.quarter_chord, segment, next_seg, wing, old_ref_chord_fraction=0.25, new_ref_chord_fraction=1.0) 
            leading_edge_sweep  = convert_sweep_segments(segment.sweeps.quarter_chord, segment, next_seg, wing, old_ref_chord_fraction=0.25, new_ref_chord_fraction=0.0) 

            projected_root_chord = segment_root_chord + segnent_start_span/2 * (np.tan(leading_edge_sweep) - np.tan(trailing_edge_sweep))
            wing.areas.reference = (projected_root_chord + segment_tip_chord)/2 * reference_wing_span
            wing.chords.mean_aerodynamic =   2./3.*( projected_root_chord+segment_tip_chord - projected_root_chord*segment_tip_chord/(projected_root_chord+segment_tip_chord) )
            
            # estimating aerodynamic center coordinates
            outboard_segment_origin =  wing.segments[seg_keys[tag+1]].origin
            span = wing.spans.projected
            taper = segment_tip_chord/projected_root_chord
            y_coord = span / 6. * (( 1. + 2. * taper ) / (1. + taper))
            x_coord = wing.chords.mean_aerodynamic * 0.25 + y_coord * np.tan(leading_edge_sweep) 
            LEMAC = outboard_segment_origin[0][0] + np.tan(leading_edge_sweep)*(y_coord - wing.segments[seg_keys[tag+1]].percent_span_location * wing.spans.projected/2)
            # estimate LEMAC
            wing.LEMAC =  LEMAC

    return wing

def segment_properties(wing):
    """Computes detailed segment properties. These are currently used for parasite drag calculations.

    Assumptions:
    Segments are trapezoids

    Source:
    http://aerodesign.stanford.edu/aircraftdesign/aircraftdesign.html (Stanford AA241 A/B Course Notes)

    Inputs:
    wing.
      percent_span_unexposed [m]
      symmetric                 [-]
      spans.projected           [m]
      thickness_to_chord        [-]
      areas.wetted              [m^2]
      chords.root               [m]
      Segments.
        percent_span_location   [-]
        root_chord_percent      [-]

    Outputs:
    wing.areas.wetted           [m^2]
    wing.areas.reference        [m^2]
    wing.segments.
      taper                     [-]
      chords.mean_aerodynamic   [m]
      areas.
        reference               [m^2]
        exposed                 [m^2]
        wetted                  [m^2]
        

    Properties Used:
    N/A
    """  
        
    # Unpack wing
    percent_span_unexposed    = wing.percent_span_unexposed
    symm                      = wing.xz_plane_symmetric
    semispan                  = wing.spans.projected*0.5 * (2 - symm)
    segments                  = wing.segments
    wing_root_chord           = wing.chords.root  
    segment_names             = list(segments.keys())
    num_segments              = len(segment_names)
    
    # initialize areas to 0
    total_wetted_area         = 0.0 
    center_body_area          = 0.0
    total_reference_area      = 0.0
    aft_center_body_area      = 0.0    
    
    for seg_idx in range(num_segments):
        if seg_idx == num_segments-1:
            continue 
        else:
            # unpack segment 
            inboard_segment   = segments[segment_names[seg_idx]]
            outboard_segment  = segments[segment_names[seg_idx+1]]
            
            # segment thickness to chord ratio 
            t_c_w     = inboard_segment.thickness_to_chord
            
            # segment span
            span_seg  = semispan*(outboard_segment.percent_span_location - inboard_segment.percent_span_location ) 
            
            # compute the exposed wing segment area 
            chord_root    = wing_root_chord*inboard_segment.root_chord_percent
            chord_tip     = wing_root_chord*outboard_segment.root_chord_percent 
            taper         = chord_tip/chord_root              
            mac_seg       = chord_root * 2/3 * (( 1 + taper  + taper**2 )/( 1 + taper))
            Sref_seg      = span_seg*(chord_root+chord_tip)*0.5
            if seg_idx == 0:  
                wing_root     = chord_root + percent_span_unexposed*((chord_tip - chord_root)/span_seg)   
                S_exposed_seg = (span_seg-percent_span_unexposed)*(wing_root+chord_tip)*0.5      
            else:   
                S_exposed_seg = Sref_seg
            
            # multiply if wing is symmetric  
            if wing.xz_plane_symmetric:
                Sref_seg      = Sref_seg*2
                S_exposed_seg = S_exposed_seg*2
            
            # compute wetted area of segment
            if t_c_w < 0.05:
                Swet_seg = 2.003* S_exposed_seg
            else:
                Swet_seg = (1.977 + 0.52*t_c_w) * S_exposed_seg
             
            # store segment properties   
            inboard_segment.taper                          = taper 
            inboard_segment.chords.mean_aerodynamic        = mac_seg 
            inboard_segment.areas.reference                = Sref_seg
            inboard_segment.aspect_ratio                   = (span_seg **2) / Sref_seg
            inboard_segment.areas.exposed                  = S_exposed_seg
            inboard_segment.areas.wetted                   = Swet_seg
            total_wetted_area                              += Swet_seg  
           
            # compute wing mean aerodynamic chord  
            MAC = wing.chords.mean_aerodynamic
            if (MAC < chord_root) and   (MAC > chord_tip):
                x_0        = segments[segment_names[seg_idx]].origin[0][0]  +  wing.origin[0][0]
                dy         = ( MAC -  chord_root) / ( (chord_tip - chord_root) / span_seg)
                LEMAC      =  x_0 + np.tan(segments[segment_names[seg_idx]].sweeps.leading_edge) *dy
                wing.LEMAC = LEMAC  
            
            if isinstance(outboard_segment, RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment):
                
                # center body 
                center_body_chord_root    = wing_root_chord*inboard_segment.root_chord_percent  -  wing.aft_center_body.length 
                center_body_chord_tip     = wing_root_chord*outboard_segment.root_chord_percent -  wing.aft_center_body.length  
                center_body_Sref_seg      = span_seg*(center_body_chord_root+center_body_chord_tip)*0.5 
                
                # aft center body 
                aft_center_body_chord_root    = wing.aft_center_body.length
                aft_center_body_chord_tip     = wing.aft_center_body.length
                aft_center_body_Sref_seg      = span_seg*(aft_center_body_chord_root+aft_center_body_chord_tip)*0.5 
               
                # double area if symmetric 
                if wing.xz_plane_symmetric:
                    center_body_Sref_seg = center_body_Sref_seg*2 
                    aft_center_body_Sref_seg = aft_center_body_Sref_seg*2  
                
                center_body_area += center_body_Sref_seg
                aft_center_body_area +=  aft_center_body_Sref_seg 
            total_reference_area += Sref_seg   

    wing.areas.reference   = total_reference_area
    wing.areas.projected   = total_reference_area
    if isinstance(wing,RCAIDE.Library.Components.Wings.Blended_Wing_Body):
        wing.center_body.area     = center_body_area
        wing.aft_center_body.area = aft_center_body_area 

    wing.areas.wetted    = total_wetted_area
        
    return wing
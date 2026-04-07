# RCAIDE/Framework/External_Interfaces/OpenVSP/vsp_boom.py

# Created:  Jun 2018, T. St Francis
# Modified: Aug 2018, T. St Francis
#           Jan 2020, T. MacDonald
#           Jul 2020, E. Botero

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units, Data  
import numpy as np
try:
    import vsp as vsp
except ImportError:
    try:
        import openvsp as vsp
    except ImportError:
        # This allows RCAIDE to build without OpenVSP
        pass
    
# ---------------------------------------------------------------------------------------------------------------------- 
#  vsp read boom
# ---------------------------------------------------------------------------------------------------------------------- 
def read_vsp_boom(b_id,fux_idx,sym_flag, units_type='SI', fineness=True, use_scaling=True):
    """This reads an OpenVSP boom geometry and writes it to a RCAIDE boom format.

    Assumptions:
    1. OpenVSP boom is "conventionally shaped" (generally narrow at nose and tail, wider in center). 
    2. Boom is designed in VSP as it appears in real life. That is, the VSP model does not rely on
       superficial elements such as canopies, stacks, or additional booms to cover up internal lofting oddities.
    3. This program will NOT account for multiple geometries comprising the boom. For example: a wingbox mounted beneath
       is a separate geometry and will NOT be processed.
    4. Boom origin is located at nose. VSP file origin can be located anywhere, preferably at the forward tip
       of the vehicle or in front (to make all X-coordinates of vehicle positive).
    5. Written for OpenVSP 3.21.1

    Source:
    N/A

    Inputs:
    0. Pre-loaded VSP vehicle in memory, via import_vsp_vehicle.
    1. VSP 10-digit geom ID for boom.
    2. Units_type set to 'SI' (default) or 'Imperial'.
    3. Boolean for whether or not to compute boom finenesses (default = True).
    4. Boolean for whether or not to use the scaling from OpenVSP (default = True).

    Outputs:

    Writes RCAIDE boom, with these geometries:           (all defaults are SI, but user may specify Imperial)

    	Booms.Boom.			
    		origin                                  [m] in all three dimensions
    		width                                   [m]
    		lengths.
    		  total                                 [m]
    		  nose                                  [m]
    		  tail                                  [m]
    		heights.
    		  maximum                               [m]
    		  at_quarter_length                     [m]
    		  at_three_quarters_length              [m]
    		effective_diameter                      [m]
    		fineness.nose                           [-] ratio of nose section length to boom effective diameter
    		fineness.tail                           [-] ratio of tail section length to boom effective diameter
    		areas.wetted                            [m^2]
    		tag                                     <string>
    		segment[].   (segments are in ordered container and callable by number)
    		  vsp.shape                               [point,circle,round_rect,general_fuse,fuse_file]
    		  vsp.xsec_id                             <10 digit string>
    		  percent_x_location
    		  percent_z_location
    		  height
    		  width
    		  length
    		  effective_diameter
    		  tag
    		vsp.xsec_num                              <integer of boom segment quantity>
    		vsp.xsec_surf_id                          <10 digit string>

    Properties Used:
    N/A
    """  	
    boom = RCAIDE.Library.Components.Booms.Boom()	

    if units_type == 'SI':
        units_factor = Units.meter * 1.
    elif units_type == 'imperial':
        units_factor = Units.foot * 1.
    elif units_type == 'inches':
        units_factor = Units.inch * 1.	 
     
    if vsp.GetGeomName(b_id):
        boom.tag = vsp.GetGeomName(b_id) + '_' + str(fux_idx+1)
    else: 
        boom.tag = 'BoomGeom' + '_' + str(fux_idx+1)	
        
    if use_scaling:
        scaling       = vsp.GetParmVal(b_id, 'Scale', 'XForm')  
    else:
        scaling       = 1.
    units_factor      = units_factor*scaling

    boom.origin[0][0] = vsp.GetParmVal(b_id, 'X_Location', 'XForm') * units_factor
    boom.origin[0][1] = vsp.GetParmVal(b_id, 'Y_Location', 'XForm') * units_factor*sym_flag
    boom.origin[0][2] = vsp.GetParmVal(b_id, 'Z_Location', 'XForm') * units_factor

    boom.lengths.total         = vsp.GetParmVal(b_id, 'Length', 'Design') * units_factor
    boom.vsp_data.xsec_surf_id = vsp.GetXSecSurf(b_id, 0) 			        # There is only one XSecSurf in geom.
    boom.vsp_data.xsec_num     = vsp.GetNumXSec(boom.vsp_data.xsec_surf_id) 		# Number of xsecs in boom.	 

        
    x_locs    = []
    heights   = []
    widths    = []
    eff_diams = []
    lengths   = []

    # -----------------
    # Boom segments
    # -----------------

    for ii in range(0, boom.vsp_data.xsec_num): 
        # Create the segment
        x_sec                     = vsp.GetXSec(boom.vsp_data.xsec_surf_id, ii) # VSP XSec ID.
        segment                   = RCAIDE.Library.Components.Booms.Segments.Segment()
        segment.vsp_data.xsec_id  = x_sec 
        segment.tag               = 'segment_' + str(ii)

        # Pull out Parms that will be needed
        X_Loc_P = vsp.GetXSecParm(x_sec, 'XLocPercent')
        Z_Loc_P = vsp.GetXSecParm(x_sec, 'ZLocPercent')

        segment.percent_x_location = vsp.GetParmVal(X_Loc_P) # Along boom length.
        segment.percent_z_location = vsp.GetParmVal(Z_Loc_P ) # Vertical deviation of boom center.
        segment.height             = vsp.GetXSecHeight(segment.vsp_data.xsec_id) * units_factor
        segment.width              = vsp.GetXSecWidth(segment.vsp_data.xsec_id) * units_factor
        segment.effective_diameter = (segment.height+segment.width)/2. 

        x_locs.append(segment.percent_x_location)	 # Save into arrays for later computation.
        heights.append(segment.height)
        widths.append(segment.width)
        eff_diams.append(segment.effective_diameter)

        if ii != (boom.vsp_data.xsec_num-1): # Segment length: stored as length since previous segment. (last segment will have length 0.0.)
            next_xsec = vsp.GetXSec(boom.vsp_data.xsec_surf_id, ii+1)
            X_Loc_P_p = vsp.GetXSecParm(next_xsec, 'XLocPercent')
            percent_x_loc_p1 = vsp.GetParmVal(X_Loc_P_p) 
            segment.length = boom.lengths.total*(percent_x_loc_p1 - segment.percent_x_location) * units_factor
        else:
            segment.length = 0.0
        lengths.append(segment.length)

        shape	   = vsp.GetXSecShape(segment.vsp_data.xsec_id)
        shape_dict = {0:'point',1:'circle',2:'ellipse',3:'super ellipse',4:'rounded rectangle',5:'general fuse',6:'fuse file'}
        segment.vsp_data.shape = shape_dict[shape]	

        boom.segments.append(segment)

    boom.heights.at_quarter_length          = get_boom_height(boom, .25)  # Calls get_boom_height function (below).
    boom.heights.at_three_quarters_length   = get_boom_height(boom, .75) 
    boom.heights.at_wing_root_quarter_chord = get_boom_height(boom, .4) 

    boom.heights.maximum    = max(heights)          # Max segment height.	
    boom.width              = max(widths)           # Max segment width.
    boom.effective_diameter = max(eff_diams)        # Max segment effective diam.

    boom.areas.front_projected  = np.pi*((boom.effective_diameter)/2)**2

    eff_diam_gradients_fwd = np.array(eff_diams[1:]) - np.array(eff_diams[:-1])		# Compute gradients of segment effective diameters.
    eff_diam_gradients_fwd = np.multiply(eff_diam_gradients_fwd, lengths[:-1])

    boom = compute_boom_fineness(boom, x_locs, eff_diams, eff_diam_gradients_fwd)	

    return boom

# ---------------------------------------------------------------------------------------------------------------------- 
# Write VSP boom
# ---------------------------------------------------------------------------------------------------------------------- 
def write_vsp_boom(boom,area_tags, OML_set_ind):
    """This writes a boom into OpenVSP format.

    Assumptions:
    None

    Source:
    N/A

    Inputs:
    boom
      width                                   [m]
      lengths.total                           [m]
      heights.
        maximum                               [m]
        at_quarter_length                     [m]
        at_wing_root_quarter_chord            [m]
        at_three_quarters_length              [m]
      effective_diameter                      [m]
      fineness.nose                           [-] ratio of nose section length to boom width
      fineness.tail                           [-] ratio of tail section length to boom width
      tag                                     <string>
      OpenVSP_values.  (optional)
        nose.top.angle                        [degrees]
        nose.top.strength                     [-] this determines how much the specified angle influences that shape
        nose.side.angle                       [degrees]
        nose.side.strength                    [-]
        nose.TB_Sym                           <boolean> determines if top angle is mirrored on bottom
        nose.z_pos                            [-] z position of the nose as a percentage of boom length (.1 is 10%)
        tail.top.angle                        [degrees]
        tail.top.strength                     [-]
        tail.z_pos (optional, 0.02 default)   [-] z position of the tail as a percentage of boom length (.1 is 10%)
      Segments. (optional)
        width                                 [m]
        height                                [m]
        percent_x_location                    [-] .1 is 10% length
        percent_z_location                    [-] .1 is 10% length
    area_tags                                 <dict> used to keep track of all tags needed in wetted area computation           
    main_wing.origin                          [m]
    main_wing.chords.root                     [m]
    fuel_tank_set_index                       <int> OpenVSP object set containing the fuel tanks    

    Outputs:
    Operates on the active OpenVSP model, no direct output

    Properties Used:
    N/A
    """     

    num_segs           = len(boom.segments)
    length             = boom.lengths.total
    fuse_x             = boom.origin[0][0]    
    fuse_y             = boom.origin[0][1]
    fuse_z             = boom.origin[0][2]
    fuse_x_rotation    = boom.x_rotation   
    fuse_y_rotation    = boom.y_rotation
    fuse_z_rotation    = boom.z_rotation
    
    widths  = []
    heights = []
    x_poses = []
    z_poses = []
    segs = boom.segments
    for seg in segs:
        widths.append(seg.width)
        heights.append(seg.height)
        x_poses.append(seg.percent_x_location)
        z_poses.append(seg.percent_z_location)

    end_ind = num_segs-1

    b_id = vsp.AddGeom("BOOM") 
    vsp.SetGeomName(b_id, boom.tag)
    area_tags[boom.tag] = ['booms',boom.tag]

    tail_z_pos = 0.02 # default value

    # set boom relative location and rotation
    vsp.SetParmVal( b_id,'X_Rel_Rotation','XForm',fuse_x_rotation)
    vsp.SetParmVal( b_id,'Y_Rel_Rotation','XForm',fuse_y_rotation)
    vsp.SetParmVal( b_id,'Z_Rel_Rotation','XForm',fuse_z_rotation)

    vsp.SetParmVal( b_id,'X_Rel_Location','XForm',fuse_x)
    vsp.SetParmVal( b_id,'Y_Rel_Location','XForm',fuse_y)
    vsp.SetParmVal( b_id,'Z_Rel_Location','XForm',fuse_z)


    if 'OpenVSP_values' in boom:        
        vals = boom.OpenVSP_values

        # for wave drag testing
        boom.OpenVSP_ID = b_id

        # Nose
        vsp.SetParmVal(b_id,"TopLAngle","XSec_0",vals.nose.top.angle)
        vsp.SetParmVal(b_id,"TopLStrength","XSec_0",vals.nose.top.strength)
        vsp.SetParmVal(b_id,"RightLAngle","XSec_0",vals.nose.side.angle)
        vsp.SetParmVal(b_id,"RightLStrength","XSec_0",vals.nose.side.strength)
        vsp.SetParmVal(b_id,"TBSym","XSec_0",vals.nose.TB_Sym)
        vsp.SetParmVal(b_id,"ZLocPercent","XSec_0",vals.nose.z_pos)
        if not vals.nose.TB_Sym:
            vsp.SetParmVal(b_id,"BottomLAngle","XSec_0",vals.nose.bottom.angle)
            vsp.SetParmVal(b_id,"BottomLStrength","XSec_0",vals.nose.bottom.strength)           
 
        if 'z_pos' in vals.tail:
            tail_z_pos = vals.tail.z_pos
        else:
            pass # use above default

 
    # OpenVSP vals do not exist:
    vals                   = Data()
    vals.nose              = Data()
    vals.tail              = Data()
    vals.tail.top          = Data()

    vals.nose.z_pos        = 0.0
    vals.tail.top.angle    = 0.0
    vals.tail.top.strength = 0.0

    #if len(np.unique(x_poses)) != len(x_poses):
        #raise ValueError('Duplicate boom section positions detected.')
    vsp.SetParmVal(b_id,"Length","Design",length)
    if num_segs != 5: # reduce to only nose and tail
        vsp.CutXSec(b_id,1) # remove extra default section
        vsp.CutXSec(b_id,1) # remove extra default section
        vsp.CutXSec(b_id,1) # remove extra default section
        for i in range(num_segs-2): # add back the required number of sections
            vsp.InsertXSec(b_id, 0, vsp.XS_ELLIPSE)           
            vsp.Update()
    for i in range(num_segs-2):
        # Bunch sections to allow proper length settings in the next step
        # This is necessary because OpenVSP will not move a section past an adjacent section
        vsp.SetParmVal(b_id, "XLocPercent", "XSec_"+str(i+1),1e-6*(i+1))
        vsp.Update()
    if x_poses[1] < (num_segs-2)*1e-6:
        print('Warning: Second boom section is too close to the nose. OpenVSP model may not be accurate.')
    for i in reversed(range(num_segs-2)):
        # order is reversed because sections are initially bunched in the front and cannot be extended passed the next
        vsp.SetParmVal(b_id, "XLocPercent", "XSec_"+str(i+1),x_poses[i+1])
        vsp.SetParmVal(b_id, "ZLocPercent", "XSec_"+str(i+1),z_poses[i+1])
        vsp.SetParmVal(b_id, "Ellipse_Width", "XSecCurve_"+str(i+1), widths[i+1])
        vsp.SetParmVal(b_id, "Ellipse_Height", "XSecCurve_"+str(i+1), heights[i+1])   
        vsp.Update()             
        set_section_angles(i, vals.nose.z_pos, tail_z_pos, x_poses, z_poses, heights, widths,length,end_ind,b_id)            

    vsp.SetParmVal(b_id, "XLocPercent", "XSec_"+str(0),x_poses[0])
    vsp.SetParmVal(b_id, "ZLocPercent", "XSec_"+str(0),z_poses[0])
    vsp.SetParmVal(b_id, "XLocPercent", "XSec_"+str(end_ind),x_poses[-1])
    vsp.SetParmVal(b_id, "ZLocPercent", "XSec_"+str(end_ind),z_poses[-1])    

    # Tail
    if heights[-1] > 0.: 
        pos = len(heights)-1
        vsp.InsertXSec(b_id, pos-1, vsp.XS_ELLIPSE)
        vsp.Update()
        vsp.SetParmVal(b_id, "Ellipse_Width", "XSecCurve_"+str(pos), widths[-1])
        vsp.SetParmVal(b_id, "Ellipse_Height", "XSecCurve_"+str(pos), heights[-1])
        vsp.SetParmVal(b_id, "XLocPercent", "XSec_"+str(pos),x_poses[-1])
        vsp.SetParmVal(b_id, "ZLocPercent", "XSec_"+str(pos),z_poses[-1])              

        xsecsurf = vsp.GetXSecSurf(b_id,0)
        vsp.ChangeXSecShape(xsecsurf,pos+1,vsp.XS_POINT)
        vsp.Update()           
        vsp.SetParmVal(b_id, "XLocPercent", "XSec_"+str(pos+1),x_poses[-1])
        vsp.SetParmVal(b_id, "ZLocPercent", "XSec_"+str(pos+1),z_poses[-1])     

        # update strengths to make end flat
        vsp.SetParmVal(b_id,"TopRStrength","XSec_"+str(pos), 0.)
        vsp.SetParmVal(b_id,"RightRStrength","XSec_"+str(pos), 0.)
        vsp.SetParmVal(b_id,"BottomRStrength","XSec_"+str(pos), 0.)
        vsp.SetParmVal(b_id,"TopLStrength","XSec_"+str(pos+1), 0.)
        vsp.SetParmVal(b_id,"RightLStrength","XSec_"+str(pos+1), 0.)            

    else:
        vsp.SetParmVal(b_id,"TopLAngle","XSec_"+str(end_ind),vals.tail.top.angle)
        vsp.SetParmVal(b_id,"TopLStrength","XSec_"+str(end_ind),vals.tail.top.strength)
        vsp.SetParmVal(b_id,"AllSym","XSec_"+str(end_ind),1)
        vsp.Update()


    if 'z_pos' in vals.tail:
        tail_z_pos = vals.tail.z_pos
    else:
        pass # use above default
    
    vsp.SetSetFlag(b_id, OML_set_ind, True)

    return area_tags

# ---------------------------------------------------------------------------------------------------------------------- 
# set_section_angles 
# ---------------------------------------------------------------------------------------------------------------------- 
def set_section_angles(i,nose_z,tail_z,x_poses,z_poses,heights,widths,length,end_ind,b_id):
    """Set boom section angles to create a smooth (in the non-technical sense) boom shape.
    Note that i of 0 corresponds to the first section that is not the end point.

    Assumptions:
    May fail to give reasonable angles for very irregularly shaped booms
    Does not work on the nose and tail sections.

    Source:
    N/A

    Inputs:  
    nose_z   [-] # 0.1 is 10% of the boom length
    widths   np.array of [m]
    heights  np.array of [m]
    tail_z   [-] # 0.1 is 10% of the boom length

    Outputs:
    Operates on the active OpenVSP model, no direct output

    Properties Used:
    N/A
    """    
    w0 = widths[i]
    h0 = heights[i]
    x0 = x_poses[i]
    z0 = z_poses[i]   
    w2 = widths[i+2]
    h2 = heights[i+2]
    x2 = x_poses[i+2]
    z2 = z_poses[i+2]

    x0 = x0*length
    x2 = x2*length
    z0 = z0*length
    z2 = z2*length

    top_z_diff = (h2/2+z2)-(h0/2+z0)
    bot_z_diff = (z2-h2/2)-(z0-h0/2)
    y_diff     = w2/2-w0/2
    x_diff     = x2-x0

    top_angle  = np.tan(top_z_diff/x_diff)/Units.deg
    bot_angle  = np.tan(-bot_z_diff/x_diff)/Units.deg
    side_angle = np.tan(y_diff/x_diff)/Units.deg

    vsp.SetParmVal(b_id,"TBSym","XSec_"+str(i+1),0)
    vsp.SetParmVal(b_id,"TopLAngle","XSec_"+str(i+1),top_angle)
    vsp.SetParmVal(b_id,"TopLStrength","XSec_"+str(i+1),0.75)
    vsp.SetParmVal(b_id,"BottomLAngle","XSec_"+str(i+1),bot_angle)
    vsp.SetParmVal(b_id,"BottomLStrength","XSec_"+str(i+1),0.75)   
    vsp.SetParmVal(b_id,"RightLAngle","XSec_"+str(i+1),side_angle)
    vsp.SetParmVal(b_id,"RightLStrength","XSec_"+str(i+1),0.75)   

    return  

# ---------------------------------------------------------------------------------------------------------------------- 
# compute_boom_fineness
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_boom_fineness(boom, x_locs, eff_diams, eff_diam_gradients_fwd):
    """This computes boom finenesses for nose and tail.

    Assumptions:
    Written for OpenVSP 3.16.1

    Source:
    N/A

    Inputs:
    0. Pre-loaded VSP vehicle in memory, via import_vsp_vehicle.
    1. RCAIDE boom [object].
    2. Array of x_locations of boom segments. (length = L)
    3. Array of effective diameters of boom segments. (length = L)
    4. Array of effective diameter gradients from nose to tail. (length = L-1)

    Outputs:
    Writes fineness values to RCAIDE boom, returns boom.

    Properties Used:
    N/A
    """

    segment_list       = list(boom.segments.keys())
    
    # Compute nose fineness.    
    x_locs    = np.array(x_locs)					# Make numpy arrays.
    eff_diams = np.array(eff_diams)
    min_val   = np.min(eff_diam_gradients_fwd[x_locs[:-1]<=0.5])	# Computes smallest eff_diam gradient value in front 50% of boom.
    x_loc     = x_locs[:-1][eff_diam_gradients_fwd==min_val][0]		# Determines x-location of the first instance of that value (if gradient=0, Segments[segment_list[0]]ost x-loc).
    boom.lengths.nose  = (x_loc-boom.segments[segment_list[0]].percent_x_location)*boom.lengths.total	# Subtracts first segment x-loc in case not at global origin.
    boom.fineness.nose = boom.lengths.nose/(eff_diams[x_locs==x_loc][0])

    # Compute tail fineness.
    x_locs_tail		    = x_locs>=0.5						# Searches aft 50% of boom.
    eff_diam_gradients_fwd_tail = eff_diam_gradients_fwd[x_locs_tail[1:]]			# Smaller array of tail gradients.
    min_val 		    = np.min(-eff_diam_gradients_fwd_tail)			# Computes min gradient, where boom tapers (minus sign makes positive).
    x_loc = x_locs[np.hstack([False,-eff_diam_gradients_fwd==min_val])][-1]			# Saves aft-most value (useful for straight boom with multiple zero gradients.)
    boom.lengths.tail       = (1.-x_loc)*boom.lengths.total
    boom.fineness.tail      = boom.lengths.tail/(eff_diams[x_locs==x_loc][0])	# Minus sign converts tail fineness to positive value.

    return boom

# ---------------------------------------------------------------------------------------------------------------------- 
# get_boom_height
# ---------------------------------------------------------------------------------------------------------------------- 
def get_boom_height(boom, location):
    """This linearly estimates boom height at any percentage point (0,100) along boom length.

    Assumptions:
    Written for OpenVSP 3.16.1

    Source:
    N/A

    Inputs:
    0. Pre-loaded VSP vehicle in memory, via import_vsp_vehicle.
    1. RCAIDE boom [object], containing boom.vsp_data.xsec_num in its data structure.
    2. boom percentage point [float].

    Outputs:
    height [m]

    Properties Used:
    N/A
    """

    segment_list   = list(boom.segments.keys())       
    for jj in range(1, boom.vsp_data.xsec_num):		# Begin at second section, working toward tail.
        if boom.segments[segment_list[jj]].percent_x_location>=location and boom.segments[segment_list[jj-1]].percent_x_location<location:  
            # Find two sections on either side (or including) the desired boom length percentage.
            a        = boom.segments[segment_list[jj]].percent_x_location							
            b        = boom.segments[segment_list[jj-1]].percent_x_location
            a_height = boom.segments[segment_list[jj]].height		# Linear approximation.
            b_height = boom.segments[segment_list[jj-1]].height
            slope    = (a_height - b_height)/(a-b)
            height   = ((location-b)*(slope)) + (b_height)	
            break
    return height

# ---------------------------------------------------------------------------------------------------------------------- 
# find_fuse_u_coordinate
# ---------------------------------------------------------------------------------------------------------------------- 
def find_fuse_u_coordinate(x_target,b_id,fuel_tank_tag):
    """Determines the u coordinate of an OpenVSP boom that matches an x coordinate

    Assumptions:
    boom is aligned with the x axis

    Source:
    N/A

    Inputs:
    x_target      [m]
    b_id       <str>
    fuel_tank_tag <str>

    Outputs:
    u_current     [-] u coordinate for the requests x position

    Properties Used:
    N/A
    """     
    tol   = 1e-3
    diff  = 1000    
    u_min = 0
    u_max = 1    
    while np.abs(diff) > tol:
        u_current = (u_max+u_min)/2
        probe_id = vsp.AddProbe(b_id,0,u_current,0,fuel_tank_tag+'_probe')
        vsp.Update()
        x_id  = vsp.FindParm(probe_id,'X','Measure')
        x_pos = vsp.GetParmVal(x_id) 
        diff = x_target-x_pos
        if diff > 0:
            u_min = u_current
        else:
            u_max = u_current
        vsp.DelProbe(probe_id)
    return u_current


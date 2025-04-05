# RCAIDE/Framework/External_Interfaces/OpenVSP/import_vsp_vehicle.py

# Created:  Jun 2018, T. St Francis
# Modified: Aug 2018, T. St Francis
#           Jan 2020, T. MacDonald
#           Jul 2020, E. Botero
#           Sep 2021, R. Erhard
#           Dec 2021, E. Botero

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
# RCAIDE imports 
import RCAIDE
from  RCAIDE.Framework.Core                   import Units, Data 
from RCAIDE.Library.Components                import Component
from RCAIDE.Library.Components.Component      import Container 
from RCAIDE.Framework.External_Interfaces.OpenVSP.vsp_rotor           import read_vsp_rotor
from RCAIDE.Framework.External_Interfaces.OpenVSP.vsp_fuselage         import read_vsp_fuselage
from RCAIDE.Framework.External_Interfaces.OpenVSP.vsp_wing             import read_vsp_wing
from RCAIDE.Framework.External_Interfaces.OpenVSP.vsp_nacelle          import read_vsp_nacelle
from RCAIDE.Framework.External_Interfaces.OpenVSP.get_vsp_measurements import get_vsp_measurements

import numpy as np
from copy import deepcopy
import sys
import os

try:
    import vsp as vsp
except ImportError:
    try:
        import openvsp as vsp
    except ImportError:
        # This allows RCAIDE to build without OpenVSP
        pass
    
# ---------------------------------------------------------------------------------------------------------------------- 
#  vsp read 
# ---------------------------------------------------------------------------------------------------------------------- 
def import_vsp_vehicle(tag,main_wing_tag = None, network_type=None, propulsor_type = None, units_type='SI',use_scaling=True,calculate_wetted_area=True): 
    """This reads an OpenVSP vehicle geometry and writes it into a RCAIDE vehicle format.
    Includes wings, fuselages, and rotors.

    Assumptions:
    1. OpenVSP vehicle is composed of conventionally shaped fuselages, wings, and rotors. 
    1a. OpenVSP fuselage: generally narrow at nose and tail, wider in center). 
    1b. Fuselage is designed in VSP as it appears in real life. That is, the VSP model does not rely on
       superficial elements such as canopies, stacks, or additional fuselages to cover up internal lofting oddities.
    1c. This program will NOT account for multiple geometries comprising the fuselage. For example: a wingbox mounted beneath
       is a separate geometry and will NOT be processed.
    2. Fuselage origin is located at nose. VSP file origin can be located anywhere, preferably at the forward tip
       of the vehicle or in front (to make all X-coordinates of vehicle positive). 
    4. Written for OpenVSP 3.21.1

    Source:
    N/A

    Inputs:
    1. A tag for an XML file in format .vsp3.
    2. Units_type set to 'SI' (default) or 'Imperial'
    3. User-specified network
    4. Boolean for whether or not to use the scaling from OpenVSP (default = True).

    Outputs:
    Writes RCAIDE vehicle with these geometries from VSP:    (All values default to SI. Any other 2nd argument outputs Imperial.)
    	Wings.Wing.    (* is all keys)
    		origin                                  [m] in all three dimensions
    		spans.projected                         [m]
    		chords.root                             [m]
    		chords.tip                              [m]
    		aspect_ratio                            [-]
    		sweeps.quarter_chord                    [radians]
    		twists.root                             [radians]
    		twists.tip                              [radians]
    		thickness_to_chord                      [-]
    		dihedral                                [radians]
    		symmetric                               <boolean>
    		tag                                     <string>
    		areas.reference                         [m^2]
    		areas.wetted                            [m^2]
    		Segments.
    		  tag                                   <string>
    		  twist                                 [radians]
    		  percent_span_location                 [-]  .1 is 10%
    		  root_chord_percent                    [-]  .1 is 10%
    		  dihedral_outboard                     [radians]
    		  sweeps.quarter_chord                  [radians]
    		  thickness_to_chord                    [-]
    		  airfoil                               <NACA 4-series, 6 series, or airfoil file>

    	Fuselages.Fuselage.			
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
    		fineness.nose                           [-] ratio of nose section length to fuselage effective diameter
    		fineness.tail                           [-] ratio of tail section length to fuselage effective diameter
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
    		vsp.xsec_num                              <integer of fuselage segment quantity>
    		vsp.xsec_surf_id                          <10 digit string>

    	Propellers.Propeller.
    		location[X,Y,Z]                            [radians]
    		rotation[X,Y,Z]                            [radians]
    		tip_radius                                 [m]
    	        hub_radius                                 [m] 
 
    """  	

    if isinstance(network_type,RCAIDE.Framework.Networks.Network) != True:
        raise Exception('Vehicle energy network type must be defined. \n Choose from list in RCAIDE.Framework.Networks') 

    if isinstance(propulsor_type,RCAIDE.Library.Components.Powertrain.Propulsors.Propulsor ) != True:
        raise Exception('Vehicle propulsor type must be defined. \n Choose from list in RCAIDE.Library.Compoments.Propulsors')     

    # Get the last path from sys.path
    system_path = sys.path[0]
    # Append the system path to the filename
    tag = os.path.join(system_path, tag)


    vsp.ClearVSPModel() 
    
    # Get the last path from sys.path
    system_path = sys.path[-1]
    # Append the system path to the filename
    tag = os.path.join(system_path, tag)
    vsp.ReadVSPFile(tag)	

    vsp_fuselages     = []
    vsp_wings         = []	
    vsp_rotors         = [] 
    vsp_nacelles      = [] 
    vsp_nacelle_type  = []
    
    vsp_geoms         = vsp.FindGeoms()
    geom_names        = []

    vehicle           = RCAIDE.Vehicle()
    vehicle.tag       = tag 

    if units_type == 'SI':
        units_type = 'SI' 
    elif units_type == 'inches':
        units_type = 'inches'	
    else:
        units_type = 'imperial'	

    # The two for-loops below are in anticipation of an OpenVSP API update with a call for GETGEOMTYPE.
    # This print function allows user to enter VSP GeomID manually as first argument in import_vsp_vehicle functions.

    print("VSP geometry IDs: ")	

    # Label each geom type by storing its VSP geom ID. 

    for geom in vsp_geoms: 
        geom_name = vsp.GetGeomName(geom)
        geom_names.append(geom_name)
        print(str(geom_name) + ': ' + geom)
        
    
    # ------------------------------------------------------------------        
    # Use OpenVSP to calculate wetted area
    # ------------------------------------------------------------------
    if calculate_wetted_area:
        measurements = get_vsp_measurements()
        if units_type == 'SI':
            units_factor = Units.meter * 1.
        elif units_type == 'imperial':
            units_factor = Units.foot * 1.
        elif units_type == 'inches':
            units_factor = Units.inch * 1.	         
        
    # ------------------------------------------------------------------
    # AUTOMATIC VSP ENTRY & PROCESSING
    # ------------------------------------------------------------------		

    for geom in vsp_geoms:
        geom_name = vsp.GetGeomName(geom)
        geom_type = vsp.GetGeomTypeName(str(geom))

        if geom_type == 'Fuselage':
            vsp_fuselages.append(geom)
        if geom_type == 'Wing':
            vsp_wings.append(geom)
        if geom_type == 'Propeller':
            vsp_rotors.append(geom) 
        if (geom_type == 'Stack') or (geom_type == 'BodyOfRevolution'):
            vsp_nacelle_type.append(geom_type)
            vsp_nacelles.append(geom) 
        
    # ------------------------------------------------------------------			
    # Read Fuselages 
    # ------------------------------------------------------------------			    
    for fuselage_id in vsp_fuselages:
        sym_planar = vsp.GetParmVal(fuselage_id, 'Sym_Planar_Flag', 'Sym') # Check for symmetry
        sym_origin = vsp.GetParmVal(fuselage_id, 'Sym_Ancestor_Origin_Flag', 'Sym') 
        if sym_planar == 2. and sym_origin == 1.:  
            num_fus  = 2 
            sym_flag = [1,-1]
        else: 
            num_fus  = 1 
            sym_flag = [1] 
        for fux_idx in range(num_fus):	# loop through fuselages on aircraft 
            fuselage = read_vsp_fuselage(fuselage_id,fux_idx,sym_flag[fux_idx],units_type,use_scaling)
            
            if calculate_wetted_area:
                fuselage.areas.wetted = measurements[vsp.GetGeomName(fuselage_id)] * (units_factor**2)
            
            vehicle.append_component(fuselage)
        
    # ------------------------------------------------------------------			    
    # Read Wings 
    # ------------------------------------------------------------------			
    for wing_id in vsp_wings:
        wing = read_vsp_wing(wing_id, main_wing_tag, units_type,use_scaling)
        if calculate_wetted_area:
            wing.areas.wetted = measurements[vsp.GetGeomName(wing_id)] * (units_factor**2)  
        vehicle.append_component(wing)		 
        
    # ------------------------------------------------------------------			    
    # Read Enegy Network 
    # ------------------------------------------------------------------
    network =  deepcopy(network_type)  
    i       = 0
    # Condition when equal number of rotors and nacelles are defined 
    if len(vsp_rotors) == len(vsp_nacelles):
        for idx , (rotor_id,nacelle_id) in enumerate(zip(vsp_rotors, vsp_nacelles)):
            # define new propulsor 
            propulsor = deepcopy(propulsor_type) 
            propulsor.tag =  'propulsor_' +  str(i+1)
            
            # Rotor 
            rotor           = read_vsp_rotor(rotor_id,units_type)
            rotor.tag       = vsp.GetGeomName(rotor_id) 
            propulsor.rotor = rotor
            
            # Nacelle 
            nacelle = read_vsp_nacelle(nacelle_id,vsp_nacelle_type[idx], units_type)
            if calculate_wetted_area:
                nacelle.areas.wetted = measurements[vsp.GetGeomName(nacelle_id)] * (units_factor**2)           
            propulsor.nacelle = nacelle          
             
            # Append to Network 
            network.propulsors.append(propulsor)
            
            
            # If symmetry is defined
            if vsp.GetParmVal(rotor_id, 'Sym_Planar_Flag', 'Sym')== 2.0:
                # update index 
                i += 1
                
                # define new propulsor 
                propulsor =  deepcopy(propulsor_type) 
                propulsor.tag =  'propulsor_' +  str(i+1)
                
                
                # Rotor 
                rotor_sym = deepcopy(rotor)
                rotor_sym.origin[0][1] = - rotor_sym.origin[0][1] 
                propulsor.rotor = rotor_sym
                
        
                # Nacelle 
                nacelle_sym = deepcopy(nacelle)
                nacelle_sym.origin[0][1] = - nacelle_sym.origin[0][1]  
                nacelle_sym.areas.wetted = nacelle.areas.wetted
                propulsor.nacelle = nacelle          
                 
                # Append to Network             
                network.propulsors.append(propulsor)
                
    # Condition when only nacelles are defined 
    elif (len(vsp_rotors) ==  0) and (len(vsp_nacelles) > 0):

        for idx ,  nacelle_id in enumerate(vsp_nacelles):
            # define new propulsor 
            propulsor = deepcopy(propulsor_type) 
            propulsor.tag =  'propulsor_' +  str(i+1)
             
            # Nacelle 
            nacelle = read_vsp_nacelle(nacelle_id,vsp_nacelle_type[idx], units_type)
            if calculate_wetted_area:
                nacelle.areas.wetted = measurements[vsp.GetGeomName(nacelle_id)] * (units_factor**2)           
            propulsor.nacelle = nacelle          
             
            # Append to Network 
            network.propulsors.append(propulsor)
            
            
            # If symmetry is defined
            if vsp.GetParmVal(nacelle_id, 'Sym_Planar_Flag', 'Sym')== 2.0:
                # update index 
                i += 1
                
                # define new propulsor 
                propulsor =  deepcopy(propulsor_type) 
                propulsor.tag =  'propulsor_' +  str(i+1)
                 
        
                # Nacelle 
                nacelle_sym = deepcopy(nacelle)
                nacelle_sym.origin[0][1] *= -1
                nacelle_sym.areas.wetted = nacelle.areas.wetted
                propulsor.nacelle = nacelle_sym          
                 
                # Append to Network             
                network.propulsors.append(propulsor)

    # Condition when only rotors are defined 
    elif (len(vsp_rotors) >  0) and (len(vsp_nacelles) == 0):
        for idx , rotor_id in enumerate( vsp_rotors):
            # define new propulsor 
            propulsor = deepcopy(propulsor_type) 
            propulsor.tag =  'propulsor_' +  str(i+1)
            
            # Rotor 
            rotor           = read_vsp_rotor(rotor_id,units_type)
            rotor.tag       = vsp.GetGeomName(rotor_id) 
            propulsor.rotor = rotor
            
            # Append to Network 
            network.propulsors.append(propulsor)
            
            
            # If symmetry is defined
            if vsp.GetParmVal(rotor_id, 'Sym_Planar_Flag', 'Sym')== 2.0:
                # update index 
                i += 1
                
                # define new propulsor 
                propulsor =  deepcopy(propulsor_type) 
                propulsor.tag =  'propulsor_' +  str(i+1)
                
                
                # Rotor 
                rotor_sym = deepcopy(rotor)
                rotor_sym.origin[0][1] = - rotor_sym.origin[0][1] 
                propulsor.rotor = rotor_sym  
                propulsor.nacelle = nacelle          
                 
                # Append to Network             
                network.propulsors.append(propulsor) 
            
    else:
        print ('Unequal numbers of rotors and nacelles defined. Skipping propulsor definition.') 
                
    vehicle.networks.append(network)
   
    
    # get origin of fuselage
    vsp_origin = 0
    for fuselage in vehicle.fuselages:
        vsp_origin = np.minimum(vsp_origin, fuselage.origin[0][0])
        
    # shift all compoments to new origin
    origin_shift =  -vsp_origin
    

    for network in vehicle.networks:
        for p_tag, p_item in network.items():
            update_origin(p_item,origin_shift)
            
    for wing in vehicle.wings:
        update_origin(wing,origin_shift) 
    
    for boom in vehicle.booms:
        update_origin(boom,origin_shift)
    
    for fuselage in vehicle.fuselages:
        update_origin(fuselage,origin_shift)   
        
    return vehicle

def update_origin(p_item,origin_shift):  
    if isinstance(p_item,Component): 
        p_item.origin[0][0] += origin_shift 
        for p_sub_tag, p_sub_item in p_item.items():
            update_origin(p_sub_item,origin_shift)    
    elif isinstance(p_item,Container): 
        for p_sub_tag, p_sub_item in p_item.items():
            update_origin(p_sub_item,origin_shift)

# RCAIDE/Framework/External_Interfaces/OpenVSP/run_vsp_mesh.py
# 
# Created:  Aug 2025, W. Zheng

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
try:
    import vsp as vsp
except ImportError:
    try:
        import openvsp as vsp
    except ImportError:
        # This allows RCAIDE to build without OpenVSP
        pass
    
import fileinput
from RCAIDE.Framework.External_Interfaces.OpenVSP.write_vsp_mesh import set_sources

# ----------------------------------------------------------------------------------------------------------------------
#  run_vsp_mesh
# ----------------------------------------------------------------------------------------------------------------------  
def run_vsp_mesh(geom,vsp_file, minedge,maxedge, sym=False, 
                 farfield_scale= 2.0, farfield= False,source=False):
    if sym:
        f = fileinput.input(vsp_file,inplace=1)
        for line in f:
            if 'SymmetrySplitting' in line:
                print(line[0:34] + '1' + line[35:-1])
            else:
                print(line)
    vsp.ClearVSPModel()
    vsp.ReadVSPFile(vsp_file)
    
    #Add in a function to modify all wing-like geometry and BOR to finite trailing edge
    geom_ids = vsp.FindGeoms()
    for geom_id in geom_ids:
        geom_type = vsp.GetGeomTypeName(geom_id)
        if geom_type in ["Wing"]:
            geom_name = vsp.GetGeomName(geom_id)
            print(f"Processing {geom_name} ({geom_type})")
            subsurf_ids = vsp.GetSubSurfIDVec(geom_id)
            for subsurf_id in subsurf_ids:
                vsp.DeleteSubSurf(geom_id, subsurf_id)
                print(f"   Deleted subsurface {subsurf_id}")
            # Get number of sections
            xsecsurf_id = vsp.GetXSecSurf(geom_id, 0)
            num_sections = vsp.GetNumXSec(xsecsurf_id)

            # for i in range(num_sections):
            #     group = f"Close_{i}"

            #     # TE closure type (3 = SKEW_BOTH)
            #     te_type = vsp.GetParm(geom_id, "TE_Close_Type", group)
            #     vsp.SetParmVal(te_type, vsp.CLOSE_SKEWBOTH)
            #     #ensure relative thickness is used
            #     te_absrel = vsp.GetParm(geom_id, "TE_Close_AbsRel", group)
            #     vsp.SetParmVal(te_absrel, 0)
            #     # TE thickness
            #     te_thick = vsp.GetParm(geom_id, "TE_Close_Thick", group)
            #     vsp.SetParmVal(te_thick, minedge * 2)
            # print(f"  Applied TE closure settings to all {num_sections} sections.")
        # if geom_type in ["BodyOfRevolution"]:
        #     geom_name = vsp.GetGeomName(geom_id)
        #     print(f"Processing {geom_name} ({geom_type})")

        #     # Get number of sections
        #     xsecsurf_id = vsp.GetXSecSurf(geom_id, 0)
        #     num_sections = vsp.GetNumXSec(xsecsurf_id)

            
        #     group = "Close"

        #     # TE closure type (3 = SKEW_BOTH)
        #     te_type = vsp.GetParm(geom_id, "TE_Close_Type", group)
        #     vsp.SetParmVal(te_type, vsp.CLOSE_SKEWBOTH)
        #     #ensure relative thickness is used
        #     te_absrel = vsp.GetParm(geom_id, "TE_Close_AbsRel", group)
        #     vsp.SetParmVal(te_absrel, 0)
        #     # TE thickness
        #     te_thick = vsp.GetParm(geom_id, "TE_Close_Thick", group)
        #     vsp.SetParmVal(te_thick, 0.01)
                

        #    print(f"  Applied TE closure settings to all {num_sections} sections.")

    vsp.Update()
    print("Trailing edge settings updated for all wings and bodies of revolution.")
    if source==True:
        set_sources(geom)
    vehicle_cont = vsp.FindContainer('Vehicle',0)
    STL_multi    = vsp.FindParm(vehicle_cont, 'MultiSolid', 'STLSettings')
    vsp.SetParmVal(STL_multi, 1.0)
    vsp.SetCFDMeshVal(vsp.CFD_MIN_EDGE_LEN, minedge )
    vsp.SetCFDMeshVal( vsp.CFD_MAX_EDGE_LEN, maxedge)
    vsp.SetCFDMeshVal( vsp.CFD_GROWTH_RATIO , 1.1 )
    #vsp.SetCFDMeshVal(vsp.CFD_MAX_GAP, maxedge*0.01 )
    
    vsp.SetCFDMeshVal( vsp.CFD_HALF_MESH_FLAG , sym)
    vsp.SetCFDMeshVal( vsp.CFD_FAR_FIELD_FLAG  , farfield)
    if farfield:
        vsp.SetCFDMeshVal( vsp.CFD_FAR_MAX_EDGE_LEN , farfield_scale/2)
        vsp.SetCFDMeshVal( vsp.CFD_FAR_X_SCALE  , farfield_scale)
        vsp.SetCFDMeshVal( vsp.CFD_FAR_Y_SCALE  , farfield_scale)
        vsp.SetCFDMeshVal( vsp.CFD_FAR_Z_SCALE  , 2*farfield_scale)  
    vsp.SetCFDMeshVal( vsp.CFD_INTERSECT_SUBSURFACE_FLAG , True)
    vsp.SetComputationFileName(vsp.CFD_STL_FILE_NAME ,vsp_file)
    vsp.Update()
    vsp.ComputeCFDMesh(vsp.SET_ALL,vsp.SET_NONE,vsp.CFD_STL_TYPE)
    
    return 
    
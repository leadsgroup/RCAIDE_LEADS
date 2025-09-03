# SU2.py
#     
# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# package imports  
from RCAIDE.Framework.Core import Data ,  Units
from RCAIDE.Framework.External_Interfaces.SU2.generate_SU2_Euler_cfg import generate_SU2_Euler_cfg 
import os
import subprocess  

# ----------------------------------------------------------------------
#  SU2
# ----------------------------------------------------------------------
def SU2(conditions,settings,geometry):
  
    # unpack settings---------------------------------------------------------------- 
    S_ref      = geometry.reference_area
    num_procs  = settings.number_of_processors
    cfg_file   = settings.SU2_config_filename

    # unpack geometry----------------------------------------------------------------
    # define point about which moment coefficient is computed
    if 'main_wing' in geometry.wings:
        c_bar      = geometry.wings['main_wing'].chords.mean_aerodynamic
        x_mac      = geometry.wings['main_wing'].aerodynamic_center[0] + geometry.wings['main_wing'].origin[0][0]
        z_mac      = geometry.wings['main_wing'].aerodynamic_center[2] + geometry.wings['main_wing'].origin[0][2]
        b_ref      = geometry.wings['main_wing'].spans.projected
    else:
        c_bar  = 0.
        x_mac  = 0.
        b_ref = 0.
        for wing in geometry.wings:
            if wing.vertical == False:
                if c_bar <= wing.chords.mean_aerodynamic:
                    c_bar  = wing.chords.mean_aerodynamic
                    x_mac  = wing.aerodynamic_center[0] + wing.origin[0][0]
                    z_mac  = wing.aerodynamic_center[2] + wing.origin[0][2]
                    b_ref  = wing.spans.projected

    x_cg       = geometry.mass_properties.center_of_gravity[0][0]
    z_cg       = geometry.mass_properties.center_of_gravity[0][2]
    if x_cg == 0.0:
        x_m = x_mac 
        z_m = z_mac
    else:
        x_m = x_cg
        z_m = z_cg
         
    aoa      = conditions.aerodynamics.angles.alpha      
    beta     = conditions.aerodynamics.angles.beta       
    mach     = conditions.freestream.mach_number         
    temp     = conditions.freestream.temperature         
    pressure = conditions.freestream.pressure   
    len_mach = len(mach)
    
    """Run SU2 for a range of angles of attack and collect CL, CD, CMz."""
    
    generate_SU2_Euler_cfg(
        "boundary_marker.txt",
        settings.SU2_config_filename,
        settings.SU2_filename,
        mach[0][0],  # need to update 
        0,
        sideslip_angle=beta[0][0],  # need to update 
        freestream_pressure=pressure[0][0],
        freestream_temperature=temp[0][0], 
        ref_origin=(x_mac, 0.0, z_mac),  # need to update 
        ref_length=c_bar,   # need to update 
        ref_area=S_ref,    # need to update 
        ref_dimensionality="FREESTREAM_VEL_EQ_ONE",
        sym=False,
        restart=False
    )  

    SU2_results = Data()
    for wing in geometry.wings:
        SU2_results[wing.tag] = []
    SU2_results['CL_Total'] = []
    SU2_results['CD_Total'] = []
    SU2_results['CM_Total'] = []

    for i in range(len_mach):
            if aoa[i][0] ==aoa[0][0]:
                restart=False
            else:
                restart=True
            #restart = i > 0  # Restart from the second case onwards
            modify_SU2_cfg(cfg_file, aoa[i][0]/Units.degrees, mach[i][0], restart)
            # Run SU2 with MPI
            try: 
                command = ["mpiexec", "-n", str(num_procs), "SU2_CFD", cfg_file] # Parallel
                process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            except:
                command = ["SU2_CFD", cfg_file] # Serial
                process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            # Stream the output to terminal in real-time
            for line in iter(process.stdout.readline, ""):
                print(line, end="")  # Print line by line without extra newlines

            process.stdout.close()
            process.wait()  # Ensure SU2_CFD finishes before proceeding
        
            # Extract aerodynamic coefficients
            for wing in geometry.wings:
                _, _, _, clw, cdw  = extract_SU2_forces("forces_breakdown.dat",wing.tag)
                SU2_results[wing.tag].append([mach[i][0],aoa[i][0], clw,cdw])

            cl, cd, cmz, clw, cdw  = extract_SU2_forces("forces_breakdown.dat",wing.tag)
            SU2_results['CL_Total'].append(cl)
            SU2_results['CD_Total'].append(-cd)
            SU2_results['CM_Total'].append(cmz)
    # ---------------------------------------------------------------------------------------
    # Pack outputs
    # ------------------ --------------------------------------------------------------------
    
    results =  Data()    
    results.S_ref             = S_ref 
    results.b_ref             = b_ref
    results.c_ref             = c_bar  
    results.X_ref             = x_m
    results.Y_ref             = 0
    results.Z_ref             = z_m 
    results.CLift             = SU2_results['CL_Total']
    results.CDift             = SU2_results['CD_Total']
    results.CLift_wings       = Data()
    for wing in geometry.wings:
        results.CLift_wings[wing.tag] = [x[2] for x in SU2_results[wing.tag]] # make it wing tag independed later 
    results.CDrag_induced_wings = Data()
    for wing in geometry.wings:
        results.CDrag_induced_wings[wing.tag] = [x[3] for x in SU2_results[wing.tag]]
    results.CX                = 0
    results.CY                = 0
    results.CZ                = 0
    results.CL                = 0
    results.CM                = SU2_results['CM_Total']
    results.CN                = 0
    results.chord_sections    = 0
    results.spanwise_stations = 0
    results.CLift_wing        = 0
    results.sectional_CLift   = 0
    results.CP                = 0
    results.gamma             = 0
    results.VD                = 0
    results.V_distribution    = 0
    results.V_x               = 0
    results.V_z               = 0
    
    return results
     

def modify_SU2_cfg(cfg_file, aoa, mach, restart):
    """Modify the SU2 configuration file for a given angle of attack and restart setting."""
    with open(cfg_file, 'r') as file:
        lines = file.readlines()
    
    with open(cfg_file, 'w') as file:
        for line in lines:
            if line.startswith('AOA='):
                file.write(f'AOA=  {aoa}\n')
            elif line.startswith('MACH_NUMBER='):
                file.write(f'MACH_NUMBER= {mach}\n')
            elif line.startswith('RESTART_SOL'):
                file.write(f'RESTART_SOL= {"YES" if restart else "NO"}\n')
            else:
                file.write(line)

def extract_SU2_forces(filename,tag):
    """Extract CL, CD, and CMz from forces_breakdown.dat using string splitting."""
    cl, cd, cmz = None, None, None
    clw,cdw = None,None
    if not os.path.exists(filename):
        print(f"Warning: {filename} not found!")
        return cl, cd, cmz
    tagflag=False
    with open(filename, 'r') as file:
        for line in file:
            if line.startswith("Total CL:"):
                parts = line.split("|")
                cl = float(parts[0].split(":")[-1].strip())
            elif line.startswith("Total CD:"):
                parts = line.split("|")
                cd = float(parts[0].split(":")[-1].strip())
            elif line.startswith("Total CMz:"):
                parts = line.split("|")
                cmz = float(parts[0].split(":")[-1].strip())
            elif line.startswith(f"Surface name: {tag}"):
                tagflag=True
            elif tagflag and line.startswith("Total CL "):
                parts = line.split("|")
                clw = float(parts[0].split(":")[-1].strip())
            elif tagflag and line.startswith("Total CD"):
                parts = line.split("|")
                cdw = float(parts[0].split(":")[-1].strip())
            elif line.startswith("Surface name") and not line.startswith(f"Surface name: {tag}"):
                tagflag=False
    return cl, cd, cmz, clw, cdw 
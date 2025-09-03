# RCAIDE/Framework/External_Interfaces/SU2/generate_SU2_Euler_cfg.py
# 
# Created:  Aug 2025, W. Zheng

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  

import os
import shutil

# ----------------------------------------------------------------------------------------------------------------------
#  generate_SU2_Euler_cfg
# ----------------------------------------------------------------------------------------------------------------------  
def generate_SU2_Euler_cfg(boundary_marker_path,
                           output_cfg_path,
                           mesh_filename,
                           mach_number,
                           aoa,
                           sideslip_angle=0,
                           freestream_pressure=101325,
                           freestream_temperature=288.15,
                           ref_origin=(0.0, 0.0, 0.0),
                           ref_length=1.0,
                           ref_area=0.0,
                           ref_dimensionality="FREESTREAM_VEL_EQ_ONE",
                           sym=False,
                           restart=False):
    workdir = "__SU2_WORKDIR__"

    # === Purge if exists ===
    if os.path.exists(workdir):
        print(f"Purging existing {workdir}...")
        shutil.rmtree(workdir)

    # === Create fresh workdir ===
    os.makedirs(workdir)
    print(f"Created {workdir}.")

    # === Move files into workdir ===
    for file_path in [boundary_marker_path, mesh_filename]:
        if os.path.exists(file_path):
            shutil.move(file_path, os.path.join(workdir, os.path.basename(file_path)))
            print(f"Moved {file_path} -> {workdir}/")
        else:
            print(f"Warning: {file_path} does not exist and cannot be moved.")

    # === Change working directory ===
    os.chdir(workdir)
    
    
    # === Parse boundary_marker.txt ===
    euler_markers = []
    farfield_found = False

    with open(boundary_marker_path, 'r') as f:
        for line in f:
            if "Assigned Physical Surface" in line:
                parts = line.split("'")
                marker_name = parts[1]
                if marker_name == "FarField":
                    farfield_found = True
                else:
                    euler_markers.append(marker_name)

    cfg_lines = []

    # ---------- HEADER ----------
    cfg_lines.append("% -------------- SU2_EULER CONFIGURATION FILE --------------\n\n")

    # ---------- SOLVER DEFINITION ----------
    cfg_lines.append("SOLVER= EULER\n")
    cfg_lines.append("MATH_PROBLEM= DIRECT\n")
    cfg_lines.append(f"RESTART_SOL= {'YES' if restart else 'NO'}\n")
    cfg_lines.append("\n")

    # ---------- FREESTREAM CONDITIONS ----------
    cfg_lines.append(f"MACH_NUMBER= {mach_number}\n")
    cfg_lines.append(f"AOA= {aoa}\n")
    cfg_lines.append(f"SIDESLIP_ANGLE= {sideslip_angle}\n")
    cfg_lines.append(f"FREESTREAM_PRESSURE= {freestream_pressure}\n")
    cfg_lines.append(f"FREESTREAM_TEMPERATURE= {freestream_temperature}\n")
    cfg_lines.append("GAMMA_VALUE= 1.4\n")
    cfg_lines.append("GAS_CONSTANT= 287.87\n")
    cfg_lines.append("\n")

    # ---------- REFERENCE VALUES ----------
    cfg_lines.append(f"REF_ORIGIN_MOMENT_X= {ref_origin[0]}\n")
    cfg_lines.append(f"REF_ORIGIN_MOMENT_Y= {ref_origin[1]}\n")
    cfg_lines.append(f"REF_ORIGIN_MOMENT_Z= {ref_origin[2]}\n")
    cfg_lines.append(f"REF_LENGTH= {ref_length}\n")
    cfg_lines.append(f"REF_AREA= {ref_area}\n")
    cfg_lines.append(f"REF_DIMENSIONALIZATION= {ref_dimensionality}\n")
    cfg_lines.append("\n")

    # ---------- NUMERICAL METHOD ----------
    cfg_lines.append("NUM_METHOD_GRAD= WEIGHTED_LEAST_SQUARES\n")
    cfg_lines.append("OBJECTIVE_FUNCTION= DRAG\n")
    cfg_lines.append("CFL_NUMBER= 25.0\n")
    cfg_lines.append("CFL_ADAPT= YES\n")
    cfg_lines.append("CFL_ADAPT_PARAM= ( 0.1, 2.0, 20, 1e10 )\n")
    cfg_lines.append("RK_ALPHA_COEFF= ( 0.66667, 0.66667, 1.000000 )\n")
    cfg_lines.append("ITER= 9999\n")
    cfg_lines.append("LINEAR_SOLVER= FGMRES\n")
    cfg_lines.append("LINEAR_SOLVER_PREC= ILU\n")
    cfg_lines.append("LINEAR_SOLVER_ERROR= 1E-6\n")
    cfg_lines.append("LINEAR_SOLVER_ITER= 10\n")
    cfg_lines.append("\n")

    # ---------- CONVECTIVE SCHEME ----------
    cfg_lines.append("CONV_NUM_METHOD_FLOW= JST\n")
    cfg_lines.append("JST_SENSOR_COEFF= ( 0.5, 0.02 )\n")
    cfg_lines.append("TIME_DISCRE_FLOW= EULER_IMPLICIT\n")
    cfg_lines.append("\n")

    # ---------- CONVERGENCE ----------
    cfg_lines.append("CONV_FIELD= RMS_DENSITY\n")
    cfg_lines.append("CONV_RESIDUAL_MINVAL= -6\n")
    cfg_lines.append("CONV_STARTITER= 25\n")
    cfg_lines.append("CONV_CAUCHY_ELEMS= 100\n")
    cfg_lines.append("CONV_CAUCHY_EPS= 1E-10\n")
    cfg_lines.append("\n")

    # ---------- MARKERS ----------
    if euler_markers:
        cfg_lines.append(f"MARKER_EULER= ( {', '.join(euler_markers)} )\n")
        cfg_lines.append(f"MARKER_PLOTTING= ( {', '.join(euler_markers)} )\n")
        cfg_lines.append(f"MARKER_MONITORING= ( {', '.join(euler_markers)} )\n")
    if farfield_found:
        cfg_lines.append("MARKER_FAR= ( FarField)\n")
    if sym:
        cfg_lines.append("MARKER_SYM= ( SYMMETRY_FACE )\n")
    else:
        cfg_lines.append("% MARKER_SYM= ( SYMMETRY_FACE )\n")
    cfg_lines.append("\n")


    # ---------- MESH AND OUTPUT ----------
    cfg_lines.append(f"MESH_FILENAME= {mesh_filename}\n")
    cfg_lines.append("MESH_OUT_FILENAME= mesh_out.su2\n")
    cfg_lines.append("SOLUTION_FILENAME= solution_flow.dat\n")
    cfg_lines.append("SOLUTION_ADJ_FILENAME= solution_adj.dat\n")
    cfg_lines.append("RESTART_FILENAME= solution_flow.dat\n")
    cfg_lines.append("RESTART_ADJ_FILENAME= solution_adj.dat\n")
    cfg_lines.append("VOLUME_FILENAME= flow\n")
    cfg_lines.append("VOLUME_ADJ_FILENAME= adjoint\n")
    cfg_lines.append("GRAD_OBJFUNC_FILENAME= of_grad.dat\n")
    cfg_lines.append("SURFACE_FILENAME= surface_floW\n")
    cfg_lines.append("TABULAR_FORMAT= CSV\n")
    cfg_lines.append("OUTPUT_WRT_FREQ= 1000\n")
    cfg_lines.append("\n")

    # ---------- RESTART CONTROL ----------
    
    
    
    cfg_lines.append("\n")


    # ---------- OUTPUT VARIABLES ----------
    cfg_lines.append("WRT_FORCES_BREAKDOWN= YES\n")
    cfg_lines.append("\n")


    cfg_lines.append("% -------------- END OF CONFIGURATION FILE --------------\n")

    # === Write to cfg file ===
    with open(output_cfg_path, 'w') as f_out:
        f_out.writelines(cfg_lines)

    print(f"Generated SU2_EULER cfg at: {output_cfg_path}")
    
    return 
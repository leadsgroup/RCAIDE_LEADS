# RCAIDE/Library/Methods/Aerodynamics/Athena_Vortex_Lattice/write_avl_airfoil_file.py
#
# Created: Oct 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE 
from RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.purge_files  import purge_files
from RCAIDE.Library.Methods.Geometry.Airfoil.import_airfoil_geometry        import import_airfoil_geometry 

# python imports
import os

# ----------------------------------------------------------------------------------------------------------------------
#  write_avl_airfoil_file
# ---------------------------------------------------------------------------------------------------------------------- 
def write_avl_airfoil_file(rcaide_airfoil_filename):
    """
    Converts RCAIDE airfoil coordinate files to AVL-compatible format for vortex lattice analysis.

    Parameters
    ----------
    rcaide_airfoil_filename : str
        Path to RCAIDE airfoil coordinate file in standard format
        File should contain airfoil name on first line followed by x,y coordinate pairs

    Returns
    -------
    avl_airfoil_filename : str
        Generated AVL airfoil filename with .dat extension

    Notes
    -----
    This function translates airfoil coordinate files from the standard airfoil
    tools format used by RCAIDE to the specific format required by AVL. The
    conversion process includes coordinate reformatting and proper spacing
    adjustments for AVL compatibility.

    The function automatically generates the output filename by extracting the
    base name from the input file and appending the .dat extension. Any existing
    files with the same name are automatically purged before writing new data.

    Coordinate formatting includes proper tab spacing and decimal precision
    optimized for AVL's input parser. Negative y-coordinates (lower surface)
    receive different spacing treatment than positive coordinates (upper surface).

    The function skips the midpoint coordinate (typically the trailing edge)
    to avoid duplicate points that can cause AVL parsing issues.

    **Major Assumptions**
        * Input file follows standard airfoil tools format
        * First line contains airfoil name or identifier
        * Coordinates are normalized (0 ≤ x ≤ 1)
        * Coordinate ordering starts from trailing edge, goes over upper surface to leading edge, then under lower surface back to trailing edge
        * File contains valid floating-point coordinate data

    See Also
    --------
    RCAIDE.Library.Methods.Geometry.Airfoil.import_airfoil_geometry : Function for reading airfoil coordinate data
    RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.purge_files : Utility function for cleaning up existing files
    """       
    

    # unpack avl_inputs
    avl_airfoil_filename =  rcaide_airfoil_filename.split(".")[-2].split("/")[-1] + '.dat'
    
    # purge file 
    purge_files([avl_airfoil_filename]) 
    
    # read airfoil file header 
    origin  = os.getcwd()
    os_path = os.path.split(origin)[0]
    f_path  = rcaide_airfoil_filename
    f = open(f_path)  
    data_block = f.readlines() 
    f.close()   
    airfoil_name = data_block[0].strip()
    
    # import airfoil coordinates 
    airfoil_geometry_data = import_airfoil_geometry(f_path)
    dim = len(airfoil_geometry_data.x_coordinates)
              
    # write file  
    with open(avl_airfoil_filename,'w') as afile:
            afile.write(airfoil_name + "\n")  
            for i in range(dim - 1):
                if i == int(dim/2):
                    pass  
                elif airfoil_geometry_data.y_coordinates[i] < 0.0:
                    case_text = '\t' + format(airfoil_geometry_data.x_coordinates[i], '.7f')+ "   " + format(airfoil_geometry_data.y_coordinates[i], '.7f') + "\n" 
                    afile.write(case_text)
                else:   
                    case_text = '\t' + format(airfoil_geometry_data.x_coordinates[i], '.7f')+ "    " + format(airfoil_geometry_data.y_coordinates[i], '.7f') + "\n" 
                    afile.write(case_text)
    afile.close()
    return avl_airfoil_filename 
 

# generate_airfoil_transitions.py
# 
# Created:  March 2021, R. Erhard
# Modified:  

from RCAIDE.Library.Methods.Geometry.Airfoil.generate_interpolated_airfoils  import generate_interpolated_airfoils
from RCAIDE.Library.Methods.Geometry.Airfoil.import_airfoil_geometry         import  import_airfoil_geometry
from RCAIDE.Library.Plots.Geometry import plot_airfoil
import os
import sys
import numpy as np
import matplotlib.pyplot as plt 

def main():

    separator     = os.path.sep 
    if  os.path.split(sys.path[0])[1] == 'geometry':
        airfoils_path =  os.path.split(os.path.split(sys.path[0])[0])[0] + separator +  'Vehicles' + os.path.sep + 'Airfoils' + os.path.sep
    else:
        airfoils_path = sys.path[0] + separator +  'Vehicles' + os.path.sep + 'Airfoils' + os.path.sep
    a_labels      = ["Clark_y", "E63"]
    nairfoils     = 4   # number of total airfoils
    
    a1            = airfoils_path + a_labels[0]+ ".txt"
    a2            = airfoils_path + a_labels[1]+ ".txt"
    airfoil_files = generate_interpolated_airfoils(a1, a2, nairfoils,npoints=101,save_filename="Transition")

    for airfoil_file in airfoil_files:
        name = os.path.basename(airfoil_file)
        plot_airfoil(airfoil_file,save_filename = name[:-4])  
        
    # import the new airfoil geometries and compare to the regression:
    airfoil_data_1   = import_airfoil_geometry(airfoil_files[1],npoints=101)
    airfoil_data_2   = import_airfoil_geometry(airfoil_files[2],npoints=101)    
    airfoil_data_1_r = import_airfoil_geometry("Transition1_regression.txt",npoints=101)
    airfoil_data_2_r = import_airfoil_geometry("Transition2_regression.txt",npoints=101)
    
    # ensure coordinates are the same:   
    assert( max(abs(airfoil_data_1.x_coordinates - airfoil_data_1_r.x_coordinates)) < 1e-5)
    assert( max(abs(airfoil_data_2.x_coordinates - airfoil_data_2_r.x_coordinates)) < 1e-5)
    assert( max(abs(airfoil_data_1.y_coordinates - airfoil_data_1_r.y_coordinates)) < 1e-5)
    assert( max(abs(airfoil_data_2.y_coordinates - airfoil_data_2_r.y_coordinates)) < 1e-5)
    
    return


if __name__ == "__main__":
    main()
    plt.show()
# test_take_off_field_length.py
#
# Created: Dec 2024, M Clarke   

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# RCAIDE Imports
import RCAIDE
from RCAIDE.Framework.Core   import Data,Units 
from RCAIDE.Library.Methods.Performance.estimate_take_off_field_length import estimate_take_off_field_length

# package imports
import numpy as np
import pylab as plt 
import sys
import os
import numpy as np
from  copy import  deepcopy

# import vehicle file
base_dir = os.path.dirname(os.path.abspath(__file__))

vehicles_path = os.path.abspath(
    os.path.join(base_dir, "..", "..", "Vehicles")
)

if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)
from Embraer_190 import vehicle_setup, configs_setup

# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------
def main():

    # define vehicle 
    vehicle   = vehicle_setup()   
  
    # Set up vehicle configs
    configs  = configs_setup(vehicle)

    # create analyses
    analyses = analyses_setup(configs) 
            
    takeoff_field_length ,second_seg_clb_grad  =  estimate_take_off_field_length(analyses= analyses.takeoff,
                                                                                 compute_2nd_seg_climb = True)

<<<<<<< HEAD
    takeoff_weight=vehicle.mass_properties.takeoff             
    print('Weight (kg): ',takeoff_weight)
    
    print('Takeoff field length (m): ', takeoff_field_length)    
    truth_TOFL = 1178.071961920313
    TOFL_error = np.max(np.abs(takeoff_field_length-truth_TOFL)) 
    assert(TOFL_error<1e-6)
=======
        for fuel_line in  network.fuel_lines: 
            fuel_line.assigned_propulsors = []           
    
    for id_eng,engine_number in enumerate(engines):
        propulsor_list = []
        # append propulsors 
        for network in  configuration.networks:
            for i in  range(engine_number):
                propulsor =  deepcopy(baseline_propulsor)
                propulsor.tag = 'propulsor_' +  str(i+1)
                network.propulsors.append(propulsor)
                propulsor_list.append(propulsor.tag) 
                
            for fuel_line in  network.fuel_lines:  
                fuel_line.assigned_propulsors =  [propulsor_list]
                
        for id_w,weight in enumerate(w_vec):
            configuration.mass_properties.takeoff = weight
            takeoff_field_length[id_w,id_eng],second_seg_clb_grad[id_w,id_eng] =  estimate_take_off_field_length(configuration,analyses,compute_2nd_seg_climb = True)
    
        # delete propulsors again 
        for propulsor in  network.propulsors: 
            baseline_propulsor = deepcopy(propulsor) 
            del network.propulsors[propulsor.tag] 
    
        for fuel_line in  network.fuel_lines: 
            fuel_line.assigned_propulsors = []                       
        
    truth_TOFL =  np.array([[ 798.53848007,  535.74840837,  389.19055604],
       [ 836.39913349,  558.69048478,  405.69702767],
       [ 875.69438164,  582.44181867,  422.76902537],
       [ 916.44292727,  607.00793181,  440.40890368],
       [ 958.66406687,  632.39452126,  458.61909191],
       [1002.37769068,  658.60745936,  477.40209408],
       [1047.60428266,  685.65279377,  496.76048898],
       [1094.36492054,  713.53674744,  516.69693015],
       [1142.68127575,  742.2657186 ,  537.21414587],
       [1192.57561349,  771.84628077,  558.31493916]])
    
    print(' takeoff_field_length = ',  takeoff_field_length)
    print(' second_seg_clb_grad  = ', second_seg_clb_grad)                      
                             
    truth_clb_grad =  np.array([[0.2447345 , 0.57880427, 0.9135685 ],
       [0.23432311, 0.55733283, 0.88131709],
       [0.22422833, 0.53718876, 0.85106376],
       [0.21473671, 0.51825249, 0.82262823],
       [0.2057956 , 0.50041838, 0.79585127],
       [0.19735829, 0.48359277, 0.77059173],
       [0.18938323, 0.46769235, 0.74672404],
       [0.18183331, 0.45264269, 0.72413612],
       [0.1746753 , 0.43837719, 0.70272758],
       [0.16787935, 0.424836  , 0.68240832]])


    TOFL_error = np.max(np.abs(truth_TOFL-takeoff_field_length)/truth_TOFL)                           
    GRAD_error = np.max(np.abs(truth_clb_grad-second_seg_clb_grad)/truth_clb_grad)
    
    print('Maximum Take OFF Field Length Error= %.4e' % TOFL_error)
    print('Second Segment Climb Gradient Error= %.4e' % GRAD_error)    
    
    title = "TOFL vs W"
    plt.figure(1); 
    plt.plot(w_vec,takeoff_field_length[:,0], 'k-', label = '2 Engines')
    plt.plot(w_vec,takeoff_field_length[:,1], 'r-', label = '3 Engines')
    plt.plot(w_vec,takeoff_field_length[:,2], 'b-', label = '4 Engines')

    plt.title(title); plt.grid(True)
    plt.plot(w_vec,truth_TOFL[:,0], 'k--o', label = '2 Engines [truth]')
    plt.plot(w_vec,truth_TOFL[:,1], 'r--o', label = '3 Engines [truth]')
    plt.plot(w_vec,truth_TOFL[:,2], 'b--o', label = '4 Engines [truth]')
    legend = plt.legend(loc='lower right')
    plt.xlabel('Weight (kg)')
    plt.ylabel('Takeoff field length (m)')    
    
    title = "2nd Segment Climb Gradient vs W"
    plt.figure(2); 
    plt.plot(w_vec,second_seg_clb_grad[:,0], 'k-', label = '2 Engines')
    plt.plot(w_vec,second_seg_clb_grad[:,1], 'r-', label = '3 Engines')
    plt.plot(w_vec,second_seg_clb_grad[:,2], 'b-', label = '4 Engines')

    plt.title(title); plt.grid(True)
    plt.plot(w_vec,truth_clb_grad[:,0], 'k--o', label = '2 Engines [truth]')
    plt.plot(w_vec,truth_clb_grad[:,1], 'r--o', label = '3 Engines [truth]')
    plt.plot(w_vec,truth_clb_grad[:,2], 'b--o', label = '4 Engines [truth]')
    legend = plt.legend(loc='lower right')
    plt.xlabel('Weight (kg)')
    plt.ylabel('Second Segment Climb Gradient (%)')    
    
    assert( TOFL_error   < 1e-6 )
    assert( GRAD_error   < 1e-6 )
>>>>>>> 6f04b72118c5f63837bcb8fbde3ed393ec4a9466

    truth_SSCG = 0.1681179781570795
    print('Second Segment Climb Gradient (%): ', second_seg_clb_grad)
    SSCG_error = np.max(np.abs(second_seg_clb_grad-truth_SSCG))
    assert(SSCG_error<1e-6)
    
    return 
    


def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses 

def base_analysis(vehicle):
    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle() 
    analyses.vehicle =  vehicle
    
    #  Geometry
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()   
    analyses.append(geometry)

<<<<<<< HEAD
     # ------------------------------------------------------------------
    #  Weights 
    weights = RCAIDE.Framework.Analyses.Weights.Conventional_Transport()    
    analyses.append(weights)
=======
    # ------------------------------------------------------------------   
    #  Aerodynamics Analysis
    aerodynamics         = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.settings.number_of_spanwise_vortices    = 10 # reducing the number of vortices to speed up the test 
    aerodynamics.settings.number_of_chordwise_vortices   = 5  # reducing the number of vortices to speed up the test 
    analyses.append(aerodynamics)
>>>>>>> 6f04b72118c5f63837bcb8fbde3ed393ec4a9466

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis  
    aerodynamics          = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.settings.maximum_lift_coefficient_factor = 0.90
    analyses.append(aerodynamics) 

    # ------------------------------------------------------------------
    #  Energy
    energy          = RCAIDE.Framework.Analyses.Energy.Energy() 
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)   

    # done!
    return analyses 


# ----------------------------------------------------------------------        
#   Call Main
# ----------------------------------------------------------------------    

if __name__ == '__main__':
    main()
    plt.show()
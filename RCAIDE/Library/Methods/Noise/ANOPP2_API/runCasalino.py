from RCAIDE.Framework.Core                                import Data
from RCAIDE.Library.Methods.Noise.ANOPP2_API.CompactF1A   import CompactF1A

# Local imports
from CasalinoRotorGeometry import Casalino_Rotor
from CasalinoRunAerodynamics import Run_Aero
from DCode.Common.plottingFunctions import plot2dDiscPerformance

import numpy as np
import os

import pickle
import pandas as pd
import pylab as plt
import os

def run_rotor_simulation(f, a, run_aero=True):
    """
    Runs the full aerodynamic and aeroacoustic simulation.
    """

    # ------------------------------------------------------------------------------------------------------------
    # Setup geometry and file locations
    # ------------------------------------------------------------------------------------------------------------    
    base_loc = os.path.dirname(__file__)
    data_loc = base_loc + "/data/" 
    filename = data_loc+"/aero_F{}_A{}.pkl"

    rotor = Casalino_Rotor(fidelity=f, plot_figures=False)     
    # ------------------------------------------------------------------------------------------------------------        
    # Get rotor aerodynamics 
    # ------------------------------------------------------------------------------------------------------------   
    if run_aero:
        inputs = Data()
        inputs.Alpha_deg = a
        inputs.J = 0.6
        inputs.V = 20.
        rotor, conditions = Run_Aero(rotor, inputs)
        
        # store outputs
        data = Data()
        data.rotor = rotor
        data.conditions = conditions
        with open(filename,'wb') as file:
            pickle.dump(data,file)        
        
    else:
        with open(filename, "rb") as file:
            data = pickle.load(file)   
            rotor = data.rotor
            conditions = data.conditions

    # ------------------------------------------------------------------------------------------------------------    
    # Plot rotor aerodynamic performance
    # ------------------------------------------------------------------------------------------------------------     

    plot2dDiscPerformance(rotor, rotor.outputs.disc_radial_distribution, rotor.outputs.disc_azimuthal_distribution, 
                        rotor.outputs.disc_thrust_distribution, "Thrust", cpt=0, levels=100, save_name=base_loc+"/plots/A{}_Thrust".format(a))     

    plot2dDiscPerformance(rotor, rotor.outputs.disc_radial_distribution, rotor.outputs.disc_azimuthal_distribution, 
                        rotor.outputs.disc_torque_distribution, "Torque", cpt=0, levels=100, save_name=base_loc+"/plots/A{}_Torque".format(a))      

    # ------------------------------------------------------------------------------------------------------------
    # Run acoustics
    # ------------------------------------------------------------------------------------------------------------    
    problem = CompactF1A()
    problem.evaluate(rotor, conditions)
    

    # post-process plot the pressure time history
    totalPressureFile = os.path.dirname(__file__) + "/Total.out.dat"
    data = pd.read_table(totalPressureFile,skiprows=4, sep="\s+", names=['ObserverTime','AcousticPressure'])
    plt.figure("PTH")
    plt.plot(data.ObserverTime, data.AcousticPressure, label="F{}, ".format(f)+'$\\alpha=$'+str(a)+"$^\\circ$")

    # post-process plot the power spectral density
    totalPressureFile = os.path.dirname(__file__) + "/Psd.apth.pa.out.dat"
    data = pd.read_table(totalPressureFile, sep="\s+", names=['Frequency','PSD'])
    plt.figure("PSD")
    plt.plot(data.Frequency, data.PSD, label="F{}, ".format(f)+'$\\alpha=$'+str(a)+"$^\\circ$")   

    return

def main():
    # get rotor geometry
    fidelities=[0,1]

    alphas = np.array([85.]) #0., 30., 60., 80.])

    for f in fidelities:
        for a in alphas:
            run_rotor_simulation(f, a)


    plt.figure("PTH")
    plt.xlabel("Observer Time")
    plt.ylabel("Acoustic Pressure (Pa)")
    plt.legend()
    plt.tight_layout()


    plt.figure("PSD")
    plt.xlabel("Frequency")
    plt.ylabel("PSD $(Pa^2/Hz)$") # plt.ylabel("PSD ($Pa^2/Hz$)")
    plt.xscale("log")
    plt.legend()
    plt.tight_layout()

    plt.show()
    return

if __name__ == '__main__':
    main()

import numpy as np
import matplotlib.pyplot as plt

# Try to import RCAIDE components. 
# Ensure this script is run in an environment where RCAIDE and your jet_noise.py module are accessible.
try:
    from RCAIDE.Framework.Core import Units, Data
    # Adjust this import to match the location of your compute_jet_noise_new function
    from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.compute_jet_noise_new import compute_jet_noise_new
except ImportError:
    print("Warning: RCAIDE or the jet_noise module could not be imported. Ensure your path is correct.")

# =============================================================================
# Helper class to mock RCAIDE's nested Data structures for validation
# =============================================================================
class MockData:
    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)

# =============================================================================
# Setup Input Parameters (From image_df279a.png)
# =============================================================================
def setup_validation_data():
    
    # 1. Turbofan Geometry (Table 5-1: Top Nozzle Configuration)
    turbofan = MockData(
        core_nozzle = MockData(diameter=0.095), # [m]
        fan_nozzle  = MockData(diameter=0.200), # [m]
        origin      = [[0.0, 0.0, 2.0]],        # Assuming a 2m engine height
        length      = 2.0,                      # Mock length [m]
        diameter    = 0.5,                      # Mock engine diameter [m]
        plug_diameter = 0.0,                    # 0.0 for the top nozzle without plug
        geometry_xe = 0.0,                      # Mock installation effect params
        geometry_ye = 0.0,
        geometry_Ce = 0.0
    )
    
    # 2. Aeroacoustic Data / Operating Conditions (Table 5-2: Condition #1 Isothermal)
    # Note: Ttp and Tts are assumed to be the stagnation (total) temperatures
    aeroacoustic_data = MockData(
        fan = MockData(angular_velocity=5000 * Units.rpm), # Mock RPM
        core_nozzle = MockData(
            exit_velocity=217.2,                           # V_p [m/s]
            exit_stagnation_temperature=311.1,             # T_tp [K]
            exit_stagnation_pressure=101325 + 20000        # Mock Pressure [Pa]
        ),
        fan_nozzle = MockData(
            exit_velocity=216.8,                           # V_s [m/s]
            exit_stagnation_temperature=310.2,             # T_ts [K]
            exit_stagnation_pressure=101325 + 15000        # Mock Pressure [Pa]
        )
    )
    
    # 3. Flight Segment Conditions (Static test assumed)
    segment = MockData(
        conditions = MockData(
            freestream = MockData(
                velocity=0.0,                              # Static test V_aircraft = 0
                mach_number=0.0,
                speed_of_sound=343.0,                      # [m/s]
                density=1.225,                             # [kg/m^3]
                pressure=101325                            # [Pa]
            ),
            aerodynamics = MockData(
                angles = MockData(alpha = 0.0 * Units.deg) # AOA
            ),
            frames = MockData(
                inertial = MockData(time=np.array([0.0]))  # Single time step
            )
        )
    )
    
    # 4. Settings (1/3 Octave Band Center Frequencies)
    # The function uses settings.center_frequencies[5:]. 
    # To start at 100 Hz, we structure the array such that index 5 is 100 Hz.
    freqs = [31.5, 40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500, 
             630, 800, 1000, 1250, 1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000, 10000]
    settings = MockData(center_frequencies=np.array(freqs))
    
    # 5. Microphone Locations
    # Assume a standard microphone placed 13 meters away at a 90-degree angle
    microphone_locations = np.array([[0.0, 13.0, 0.0]])
    
    return microphone_locations, turbofan, aeroacoustic_data, segment, settings

# =============================================================================
# Run Validation and Plot (Matching image_df273a.png)
# =============================================================================
def run_and_plot():
    # Setup inputs
    mic_locs, turbofan, aero_data, segment, settings = setup_validation_data()
    
    # Run the model
    print("Running jet noise prediction...")
    engine_noise = compute_jet_noise_new(mic_locs, turbofan, aero_data, segment, settings)
    
    # Extract SPL spectrum for the first time step and first microphone
    # engine_noise.SPL_1_3_spectrum shape is (n_cpts, n_mic, n_freq)
    spl_spectrum = engine_noise.SPL_1_3_spectrum[0, 0, :]
    frequencies = settings.center_frequencies[:]
    
    # Plotting
    print("Generating plot...")
    plt.figure(figsize=(10, 6))
    
    # Plot line with markers to match the style of image_df273a.png
    plt.plot(frequencies, spl_spectrum, 'k-o', linewidth=2.5, markerfacecolor='w', 
             markeredgecolor='k', markersize=6, label='Model Output')
    
    # Formatting axes to match the reference image exactly
    plt.xscale('log')
    
    # Tick formatting
    plt.ylim([70,120])
    plt.xlim([100,10000])

    plt.title('13m at 90')
    
    # Grid formatting
    plt.grid(True, which='both', linestyle='--', alpha=0.6)
    
    # Labels
    plt.xlabel('Frequency (Hz)', fontweight='bold', fontsize=12)
    plt.ylabel('Sound Pressure Level (dB)', fontweight='bold', fontsize=12)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_and_plot()
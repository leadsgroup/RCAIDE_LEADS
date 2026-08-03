import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.tri as tri
from scipy.interpolate import griddata

def main():
    # ---------------------------------------------------------
    # 1. Load Ground Truth Data
    # ---------------------------------------------------------
    truth_file = '/Users/siripunn/Desktop/LEADS_WORK/LEADS_Research/RCAIDE_LEADS/VnV/Verification/analysis_aeroacoustics/b737_sim_noise_SEL_N_TR (1).csv'
    truth_df = pd.read_csv(truth_file)
    
    # NOTE: Update 'SEL' below to the exact column name in your CSV if it differs
    lon_truth = truth_df['Longitude (deg)'].values
    lat_truth = truth_df['Latitude (deg)'].values
    sel_truth = truth_df['Noise Level (dB)'].values 

    # ---------------------------------------------------------
    # 2. Load Simulated Data
    # ---------------------------------------------------------
    # Loading the compressed file saved from the previous step
    sim_data = np.load('/Users/siripunn/Desktop/LEADS_WORK/LEADS_Research/RCAIDE_LEADS/VnV/Verification/analysis_aeroacoustics/b737_high_res_footprint_no_inf.npz')
    lon_sim = sim_data['longitude']
    lat_sim = sim_data['latitude']
    sel_sim = sim_data['sel_dBA']

    # ---------------------------------------------------------
    # 3. Align Data Grids
    # ---------------------------------------------------------
    # Interpolate simulated data onto the exact grid coordinates of the ground truth
    # This prevents matrix mismatch errors if the CSVs are sorted differently
    sel_sim_aligned = griddata((lon_sim, lat_sim), sel_sim, (lon_truth, lat_truth), method='nearest')

    # ---------------------------------------------------------
    # 4. Decibel Arithmetic
    # ---------------------------------------------------------
    # Option A: Standard Level Difference (Over/Under Prediction)
    delta_sel = sel_sim_aligned - sel_truth

    # Option B: Energy Difference (Uncomment if you need absolute energy delta)
    # energy_diff = np.abs(10**(sel_sim_aligned / 10.0) - 10**(sel_truth / 10.0))
    # delta_sel = 10 * np.log10(np.where(energy_diff > 0, energy_diff, 1e-12))

    # ---------------------------------------------------------
    # 5. Plotting the Difference Contour
    # ---------------------------------------------------------
    fig, ax = plt.subplots(figsize=(10, 8), dpi=120)
    
    # Create triangulation for the truth grid
    triangulation = tri.Triangulation(lon_truth, lat_truth)
    
    # Define a diverging colormap centered at zero for easy error identification
    # Setting the range from -15 dB (under-prediction) to +15 dB (over-prediction)
    levels = np.linspace(-15, 15, 31) 
    
    # 'coolwarm' shows blue for negative (under) and red for positive (over)
    heatmap = ax.tricontourf(triangulation, delta_sel, levels=levels, cmap='jet', extend='both')
    
    # Format the colorbar
    cbar = fig.colorbar(heatmap, ax=ax)
    cbar.set_label('Δ SEL (Simulated - Truth) [dBA]', fontsize=12, fontweight='bold')
    
    # Set titles and labels
    ax.set_title('B737 Noise Footprint Error Profile', fontsize=14, fontweight='bold', pad=15)
    ax.set_xlabel('Longitude', fontsize=12)
    ax.set_ylabel('Latitude', fontsize=12)
    
    # Format axes 
    ax.grid(True, linestyle='--', alpha=0.5, color='gray')
    
    # Maintain proportional geographic scaling
    mean_lat = np.mean(lat_truth)
    ax.set_aspect(1.0 / np.cos(np.radians(mean_lat)))
    
    plt.tight_layout()
    plt.show()

    # Print summary statistics to the console
    print("--- Error Statistics ---")
    print(f"Mean Error: {np.mean(delta_sel):.2f} dBA")
    print(f"Max Over-prediction: {np.max(delta_sel):.2f} dBA")
    print(f"Max Under-prediction: {np.min(delta_sel):.2f} dBA")
    print(f"Mean Absolute Error (MAE): {np.mean(np.abs(delta_sel)):.2f} dBA")

if __name__ == '__main__':
    main()
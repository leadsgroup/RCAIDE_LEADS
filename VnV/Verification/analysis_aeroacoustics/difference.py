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
    
    print("Loading Truth Data...")
    truth_df = pd.read_csv(truth_file)
    
    # Drop rows with missing values to prevent NaNs from breaking the arrays
    truth_df = truth_df.dropna(subset=['Longitude (deg)', 'Latitude (deg)', 'Noise Level (dB)'])
    
    # Extract and force to 1D flat arrays
    lon_truth = truth_df['Longitude (deg)'].values.flatten()
    lat_truth = truth_df['Latitude (deg)'].values.flatten()
    sel_truth = truth_df['Noise Level (dB)'].values.flatten()
    
    print(f"Truth points available: {len(sel_truth)}")

    # ---------------------------------------------------------
    # 2. Load Simulated Data
    # ---------------------------------------------------------
    sim_file = '/Users/siripunn/Desktop/LEADS_WORK/LEADS_Research/RCAIDE_LEADS/VnV/Verification/analysis_aeroacoustics/b737_high_res_footprint_3.npz'
    print("Loading Simulated Data...")
    sim_data = np.load(sim_file)

    # Flatten everything to 1D arrays
    lon_sim = sim_data['longitude'].flatten()
    lat_sim = sim_data['latitude'].flatten()
    sel_sim = sim_data['sel_dBA'].flatten()
    
    # Combine (x, y) coordinates into a strict (N, 2) shaped matrix
    points_sim = np.column_stack((lon_sim, lat_sim))
    points_truth = np.column_stack((lon_truth, lat_truth))

    # Run the nearest-neighbor interpolation
    sel_sim_aligned = griddata(
        points_sim, 
        sel_sim, 
        points_truth, 
        method='nearest'
    )

    # Clean any potential NaN/Inf values that might have occurred
    valid_mask = np.isfinite(sel_sim_aligned) & np.isfinite(sel_truth)

    plt.figure(figsize=(8, 6))
    plt.scatter(lon_sim, lat_sim, c=sel_sim, cmap='jet', s=10)
    plt.colorbar(label='Simulated SEL (dBA)')
    plt.title("DEBUG: Raw Simulated Data")
    plt.show()
    
    lon_truth = lon_truth[valid_mask]
    lat_truth = lat_truth[valid_mask]
    sel_truth = sel_truth[valid_mask]
    sel_sim_aligned = sel_sim_aligned[valid_mask]
    
    print(f"Successfully aligned {len(sel_sim_aligned)} grid points.")

    # ---------------------------------------------------------
    # 4. Decibel Arithmetic
    # ---------------------------------------------------------
    # Level Difference (Over / Under Prediction)
    delta_sel = sel_sim_aligned - sel_truth

    # ---------------------------------------------------------
    # 5. Plotting the Difference Contour
    # ---------------------------------------------------------
    print("Generating Error Contour Plot...")
    fig, ax = plt.subplots(figsize=(10, 8), dpi=120)
    
    triangulation = tri.Triangulation(lon_truth, lat_truth)
    
    # Scale from -15 dB (under-prediction) to +15 dB (over-prediction)
    levels = np.linspace(np.min(delta_sel),np.max(delta_sel), 51) 
    
    heatmap = ax.tricontourf(triangulation, delta_sel, levels=levels, cmap='coolwarm', extend='both')
    
    # Format colorbar
    cbar = fig.colorbar(heatmap, ax=ax)
    cbar.set_label('Δ SEL (Simulated - Truth) [dBA]', fontsize=12, fontweight='bold')
    
    # Set titles and labels
    ax.set_title('B737 Noise Footprint Error Profile', fontsize=14, fontweight='bold', pad=15)
    ax.set_xlabel('Longitude', fontsize=12)
    ax.set_ylabel('Latitude', fontsize=12)
    
    ax.grid(True, linestyle='--', alpha=0.5, color='gray')
    
    # Proportional geographic scaling
    mean_lat = np.mean(lat_truth)
    ax.set_aspect(1.0 / np.cos(np.radians(mean_lat)))
    
    plt.tight_layout()
    plt.show()

    # ---------------------------------------------------------
    # 6. Summary Statistics
    # ---------------------------------------------------------
    print("\n--- Error Statistics ---")
    print(f"Mean Error: {np.mean(delta_sel):.2f} dBA")
    print(f"Max Over-prediction: {np.max(delta_sel):.2f} dBA")
    print(f"Max Under-prediction: {np.min(delta_sel):.2f} dBA")
    print(f"Mean Absolute Error (MAE): {np.mean(np.abs(delta_sel)):.2f} dBA")

if __name__ == '__main__':
    main()
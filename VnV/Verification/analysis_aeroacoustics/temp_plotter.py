import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.tri as tri

def plot_noise_heatmap(csv_filename):
    # 1. Load the data
    try:
        df = pd.read_csv(csv_filename)
        print(f"Successfully loaded '{csv_filename}'.")
    except FileNotFoundError:
        print(f"Error: Could not find '{csv_filename}'. Ensure it's in the same directory.")
        return

    # 2. Extract specific columns based on the known CSV structure
    x_col = 'Longitude'
    y_col = 'Latitude'
    z_col = 'dB'
    
    print(f"Using columns -> X: '{x_col}', Y: '{y_col}', Noise: '{z_col}'")

    # Extract arrays
    x = df[x_col].values
    y = df[y_col].values
    z = df[z_col].values

    # 3. Setup the plot
    fig, ax = plt.subplots(figsize=(10, 8), dpi=120)

    # 4. Create an unstructured triangulation grid and plot the heatmap
    triangulation = tri.Triangulation(x, y)
    
    # Generate the filled contour (heatmap)
    # 'jet' is a standard colormap for aeroacoustic footprints
    levels = np.linspace(np.min(z), np.max(z), 40) # 40 smooth color transitions
    heatmap = ax.tricontourf(triangulation, z, levels=levels, cmap='jet', extend='both')

    # 5. Add colorbar and labels
    cbar = fig.colorbar(heatmap, ax=ax)
    cbar.set_label(f'{z_col} Level (Exposure)', fontsize=12, fontweight='bold')

    ax.set_title('B737 Simulated Noise Footprint', fontsize=14, fontweight='bold', pad=15)
    ax.set_xlabel('Longitude', fontsize=12)
    ax.set_ylabel('Latitude', fontsize=12)
    
    # Format axes with a subtle grid
    ax.grid(True, linestyle='--', alpha=0.5, color='gray')
    
    # Keep the geographic spatial scales proportional based on the center latitude
    # This approximates a Mercator projection so the map isn't stretched
    mean_lat = np.mean(y)
    ax.set_aspect(1.0 / np.cos(np.radians(mean_lat)))

    # 6. Display the plot
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_noise_heatmap("/Users/siripunn/Desktop/LEADS_WORK/LEADS_Research/RCAIDE_LEADS/VnV/Verification/analysis_aeroacoustics/b737_sim_noise.csv")
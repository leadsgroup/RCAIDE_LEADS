import pandas as pd
import numpy as np

def interpolate_flight_segment(input_csv, output_csv, start_lon, end_lon, num_new_points=10):
    """
    Reads a flight track CSV, finds the segment between start_lon and end_lon,
    interpolates 10 new points linearly for Latitude, Longitude, and Altitude,
    and saves the updated track to a new CSV.
    """
    try:
        df = pd.read_csv(input_csv)
        print(f"Successfully loaded '{input_csv}'.")
    except FileNotFoundError:
        print(f"Error: Could not find '{input_csv}'. Ensure it is in the same directory.")
        return

    # 1. Identify the boundary points in the dataset
    # Assuming the track is ordered sequentially. We need to find the rows that most
    # closely match the provided start and end longitudes.
    
    # Calculate absolute differences to find the closest indices
    start_idx = (np.abs(df['Longitude (deg)'] - start_lon)).argmin()
    end_idx = (np.abs(df['Longitude (deg)'] - end_lon)).argmin()
    
    # Ensure start_idx is before end_idx in the dataframe
    if start_idx > end_idx:
        start_idx, end_idx = end_idx, start_idx

    print(f"Interpolating between Index {start_idx} (Lon: {df.loc[start_idx, 'Longitude (deg)']}) "
          f"and Index {end_idx} (Lon: {df.loc[end_idx, 'Longitude (deg)']})")

    # 2. Extract the boundary values
    start_row = df.loc[start_idx]
    end_row = df.loc[end_idx]

    # 3. Generate the interpolated values
    # We use num_new_points + 2 to include the start and end points in the linspace,
    # then slice [1:-1] to get only the new intermediate points.
    interp_lons = np.linspace(start_row['Longitude (deg)'], end_row['Longitude (deg)'], num_new_points + 2)[1:-1]
    interp_lats = np.linspace(start_row['Latitude (deg)'], end_row['Latitude (deg)'], num_new_points + 2)[1:-1]
    interp_alts = np.linspace(start_row['Altitude MSL (ft)'], end_row['Altitude MSL (ft)'], num_new_points + 2)[1:-1]

    # 4. Create a DataFrame for the new points
    # Copy the structure of the original dataframe
    new_rows = pd.DataFrame(columns=df.columns)
    new_rows['Longitude (deg)'] = interp_lons
    new_rows['Latitude (deg)'] = interp_lats
    new_rows['Altitude MSL (ft)'] = interp_alts
    
    # Fill other non-important columns (like speed, thrust) with NaN or copy from start_row
    # Here, we'll just backfill them from the start_row for continuity, though you mentioned
    # they are not important.
    for col in df.columns:
        if col not in ['Longitude (deg)', 'Latitude (deg)', 'Altitude MSL (ft)']:
            new_rows[col] = start_row[col]

    # 5. Insert the new points into the original dataframe
    # Split the original dataframe, insert the new rows, and concatenate
    df_top = df.iloc[:start_idx + 1]
    df_bottom = df.iloc[start_idx + 1:]
    
    df_updated = pd.concat([df_top, new_rows, df_bottom]).reset_index(drop=True)

    # 6. Save the new dataframe
    df_updated.to_csv(output_csv, index=False)
    print(f"Successfully added {num_new_points} interpolated points.")
    print(f"Updated track saved to '{output_csv}'.")

if __name__ == "__main__":
    # Define your target longitudes based on your request
    target_start_lon = -87.80712032
    target_end_lon = -87.89908325
    
    input_file = "/Users/siripunn/Desktop/LEADS_WORK/LEADS_Research/RCAIDE_LEADS/VnV/Verification/analysis_aeroacoustics/b737_sim_track.csv"
    output_file = "/Users/siripunn/Desktop/LEADS_WORK/LEADS_Research/RCAIDE_LEADS/VnV/Verification/analysis_aeroacoustics/b737_sim_track_interpolated.csv"
    
    interpolate_flight_segment(input_file, output_file, target_start_lon, target_end_lon, num_new_points=20)

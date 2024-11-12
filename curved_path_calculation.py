import numpy as np
import matplotlib.pyplot as plt
from geographiclib.geodesic import Geodesic

# Constants and Configurations
LAT_A, LON_A = 49.64, 6.14  # Coordinates of Point A
LAT_B, LON_B = 50.02, 4.93  # Coordinates of Point B
MAX_TURN_RATE = 3           # Maximum heading change rate in degrees per step
POINT_INTERVAL_METERS = 300  # Interval between points in meters
geod = Geodesic.WGS84        # WGS84 model for geodesic calculations

def calculate_path_segment(start_lat, start_lon, start_heading, end_lat, end_lon, end_heading):
    """
    Generates a path segment with heading adjustment towards a target heading.
    """
    total_distance = geod.Inverse(start_lat, start_lon, end_lat, end_lon)['s12']
    n_points = max(1, int(total_distance / POINT_INTERVAL_METERS))
    step_distance = total_distance / (2 * n_points)
    
    lats, lons = [start_lat], [start_lon]
    heading = start_heading
    for _ in range(n_points):
        g = geod.Inverse(lats[-1], lons[-1], end_lat, end_lon)
        target_heading = g['azi1']
        
        # Adjust heading gradually towards the target heading
        heading_change = (target_heading - heading + 180) % 360 - 180
        heading += min(MAX_TURN_RATE, abs(heading_change)) * np.sign(heading_change)
        
        # Calculate the next point in the path
        new_position = geod.Direct(lats[-1], lons[-1], heading, step_distance)
        lats.append(new_position['lat2'])
        lons.append(new_position['lon2'])
        
        # End segment if the heading is close enough to the final target heading
        if abs(heading - end_heading) < 0.1:
            break
            
    return lats, lons

def generate_complete_path(start_lat, start_lon, start_heading, end_lat, end_lon, end_heading):
    """
    Generates the full curved path by connecting two adjusted path segments.
    """
    # Calculate Phase 1: From start point to an intermediate connection
    phase_1_lats, phase_1_lons = calculate_path_segment(
        start_lat, start_lon, start_heading, end_lat, end_lon, end_heading)
    
    # Calculate Phase 2: From end point back towards the endpoint of Phase 1
    phase_2_lats, phase_2_lons = calculate_path_segment(
        end_lat, end_lon, (end_heading + 180) % 360, phase_1_lats[-1], phase_1_lons[-1], end_heading)

    end_of_phase_2_lat = phase_2_lats[-1] if LAT_B == phase_2_lats[0] else phase_2_lats[0]
    end_of_phase_2_lon = phase_2_lons[-1] if LON_B == phase_2_lons[0] else phase_2_lons[0]

    # IMPORTANT to recalculate 
    # Recalculate Phase 1: From start point to an intermediate connection
    phase_1_lats_recalculated, phase_1_lons_recalculated = calculate_path_segment(
        start_lat, start_lon, start_heading, end_of_phase_2_lat, end_of_phase_2_lon, end_heading)

    # Recalculate Phase 2: From end point back towards the endpoint of Phase 1
    phase_2_lats_recalculated, phase_2_lons_recalculated = calculate_path_segment(
        end_lat, end_lon, (end_heading + 180) % 360, phase_1_lats_recalculated[-1], phase_1_lons_recalculated[-1], end_heading)
    
    # Combine phases for the full path
    full_lats = phase_1_lats_recalculated + phase_2_lats_recalculated[::-1]
    full_lons = phase_1_lons_recalculated + phase_2_lons_recalculated[::-1]
    
    return full_lats, full_lons

def plot_curved_path(start_lat, start_lon, start_heading, end_lat, end_lon, end_heading):
    """
    Plots the curved path between two points with initial and final headings.
    """
    full_lats, full_lons = generate_complete_path(start_lat, start_lon, start_heading, end_lat, end_lon, end_heading)
    print(tuple(zip(full_lats, full_lons)))
    great_circle_distance = geod.Inverse(start_lat, start_lon, end_lat, end_lon)['s12']
    
    # Plot the path and great-circle route
    plt.figure(figsize=(10, 6))
    plt.plot(full_lons, full_lats, label=f"Curved Path (Initial {start_heading}° to Final {end_heading}°)", color="purple")
    plt.plot([start_lon, end_lon], [start_lat, end_lat], label=f"Great Circle ({great_circle_distance:.2f} meters)", linestyle="--", color="blue")
    
    # Mark Points A and B
    plt.scatter([start_lon, end_lon], [start_lat, end_lat], color="red", marker="o")
    plt.annotate("A", (start_lon, start_lat), textcoords="offset points", xytext=(-10,5), ha='center')
    plt.annotate("B", (end_lon, end_lat), textcoords="offset points", xytext=(-10,5), ha='center')
    
    # Heading indicators
    plt.arrow(start_lon, start_lat, 0.02 * np.cos(np.radians(start_heading)), 0.02 * np.sin(np.radians(start_heading)),
              head_width=0.02, head_length=0.02, color="green", label=f"Initial Heading ({start_heading}°)")
    plt.arrow(end_lon, end_lat, 0.02 * np.cos(np.radians(end_heading)), 0.02 * np.sin(np.radians(end_heading)),
              head_width=0.02, head_length=0.02, color="orange", label=f"Final Heading ({end_heading}°)")
    
    # Display plot
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.legend()
    plt.title(f"Curved Path from Heading {start_heading}° to {end_heading}°")
    plt.grid(True)
    plt.show()

# Example usage for different initial and final headings
direction_data = [
    (295, 135), (195, 335), (315, 45), (135, 45), (90, 90), (0, 0), (0, 180), (270, 90)
]
for initial_heading, final_heading in direction_data:
    plot_curved_path(LAT_A, LON_A, initial_heading, LAT_B, LON_B, final_heading)

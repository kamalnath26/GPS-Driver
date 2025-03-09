import rosbag
import matplotlib.pyplot as plt
import numpy as np
import os

# File paths to your bag files
current_path = os.path.join(os.getcwd(), "../data")
bag_file_with_buildings = os.path.join(current_path, 'with_building_stationary.bag')
bag_file_without_buildings = os.path.join(current_path, 'without_building_stationary.bag')

# Function to read bag file and return processed data
def read_bag_data(bag_file):
    utm_eastings = []
    utm_northings = []
    
    # Read the bag file
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/gps']):
            utm_eastings.append(msg.utm_easting)
            utm_northings.append(msg.utm_northing)
    
    # Convert to numpy arrays for easier manipulation
    utm_eastings = np.array(utm_eastings)
    utm_northings = np.array(utm_northings)
    
    # Subtract the first point to remove the offset
    utm_eastings -= utm_eastings[0]
    utm_northings -= utm_northings[0]

    return utm_eastings, utm_northings

# Function to read bag file and return processed data
def read_bag_data_altitude(bag_file):
    altitudes = []
    timestamps = []
    
    # Read the bag file
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/gps']):
            altitudes.append(msg.altitude)
            timestamps.append(t.to_sec())  # Convert ROS time to seconds
    
    # Convert to numpy arrays for easier manipulation
    altitudes = np.array(altitudes)
    timestamps = np.array(timestamps)
    
    # Subtract the first point to remove the offset
    altitudes -= altitudes[0]
    timestamps -= timestamps[0]

    return altitudes, timestamps

# Read data from both bag files
utm_eastings_with, utm_northings_with = read_bag_data(bag_file_with_buildings)
utm_eastings_without, utm_northings_without = read_bag_data(bag_file_without_buildings)
altitudes_with, timestamps_with = read_bag_data_altitude(bag_file_with_buildings)
altitudes_without, timestamps_without = read_bag_data_altitude(bag_file_without_buildings)

# Calculate centroids
centroid_easting_with = np.mean(utm_eastings_with)
centroid_northing_with = np.mean(utm_northings_with)

centroid_easting_without = np.mean(utm_eastings_without)
centroid_northing_without = np.mean(utm_northings_without)

# Calculate deviations from the centroid
deviation_easting_with = utm_eastings_with - centroid_easting_with
deviation_northing_with = utm_northings_with - centroid_northing_with

deviation_easting_without = utm_eastings_without - centroid_easting_without
deviation_northing_without = utm_northings_without - centroid_northing_without

# Print centroid values
print(f'With Buildings Centroid: ({centroid_easting_with:.2f}, {centroid_northing_with:.2f})')
print(f'Without Buildings Centroid: ({centroid_easting_without:.2f}, {centroid_northing_without:.2f})')

# Print some altitude statistics
print(f'Min Altitude with Buildings: {np.min(altitudes_with):.2f} m')
print(f'Max Altitude with Buildings: {np.max(altitudes_with):.2f} m')
print(f'Mean Altitude with Buildings: {np.mean(altitudes_with):.2f} m')

print(f'Min Altitude without Buildings: {np.min(altitudes_without):.2f} m')
print(f'Max Altitude without Buildings: {np.max(altitudes_without):.2f} m')
print(f'Mean Altitude without Buildings: {np.mean(altitudes_without):.2f} m')

 # Calculate Euclidean distance from each point to the centroid for with Buildings
with_distances = np.sqrt((utm_eastings_with - centroid_easting_with)**2 + (utm_northings_with - centroid_northing_with)**2)
with_distances_deviations = np.sqrt((deviation_easting_with - centroid_easting_with)**2 + (deviation_northing_with - centroid_northing_with)**2)

# Calculate Euclidean distance from each point to the centroid for without Buildings
without_distances = np.sqrt((utm_eastings_without - centroid_easting_without)**2 + (utm_northings_without - centroid_northing_without)**2)
without_distances_deviations = np.sqrt((deviation_easting_without - centroid_easting_without)**2 + (deviation_northing_without - centroid_northing_without)**2)



# Plotting
# plt.figure(figsize=(10, 6))
plt.figure()
# Plot data with buildings
plt.scatter(utm_eastings_with, utm_northings_with, alpha=0.7, label='With Buildings', color='blue')
plt.scatter(deviation_easting_with, deviation_northing_with, alpha=0.7, label='With Buildings Deviations', color='purple')
plt.scatter(centroid_easting_with, centroid_northing_with, color='red', s=100, label='Centroid With Buildings', marker='X')

# Plot data without buildings
plt.scatter(utm_eastings_without, utm_northings_without, alpha=0.7, label='Without Buildings', color='green')
plt.scatter(deviation_easting_without, deviation_northing_without, alpha=0.7, label='Without Buildings Deviations', color= 'yellow')
plt.scatter(centroid_easting_without, centroid_northing_without, color='orange', s=100, label='Centroid Without Buildings', marker='X')

# Annotate centroids with values
plt.text(centroid_easting_with, centroid_northing_with, 
         f'({centroid_easting_with:.2f}, {centroid_northing_with:.2f})',
         fontsize=10, ha='right', color='red')

plt.text(centroid_easting_without, centroid_northing_without, 
         f'({centroid_easting_without:.2f}, {centroid_northing_without:.2f})',
         fontsize=10, ha='right', color='orange')

plt.annotate(f'Centroid With Buildings\n({centroid_easting_with:.2f}, {centroid_northing_with:.2f})',
             xy=(centroid_easting_with, centroid_northing_with),
             xytext=(centroid_easting_with + 1, centroid_northing_with + 1),
             arrowprops=dict(facecolor='black', shrink=0.05))

plt.annotate(f'Centroid Without Buildings\n({centroid_easting_without:.2f}, {centroid_northing_without:.2f})',
             xy=(centroid_easting_without, centroid_northing_without),
             xytext=(centroid_easting_without + 1, centroid_northing_without + 1),
             arrowprops=dict(facecolor='black', shrink=0.05))

# Add labels and title
plt.suptitle('All Data in same plot')
plt.title('Stationary Northing vs. Easting Scatterplot with and without Buildings')
plt.xlabel('Easting (meters)')
plt.ylabel('Northing (meters)')
plt.axhline(0, color='grey', lw=0.5, ls='--')
plt.axvline(0, color='grey', lw=0.5, ls='--')
plt.grid()
plt.legend()
plt.axis('equal')


plt.figure()
# Scatter plot of the original GPS data points
plt.subplot(2, 2, 1)
plt.scatter(utm_eastings_with, utm_northings_with, alpha=0.7, label='GPS Data Points with  Buildings', color = 'blue')
plt.scatter(centroid_easting_with, centroid_northing_with, color='red', s=100, label='Centroid with Buildings', marker='X')

# Annotate centroid
plt.text(centroid_easting_with, centroid_northing_with, 
         f'({centroid_easting_with:.2f}, {centroid_northing_with:.2f})',
         fontsize=10, ha='right', color='red')

# plt.annotate(f'Centroid\n({centroid_easting:.2f}, {centroid_northing:.2f})',
#              xy=(centroid_easting, centroid_northing),
#              xytext=(centroid_easting + 1, centroid_northing + 1),
#              arrowprops=dict(facecolor='black', shrink=0.05))

plt.title('Stationary Northing vs. Easting Scatterplot With Buildings')
plt.xlabel('Easting (meters)')
plt.ylabel('Northing (meters)')
plt.axhline(0, color='grey', lw=0.5, ls='--')
plt.axvline(0, color='grey', lw=0.5, ls='--')
plt.grid()
plt.legend()
plt.axis('equal')

# Plotting deviations
plt.subplot(2, 2, 2)
plt.scatter(deviation_easting_with, deviation_northing_with, alpha=0.7, label='Deviations with Buildings', color = 'purple')
plt.scatter(centroid_easting_with, centroid_northing_with, color='red', s=100, label='Centroid with Buildings', marker='X')

#Annotate centroid
plt.text(centroid_easting_with, centroid_northing_with, 
         f'({centroid_easting_with:.2f}, {centroid_northing_with:.2f})',
         fontsize=10, ha='right', color='red')

# plt.annotate(f'Centroid\n({centroid_easting:.2f}, {centroid_northing:.2f})',
#              xy=(centroid_easting, centroid_northing),
#              xytext=(centroid_easting + 1, centroid_northing + 1),
#              arrowprops=dict(facecolor='black', shrink=0.05))

# Add labels and title for deviations
plt.title('Deviation from Centroid with Buildings')
plt.xlabel('Deviation Easting (meters)')
plt.ylabel('Deviation Northing (meters)')
plt.axhline(0, color='grey', lw=0.5, ls='--')
plt.axvline(0, color='grey', lw=0.5, ls='--')
plt.grid()
plt.legend()
plt.axis('equal')


# Scatter plot of the original GPS data points
plt.subplot(2, 2, 3)
plt.scatter(utm_eastings_without, utm_northings_without, alpha=0.7, label='GPS Data Points without Buildings', color = 'green')
plt.scatter(centroid_easting_without, centroid_northing_without, color='red', s=100, label='Centroid without Buildings', marker='X')

# Annotate centroid
plt.text(centroid_easting_without, centroid_northing_without, 
         f'({centroid_easting_without:.2f}, {centroid_northing_without:.2f})',
         fontsize=10, ha='right', color='red')

# plt.annotate(f'Centroid\n({centroid_easting:.2f}, {centroid_northing:.2f})',
#              xy=(centroid_easting, centroid_northing),
#              xytext=(centroid_easting + 1, centroid_northing + 1),
#              arrowprops=dict(facecolor='black', shrink=0.05))

plt.title('Stationary Northing vs. Easting Scatterplot without Buildings')
plt.xlabel('Easting (meters)')
plt.ylabel('Northing (meters)')
plt.axhline(0, color='grey', lw=0.5, ls='--')
plt.axvline(0, color='grey', lw=0.5, ls='--')
plt.grid()
plt.legend()
plt.axis('equal')

# Plotting deviations
plt.subplot(2, 2, 4)
plt.scatter(deviation_easting_without, deviation_northing_without, alpha=0.7, label='Deviations without Buildings', color = 'gold')
plt.scatter(centroid_easting_without, centroid_northing_without, color='red', s=100, label='Centroid without Buildings', marker='X')

#Annotate centroid
plt.text(centroid_easting_without, centroid_northing_without, 
         f'({centroid_easting_without:.2f}, {centroid_northing_without:.2f})',
         fontsize=10, ha='right', color='red')

# plt.annotate(f'Centroid\n({centroid_easting:.2f}, {centroid_northing:.2f})',
#              xy=(centroid_easting, centroid_northing),
#              xytext=(centroid_easting + 1, centroid_northing + 1),
#              arrowprops=dict(facecolor='black', shrink=0.05))

# Add labels and title for deviations
plt.title('Deviation from Centroid without Buildings')
plt.xlabel('Deviation Easting (meters)')
plt.ylabel('Deviation Northing (meters)')
plt.axhline(0, color='grey', lw=0.5, ls='--')
plt.axvline(0, color='grey', lw=0.5, ls='--')
plt.grid()
plt.legend()
plt.axis('equal')

# Show the plots
plt.tight_layout()


# plt.figure(figsize=(10, 6))
plt.figure()
plt.plot(timestamps_with, altitudes_with, marker='o', linestyle='-', color='red', label='Altitude vs. Time with Buildings')
plt.title('Altitude vs. Time with Buildings')
plt.xlabel('Time (seconds)')
plt.ylabel('Altitude (meters)')
plt.axhline(0, color='grey', lw=0.5, ls='--')  # Optional: horizontal line at y=0
plt.grid()
plt.legend()
plt.tight_layout()

# plt.figure(figsize=(10, 6))
plt.figure()
plt.plot(timestamps_without, altitudes_without, marker='o', linestyle='-', color='green', label='Altitude vs. Time without Buildings')
plt.title('Altitude vs. Time without Buildings')
plt.xlabel('Time (seconds)')
plt.ylabel('Altitude (meters)')
plt.axhline(0, color='grey', lw=0.5, ls='--')  # Optional: horizontal line at y=0
plt.grid()
plt.legend()
plt.tight_layout()


# Altitude with and without buildings in same  plot
plt.figure()
plt.plot(timestamps_with, altitudes_with, marker='o', linestyle='-', color='red', label='Altitude vs. Time with Buildings')
plt.plot(timestamps_without, altitudes_without, marker='o', linestyle='-', color='green', label='Altitude vs. Time without Buildings')
plt.title('Altitude vs. Time with and without Buildings in same plot')
plt.xlabel('Time (seconds)')
plt.ylabel('Altitude (meters)')
plt.axhline(0, color='grey', lw=0.5, ls='--')  # Optional: horizontal line at y=0
plt.grid()
plt.legend()
plt.tight_layout()


plt.figure()
plt.subplot(1, 2, 1)
# plt.scatter(timestamps_with, altitudes_with, alpha=0.7, label='Altitude vs. Time with Buildings', color = 'blue')
plt.plot(timestamps_with, altitudes_with, marker='o', linestyle='-', color='red', label='Altitude vs. Time with Buildings')

# Add labels and title for deviations
plt.title('Altitude vs. Time with Buildings')
plt.xlabel('Time (seconds)')
plt.ylabel('Altitude (meters)')
plt.axhline(0, color='grey', lw=0.5, ls='--')
plt.axvline(0, color='grey', lw=0.5, ls='--')
plt.grid()
plt.legend()
plt.axis('equal')

# Show the plots
plt.tight_layout()

plt.subplot(1, 2, 2)
# plt.scatter(timestamps_without, altitudes_without, alpha=0.7, label='Altitude vs. Time with Buildings', color = 'green')
plt.plot(timestamps_without, altitudes_without, marker='o', linestyle='-', color='green', label='Altitude vs. Time with Buildings')

# Add labels and title for deviations
plt.title('Altitude vs. Time without Buildings')
plt.xlabel('Time (seconds)')
plt.ylabel('Altitude (meters)')
plt.axhline(0, color='grey', lw=0.5, ls='--')
plt.axvline(0, color='grey', lw=0.5, ls='--')
plt.grid()
plt.legend()
plt.axis('equal')

# Show the plots
plt.tight_layout()

#  Show the plots of Eculidian distance
plt.figure()
plt.hist(with_distances, bins=30, alpha=0.7, color='blue', edgecolor='black')
plt.title('Histogram of Distances to Centroid with Buildings')
plt.xlabel('Distance (meters)')
plt.ylabel('Frequency')
plt.grid()
plt.axvline(np.mean(with_distances), color='red', linestyle='dashed', linewidth=1, label='Mean Distance with Buildings')
plt.legend()
plt.tight_layout()

plt.figure()
plt.hist(without_distances, bins=30, alpha=0.7, color='green', edgecolor='black')
plt.title('Histogram of Distances to Centroid without Buildings')
plt.xlabel('Distance (meters)')
plt.ylabel('Frequency')
plt.grid()
plt.axvline(np.mean(without_distances), color='red', linestyle='dashed', linewidth=1, label='Mean Distance without Buildings')
plt.legend()
plt.tight_layout()

plt.figure()
plt.hist(with_distances_deviations, bins=30, alpha=0.7, color='blue', edgecolor='black')
plt.title('Histogram of Deviated Distances to Centroid with Buildings')
plt.xlabel('Distance (meters)')
plt.ylabel('Frequency')
plt.grid()
plt.axvline(np.mean(with_distances_deviations), color='red', linestyle='dashed', linewidth=1, label='Deviated Mean Distance with Buildings')
plt.legend()
plt.tight_layout()

plt.figure()
plt.hist(without_distances_deviations, bins=30, alpha=0.7, color='green', edgecolor='black')
plt.title('Histogram of Deviated Distances to Centroid without Buildings')
plt.xlabel('Distance (meters)')
plt.ylabel('Frequency')
plt.grid()
plt.axvline(np.mean(without_distances_deviations), color='red', linestyle='dashed', linewidth=1, label='Deviated Mean Distance without Buildings')
plt.legend()
plt.tight_layout()

plt.figure()
plt.subplot(2, 2, 1)
plt.hist(with_distances, bins=30, alpha=0.7, color='blue', edgecolor='black')
plt.title('Histogram of Distances to Centroid with Buildings')
plt.xlabel('Distance (meters)')
plt.ylabel('Frequency')
plt.grid()
plt.axvline(np.mean(with_distances), color='red', linestyle='dashed', linewidth=1, label='Mean Distance with Buildings')
plt.legend()
plt.tight_layout()

plt.subplot(2, 2, 2)
plt.hist(without_distances, bins=30, alpha=0.7, color='green', edgecolor='black')
plt.title('Histogram of Distances to Centroid without Buildings')
plt.xlabel('Distance (meters)')
plt.ylabel('Frequency')
plt.grid()
plt.axvline(np.mean(without_distances), color='red', linestyle='dashed', linewidth=1, label='Mean Distance without Buildings')
plt.legend()
plt.tight_layout()

plt.subplot(2, 2, 3)
plt.hist(with_distances_deviations, bins=30, alpha=0.7, color='blue', edgecolor='black')
plt.title('Histogram of Deviated Distances to Centroid with Buildings')
plt.xlabel('Distance (meters)')
plt.ylabel('Frequency')
plt.grid()
plt.axvline(np.mean(with_distances_deviations), color='red', linestyle='dashed', linewidth=1, label='Deviated Mean Distance with Buildings')
plt.legend()
plt.tight_layout()

plt.subplot(2, 2, 4)
plt.hist(without_distances_deviations, bins=30, alpha=0.7, color='green', edgecolor='black')
plt.title('Histogram of Deviated Distances to Centroid without Buildings')
plt.xlabel('Distance (meters)')
plt.ylabel('Frequency')
plt.grid()
plt.axvline(np.mean(without_distances_deviations), color='red', linestyle='dashed', linewidth=1, label='Deviated Mean Distance without Buildings')
plt.legend()
plt.tight_layout()

plt.show()

import rosbag
import matplotlib.pyplot as plt
import numpy as np
import os

# File paths to your bag files
current_path = os.path.join(os.getcwd(), "../data")
moving_data = os.path.join(current_path, 'moving_data.bag')


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

utm_eastings, utm_northings = read_bag_data(moving_data)
altitudes, timestamps = read_bag_data_altitude(moving_data)

# Calculate the line of best fit
fit = np.polyfit(utm_eastings, utm_northings, 1)  # 1st degree polynomial (line)
fit_line = np.polyval(fit, utm_eastings)

# Plotting
# plt.figure(figsize=(10, 6))
plt.figure()
plt.scatter(utm_eastings, utm_northings, alpha=0.7, label='GPS Data Points', color='blue')
plt.plot(utm_eastings, fit_line, color='red', label='Line of Best Fit', linewidth=2)

# Add labels and title
plt.suptitle('Moving Data')
plt.title('Northing vs. Easting Scatterplot with Line of Best Fit')
plt.xlabel('Easting (m)')
plt.ylabel('Northing (m)')
plt.axhline(0, color='grey', lw=0.5, ls='--')  # Optional: horizontal line at y=0
plt.axvline(0, color='grey', lw=0.5, ls='--')  # Optional: vertical line at x=0
plt.grid()
plt.legend()
plt.tight_layout()


# Print coefficients of the line of best fit
print(f'Line of Best Fit: y = {fit[0]:.4f} * x + {fit[1]:.4f}')
# Print some altitude statistics
print(f'Min Altitude: {np.min(altitudes):.2f} m')
print(f'Max Altitude: {np.max(altitudes):.2f} m')
print(f'Mean Altitude: {np.mean(altitudes):.2f} m')

plt.figure()
plt.plot(timestamps, altitudes, color='blue', marker='o', linestyle='-', alpha=0.7)

# Add labels and title
plt.suptitle('Moving Data')
plt.title('Altitude vs. Time Plot')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.grid()
plt.tight_layout()
plt.show()


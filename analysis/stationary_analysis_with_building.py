import rosbag
import matplotlib.pyplot as plt
import numpy as np
import os

# File path to your bag file
current_path = os.path.join(os.getcwd(),"../data")
bag_file = os.path.join(current_path,'with_building_stationary.bag')

# Initialize lists to store GPS data
latitudes = []
longitudes = []
utm_eastings = []
utm_northings = []

# Read the bag file
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/gps']):
        latitudes.append(msg.latitude)
        longitudes.append(msg.longitude)
        utm_eastings.append(msg.utm_easting)
        utm_northings.append(msg.utm_northing)

# Convert to numpy arrays for easier manipulation
utm_eastings = np.array(utm_eastings)
utm_northings = np.array(utm_northings)

# Subtract the first point to remove the offset
utm_eastings -= utm_eastings[0]
utm_northings -= utm_northings[0]

# Calculate the centroid
centroid_easting = np.mean(utm_eastings)
centroid_northing = np.mean(utm_northings)

# Calculate deviations from the centroid
deviation_easting = utm_eastings - centroid_easting
deviation_northing = utm_northings - centroid_northing

# Print centroid values
print(f'Centroid Easting: {centroid_easting}, Centroid Northing: {centroid_northing}')

# Plotting
# plt.figure(figsize=(10, 6))
plt.figure()
plt.scatter(utm_eastings, utm_northings, alpha=0.7, label='GPS Data Points')
plt.scatter(deviation_easting, deviation_northing, alpha=0.7, label='Deviations GPS Data Points')
plt.scatter(centroid_easting, centroid_northing, color='red', s=100, label='Centroid', marker='X')

# Annotate centroid
plt.text(centroid_easting, centroid_northing, 
         f'({centroid_easting:.2f}, {centroid_northing:.2f})',
         fontsize=10, ha='right', color='red')


plt.annotate(f'Centroid\n({centroid_easting:.2f}, {centroid_northing:.2f})',
             xy=(centroid_easting, centroid_northing),
             xytext=(centroid_easting + 1, centroid_northing + 1),
             arrowprops=dict(facecolor='black', shrink=0.05))

# Add labels and title
plt.title('Stationary Northing vs. Easting Scatterplot')
plt.xlabel('Easting (m)')
plt.ylabel('Northing (m)')
plt.axhline(0, color='grey', lw=0.5, ls='--')
plt.axvline(0, color='grey', lw=0.5, ls='--')
plt.grid()
plt.legend()
plt.axis('equal')

# plt.figure(figsize=(12, 6))
plt.figure()

# Scatter plot of the original GPS data points
plt.subplot(1, 2, 1)
plt.scatter(utm_eastings, utm_northings, alpha=0.7, label='GPS Data Points')
plt.scatter(centroid_easting, centroid_northing, color='red', s=100, label='Centroid', marker='X')

# Annotate centroid
plt.text(centroid_easting, centroid_northing, 
         f'({centroid_easting:.2f}, {centroid_northing:.2f})',
         fontsize=10, ha='right', color='red')

# plt.annotate(f'Centroid\n({centroid_easting:.2f}, {centroid_northing:.2f})',
#              xy=(centroid_easting, centroid_northing),
#              xytext=(centroid_easting + 1, centroid_northing + 1),
#              arrowprops=dict(facecolor='black', shrink=0.05))

plt.title('Stationary Northing vs. Easting Scatterplot')
plt.xlabel('Easting (m)')
plt.ylabel('Northing (m)')
plt.axhline(0, color='grey', lw=0.5, ls='--')
plt.axvline(0, color='grey', lw=0.5, ls='--')
plt.grid()
plt.legend()
plt.axis('equal')

# Plotting deviations
plt.subplot(1, 2, 2)
plt.scatter(deviation_easting, deviation_northing, alpha=0.7, label='Deviations')
plt.scatter(centroid_easting, centroid_northing, color='red', s=100, label='Centroid', marker='X')

#Annotate centroid
plt.text(centroid_easting, centroid_northing, 
         f'({centroid_easting:.2f}, {centroid_northing:.2f})',
         fontsize=10, ha='right', color='red')

# plt.annotate(f'Centroid\n({centroid_easting:.2f}, {centroid_northing:.2f})',
#              xy=(centroid_easting, centroid_northing),
#              xytext=(centroid_easting + 1, centroid_northing + 1),
#              arrowprops=dict(facecolor='black', shrink=0.05))

# Add labels and title for deviations
plt.title('Deviation from Centroid')
plt.xlabel('Deviation Easting (m)')
plt.ylabel('Deviation Northing (m)')
plt.axhline(0, color='grey', lw=0.5, ls='--')
plt.axvline(0, color='grey', lw=0.5, ls='--')
plt.grid()
plt.legend()
plt.axis('equal')

# Show the plots
plt.tight_layout()
plt.show()

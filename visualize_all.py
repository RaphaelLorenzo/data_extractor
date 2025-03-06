import numpy as np
import matplotlib.pyplot as plt
import os
from mpl_toolkits.axes_grid1 import make_axes_locatable
import cv2

# Load image paths
images = [os.path.join(dp, f) for dp, dn, fn in os.walk(os.path.expanduser("./saved_images")) for f in fn if f.endswith(".png")]
images.sort()

# Load scale weight paths
scale_weights = [os.path.join(dp, f) for dp, dn, fn in os.walk(os.path.expanduser("./saved_scale_weights")) for f in fn if f.endswith(".txt")]
scale_weights.sort()

# Load point cloud paths
pcls = [os.path.join(dp, f) for dp, dn, fn in os.walk(os.path.expanduser("./saved_point_clouds")) for f in fn if f.endswith(".bin")]
pcls.sort()

# Create a figure with 3 subplots: image, point cloud, and scale weight history
fig = plt.figure(figsize=(18, 12))
ax1 = plt.subplot2grid((2, 2), (0, 0), colspan=1, rowspan=1)  # Image
ax2 = plt.subplot2grid((2, 2), (0, 1), colspan=1, rowspan=1)  # Point cloud
ax3 = plt.subplot2grid((2, 2), (1, 0), colspan=2, rowspan=1)  # Scale weight history

fig.suptitle(f"Point cloud and image visualizer", fontsize=16)

# Number of frames to show in the scale weight history plot
N = 20
scale_weight_history = []

# Parameters for moving average
window_size = 3  # Size of the moving average window
threshold = 0.01  # Threshold for detecting significant changes

for i in range(len(images)):
    
    # Find corresponding scale weight and point cloud based on progression
    progression_prop = i / len(images)
    
    # Get scale weight
    closest_scale_weight_index = int(progression_prop * len(scale_weights))
    scale_weight = float(open(scale_weights[closest_scale_weight_index]).read())
    
    # Get point cloud
    closest_pcl_index = int(progression_prop * len(pcls))
    pcl_path = pcls[closest_pcl_index]
    
    # Add current scale weight to history
    scale_weight_history.append(scale_weight)
    if len(scale_weight_history) > N:
        scale_weight_history.pop(0)
    
    # Clear all axes
    ax1.clear()
    ax2.clear()
    ax3.clear()

    # Display image
    image_path = images[i]
    image = cv2.imread(image_path)
    image = np.array(image)[:, :, ::-1]  # BGR to RGB
    
    print(f"Image shape: {image.shape}")
    
    ax1.imshow(image)
    ax1.axis('off')
    ax1.set_title(f'Image {i} | scale weight: {scale_weight:.2f}')
    
    # Display point cloud
    xyzi = np.fromfile(pcl_path, dtype=np.float32).reshape(-1, 4)
    print(f"Point cloud min/max: {xyzi.min(axis=0)}, {xyzi.max(axis=0)}")
    sub_xyzi = xyzi[::8, :] # skip some points to avoid overplotting
    
    scatter = ax2.scatter(sub_xyzi[:, 0], sub_xyzi[:, 1], c=sub_xyzi[:, 2], cmap='viridis', s=0.5, vmin=-2, vmax=3)
    ax2.set_xlim(-6, 6)
    ax2.set_ylim(-4, 8)
    ax2.set_title(f'Point cloud {closest_pcl_index}')
    ax2.set_aspect('equal')
    
    # Plot scale weight history with change detection
    x_values = list(range(max(0, i-N+1), i+1))
    
    # Calculate moving averages if we have enough data
    if len(scale_weight_history) >= window_size:
        moving_avgs = []
        for j in range(len(scale_weight_history) - window_size + 1):
            window = scale_weight_history[j:j+window_size]
            moving_avgs.append(sum(window) / window_size)
        
        # Plot the basic line
        ax3.plot(x_values, scale_weight_history, 'b-', alpha=0.5)
        
        # Plot segments with color based on trend
        for j in range(1, len(moving_avgs)):
            idx1 = j + window_size - 2
            idx2 = j + window_size - 1
            if idx1 < len(scale_weight_history) and idx2 < len(scale_weight_history):
                x_segment = [x_values[idx1], x_values[idx2]]
                y_segment = [scale_weight_history[idx1], scale_weight_history[idx2]]
                
                # Determine color based on change in moving average
                change = moving_avgs[j] - moving_avgs[j-1]
                if abs(change) > threshold:  # Only highlight significant changes
                    color = 'green' if change > 0 else 'red'
                    ax3.plot(x_segment, y_segment, color=color, linewidth=2)
        
        # Plot points
        ax3.scatter(x_values, scale_weight_history, color='blue', s=30, zorder=5)
    else:
        # If not enough data for moving average, just plot the line
        ax3.plot(x_values, scale_weight_history, 'b-', marker='o')
    
    ax3.set_title(f'Scale Weight History (Last {N} Frames) - Green: Increase, Red: Decrease')
    ax3.set_xlabel('Frame Number')
    ax3.set_ylabel('Scale Weight')
    ax3.set_ylim(-1, 0)  # Set y-axis limits from -1 to 0
    ax3.grid(True)
    
    plt.tight_layout(rect=[0, 0, 1, 0.95])  # Adjust layout to make room for the suptitle
    plt.pause(0.05)

plt.close(fig)

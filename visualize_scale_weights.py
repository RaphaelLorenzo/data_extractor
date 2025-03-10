import numpy as np
import matplotlib.pyplot as plt
import os
from matplotlib.patches import Rectangle

# Load all scale weight paths
scale_weights = [os.path.join(dp, f) for dp, dn, fn in os.walk(os.path.expanduser("./saved_scale_weights")) for f in fn if f.endswith(".txt")]
scale_weights.sort()

# Read all scale weight values
weights = []
for sw_path in scale_weights:
    try:
        weight = float(open(sw_path).read())
        weights.append(weight)
    except Exception as e:
        print(f"Error reading {sw_path}: {e}")

# Parameters for moving average and trend detection
window_size = 5  # Size of the moving average window (larger for overall trend)
threshold = 0.005  # Threshold for detecting significant changes

# Create figure
plt.figure(figsize=(15, 8))
ax = plt.gca()

# Plot the basic weight line
x_values = list(range(len(weights)))
plt.plot(x_values, weights, 'b-', alpha=0.5, label='Scale Weight')

# Calculate moving averages
if len(weights) >= window_size:
    moving_avgs = []
    for i in range(len(weights) - window_size + 1):
        window = weights[i:i+window_size]
        moving_avgs.append(sum(window) / window_size)
    
    # Plot moving average
    ma_x_values = list(range(window_size-1, len(weights)))
    plt.plot(ma_x_values, moving_avgs, 'k-', alpha=0.7, label=f'Moving Avg (window={window_size})')
    
    # Identify trend periods
    trend_periods = []
    current_trend = None
    start_idx = None
    
    for i in range(1, len(moving_avgs)):
        change = moving_avgs[i] - moving_avgs[i-1]
        
        # Determine if this is a significant change
        if abs(change) > threshold:
            trend = 'increase' if change > 0 else 'decrease'
            
            # Start a new trend or continue current one
            if current_trend is None:
                current_trend = trend
                start_idx = i + window_size - 2
            elif trend != current_trend:
                # End previous trend and start new one
                end_idx = i + window_size - 2
                trend_periods.append((start_idx, end_idx, current_trend))
                current_trend = trend
                start_idx = i + window_size - 2
        
        # If we reach the end with an active trend, close it
        if i == len(moving_avgs) - 1 and current_trend is not None:
            end_idx = i + window_size - 1
            trend_periods.append((start_idx, end_idx, current_trend))
    
    # Add gray overlay for trend periods
    for start, end, trend in trend_periods:
        if start < len(x_values) and end < len(x_values):
            x_start = x_values[start]
            x_end = x_values[end]
            width = x_end - x_start
            
            # Create rectangle overlay
            rect = Rectangle((x_start, min(weights)), width, max(weights) - min(weights), 
                             color='lightgray', alpha=0.3, zorder=1)
            ax.add_patch(rect)
            
            # Add trend label
            trend_label = "↑" if trend == 'increase' else "↓"
            plt.text(x_start + width/2, np.mean(weights), trend_label, 
                     ha='center', va='center', fontsize=12, 
                     color='green' if trend == 'increase' else 'red',
                     fontweight='bold')
    
    # Plot segments with color based on trend
    for i in range(1, len(moving_avgs)):
        idx1 = i + window_size - 2
        idx2 = i + window_size - 1
        if idx1 < len(weights) and idx2 < len(weights):
            x_segment = [x_values[idx1], x_values[idx2]]
            y_segment = [weights[idx1], weights[idx2]]
            
            # Determine color based on change in moving average
            change = moving_avgs[i] - moving_avgs[i-1]
            if abs(change) > threshold:  # Only highlight significant changes
                color = 'green' if change > 0 else 'red'
                plt.plot(x_segment, y_segment, color=color, linewidth=2)

# Add scatter points for all weight values
plt.scatter(x_values, weights, color='blue', s=10, zorder=5)

# Add labels and title
plt.title('Scale Weight Measurements Over Time', fontsize=16)
plt.xlabel('Frame Number', fontsize=12)
plt.ylabel('Scale Weight', fontsize=12)
plt.grid(True, alpha=0.3)
plt.legend()

# Add annotations for significant events
min_idx = np.argmin(weights)
max_idx = np.argmax(weights)
plt.annotate(f'Min: {weights[min_idx]:.3f}', 
             xy=(min_idx, weights[min_idx]),
             xytext=(min_idx, weights[min_idx] - 0.05),
             arrowprops=dict(facecolor='black', shrink=0.05),
             ha='center')

plt.annotate(f'Max: {weights[max_idx]:.3f}', 
             xy=(max_idx, weights[max_idx]),
             xytext=(max_idx, weights[max_idx] + 0.05),
             arrowprops=dict(facecolor='black', shrink=0.05),
             ha='center')

# Show the plot
plt.tight_layout()
plt.savefig('scale_weights_analysis.png', dpi=150)
plt.show()

# Print some statistics
print(f"Total measurements: {len(weights)}")
print(f"Min weight: {min(weights):.3f} at frame {np.argmin(weights)}")
print(f"Max weight: {max(weights):.3f} at frame {np.argmax(weights)}")
print(f"Average weight: {np.mean(weights):.3f}")
print(f"Standard deviation: {np.std(weights):.3f}")

# Identify significant changes (more than 3 standard deviations from mean)
std_dev = np.std(weights)
mean_weight = np.mean(weights)
significant_changes = []

for i in range(1, len(weights)):
    change = weights[i] - weights[i-1]
    if abs(change) > 3 * std_dev:
        significant_changes.append((i, weights[i-1], weights[i], change))

print(f"\nSignificant changes (> 3 std dev):")
for idx, prev, curr, change in significant_changes:
    print(f"Frame {idx}: {prev:.3f} → {curr:.3f} (Δ{change:.3f})")

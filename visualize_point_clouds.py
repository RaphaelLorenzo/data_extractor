import numpy as np
import matplotlib.pyplot as plt
import os
from mpl_toolkits.axes_grid1 import make_axes_locatable

pcls = [os.path.join(dp, f) for dp, dn, fn in os.walk(os.path.expanduser("./saved_point_clouds")) for f in fn if f.endswith(".bin")]
pcls.sort()


fig, (ax) = plt.subplots(1, 1, figsize=(10, 10))
fig.suptitle(f"Point cloud visualizer", fontsize=14)

for i in range(len(pcls)):
    ax.clear()

    pcl_path = pcls[i]
    xyzi = np.fromfile(pcl_path, dtype=np.float32).reshape(-1, 4)
    
    print(xyzi.min(axis=0), xyzi.max(axis=0))
    
    ax.scatter(xyzi[:, 0], xyzi[:, 1], c=xyzi[:, 2], cmap='viridis', s=0.5, vmin=-2, vmax=3)
    
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_title('Point cloud {}'.format(i))
    plt.pause(0.1)

plt.close(fig)

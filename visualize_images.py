import numpy as np
import matplotlib.pyplot as plt
import os
from mpl_toolkits.axes_grid1 import make_axes_locatable
import cv2
images = [os.path.join(dp, f) for dp, dn, fn in os.walk(os.path.expanduser("./saved_images")) for f in fn if f.endswith(".png")]
images.sort()


fig, (ax) = plt.subplots(1, 1, figsize=(10, 10))
fig.suptitle(f"Point cloud visualizer", fontsize=14)

for i in range(len(images)):
    ax.clear()

    image_path = images[i]
    image = cv2.imread(image_path)
    image = np.array(image)[:, :, ::-1] # BGR to RGB
    
    print(image.shape)
    
    ax.imshow(image)
    
    ax.axis('off')
    ax.set_title('Point cloud {}'.format(i))
    plt.pause(0.05)

plt.close(fig)

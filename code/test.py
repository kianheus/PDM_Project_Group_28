import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
from tqdm import trange

xx = np.arange(0, 6, 0.1)
yy = np.sin(xx)
fig = plt.figure()
frames=[]

for i in trange(len(xx)):
    plt.plot(xx[:i+1], yy[:i+1])
    plt.xlim((min(xx), max(xx)))
    plt.ylim((min(yy), max(yy)))
    # print(f"../export/test/test{i:05}.png")
    plt.savefig(f"../export/test/test{i:05}.png")
    plt.close()

# ani = animation.ArtistAnimation(fig, frames, interval=50, blit=True,
                                # repeat_delay=1000)
# ani.save('movie.mp4')
# plt.show()

# ffmpeg -framerate 8 -pattern_type glob -i '../export/tree/tree*.png' -c:v libx264 -r 30 -pix_fmt yuv420p tree.mp4
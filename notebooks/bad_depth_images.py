# ---
# jupyter:
#   jupytext:
#     formats: ipynb,py:light
#     text_representation:
#       extension: .py
#       format_name: light
#       format_version: '1.5'
#       jupytext_version: 1.4.1
#   kernelspec:
#     display_name: Python 2
#     language: python
#     name: python2
# ---

import os
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import ArtistAnimation
# %matplotlib notebook

base_path = '/home/itayb/simulator/gen_dumps/carla_scripts/output_iter2/'

sector = 'main'
clips = os.listdir(base_path)
center_view = "%s_to_%s" % (sector, sector)
# clips = ['Town03_1584992806', 'Town02_1584900602', 'Town05_1585010636', 'Town03_1584974839',
#         'Town04_1585049035', 'Town03_1584988834', 'Town04_1585049638', 'Town03_1584987013']

# +
bad_frames = []

for clip in clips:
    print("Clip: %s" % clip)
    frames_path = os.path.join(base_path, clip, center_view)
    frames = os.listdir(frames_path)
    for frame in frames:
        frame_depth = np.load(os.path.join(frames_path,frame))["sim_depth"]
        if np.median(frame_depth) < 4 and np.median(frame_depth) > 3:
            print("Found bad img: %s" % frame, "median: %s" % np.median(frame_depth))
            bad_frames.append(np.load(os.path.join(frames_path,frame))['image'])
# -

fig = plt.figure()
ax = fig.add_subplot(111)
plt.tight_layout()
ims = []
for img in bad_frames:
    ims.append([ax.imshow(img, animated=True, cmap='gray', origin='low')])
ani = ArtistAnimation(fig, ims, interval=1000, blit=True)



# ---
# jupyter:
#   jupytext:
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

# +
# %matplotlib notebook

import numpy as np
import os, sys
import re, math, random
import matplotlib.pyplot as plt 
from matplotlib import animation
import cv2
import glob
from IPython.core.display import display, HTML
display(HTML("<style>.container { width:100% !important; }</style>"))
# -

sim_id = 'last'
sector = 'frontCornerLeft'
car_name = 'Alfred'

# +
base_output_dir = os.path.join('../carla_scripts/output/', car_name, sector)

if sim_id == 'last':
    output_path = os.path.join(os.getcwd(), base_output_dir)
    print(output_path)
    list_of_clips = glob.glob(output_path + '/*')
    sim_id = max(list_of_clips, key=os.path.getmtime).split('/')[-1]
print(sim_id)

output_dir = os.path.join(os.getcwd(), base_output_dir, sim_id)

if sector == 'main':    
    views = [
        ['main_to_main', True],
        ['main_to_main', False],
        ['frontCornerLeft_to_main', False],
        ['frontCornerRight_to_main', False],
        ['parking_front_to_main', False]
    ]

elif sector == 'frontCornerLeft':
    views = [
        ['frontCornerLeft_to_frontCornerLeft', True],
        ['frontCornerLeft_to_frontCornerLeft', False],
        ['parking_front_to_frontCornerLeft', False],
        ['parking_left_to_frontCornerLeft', False]
    ]
    
elif sector == 'frontCornerRight':
    views = [
        ['frontCornerRight_to_frontCornerRight', True],
        ['frontCornerRight_to_frontCornerRight', False],
        ['parking_front_to_frontCornerRight', False],
        ['parking_right_to_frontCornerRight', False]
    ]

elif sector == 'rearCornerLeft':
    views = [
        ['rearCornerLeft_to_rearCornerLeft', True],
        ['rearCornerLeft_to_rearCornerLeft', False],
        ['parking_left_to_rearCornerLeft', False],
        ['parking_rear_to_rearCornerLeft', False]
    ]

elif sector == 'rearCornerRight':
    views = [
        ['rearCornerRight_to_rearCornerRight', True],
        ['rearCornerRight_to_rearCornerRight', False],
        ['parking_right_to_rearCornerRight', False],
        ['parking_rear_to_rearCornerRight', False]
    ]


def process_npz(ax, file_path, depth_image=False):
    f = np.load(file_path)
    if 'sim_depth' in f and depth_image:
        return [ax.imshow(np.log(f['sim_depth']), cmap='Greys', origin='lower', animated=True)]
    if 'image' in f:
        return [ax.imshow(f['image'], cmap='gray', origin='lower', animated=True)]
    else:
        return []
    
    
cols = 2
rows = int(math.ceil(len(views)/cols)+1)
fig = plt.figure(figsize=(14,12))
plt.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=0.1, hspace=0.2)
limit0 = 0
limit1 = -1

ims = []
for i, view in enumerate(views):
    ax = fig.add_subplot(rows, cols, i+1)
    ax.title.set_text(view[0])
    ax.set_yticklabels([])
    ax.set_xticklabels([])
    full_view_dir = os.path.join(output_dir, view[0])
    sorted_file_names = os.listdir(full_view_dir)
    sorted_file_names.sort(key=lambda x: re.search('_(\d+)\.npz', x).group(1))
    
    for j, file_name in enumerate(sorted_file_names[limit0:limit1][::2]):
        if file_name.endswith(".npz"):
            file_path = os.path.join(full_view_dir, file_name)
            if len(ims) > j:
                ims[j] += process_npz(ax, file_path,view[1])
            else:
                ims.append(process_npz(ax, file_path, view[1]))

anim = animation.ArtistAnimation(fig, ims, interval=100, blit=True, repeat_delay=0)
plt.show()

# +
root_path = output_dir #os.path.join('../carla_scripts/output/', sim_id)
frames = set()
views_dirs = os.listdir(root_path,)
for i, view_name in enumerate(views_dirs):
    file_path = os.path.join(root_path, view_name)
    frame_nums = [re.search('_(\d+)\.npz', file_name).group(1) for file_name in os.listdir(file_path,) if 'image' in np.load(os.path.join(file_path,file_name))]
    if i == 0:
        frames = set(frame_nums)
        print('out of %d frames in %s,' % (len(frames), view_name)),
    else:
        frames = frames.intersection(set(frame_nums))
print('%d exist in all %d views. ' % (len(frames), len(views_dirs))),

if len(frames)>0:
    frame = random.choice(list(frames))
    print('Frame %s chosen' % frame)

# +
view1 = views[0][0]
view2 = views[2][0]
im1_path = '%s/%s/%s/%s_%s_%s.npz' % (base_output_dir, sim_id, view1, sim_id, view1, frame)
im2_path = '%s/%s/%s/%s_%s_%s.npz' % (base_output_dir, sim_id, view2, sim_id, view2, frame)

im1 = np.load(im1_path)['image']
im2 = np.load(im2_path)['image']
fig = plt.figure(figsize=(10,8))
ax1 = fig.add_subplot(211)
ax2 = fig.add_subplot(212, sharex=ax1, sharey=ax1)
ax1.title.set_text("View: %s, Frame: %s" % (view1, frame))
ax1.imshow(im1*1, cmap='gray', origin='lower')
ax2.title.set_text("View: %s, Frame: %s" % (view2, frame))
ax2.imshow(im2*1, cmap='gray', origin='lower')
# -



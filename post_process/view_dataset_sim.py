# ---
# jupyter:
#   jupytext:
#     formats: py:light
#     text_representation:
#       extension: .py
#       format_name: light
#       format_version: '1.5'
#       jupytext_version: 1.3.4
#   kernelspec:
#     display_name: Python 2
#     language: python
#     name: python2
# ---

# +
from stereo.prediction.pred_clip import pred_views
from stereo.common.vis_utils import view_pptk3d

import numpy as np
import matplotlib
matplotlib.use('nbagg')
# %matplotlib notebook
from matplotlib import pyplot as plt
import matplotlib.animation

from stereo.common.vis_utils import warp_source2target_by_Z as warp
from stereo.data.lidar_utils import interp_lidar

from stereo.data.dataset_utils import ViewDatasetIndex
from random import sample
# %load_ext autoreload
# %autoreload 2


from stereo.data.view_generator import ViewGenerator
from stereo.prediction.predictor import StereoPredictor


# -

def sim2pcd(view):
    x,y = np.meshgrid(np.arange(view['image'].shape[1],dtype=np.float64), np.arange(view['image'].shape[0],dtype=np.float64))
    x = (x - view['origin'][0]) / view['focal']
    y = (y - view['origin'][1]) / view['focal']
    Z = view['sim_depth'].flatten()
    X = Z*x.flatten()
    Y = Z*y.flatten()
    pcd = np.c_[X,Y,Z]
    grayscale = view['image'].flatten()/255.
    return pcd[Z<100,:], grayscale[Z<100]


model_conf_file = '/homes/guygo/gitlab/stereo/stereo/models/conf/diet_main_sm_reg_sim_v0.0.2.3.json'
section_name = 'main'
predictor = StereoPredictor(model_conf_file, sector_name=section_name)

# dataset_dir = '/mobileye/algo_STEREO3/stereo/data/simulator/v2'
dataset_dir = '/homes/guygo/sim_sample/rcl_linked/'
test_percentage = 0.1
# dataSetIndex = ViewDatasetIndex(dataset_dir)
dataSetIndex = ViewDatasetIndex(dataset_dir,rebuild_index=True, save_index=True,  
                                test_percentage=test_percentage)

# +
# views_names_str = "main_to_main,frontCornerLeft_to_main,frontCornerRight_to_main,parking_front_to_main"
views_names_str = "rearCornerLeft_to_rearCornerLeft,parking_rear_to_rearCornerLeft,parking_left_to_rearCornerLeft"
views_names = views_names_str.split(',')

cntr_cams = ['rearCornerLeft']
cntr_views_names = {}
for cam in cntr_cams:
    cntr_views_names[cam] = cam+'_to_'+cam
    
left_view = {'rearCornerLeft_to_rearCornerLeft': 'parking_rear_to_rearCornerLeft'}

right_view = {'rearCornerLeft_to_rearCornerLeft': 'parking_rear_to_rearCornerLeft'}

# extra_view = {'rearCornerLeft_to_rearCornerLeft': 'parking_left_to_rearCornerLeft'}
# -

dataSetIndex.test_frames_list

# +
tt = 'test'
if 'train' in tt:
    frames_list = dataSetIndex.train_frames_list
else:
    frames_list = dataSetIndex.test_frames_list
frame = sample(frames_list, 1)[0]

clip_name = 'Town02_1585735847'
gi = 4709
frame = '%s@gi@%07d' %(clip_name, gi)

views = dataSetIndex.read_views(views_names, frame)

#print views['main_to_main']['clip_name'], views['main_to_main']['grab_index']

#Town04_9 13689

# -

cntr_cam = cntr_cams[0]
cntr_view = cntr_views_names[cntr_cam]
im_lidar = views[cntr_view]['sim_depth']
im_lidar[im_lidar == 0.] = np.nan
fig = plt.figure()
ax1 = fig.add_subplot(121, title=cntr_cam)
ax2 = fig.add_subplot(122, title=cntr_cam)
ax1.imshow(views[cntr_view]['image'], origin='lower', cmap='gray')
ax2.imshow(1./im_lidar, origin='lower', alpha=0.5)

pcd, grayscale, _, _ = pred_views(views, predictor, 'main')
pcd = pcd['main']
grayscale = grayscale['main']
# view_pptk3d(pcd, grayscale, fix_coords_type=1,save_to_tmp=True)


pcd_sim, grayscale_sim = sim2pcd(views['main_to_main'])
pcds = np.concatenate([pcd, pcd_sim])
grayscale_pred_only = np.concatenate([grayscale, np.zeros_like(grayscale_sim)])
grayscale_sim_only = np.concatenate([np.zeros_like(grayscale), grayscale_sim])
grayscales = [grayscale_pred_only, grayscale_sim_only]
viewer = view_pptk3d(pcds, grayscale_pred_only, fix_coords_type=1)
viewer.viewer.attributes(grayscale_sim_only, grayscale_pred_only)

# +

Z = views[cntr_view]['sim_depth']
RT_cntr_to_left = np.matmul(np.linalg.inv(views[left_view[cntr_view]]['RT_view_to_main']), 
               views[cntr_view]['RT_view_to_main'])
RT_cntr_to_right = np.matmul(np.linalg.inv(views[right_view[cntr_view]]['RT_view_to_main']), 
               views[cntr_view]['RT_view_to_main'])
try:
    RT_cntr_to_extra = np.matmul(np.linalg.inv(views[extra_view[cntr_view]]['RT_view_to_main']), 
               views[cntr_view]['RT_view_to_main'])
except:
    RT_cntr_to_extra = None
    
left_w = warp(views[left_view[cntr_view]]['image'], 
            views[left_view[cntr_view]]['origin'], 
            views[left_view[cntr_view]]['focal'], 
            RT_cntr_to_left, Z)
right_w = warp(views[right_view[cntr_view]]['image'], 
            views[right_view[cntr_view]]['origin'], 
            views[right_view[cntr_view]]['focal'], 
            RT_cntr_to_right, Z)
if RT_cntr_to_extra is not None:
    extra_w = warp(views[extra_view[cntr_view]]['image'], 
              views[extra_view[cntr_view]]['origin'], 
              views[extra_view[cntr_view]]['focal'], 
              RT_cntr_to_extra, Z)

fig = plt.figure()
ax1 = fig.add_subplot(211)
ax2 = fig.add_subplot(212)
ims = []
ims.append([ax1.imshow(views[cntr_view]['image'], origin='lower', cmap='gray', clim=(0,255)),
           ax2.imshow(views[cntr_view]['image'], origin='lower', cmap='gray', clim=(0,255))]) 
ims.append([ax1.imshow(views[right_view[cntr_view]]['image'], origin='lower', cmap='gray', clim=(0,255)),
           ax2.imshow(right_w, origin='lower', cmap='gray', clim=(0,255))]) 
ims.append([ax1.imshow(views[left_view[cntr_view]]['image'], origin='lower', cmap='gray', clim=(0,255)),
           ax2.imshow(left_w, origin='lower', cmap='gray', clim=(0,255))])
if RT_cntr_to_extra is not None:
    ims.append([ax1.imshow(views[extra_view[cntr_view]]['image'], origin='lower', cmap='gray', clim=(0,255)),
               ax2.imshow(extra_w, origin='lower', cmap='gray', clim=(0,255))])


ani = matplotlib.animation.ArtistAnimation(fig, ims, interval=800, blit=True)
# -




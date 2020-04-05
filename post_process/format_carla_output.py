import os

import shutil
from file2path import file2path

#sim_output_dir = '/mobileye/algo_STEREO3/stereo/data/simulator/Clips'
sim_output_dir = '/homes/guygo/sim_sample/rcl'
#output_dir ='/mobileye/algo_STEREO3/stereo/data/simulator/Clips_dump1_linked'
output_dir ='/homes/guygo/sim_sample/rcl_linked'
filter_clips = []

for clip_dir in os.listdir(sim_output_dir):
    clip_dir_path = os.path.join(sim_output_dir, clip_dir)
    if os.path.isdir(clip_dir_path): #and clip_dir in filter_clips:
        #print('Scanning %s' % clip_dir)
        for cam_dir in os.listdir(clip_dir_path):
            cam_dir_path = os.path.join(clip_dir_path, cam_dir)
            if os.path.isdir(cam_dir_path):
                #print('Scanning %s' % cam_dir)
                for filename in os.listdir(cam_dir_path):
                    if 'npz' in filename:

                        new_filename = str.join('_', (filename.split('_')[:-1] +
                                                      [filename.split('_')[-1].split('.')[0].zfill(7) + '.npz']))

                        orig_file_path = os.path.join(cam_dir_path, filename)
                        new_file_path = file2path(os.path.join(output_dir, new_filename ))
                        #print('copying %s' % orig_file_path),
                        #print('into %s' % new_file_path)

                        try:
                            os.symlink(orig_file_path, new_file_path)
			   #shutil.copy(orig_file_path, new_file_path)
                        except OSError as os_err:
                            os.makedirs(os.path.dirname(new_file_path))
			    os.symlink(orig_file_path, new_file_path)
                           # shutil.copy(orig_file_path, new_file_path)


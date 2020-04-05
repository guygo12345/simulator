import os
import re
import numpy as np


def remove_bad_frames(base_path, clip, frames_nums):
    clip_path = os.path.join(base_path, clip)
    views = os.listdir(clip_path)
    for view in views:
        frames_path = os.path.join(clip_path, view)
        frames = os.listdir(frames_path)
        to_be_removed = [f for f in frames if re.findall(".*_(.*).npz", f)[0] in frames_nums]
        for frame in to_be_removed:
            frame_path = os.path.join(clip_path, view, frame)
            os.remove(frame_path)#, '%s_bad/%s' % (base_path, frame))
            # print(frame_path)

def filter_frames(base_path, clip, remove_bad=False):
    clip_path = os.path.join(base_path, clip)
    views = os.listdir(clip_path)
    filtered_frames = set()
    bad_frames = set()
    for i, view in enumerate(views):
        frames_path = os.path.join(clip_path, view)
        frames = os.listdir(frames_path)
        frames_nums = set([re.findall(".*_(.*).npz", frame)[0] for frame in frames])
        if i == 0:
            filtered_frames = frames_nums
        else:
            bad_frames.update(filtered_frames.difference(frames_nums))
            bad_frames.update(frames_nums.difference(filtered_frames))
            filtered_frames = filtered_frames.intersection(frames_nums)
    if remove_bad:
        remove_bad_frames(base_path, clip, bad_frames)
    return filtered_frames


def main():
    base_output_path = '../carla_scripts/output'
    cars = os.listdir(base_output_path)
    for car in cars:
        # print("Car: " + car)
        sectors_path = os.path.join(base_output_path, car)
        sectors = os.listdir(sectors_path)
        for sector in sectors:
            # print("\tSector: " + sector)
            clips_path = os.path.join(sectors_path, sector)
            clips = os.listdir(clips_path)
            for clip in clips:
                # print("\t\tClip: " + clip)
                filtered_frames = filter_frames(clips_path, clip, remove_bad=True)


if __name__ == '__main__':
    main()
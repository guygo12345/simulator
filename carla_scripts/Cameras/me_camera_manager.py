import os
import json
import numpy as np
import cv2
import random
from scipy.spatial.transform import Rotation
from os.path import join, exists
import carla


def to_bgra_array(image, scale):
    """Convert a CARLA raw image to a BGRA numpy array."""
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    return cv2.resize(array, (0, 0), fx=float(1 / scale), fy=float(1 / scale)).astype(np.uint8)


def depth_to_array(image, scale=None):
    """
    Convert an image containing CARLA encoded depth-map to a 2D array containing
    the depth value of each pixel normalized between [0.0, 1.0].
    """
    array = to_bgra_array(image, scale)
    array = array.astype(np.float32)
    depth = np.dot(array[:, :, :3], [65536.0, 256.0, 1.0]) # (R + G * 256 + B * 256 * 256)
    depth /= 16777.215  # (256.0 * 256.0 * 256.0 - 1.0) / 1000
    if scale:
        return cv2.resize(depth, (0,0), fx=float(1/scale), fy=float(1/scale), interpolation=cv2.INTER_NEAREST).astype(np.float32)
    else:
        return depth.astype(np.float32)


def matrix_from_euler_angles(euler_angles, negate_yaw=False):
    pitch, yaw, roll = euler_angles
    if negate_yaw:
        yaw = -yaw
    r = Rotation.from_euler('xyz', (pitch, yaw, roll), degrees=True)
    matrix = r.as_matrix()
    return matrix


def euler_angles_from_matrix(rotation_matrix, negate_yaw=False):
    r = Rotation.from_matrix(rotation_matrix)
    euler_angles = r.as_euler('xyz')  # X (left-right), Y (up-down), Z (forward-backward)
    pitch, yaw, roll = np.rad2deg(euler_angles)  # Y (left-right), Z (up-down), X (forward-backward)
    if negate_yaw:
        yaw = -yaw
    return pitch, yaw, roll


class MESector(object):
    def __init__(self, sector_name, R_to_main, origin, width, height, focal, scale):
        self.sector_name = sector_name
        self.R_to_main = R_to_main
        self.origin = origin
        self.width = width
        self.height = height
        self.focal = focal
        self.scale = scale

    def get_image_width(self):
        return self.width * self.scale

    def get_image_height(self):
        return self.height * self.scale

    def get_fov(self):
        return np.rad2deg(2.0 * np.arctan(self.width / (2 * self.focal)))


class MEView(object):
    def __init__(self, cam_name, T_to_main, sector):
        self.cam_name = cam_name
        self.T_to_main = T_to_main
        self.sector = sector

    def get_view_name(self):
        return "%s_to_%s" % (self.cam_name, self.sector.sector_name)

    def is_center_view(self):
        return self.cam_name == self.sector.sector_name

    def get_RT_view_to_main(self):
        R, T = matrix_from_euler_angles(self.sector.R_to_main), self.T_to_main
        return np.r_[np.c_[R, T], np.array([0, 0, 0, 1]).reshape((1, 4))]


class MECameraManager(object):
    def __init__(self, world, player, simulation_id, sector, car_name, capture_frequency=0.5, output_dir=None):
        self.world = world
        self.player = player
        self.simulation_id = simulation_id
        self.car_name = car_name
        self.car_setup = MECameraManager.get_car_setup(self.car_name)
        self.sector = None
        self.update_sector(sector)
        self.output_dir = None
        self.update_output_dir()
        self.center_view_frame_data_with_depth = None
        self.capture_frequency = capture_frequency
        self.sensors_list = []

    @staticmethod
    def config_out_dir(car_name, sector, output_dir=None):
        if output_dir:
            return output_dir
        else:
            if not exists(os.path.join('output', car_name)):
                os.mkdir(os.path.join('output', car_name))
            return os.path.join('output', car_name, sector)

    @staticmethod
    def get_car_setup(car_name):
        conf_file_path = 'Cameras/setup/{car_name}.json'.format(car_name=car_name)
        with open(conf_file_path, 'r') as f:
            setup = json.load(f)
        return setup

    def update_sector(self, sector):
        if sector == 'rand':
            sector = random.choice(list(self.car_setup['views'].keys()))
            print("Chose random sector - %s" % sector)
        self.sector = sector

    def update_output_dir(self):
        self.output_dir = MECameraManager.config_out_dir(self.car_name, self.sector)

    def get_reset_matrix(self):
        return self.car_setup['reset_matrix']

    def get_sector_setup(self, sector):
        return self.car_setup['views'][sector]

    def get_camera_location(self, cam_name):
        return self.car_setup['cameras_locations'][cam_name]

    def init_sensors(self):
        sector_setup = self.get_sector_setup(self.sector)
        reset_matrix = self.get_reset_matrix()
        me_sector = MESector(self.sector, sector_setup['R_to_main'], sector_setup['origin'],
                             sector_setup['width'], sector_setup['height'], sector_setup['focal'],
                             sector_setup['scale'])
        sector_cams = sector_setup['cams']
        for cam in sector_cams:
            me_view = MEView(cam, self.get_camera_location(cam), me_sector)
            self.init_sensor(me_view, reset_matrix)
            if me_view.is_center_view():
                self.init_sensor(me_view, reset_matrix, sensor_type="sensor.camera.depth")

    def init_sensor(self, me_view, reset_matrix, sensor_type='sensor.camera.rgb'):
        sensor_bp = self.world.get_blueprint_library().find(sensor_type)
        sensor_bp.set_attribute('image_size_x', str(me_view.sector.get_image_width()))
        sensor_bp.set_attribute('image_size_y', str(me_view.sector.get_image_height()))
        sensor_bp.set_attribute('fov', str(me_view.sector.get_fov()))
        sensor_bp.set_attribute('sensor_tick', str(self.capture_frequency))
        rel_matrix = np.matmul(me_view.get_RT_view_to_main(), reset_matrix)
        y, z, x = rel_matrix[:3, 3]
        sensor_location = carla.Location(x=x, y=y, z=z)
        negate_yaw = 'rear' in me_view.get_view_name()
        pitch, yaw, roll = me_view.sector.R_to_main
        sensor_rotation = carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)
        rel_transform = carla.Transform(sensor_location, sensor_rotation)
        sensor = self.world.spawn_actor(sensor_bp, rel_transform, attach_to=self.player)
        self.sensors_list.append(sensor)
        sensor.listen(self.get_process_func(me_view, sensor_type))

    def destroy(self):
        for sensor in self.sensors_list:
            if sensor is not None:
                sensor.destroy()
        self.sensors_list = []

    def get_process_func(self, me_view, sensor_type):
        # Makes sure output folders exist
        if not exists(self.output_dir):
            os.mkdir(self.output_dir)
        simulation_dir_path = join(self.output_dir, self.simulation_id)
        if not exists(simulation_dir_path):
            os.mkdir(simulation_dir_path)
        cam_dir_path = join(simulation_dir_path, me_view.get_view_name())
        if not exists(cam_dir_path):
            os.mkdir(cam_dir_path)

        # Generates the actual listen function run on each clock tick.
        def process(image):
            gi = image.frame
            file_path = join(cam_dir_path, '%s_%s_%07d.npz' %
                             (self.simulation_id, me_view.get_view_name(), gi))
            data = {
                'origin': me_view.sector.origin,
                'focal': me_view.sector.focal,
                'fov': me_view.sector.get_fov(),
                'grab_index': gi,
                'RT_view_to_main': me_view.get_RT_view_to_main(),
                'clip_name': self.simulation_id
            }
            scale = me_view.sector.scale
            if sensor_type == 'sensor.camera.rgb':
                data['image'] = np.flip(cv2.cvtColor(to_bgra_array(image, scale=scale),
                                                     cv2.COLOR_BGRA2GRAY), 0)
            elif sensor_type == 'sensor.camera.depth':
                data['sim_depth'] = np.flip(depth_to_array(image, scale=scale), 0)
            self.save_frame(file_path, data)

        return process

    def save_frame(self, file_path, data):
        center_view = "%s_to_%s" % (self.sector, self.sector)
        if center_view in file_path:
            if self.center_view_frame_data_with_depth is None:
                self.center_view_frame_data_with_depth = data
                return
            else:
                data = {**self.center_view_frame_data_with_depth, **data}
                self.center_view_frame_data_with_depth = None
        np.savez(file_path, **data)
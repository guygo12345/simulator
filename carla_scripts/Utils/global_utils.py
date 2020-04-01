import sys
import json
import cv2
import numpy as np
import re
from subprocess import Popen
from time import sleep
import psutil
from scipy.spatial.transform import Rotation

carla_path = '/home/itayb/simulator/carla_0.98/PythonAPI/carla/dist/carla-0.9.8-py3.6-linux-x86_64.egg'
sys.path.append(carla_path)
import carla

SERVER_PATH = '/home/itayb/simulator/carla_0.98/Dist/CARLA_Shipping_0.9.8-34-g4bc53a7f/LinuxNoEditor/CarlaUE4.sh'


def get_carla_client(host, port, use_ini_file=True, use_opengl=True, low_quality=False):
    running_processes = list((p.name() for p in psutil.process_iter()))
    server_is_running = len([p for p in running_processes if 'CarlaUE4' in p]) != 0
    if not server_is_running:
        print('Server is down, Trying to run it')
        run_carla_server(use_ini_file, use_opengl, low_quality)
        sleep(8)
    carla_client = carla.Client(host, port)
    return carla_client


def run_carla_server(use_ini_file=True, use_opengl=True, low_quality=False):
    command = SERVER_PATH
    if use_ini_file:
        command += ' -carla-settings=CarlaSettings.ini'
    if use_opengl:
        command += ' -opengl'
    if low_quality:
        command += ' -quality-level=Low'
    Popen(command.split(" "))


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


def get_camera_setups(sector='main', car_name='Alfred'):
    conf_file_path = 'cameras/{car_name}.json'.format(car_name=car_name)
    with open(conf_file_path, 'r') as f:
        sensors = json.load(f)
    return sensors[sector]


def get_fov(width, focal):
    return np.rad2deg(2.0 * np.arctan(width / (2 * focal)))


def adjust_axes(matrix):
    flip_matrix = np.array([
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]])
    matrix = np.matmul(flip_matrix, matrix)
    return matrix


def get_reset_matrix():
    reset_matrix = np.array([
        [1.0, 0.0, 0.0, 0.0],  # X
        [0.0, 1.0, 0.0, 1.5],  # Y
        [0.0, 0.0, 1.0, 0.25],  # Z
        [0.0, 0.0, 0.0, 1.0]])
    return reset_matrix


def to_bgra_array(image, scale=None):
    """Convert a CARLA raw image to a BGRA numpy array."""
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    if scale:
        return cv2.resize(array, (0,0), fx=float(1/scale), fy=float(1/scale)).astype(np.uint8)
    else:
        return array.astype(np.uint8)


def depth_to_array(image, scale=None):
    """
    Convert an image containing CARLA encoded depth-map to a 2D array containing
    the depth value of each pixel normalized between [0.0, 1.0].
    """
    array = to_bgra_array(image)
    array = array.astype(np.float32)
    depth = np.dot(array[:, :, :3], [65536.0, 256.0, 1.0]) # (R + G * 256 + B * 256 * 256)
    depth /= 16777.215  # (256.0 * 256.0 * 256.0 - 1.0) / 1000
    if scale:
        return cv2.resize(depth, (0,0), fx=float(1/scale), fy=float(1/scale), interpolation=cv2.INTER_NEAREST).astype(np.float32)
    else:
        return depth.astype(np.float32)


# def get_reverse_RT(RT_view_to_main):
#     R, T = RT_view_to_main[:3, :3], RT_view_to_main[:3, 3]
#     reverse_RT_view_to_main = np.r_[np.c_[R.T, np.matmul(-R.T,T)], RT_view_to_main[3].reshape((1,4))]
#     return reverse_RT_view_to_main


def extract_transform_from_matrix(matrix, negate_yaw=False):
    rotation_matrix = matrix[:3, :3]
    r = Rotation.from_matrix(rotation_matrix)
    euler_angles = r.as_euler('xyz')  # X (left-right), Y (up-down), Z (forward-backward)
    pitch, yaw, roll = np.rad2deg(euler_angles)  # Y (left-right), Z (up-down), X (forward-backward)
    if negate_yaw:
        yaw = -yaw
    ty, tz, tx = matrix[:3, 3]  # From ME to Carla coordinate system
    transform = carla.Transform(
        carla.Location(x=tx, y=ty, z=tz),
        carla.Rotation(pitch=pitch, yaw=yaw, roll=roll))
    return transform

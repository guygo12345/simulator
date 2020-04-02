import sys
import numpy as np
import re
from subprocess import Popen
from time import sleep
import psutil
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

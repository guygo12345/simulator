from subprocess import Popen
from time import sleep
import psutil
import os
import carla

SERVER_PATH = '/opt/carla/bin/CarlaUE4.sh'


def get_carla_client(host, port, use_ini_file=True, use_opengl=True, low_quality=False, off_screen=False):
    running_processes = list((p.name() for p in psutil.process_iter()))
    server_is_running = len([p for p in running_processes if 'CarlaUE4' in p]) != 0
    if not server_is_running:
        print('Server is down, Trying to run it')
        run_carla_server(use_ini_file, use_opengl, low_quality, off_screen)
        sleep(5)
    carla_client = carla.Client(host, port)
    return carla_client


def run_carla_server(use_ini_file=True, use_opengl=True, low_quality=False, off_screen=False):
    command = SERVER_PATH
    if use_ini_file:
        command += ' -carla-settings=CarlaSettings.ini'
    if use_opengl:
        command += ' -opengl'
    if low_quality:
        command += ' -quality-level=Low'
    if off_screen:
        command = "SDL_VIDEODRIVER=offscreen SDL_HINT_CUDA_DEVICE=0 " + command
    print(command)
    os.system(command + " &")


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

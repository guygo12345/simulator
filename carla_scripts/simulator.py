#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.


from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================

import sys
import glob
import os
sys.path.append(os.getcwd() + "/../")

try:
    sys.path.append(glob.glob('/home/itayb/simulator/carla_0.98/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
import logging
import random
import signal
import time
from os.path import join, exists
from os import mkdir
import pygame

from carla_scripts.Sensors import *
from carla_scripts.Utils import *


# ==============================================================================
# -- Simulator ---------------------------------------------------------------------
# ==============================================================================

class Simulator(object):
    def __init__(self, client, hud, args):
        carla_world = client.get_world()
        self.world = carla_world
        self.set_map(client, args.map_id)
        self.actor_role_name = args.rolename
        self.map_id = args.map_id
        self.map = self.world.get_map()
        self.simulation_id = self.map.name + '_' + str(args.id)
        self.hud = hud
        self.player = None
        self.location = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._gamma = args.gamma
        self.me_sensor_manager = None
        self.recording_enabled = False
        self.recording_start = 0
        self.world.on_tick(hud.on_world_tick)
        self.spawn_process = None
        self.spawn_interval = args.spawn_interval
        self.clip_interval = args.clip_interval
        self.me_sensor_manager = MESensorManager(self.world, self.player, simulation_id=self.simulation_id,
                                                 sector=args.sector, car_name=args.car_name)
        self.restarting = False
        self.restart()
        self.start_time = time.time()
        self.spawn_npc()
        self.next_weather(rand=True)

    def set_map(self, client, map_id):
        all_maps = client.get_available_maps()
        world_map = all_maps[map_id]
        self.world = client.load_world(world_map)

    def set_fixed_timestep(self, t=0.2):
        settings = self.world.get_settings()
        settings.fixed_delta_seconds = t
        self.world.apply_settings(settings)

    def set_sync_mode(self):
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        self.world.apply_settings(settings)

    def unset_sync_mode(self):
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        self.world.apply_settings(settings)

    def set_no_rendering(self):
        settings = self.world.get_settings()
        settings.no_rendering_mode = True
        self.world.apply_settings(settings)

    def set_rendering(self):
        settings = self.world.get_settings()
        settings.no_rendering_mode = False
        self.world.apply_settings(settings)

    def restart(self):
        if self.restarting:
            return
        self.restarting = True
        self.unset_sync_mode()
        if self.player is not None:
            self.destroy()
            self.destroy_me_sensor()
        if self.player is None:
            self.create_player()
        self.setup_sensors()
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)
        self.set_fixed_timestep()
        self.set_sync_mode()
        self.me_sensor_manager.player = self.player
        self.me_sensor_manager.init_sensors()
        self.restarting = False

    def create_player(self, on_ground=True):
        blueprint = self.create_blueprint()
        spawn_points = self.map.get_spawn_points()
        spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
        # if on_ground:
        #     spawn_point.location.z = 0.35
        while self.player is None:
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            sleep(0.1)
        self.player.set_autopilot()
        self.location = self.player.get_location()


    def setup_sensors(self):
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.set_camera_manager()

    def create_blueprint(self):
        blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        return blueprint

    def set_camera_manager(self):
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)

    def next_weather(self, reverse=False, rand=False):
        if rand:
            dry_indices = range(0, 5)
            wet_indices = range(5, len(self._weather_presets))
            select_from = dry_indices if random.random() > 0 else wet_indices  # for now we don't use wet maps
            self._weather_index = random.choice(select_from)
        else:
            self._weather_index += -1 if reverse else 1
            self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = RadarSensor(self.player)
        elif self.radar_sensor.sensor is not None:
            self.radar_sensor.sensor.destroy()
            self.radar_sensor = None

    def get_elapsed_seconds(self):
        return time.time() - self.start_time
        #return int(self.world.get_snapshot().elapsed_seconds)

    def restart_if_needed(self):
        need_restart = False

        if self.clip_interval:
            elapsed_seconds = self.get_elapsed_seconds()
            if elapsed_seconds > self.clip_interval + 3:
                self.simulation_id = self.map.name + '_' + str(int(time.time()))
                self.start_time = time.time()
                need_restart = True

        if self.collision_sensor.get_collision_history() or \
                len(self.lane_invasion_sensor.history) > 5:
            need_restart = True

        elif self.world.get_snapshot().frame % 10 == 0:
            if self.location == self.player.get_location():
                need_restart = True
            else:
                self.location = self.player.get_location()

        elif self.spawn_interval:
            elapsed_seconds = self.get_elapsed_seconds()
            if elapsed_seconds > self.spawn_interval + 3:
                need_restart = True

        if need_restart:
            self.restart_spawn()
            self.next_weather(rand=True)

        return need_restart

    def tick(self, clock):
        self.hud.tick(self, clock)
        self.short_traffic_lights()
        if self.restart_if_needed():
            return
        self.world.tick()

    def short_traffic_lights(self):
        if self.player.is_at_traffic_light():
            self.player.get_traffic_light().set_state(carla.TrafficLightState.Green)
        else:
            traffic_lights = self.world.get_actors().filter('traffic.traffic_light')
            for tl in traffic_lights:
                distance_from_player = tl.get_location().distance(self.player.get_location())
                if distance_from_player < 20:
                    tl.set_state(carla.TrafficLightState.Green)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy_me_sensor(self):
        if self.me_sensor_manager is not None:
            self.me_sensor_manager.destroy()

    def destroy(self):
        if self.radar_sensor is not None:
            self.toggle_radar()
        if self.player is not None:
            self.player.set_autopilot(False)
        actors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor,
            self.player]
        for actor in actors:
            if actor is not None:
                actor.destroy()
        self.player = None

    def spawn_npc(self, filterv='vehicle.*', filterw='walker.pedestrian.*'):
        num_of_actors_dict = {
            0: [100, 20, True],
            1: [100, 100, False],
            2: [80, 100, False],
            3: [60, 100, False],
            4: [100, 100, False],
            5: [100, 100, False],
            6: [30, 40, False],
        }
        num_of_vehicles, num_of_walkers, safe = num_of_actors_dict[self.map_id]
        command = "python3 ./Utils/spawn_npc.py -n %s -w %s --filterv %s --filterw %s --on_ground" %\
                  (num_of_vehicles, num_of_walkers, filterv, filterw)
        if safe:
            command += ' --safe'
        if self.spawn_process is not None:
            self.destroy_spawn()
        self.spawn_process = Popen(command.split(" "))

    def destroy_spawn(self):
        if self.spawn_process is None:
            return
        self.spawn_process.send_signal(signal.SIGINT)
        sleep(1)
        self.spawn_process = None

    def restart_spawn(self):
        self.destroy_spawn()
        self.restart()
        sleep(0.1)
        self.spawn_npc()


class MESensorManager(object):
    def __init__(self, world, player, simulation_id, sector, car_name, output_dir=None):
        self.world = world
        self.player = player
        if output_dir:
            self.output_dir = output_dir
        else:
            if not exists(os.path.join('output', car_name)):
                os.mkdir(os.path.join('output', car_name))
            self.output_dir = os.path.join('output', car_name, sector)
        self.simulation_id = simulation_id
        self.sector = sector
        self.car_name = car_name
        self.center_view_frame_data_with_depth = None
        self.sensors_list = []

    def init_sensors(self):
        for i, sensor_dict in enumerate(get_camera_setups(self.sector, self.car_name)):
            if sys.version_info[0] < 3:
                sensor_dict = {k: str(v) if type(v) is unicode else v for (k, v) in sensor_dict.items()}
            sensor_dict['fov'] = get_fov(sensor_dict['width'], sensor_dict['focal'])
            sensor = self.sensor_from_dictionary(sensor_dict)
            sensor.listen(self.get_process_func(sensor_dict))

    def destroy(self):
        for sensor in self.sensors_list:
            if sensor is not None:
                sensor.destroy()
        self.sensors_list = []

    def sensor_from_dictionary(self, d):
        if 'scale' in d:
            scale = float(d['scale'])
            image_size_x = float(d['width']) * scale
            image_size_y = float(d['height']) * scale
        else:
            image_size_x = d['width']
            image_size_y = d['height']

        sensor_bp = self.world.get_blueprint_library().find(d['sensor_type'])
        sensor_bp.set_attribute('image_size_x', str(image_size_x))
        sensor_bp.set_attribute('image_size_y', str(image_size_y))
        sensor_bp.set_attribute('fov', str(d['fov']))
        sensor_bp.set_attribute('sensor_tick', str(d['sensor_tick']))

        cam_matrix = np.array(d['RT_view_to_main'])
        rts_matrix = np.matmul(cam_matrix, get_reset_matrix())

        negate_yaw = 'rear' in str(d['view_name'])
        rel_transform = extract_transform_from_matrix(rts_matrix, negate_yaw=negate_yaw)

        sensor = self.world.spawn_actor(sensor_bp, rel_transform, attach_to=self.player)
        self.sensors_list.append(sensor)
        return sensor

    def get_process_func(self, sensor_dict):
        # Makes sure output folders exist
        if not exists(self.output_dir):
            mkdir(self.output_dir)
        simulation_dir_path = join(self.output_dir, self.simulation_id)
        if not exists(simulation_dir_path):
            mkdir(simulation_dir_path)
        cam_dir_path = join(simulation_dir_path, sensor_dict['view_name'])
        if not exists(cam_dir_path):
            mkdir(cam_dir_path)

        # Generates the actual listen function run on each clock tick.
        def process(image):
            # if sensor_dict['view_name'] == 'frontCornerRight_to_main':
            #     print('%s: Start process frame %s' % (time.time(), image.frame))
            gi = image.frame
            file_path = join(cam_dir_path, '%s_%s_%07d.npz' %
                             (self.simulation_id, sensor_dict['view_name'], gi))
            data = {
                'origin': sensor_dict['origin'],
                'focal': sensor_dict['focal'],
                'fov': sensor_dict['fov'],
                'grab_index': gi,
                'RT_view_to_main': np.array(sensor_dict['RT_view_to_main']),
                'clip_name': self.simulation_id
            }
            scale = sensor_dict['scale'] if 'scale' in sensor_dict else None
            if sensor_dict['sensor_type'] == 'sensor.camera.rgb':
                data['image'] = np.flip(cv2.cvtColor(to_bgra_array(image, scale=scale), cv2.COLOR_BGRA2GRAY), 0)
            if sensor_dict['sensor_type'] == 'sensor.camera.depth':
                data['sim_depth'] = np.flip(depth_to_array(image, scale=scale), 0)
            self.save_frame(file_path, data)
            # print('Finished process frame %s in sensor: %s - %s' % (image.frame, sensor_dict['view_name'],
            #                                                      sensor_dict['sensor_type']))

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

# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.font.init()
    simulator = None

    try:
        client = get_carla_client(args.host, args.port, low_quality=args.low_quality)
        client.set_timeout(10.0)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        simulator = Simulator(client, hud, args)
        controller = KeyboardControl(simulator, args.autopilot)

        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(60)
            if controller.parse_events(client, simulator, clock):
                return
            simulator.tick(clock)
            simulator.render(display)
            pygame.display.flip()

    finally:

        if (simulator and simulator.recording_enabled):
            client.stop_recorder()

        if simulator is not None:
            simulator.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        type=bool,
        default=True,
        help='enable autopilot')
    argparser.add_argument(
        '-l', '--low_quality',
        type=bool,
        default=False,
        help='run in low quality')
    argparser.add_argument(
        '-s', '--spawn_interval',
        type=int,
        default=0,
        help='spawn every x seconds')
    argparser.add_argument(
        '-c', '--clip_interval',
        type=int,
        default=0,
        help='spawn every x seconds')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--sector',
        metavar='Sector',
        default='main',
        help='The sector of cameras')
    argparser.add_argument(
        '--car_name',
        metavar='car_name',
        default='Alfred',
        help='The ME car to simulate')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--map_id',
        metavar='MAPID',
        type=int,
        default=0,
        help='map index from world.get_maps()')
    argparser.add_argument(
        '-i', '--id',
        metavar='I',
        default=int(time.time()),
        type=int,
        help='Simulation ID, used for naming output npz files')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()

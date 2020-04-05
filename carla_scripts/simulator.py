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

import carla
import random
import signal
import time

from carla_scripts.Sensors import *
from carla_scripts.cameras import *
from carla_scripts.carla_utils import *

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
        self.debug = args.debug
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
        self.closest_vehicle_distance = None
        self.spawn_interval = args.spawn_interval
        self.clip_interval = args.clip_interval
        self.me_sensor_manager = MECameraManager(self.world, self.player, simulation_id=self.simulation_id,
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
        self.me_sensor_manager.simulation_id = self.simulation_id
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

    def get_closest_vehicle_distance(self):
        if self.hud.closest_vehicle_distance is None:
            return 100000
        return self.hud.closest_vehicle_distance

    def get_elapsed_seconds(self):
        return time.time() - self.start_time
        #return int(self.world.get_snapshot().elapsed_seconds)

    def restart_if_needed(self):
        need_restart = False

        if self.get_closest_vehicle_distance() <= 2.5:
            if self.hud.closest_vehicle_distance != self.closest_vehicle_distance:
                self.closest_vehicle_distance = self.hud.closest_vehicle_distance
                self.spawn_npc()

        if self.clip_interval:
            elapsed_seconds = self.get_elapsed_seconds()
            if elapsed_seconds > self.clip_interval + 3:
                print("Clip interval reached")
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

    def draw_sensors(self):
        for sensor in self.me_sensor_manager.sensors_list:
            self.world.debug.draw_box(carla.BoundingBox(sensor.get_transform().location,
                                                        carla.Vector3D(0.3, 0.1, 0.1)),
                                                        sensor.get_transform().rotation, 0.03,
                                                        carla.Color(255, 0, 0, 0), 0.01)

    def tick(self, clock):
        self.hud.tick(self, clock)
        self.short_traffic_lights()
        if self.restart_if_needed():
            return
        if self.debug:
            self.draw_sensors()
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

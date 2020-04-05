import sys
import os
sys.path.append(os.getcwd() + "/../")
import argparse
import logging
import pygame
import time
from carla_scripts.simulator import Simulator
from carla_scripts.Utils import *


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


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '--car_name',
        metavar='car_name',
        default='Alfred',
        help='The ME car to simulate')
    argparser.add_argument(
        '--sector',
        metavar='Sector',
        default='main',
        help='The sector of Cameras. "rand" for random choice')
    argparser.add_argument(
        '--map_id',
        metavar='MAPID',
        type=int,
        default=0,
        help='map index from world.get_maps(). currently available: 0-6')
    argparser.add_argument(
        '-c', '--clip_interval',
        type=int,
        default=0,
        help='Start new clip every x seconds')
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
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
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
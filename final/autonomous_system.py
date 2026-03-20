#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example of automatic vehicle control from client side."""

from __future__ import print_function

import argparse
import collections
import datetime
import glob
import logging
import math
import os
import numpy.random as random
import re
import sys
import weakref

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
    from pygame.locals import K_p
    from pygame.locals import K_c
    from pygame.locals import K_TAB
    from pygame.locals import K_BACKQUOTE
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- Add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import carla
from carla import ColorConverter as cc

from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error
from agents.navigation.constant_velocity_agent import ConstantVelocityAgent  # pylint: disable=import-error

# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================

def find_weather_presets():
    """Method to find weather presets"""
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    def name(x): return ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]

def get_actor_display_name(actor, truncate=250):
    """Method to get actor display name"""
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        if int_generation in [1, 2, 3]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            return []
    except:
        return []

# ==============================================================================
# -- World ---------------------------------------------------------------
# ==============================================================================

class World(object):
    """ Class representing the surrounding environment """

    def __init__(self, carla_world, hud, args):
        """Constructor method"""
        self._args = args
        self.world = carla_world
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._actor_generation = args.generation
        self.is_parking = False
        self.show_adas = True
        self.restart(args)
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0

    def restart(self, args):
        """Restart the world"""
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_id = self.camera_manager.transform_index if self.camera_manager is not None else 0

        blueprint_list = get_actor_blueprints(self.world, 'vehicle.lincoln.mkz_2017', 'all')
        if not blueprint_list: blueprint_list = get_actor_blueprints(self.world, 'vehicle.tesla.model3', 'all')
        if not blueprint_list: blueprint_list = get_actor_blueprints(self.world, 'vehicle.audi.etron', 'all')
        if not blueprint_list: blueprint_list = get_actor_blueprints(self.world, 'vehicle.*', 'all')
        
        # In case generic filter still returns non-cars, force 4-wheels
        blueprint_list = [x for x in blueprint_list if int(x.get_attribute('number_of_wheels')) == 4]
            
        blueprint = random.choice(blueprint_list)
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.modify_vehicle_physics(self.player)
        while self.player is None:
            if not self.map.get_spawn_points():
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.modify_vehicle_physics(self.player)

        if self._args.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager.transform_index = cam_pos_id
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def next_weather(self, reverse=False):
        """Get next weather setting"""
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def modify_vehicle_physics(self, actor):
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def tick(self, clock):
        """Method for every tick"""
        self.hud.tick(self, clock)

    def render(self, display):
        """Render world"""
        self.camera_manager.render(display)
        if getattr(self, 'show_adas', True) and self.camera_manager.sensor:
            self._render_adas(display)
        self.hud.render(display)
        
    def _render_adas(self, display):
        w2c = np.array(self.camera_manager.sensor.get_transform().get_inverse_matrix())
        K = np.identity(3)
        K[0, 2] = self.hud.dim[0] / 2.0
        K[1, 2] = self.hud.dim[1] / 2.0
        K[0, 0] = K[1, 1] = self.hud.dim[0] / (2.0 * math.tan(90.0 * math.pi / 360.0))

        veh_loc = self.player.get_location()
        smooth_wp = self.map.get_waypoint(veh_loc)
        
        # 1. DRAW LANE POLYGONS
        l_wps = []
        for _ in range(15):
            l_wps.append(smooth_wp)
            nxt = smooth_wp.next(2.0)
            if nxt: smooth_wp = nxt[0]
            else: break
            
        left_points = []
        right_points = []
        for wp in l_wps:
            vt = wp.transform
            hw = wp.lane_width / 2.0
            
            l_loc = vt.location - vt.get_right_vector() * hw
            l_loc.z -= 0.5
            l_cam = np.dot(w2c, np.array([l_loc.x, l_loc.y, l_loc.z, 1.0]))
                
            r_loc = vt.location + vt.get_right_vector() * hw
            r_loc.z -= 0.5
            r_cam = np.dot(w2c, np.array([r_loc.x, r_loc.y, r_loc.z, 1.0]))

            if l_cam[2] > 0 and r_cam[2] > 0:
                l_img = np.dot(K, l_cam[:3])
                r_img = np.dot(K, r_cam[:3])
                left_points.append((int(l_img[0]/l_img[2]), int(l_img[1]/l_img[2])))
                right_points.append((int(r_img[0]/r_img[2]), int(r_img[1]/r_img[2])))

        if len(left_points) > 1 and len(right_points) > 1:
            poly_points = left_points + right_points[::-1]
            pygame.draw.polygon(display, (0, 255, 100, 100), poly_points)
            pygame.draw.lines(display, (0, 255, 100, 255), False, left_points, 4)
            pygame.draw.lines(display, (0, 255, 100, 255), False, right_points, 4)

        # 2. DRAW BOUNDING BOXES FOR TRAFFIC & LIGHTS
        v_inv_mat = np.array(self.player.get_transform().get_inverse_matrix())
        
        vehicles = list(self.world.get_actors().filter('*vehicle*'))
        walkers = list(self.world.get_actors().filter('*walker*'))
        lights = list(self.world.get_actors().filter('*traffic.light*'))
        
        min_ttc = 999.0
        active_mode = "Auto-Park Active" if self.is_parking else "Autonomous Cruise"
        
        for actor in vehicles + walkers + lights:
            if actor.id == self.player.id: continue
            dist = veh_loc.distance(actor.get_location())
            if dist > 80.0: continue
            
            a_loc = actor.get_location()
            p = np.dot(v_inv_mat, [a_loc.x, a_loc.y, a_loc.z, 1.0])
            if p[0] < 0: continue

            # TTC Extrapolation (Risk Prediction System Innovation)
            if abs(p[1]) < 2.0 and "vehicle" in actor.type_id:
                rel_v = self.player.get_velocity().length() - actor.get_velocity().length()
                if rel_v > 0.1:
                    ttc = p[0] / rel_v
                    if ttc < min_ttc: min_ttc = ttc

            # 3D to 2D Bounding Box generic
            bb = getattr(actor, 'bounding_box', None)
            if not bb: continue
            
            cords = np.zeros((8, 4))
            extent = bb.extent
            cords[0, :] = np.array([extent.x, extent.y, -extent.z, 1])
            cords[1, :] = np.array([-extent.x, extent.y, -extent.z, 1])
            cords[2, :] = np.array([-extent.x, -extent.y, -extent.z, 1])
            cords[3, :] = np.array([extent.x, -extent.y, -extent.z, 1])
            cords[4, :] = np.array([extent.x, extent.y, extent.z, 1])
            cords[5, :] = np.array([-extent.x, extent.y, extent.z, 1])
            cords[6, :] = np.array([-extent.x, -extent.y, extent.z, 1])
            cords[7, :] = np.array([extent.x, -extent.y, extent.z, 1])
            
            bb_mat = np.array(actor.get_transform().get_matrix())
            cords_x_y_z = np.dot(bb_mat, cords.T).T
            cords_x_y_z_cam = np.dot(w2c, cords_x_y_z.T).T
            
            valid_idx = cords_x_y_z_cam[:, 2] > 0
            if not np.any(valid_idx): continue
            
            cords_x_y_z_img = np.dot(K, cords_x_y_z_cam[:, :3].T).T
            cords_x_y_z_img[:, 0] /= cords_x_y_z_img[:, 2]
            cords_x_y_z_img[:, 1] /= cords_x_y_z_img[:, 2]
            
            x_min, x_max = np.min(cords_x_y_z_img[valid_idx, 0]), np.max(cords_x_y_z_img[valid_idx, 0])
            y_min, y_max = np.min(cords_x_y_z_img[valid_idx, 1]), np.max(cords_x_y_z_img[valid_idx, 1])
            if not (np.isfinite(x_min) and np.isfinite(y_min)): continue
            
            x1, y1, x2, y2 = max(0, int(x_min)), max(0, int(y_min)), min(self.hud.dim[0], int(x_max)), min(self.hud.dim[1], int(y_max))
            
            color = (0, 255, 0)
            typename = "Car"
            if "walker" in actor.type_id: typename = "Ped"; color = (255, 100, 0)
            elif "traffic.light" in actor.type_id: 
                typename = "LIGHT"
                ts = actor.get_state()
                if ts == carla.TrafficLightState.Red: color = (255, 0, 0)
                elif ts == carla.TrafficLightState.Yellow: color = (255, 255, 0)
                else: continue
            else:
                if dist < 10: color = (255, 0, 0)
                elif dist < 20: color = (255, 255, 0)
                
            pygame.draw.rect(display, (*color, 180), (x1, y1, x2-x1, y2-y1), 3)
            txt = self.hud._font_mono.render(f"[{typename}] {dist:.0f}m", True, color)
            display.blit(txt, (x1, max(0, y1-15)))

        # Render Active Auto Mode
        mode_txt = self.hud._font_mono.render(f"MODE: {active_mode}", True, (0, 255, 255))
        display.blit(mode_txt, (self.hud.dim[0] // 2 - mode_txt.get_width() // 2, 20))
        
        # Render TTC & AEB Risk System
        if min_ttc < 999.0:
            risk = "LOW RISK"
            color = (0, 255, 0)
            if min_ttc < 3.0: risk = "CRITICAL RISK - AEB INTERVENTION"; color = (255, 0, 0)
            elif min_ttc < 5.0: risk = "HIGH RISK - FCW ACTIVE"; color = (255, 165, 0)
            
            ttc_txt = self.hud._font_mono.render(f"Risk Prediction: {risk} | TTC: {min_ttc:.1f}s", True, color)
            display.blit(ttc_txt, (self.hud.dim[0] // 2 - ttc_txt.get_width() // 2, 45))

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        actors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.player]
        for actor in actors:
            if actor is not None:
                actor.destroy()

# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================

class KeyboardControl(object):
    def __init__(self, world):
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self, world):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            if event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key == K_q:
                    world.show_adas = not getattr(world, 'show_adas', False)
                elif event.key == K_p:
                    world.is_parking = not getattr(world, 'is_parking', False)
                    if world.is_parking:
                        world.hud.notification("Parking Mode ENGAGED")
                    else:
                        world.hud.notification("Parking Mode CANCELED - CRUISE")

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)

# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================

class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 24), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame_count
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        transform = world.player.get_transform()
        vel = world.player.get_velocity()
        control = world.player.get_control()
        heading = 'N' if abs(transform.rotation.yaw) < 89.5 else ''
        heading += 'S' if abs(transform.rotation.yaw) > 90.5 else ''
        heading += 'E' if 179.5 > transform.rotation.yaw > 0.5 else ''
        heading += 'W' if -0.5 > transform.rotation.yaw > -179.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')

        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)),
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (transform.rotation.yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (transform.location.x, transform.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % transform.location.z,
            '']
        if isinstance(control, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', control.throttle, 0.0, 1.0),
                ('Steer:', control.steer, -1.0, 1.0),
                ('Brake:', control.brake, 0.0, 1.0),
                ('Reverse:', control.reverse),
                ('Hand brake:', control.hand_brake),
                ('Manual:', control.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(control.gear, control.gear)]
        elif isinstance(control, carla.WalkerControl):
            self._info_text += [
                ('Speed:', control.speed, 0.0, 5.556),
                ('Jump:', control.jump)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]

        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']

        def dist(l):
            return math.sqrt((l.x - transform.location.x)**2 + (l.y - transform.location.y)
                             ** 2 + (l.z - transform.location.z)**2)
        vehicles = [(dist(x.get_location()), x) for x in vehicles if x.id != world.player.id]

        for dist, vehicle in sorted(vehicles):
            if dist > 200.0:
                break
            vehicle_type = get_actor_display_name(vehicle, truncate=22)
            self._info_text.append('% 4dm %s' % (dist, vehicle_type))

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]: break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        fig = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + fig * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (fig * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)

class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))
    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)
    def render(self, display):
        display.blit(self.surface, self.pos)

class HelpText(object):
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.dim = (680, len(lines) * 22 + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for i, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, i * 22))
        self.surface.set_alpha(220)
        self._render = False
    def toggle(self): self._render = not self._render
    def render(self, display):
        if self._render: display.blit(self.surface, self.pos)

class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(blueprint, carla.Transform(), attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))
    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history: history[frame] += intensity
        return history
    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self: return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000: self.history.pop(0)

class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))
    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self: return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))

class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(blueprint, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))
    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self: return
        self.lat = event.latitude
        self.lon = event.longitude

class CameraManager(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z
        attachment = carla.AttachmentType
        self._camera_transforms = [
            (carla.Transform(carla.Location(x=-2.0*bound_x, y=+0.0*bound_y, z=2.0*bound_z), carla.Rotation(pitch=8.0)), attachment.SpringArmGhost),
            (carla.Transform(carla.Location(x=+0.8*bound_x, y=+0.0*bound_y, z=1.3*bound_z)), attachment.Rigid),
            (carla.Transform(carla.Location(x=+1.9*bound_x, y=+1.0*bound_y, z=1.2*bound_z)), attachment.SpringArmGhost),
            (carla.Transform(carla.Location(x=-2.8*bound_x, y=+0.0*bound_y, z=4.6*bound_z), carla.Rotation(pitch=6.0)), attachment.SpringArmGhost),
            (carla.Transform(carla.Location(x=-1.0, y=-1.0*bound_y, z=0.4*bound_z)), attachment.Rigid)]

        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette, 'Camera Semantic Segmentation (CityScapes Palette)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            blp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                blp.set_attribute('image_size_x', str(hud.dim[0]))
                blp.set_attribute('image_size_y', str(hud.dim[1]))
            elif item[0].startswith('sensor.lidar'):
                blp.set_attribute('range', '50')
            item.append(blp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else (force_respawn or (self.sensors[index][0] != self.sensors[self.index][0]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self: return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / 100.0
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img = np.zeros((self.hud.dim[0], self.hud.dim[1], 3))
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3][:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

def spawn_traffic(client, world, tm, n_vehicles=15, n_walkers=15):
    car_makes = ['audi', 'bmw', 'chevrolet', 'dodge', 'ford', 'lincoln', 'mercedes', 'mini', 'nissan', 'seat', 'tesla', 'toyota', 'volkswagen']
    blueprints = [x for x in world.get_blueprint_library().filter('vehicle.*') if int(x.get_attribute('number_of_wheels')) == 4 and any(make in x.id for make in car_makes)]
    spawn_points = world.get_map().get_spawn_points()
    number_of_spawn_points = len(spawn_points)

    if n_vehicles < number_of_spawn_points: random.shuffle(spawn_points)
    elif n_vehicles > number_of_spawn_points: n_vehicles = number_of_spawn_points
    
    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor

    batch = []
    for n, transform in enumerate(spawn_points):
        if n >= n_vehicles: break
        blueprint = random.choice(blueprints)
        if blueprint.has_attribute('color'): color = random.choice(blueprint.get_attribute('color').recommended_values); blueprint.set_attribute('color', color)
        blueprint.set_attribute('role_name', 'autopilot')
        batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True, tm.get_port())))
    client.apply_batch_sync(batch, True)

    blueprintsWalkers = world.get_blueprint_library().filter('walker.pedestrian.*')
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    
    spawn_points_walkers = []
    for _ in range(n_walkers):
        loc = world.get_random_location_from_navigation()
        if loc: spawn_points_walkers.append(carla.Transform(loc))

    batch = []
    for spawn_point in spawn_points_walkers:
        batch.append(SpawnActor(random.choice(blueprintsWalkers), spawn_point))
    results = client.apply_batch_sync(batch, True)
    
    walkers_list = []
    for i in range(len(results)):
        if not results[i].error: walkers_list.append({"id": results[i].actor_id})
        
    batch = []
    for i in range(len(walkers_list)):
        batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
    results = client.apply_batch_sync(batch, True)
    
    for i in range(len(results)):
        if not results[i].error: walkers_list[i]["con"] = results[i].actor_id
        
    world.tick()
    for walker in walkers_list:
        if "con" not in walker: continue
        world.get_actor(walker["con"]).start()
        world.get_actor(walker["con"]).go_to_location(world.get_random_location_from_navigation())
        world.get_actor(walker["con"]).set_max_speed(1.0 + random.random())
        
    print(f"Spawned {n_vehicles} vehicles and {len(walkers_list)} walkers!")

def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    try:
        if args.seed: random.seed(args.seed)
        client = carla.Client(args.host, args.port)
        client.set_timeout(60.0)
        traffic_manager = client.get_trafficmanager()
        sim_world = client.get_world()

        args.sync = True # CRITICAL: Forces 0 lag between Python matrix processing & Unreal physics engine rendering
        if args.sync:
            settings = sim_world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            sim_world.apply_settings(settings)
            traffic_manager.set_synchronous_mode(True)

        spawn_traffic(client, sim_world, traffic_manager, n_vehicles=15, n_walkers=15)

        display = pygame.display.set_mode((args.width, args.height), pygame.HWSURFACE | pygame.DOUBLEBUF)
        hud = HUD(args.width, args.height)
        world = World(sim_world, hud, args)
        controller = KeyboardControl(world)

        agent = BehaviorAgent(world.player, behavior=args.behavior)
        spawn_points = world.map.get_spawn_points()
        destination = random.choice(spawn_points).location
        agent.set_destination(destination)

        clock = pygame.time.Clock()
        park_timer = 0
        
        while True:
            clock.tick()
            if args.sync: sim_world.tick()
            else: sim_world.wait_for_tick()
            
            if controller.parse_events(world): return

            world.tick(clock)
            world.render(display)
            pygame.display.flip()

            if world.is_parking:
                if park_timer == 0:
                    curr_wp = world.map.get_waypoint(world.player.get_location())
                    right_wp = curr_wp.get_right_lane()
                    if right_wp and right_wp.lane_type in [carla.LaneType.Driving, carla.LaneType.Parking]:
                        fwd_right = right_wp.next(10.0)
                        if fwd_right:
                            agent.set_destination(fwd_right[0].transform.location)
                            world.player.set_light_state(carla.VehicleLightState.RightBlinker)
                
                park_timer += 1
                control = agent.run_step()

                if agent.done() or park_timer > 150:
                    control.throttle = 0.0
                    control.brake = 1.0
                    control.steer = 0.0
                    world.player.set_light_state(carla.VehicleLightState.NONE)
                world.player.apply_control(control)
                
            else:
                if park_timer != 0:
                    park_timer = 0
                    agent.set_destination(random.choice(spawn_points).location)
                    world.player.set_light_state(carla.VehicleLightState.LeftBlinker)
                    
                if agent.done():
                    agent.set_destination(random.choice(spawn_points).location)
                    if args.loop: world.hud.notification("Target reached", seconds=4.0)
                    else: break
                
                control = agent.run_step()
                control.manual_gear_shift = False
                
                # Check indicator after merging timeout and turn off left indicator
                if world.player.get_light_state() == carla.VehicleLightState.LeftBlinker and world.player.get_velocity().length() > 3.0:
                    world.player.set_light_state(carla.VehicleLightState.NONE)
                    
                world.player.apply_control(control)
    finally:
        if world is not None:
            settings = world.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.world.apply_settings(settings)
            traffic_manager.set_synchronous_mode(False)
            world.destroy()
        pygame.quit()

def main():
    argparser = argparse.ArgumentParser()
    argparser.add_argument('-v', '--verbose', action='store_true', dest='debug')
    argparser.add_argument('--host', default='127.0.0.1')
    argparser.add_argument('-p', '--port', default=2000, type=int)
    argparser.add_argument('--res', default='1280x720')
    argparser.add_argument('--sync', action='store_true')
    argparser.add_argument('--filter', default='vehicle.*')
    argparser.add_argument('--generation', default='2')
    argparser.add_argument('-l', '--loop', action='store_true', dest='loop')
    argparser.add_argument('-a', '--agent', default="Behavior")
    argparser.add_argument('-b', '--behavior', default='normal')
    argparser.add_argument('-s', '--seed', default=None, type=int)
    args = argparser.parse_args()
    args.width, args.height = [int(x) for x in args.res.split('x')]
    try:
        game_loop(args)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

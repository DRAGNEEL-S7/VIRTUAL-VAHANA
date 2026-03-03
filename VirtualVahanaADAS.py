import carla
import pygame
import numpy as np
import math
import random

# =====================================================
# INDUSTRY-GRADE FILTERS & CONTROLLERS
# =====================================================
class Kalman1D:
    def __init__(self, q=0.05, r=2.0):
        self.x = 0.0
        self.p = 1.0
        self.q = q
        self.r = r
        self.initialized = False

    def update(self, measurement):
        if not self.initialized:
            self.x = measurement
            self.initialized = True
            return self.x
        
        self.p += self.q
        k = self.p / (self.p + self.r)
        self.x += k * (measurement - self.x)
        self.p *= (1 - k)
        return self.x

# =====================================================
# CONFIGURATION
# =====================================================
SCREEN_W, SCREEN_H = 1200, 600
CAM_W, DASH_W = 800, 400

CRITICAL_TTC = 1.8    
SAFE_TTC = 3.5
ACC_MAX_SPEED = 40.0   # SPEED CAPPED TO 40 KM/H
MAX_RADAR_RANGE = 100.0

class VirtualVahanaADAS:
    def __init__(self):
        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.traffic_manager = None

        self.vehicle = None
        self.camera = None
        self.radar = None
        
        # Traffic Tracking Lists
        self.vehicles_list = []
        self.walkers = []
        self.walker_controllers = []
        
        self.surface = None
        self.speed = 0.0
        self.ttc = float('inf')
        
        self.kf_dist = Kalman1D(q=0.1, r=3.0)
        self.kf_vel = Kalman1D(q=0.05, r=2.0)
        
        self.aeb_active = False
        self.pre_warning = False
        self.acc_active = False
        
        self.cam_index = 0
        self.current_keys = []
        self.prev_control = carla.VehicleControl()

    def setup(self):
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05 
        self.world.apply_settings(settings)

        self.traffic_manager = self.client.get_trafficmanager()
        self.traffic_manager.set_synchronous_mode(True)
        self.traffic_manager.set_global_distance_to_leading_vehicle(2.5)

        bp_lib = self.world.get_blueprint_library()
        
        # 1. Spawn Ego Vehicle
        v_bp = bp_lib.find("vehicle.tesla.model3")
        ego_spawn = self.map.get_spawn_points()[10]
        self.vehicle = self.world.spawn_actor(v_bp, ego_spawn)
        self.vehicle.set_simulate_physics(True)

        # 2. Spawn Urban Traffic (Vehicles & Pedestrians)
        self.spawn_vehicle_traffic(bp_lib, num_vehicles=40, ego_spawn=ego_spawn)
        self.spawn_pedestrians(bp_lib, num_pedestrians=40)
        
        self.setup_camera()
        
        # 3. Radar Setup
        radar_bp = bp_lib.find("sensor.other.radar")
        radar_bp.set_attribute("range", str(MAX_RADAR_RANGE))
        radar_bp.set_attribute("horizontal_fov", "35") 
        radar_bp.set_attribute("vertical_fov", "15")   
        self.radar = self.world.spawn_actor(
            radar_bp, carla.Transform(carla.Location(x=2.2, z=1.0)), attach_to=self.vehicle
        )
        self.radar.listen(self._process_radar)

    def spawn_vehicle_traffic(self, bp_lib, num_vehicles, ego_spawn):
        spawn_points = self.map.get_spawn_points()
        random.shuffle(spawn_points)
        vehicle_bps = bp_lib.filter('vehicle.*')
        safe_bps = [x for x in vehicle_bps if int(x.get_attribute('number_of_wheels')) == 4]

        for sp in spawn_points:
            if len(self.vehicles_list) >= num_vehicles: break
            if sp.location.distance(ego_spawn.location) < 10.0: continue
                
            v_bp = random.choice(safe_bps)
            if v_bp.has_attribute('color'):
                color = random.choice(v_bp.get_attribute('color').recommended_values)
                v_bp.set_attribute('color', color)
                
            veh = self.world.try_spawn_actor(v_bp, sp)
            if veh:
                veh.set_autopilot(True, self.traffic_manager.get_port())
                self.vehicles_list.append(veh)
        print(f"Spawned {len(self.vehicles_list)} traffic vehicles.")

    def spawn_pedestrians(self, bp_lib, num_pedestrians):
        walker_bps = bp_lib.filter('walker.pedestrian.*')
        controller_bp = bp_lib.find('controller.ai.walker')
        
        for _ in range(num_pedestrians):
            loc = self.world.get_random_location_from_navigation()
            if loc is not None:
                spawn_point = carla.Transform(loc)
                walker_bp = random.choice(walker_bps)
                
                if walker_bp.has_attribute('is_invincible'):
                    walker_bp.set_attribute('is_invincible', 'false')
                    
                walker = self.world.try_spawn_actor(walker_bp, spawn_point)
                if walker:
                    controller = self.world.try_spawn_actor(controller_bp, carla.Transform(), walker)
                    if controller:
                        self.walkers.append(walker)
                        self.walker_controllers.append(controller)
                        
        self.world.tick()
        
        for controller in self.walker_controllers:
            controller.start()
            controller.go_to_location(self.world.get_random_location_from_navigation())
            controller.set_max_speed(1.0 + random.random())
        print(f"Spawned {len(self.walkers)} pedestrians.")

    def setup_camera(self):
        if self.camera:
            self.camera.stop()
            self.camera.destroy()
            self.world.tick()
            
        bp = self.world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(CAM_W))
        bp.set_attribute("image_size_y", str(SCREEN_H))
        
        transforms = [
            carla.Transform(carla.Location(x=-6.5, z=2.8), carla.Rotation(pitch=-15)),
            carla.Transform(carla.Location(x=1.6, z=1.2)),                             
            carla.Transform(carla.Location(z=50), carla.Rotation(pitch=-90))           
        ]
        self.camera = self.world.spawn_actor(bp, transforms[self.cam_index], attach_to=self.vehicle)
        self.camera.listen(self._process_camera)

    def _process_camera(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))[:, :, :3][:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    def _process_radar(self, data):
        raw_dist = MAX_RADAR_RANGE
        raw_vel = 0.0
        
        for d in data:
            if abs(d.azimuth) < 0.25 and abs(d.altitude) < 0.15:
                if 0.5 < d.depth < raw_dist: 
                    raw_dist = d.depth
                    raw_vel = max(0.0, -d.velocity)

        if raw_dist == MAX_RADAR_RANGE:
            self.kf_dist.initialized = False
            self.kf_vel.initialized = False
            self.radar_data = (MAX_RADAR_RANGE, 0.0)
        else:
            filt_dist = self.kf_dist.update(raw_dist)
            filt_vel = self.kf_vel.update(raw_vel)
            self.radar_data = (filt_dist, filt_vel)

    def calculate_lka(self):
        wp = self.map.get_waypoint(self.vehicle.get_location())
        fwd = wp.transform.get_forward_vector()
        loc = self.vehicle.get_location()
        
        err = (loc.y - wp.transform.location.y) * fwd.x - (loc.x - wp.transform.location.x) * fwd.y
        nudge = np.clip(-err * 0.25, -0.3, 0.3)
        return nudge

    def arbitrate(self, keys):
        dist, rel_vel = self.radar_data
        v = self.vehicle.get_velocity()
        ego_mps = math.sqrt(v.x**2 + v.y**2 + v.z**2)
        self.speed = 3.6 * ego_mps

        user_t = 0.6 if keys[pygame.K_w] else 0.0
        user_b = 1.0 if keys[pygame.K_s] else 0.0
        user_s = -0.4 if keys[pygame.K_a] else (0.4 if keys[pygame.K_d] else 0.0)

        # SPEED GOVERNOR: Limit manual speed to 40 km/h
        if self.speed >= ACC_MAX_SPEED:
            user_t = 0.0 

        final_t = user_t
        final_b = user_b
        final_s = user_s

        # 1. LANE KEEPING ASSIST (LKA)
        if abs(user_s) < 0.1:
            final_s += self.calculate_lka()

        # 2. ADAPTIVE CRUISE CONTROL (ACC)
        self.acc_active = False
        dynamic_safe_dist = max(ego_mps * 2.2, 12.0)

        if user_t > 0 and dist < dynamic_safe_dist:
            self.acc_active = True
            gap_ratio = np.clip(dist / dynamic_safe_dist, 0.0, 1.0)
            
            final_t = user_t * gap_ratio
            if gap_ratio < 0.5:
                final_b = max(user_b, 0.4 * (1.0 - gap_ratio))

        # 3. AUTONOMOUS EMERGENCY BRAKING (AEB)
        self.aeb_active = False
        self.pre_warning = False

        if dist < MAX_RADAR_RANGE:
            closing_speed = max(rel_vel, ego_mps * 0.5)
            self.ttc = dist / closing_speed if closing_speed > 0.5 else float('inf')

            # Panic threshold for pedestrians (6.0 meters) + TTC check
            if self.ttc < CRITICAL_TTC or dist < 6.0:
                self.aeb_active = True
                final_t = 0.0
                final_b = 1.0
            elif self.ttc < SAFE_TTC or dist < 12.0:
                self.pre_warning = True
        else:
            self.ttc = float('inf')

        # 4. SIGNAL SMOOTHING & ACTUATION
        alpha = 0.4
        smoothed_t = alpha * final_t + (1 - alpha) * self.prev_control.throttle
        smoothed_b = alpha * final_b + (1 - alpha) * self.prev_control.brake

        if smoothed_b > 0.1: smoothed_t = 0.0

        current_control = carla.VehicleControl(
            throttle=float(smoothed_t),
            brake=float(smoothed_b),
            steer=float(np.clip(final_s, -0.6, 0.6)),
            manual_gear_shift=False
        )
        self.vehicle.apply_control(current_control)
        self.prev_control = current_control

        try: lights = carla.VehicleLightState.NONE
        except AttributeError: lights = getattr(carla.VehicleLightState, 'None_', carla.VehicleLightState.NONE)

        if final_s < -0.15: lights |= carla.VehicleLightState.LeftBlinker
        if final_s > 0.15:  lights |= carla.VehicleLightState.RightBlinker
        if smoothed_b > 0.1: lights |= carla.VehicleLightState.Brake
        self.vehicle.set_light_state(carla.VehicleLightState(lights))

    # =================================================
    # DASHBOARD UI
    # =================================================
    def render(self, screen, font):
        if self.surface: screen.blit(self.surface, (0, 0))
        
        pygame.draw.rect(screen, (10, 14, 20), (CAM_W, 0, DASH_W, SCREEN_H))
        pygame.draw.line(screen, (0, 200, 255), (CAM_W, 0), (CAM_W, SCREEN_H), 3)
        
        screen.blit(font.render("VIRTUAL VAHANA (MANUAL + ADAS)", True, (0, 200, 255)), (CAM_W+15, 20))

        for i, k in enumerate(['W', 'A', 'S', 'D']):
            col = (0, 255, 100) if k in self.current_keys else (40, 50, 60)
            pygame.draw.rect(screen, col, (CAM_W + 90 + (i*50), 65, 40, 40), 2, border_radius=4)
            screen.blit(font.render(k, True, col), (CAM_W + 102 + (i*50), 73))

        screen.blit(font.render(f"SPEED: {int(self.speed):02d} / {int(ACC_MAX_SPEED)} KM/H", True, (255,255,255)), (CAM_W+30, 130))
        screen.blit(font.render(f"TTC  : {self.ttc:.1f} S", True, (255,255,255)), (CAM_W+30, 170))
        screen.blit(font.render(f"DIST : {self.radar_data[0]:.1f} M", True, (150, 180, 200)), (CAM_W+30, 210))

        def draw_lamp(txt, active, y_pos, color=(0, 255, 150)):
            pygame.draw.circle(screen, color if active else (30, 40, 50), (CAM_W+40, y_pos+12), 10)
            screen.blit(font.render(txt, True, (255,255,255) if active else (100,110,120)), (CAM_W+65, y_pos))

        draw_lamp("LKA ASSIST", True, 270)
        draw_lamp("ACC INTERVENTION", self.acc_active, 310, (100, 200, 255))

        if self.aeb_active:
            pygame.draw.rect(screen, (220, 20, 20), (CAM_W+20, 400, 360, 90), border_radius=8)
            screen.blit(font.render("AEB: EMERGENCY BRAKE", True, (255,255,255)), (CAM_W+45, 430))
        elif self.pre_warning:
            pygame.draw.rect(screen, (220, 180, 0), (CAM_W+20, 400, 360, 90), border_radius=8)
            screen.blit(font.render("FCW: COLLISION RISK", True, (20,20,20)), (CAM_W+60, 430))

    def run(self):
        pygame.init()
        screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
        font = pygame.font.SysFont("Verdana", 18, bold=True)
        
        self.setup()

        try:
            while True:
                self.world.tick()
                pygame.event.pump()
                keys = pygame.key.get_pressed()
                self.current_keys = [k for k in ['W','A','S','D'] if keys[getattr(pygame, f"K_{k.lower()}")] ]
                
                for event in pygame.event.get():
                    if event.type == pygame.KEYDOWN and event.key == pygame.K_c:
                        self.cam_index = (self.cam_index + 1) % 3
                        self.setup_camera()
                    if event.type == pygame.QUIT: return

                self.arbitrate(keys)
                self.render(screen, font)
                pygame.display.flip()
                
        finally:
            print("Shutting down... Cleaning up actors to prevent CARLA errors.")
            
            # 1. Stop AI Controllers first
            for controller in self.walker_controllers:
                if controller and controller.is_alive:
                    controller.stop()
                    
            # 2. Safely Batch-Destroy Actors (Prevents "not found" errors)
            destroy_cmds = []
            all_actors = self.walker_controllers + self.walkers + self.vehicles_list + [self.camera, self.radar, self.vehicle]
            
            for actor in all_actors:
                if actor and actor.is_alive:
                    destroy_cmds.append(carla.command.DestroyActor(actor.id))
                    
            if destroy_cmds:
                self.client.apply_batch(destroy_cmds)

            # 3. Reset World Settings
            st = self.world.get_settings()
            st.synchronous_mode = False
            st.fixed_delta_seconds = None
            self.world.apply_settings(st)
            
            if self.traffic_manager:
                self.traffic_manager.set_synchronous_mode(False)
            
            pygame.quit()
            print("Cleanup complete. Safe exit.")

if __name__ == "__main__":
    VirtualVahanaADAS().run()

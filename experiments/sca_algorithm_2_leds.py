import asyncio
import socket
import os

from behaviours.obstacle_avoidance import ObstacleAvoidance
from behaviours.colour_recognition import OptionGroundSensor
from behaviours.sca_algorithm_2 import SCA
from swarm_platform.controller.client import SwarmClient
from utils.communication import SwarmUDPManager

class SCAExperiment:
    """
    Implements the SCA algorithm using the Optitrack tracking system to
    get each robot's position and its distance/bearing to nearby neighbours.
    Includes the velocity bias that steers the robot towards authoritative
    neighbours (applied only when obstacle avoidance is not currently active).

    Requires `tracking: true` for this experiment in swarm_project.yaml.
    """

    # Top LED colour shown for each opinion. Black is rendered as dark
    # blue so a "black" opinion still shows up on the LED.
    OPINION_COLORS = {
        -1: (255, 0, 0),
        0: (0, 0, 139),
        1: (255, 255, 255),
        2: (50, 15, 0),
    }

    def __init__(self, robot, config=None, logger=None):
        self.robot = robot
        self.logger = logger
        self.config = config or {}

        self.running = True
        self.paused = False

        self.robot_id = socket.gethostname()
        
        coordinator_ip = os.getenv("SWARM_COORDINATOR", "10.15.2.63")
        coordinator_port = int(os.getenv("SWARM_COORDINATOR_PORT", "9100"))
        self.client = SwarmClient(coordinator_ip, coordinator_port)
        self.udp = SwarmUDPManager(port=5000)
        self.target_ips = []
        
        self.wheel_velocity = self.config.get("wheel_velocity", 200)

        self.obstacle_avoidance = ObstacleAvoidance(wheel_velocity=self.wheel_velocity)
        self.color_recognition = OptionGroundSensor()
        self.sca_algorithm = SCA()

        self.radius = 0.1

        self.tick = 0
        self.previous_opinion = -1
        self.debug_pixel_until = -1
        self.debug_pixel_colour = (0, 255, 0)

    async def refresh_peers(self):
        robots = await self.client.list_robots()
        self.target_ips = [
            info["ip"] for rid, info in robots.items() if rid != self.robot_id
        ]
        print(f"[PEERS] {self.target_ips}")

    async def run(self):
        await self.refresh_peers()

        while self.running:

            if self.paused:
                await self.robot.stop()
                await asyncio.sleep(0.05)
                continue

            prox = await self.robot.proximity_horizontal()
            left, right = self.obstacle_avoidance.step_motion(prox)
            
            ground = await self.robot.proximity_ground_reflected()
            patch, _ = self.color_recognition.detect_option(ground)

            nearby_hostnames = await self.robot.get_neighbours(self.radius)
            relative_poses = await self.robot.get_relative_poses(nearby_hostnames)
            my_pose = await self.robot.get_global_pose()

            received = self.udp.receive_messages()

            neighbours = {}
            for id, msg in received.items():
                if msg.get("id") in relative_poses:
                    rel = relative_poses[msg.get("id")]
                    msg["distance"] = rel.distance
                    msg["bearing"] = rel.bearing
                    neighbours[id] = msg
             
            left_bias, right_bias, opinion, quality, rarity, authority, buffer = self.sca_algorithm.sca_tick(patch, neighbours)

            # Debug: for 2s after an opinion change, show one pixel: green if it matches the
            # patch under the robot, magenta if it came from communication.
            if opinion != self.previous_opinion and opinion != -1:
                self.debug_pixel_colour = (0, 255, 0) if patch == opinion else (255, 0, 255)
                self.debug_pixel_until = self.tick + 40
            self.previous_opinion = opinion

            if self.robot.has_led_ring:
                colour = self.OPINION_COLORS.get(opinion, (0, 0, 0))
                if self.tick < self.debug_pixel_until:
                    pixels = [colour] * self.robot.led_ring.num_pixels
                    pixels[0] = self.debug_pixel_colour
                    await self.robot.led_ring_set_pixels(pixels)
                else:
                    await self.robot.led_ring_fill(*colour)

            avoidance_active = (self.obstacle_avoidance.turn_direction is not None or self.obstacle_avoidance.backward)
            if not avoidance_active:
                left += left_bias
                right += right_bias

            self.udp.send_to_all(
                {"id": self.robot_id, 
                 "tick": self.tick,  
                 "opinion": opinion, 
                 "quality": quality, 
                 "authority": authority}, 
                self.target_ips
            )
            
            await self.robot.drive(left, right)

            if self.logger:
                self.logger.log(
                    state={"proximity": prox,
                           "reflected": ground},
                    command={
                        "tick": self.tick,
                        "left_motor": left,
                        "right_motor": right,
                        "patch": patch,
                        "opinion": opinion,
                        "quality": quality,
                        "rarity": rarity,
                        "authority": authority,
                        "buffer": buffer,
                        "neighbours": nearby_hostnames,
                        "position": my_pose.position if my_pose else None,
                    },
                )

            await asyncio.sleep(0.05)
            self.tick += 1
        await self.robot.stop()


    async def pause(self):
        self.paused = True

    async def resume(self):
        self.paused = False

    async def stop(self):
        self.running = False
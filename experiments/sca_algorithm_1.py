import asyncio
import socket
import os
import re

from behaviours.obstacle_avoidance import ObstacleAvoidance
from behaviours.colour_recognition import OptionGroundSensor
from behaviours.sca_algorithm_1 import SCA 
from swarm_platform.controller.client import SwarmClient
from utils.communication import SwarmUDPManager 

class SCAExperiment:
    """
    Implements the SCA algorithm using the Thymio's IR communication 
    to estimate which neighbours are close. 
    The velocity bias is not implemented in this version so steering is 
    only done by ObstacleAvoidance.
    """

    def __init__(self, robot, config=None, logger=None):
        self.robot = robot
        self.logger = logger
        self.config = config or {}

        self.running = True
        self.paused = False

        self.robot_id = socket.gethostname()
        self.short_id = int(re.sub(r"\D", "", self.robot_id))
        
        coordinator_ip = os.getenv("SWARM_COORDINATOR", "10.15.2.63")
        coordinator_port = int(os.getenv("SWARM_COORDINATOR_PORT", "9100"))
        self.client = SwarmClient(coordinator_ip, coordinator_port)
        self.udp = SwarmUDPManager(port=5000)
        self.target_ips = []
        
        self.wheel_velocity = self.config.get("wheel_velocity", 200)

        self.obstacle_avoidance = ObstacleAvoidance(wheel_velocity=self.wheel_velocity)
        self.color_recognition = OptionGroundSensor()
        self.sca_algorithm = SCA()

        self.nearby_ids = {}       
        self.NEARBY_WINDOW = 20

        self.tick = 0

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

            await self.robot.send(self.short_id)

            try:
                rx, _, _, _ = await self.robot.receive()
            except Exception as e:
                print(f"Hidden error in receive() : {repr(e)}")
                rx = 0

            if rx > 0:  
                self.nearby_ids[rx] = self.tick

            delete = []
            for id, last_seen in self.nearby_ids.items():
                if self.tick - last_seen > self.NEARBY_WINDOW:
                    delete.append(id)

            for id in delete:
                del self.nearby_ids[id]

            received = self.udp.receive_messages()

            nearby_received = {}
            for i, msg in received.items():
                if msg.get("id") in self.nearby_ids:
                    nearby_received[i] = msg
             
            left_bias, right_bias, opinion, quality, rarity, authority, buffer = self.sca_algorithm.sca_tick(patch, nearby_received)

            self.udp.send_to_all(
                {"id": self.short_id, 
                 "tick": self.tick,  
                 "opinion": opinion, 
                 "quality": quality, 
                 "authority": authority}, 
                self.target_ips
            )
            
            await self.robot.drive(left + left_bias, right + right_bias)

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
                        "rx": rx
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
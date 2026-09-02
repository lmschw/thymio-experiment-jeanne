import asyncio
import socket
import os

from behaviours.obstacle_avoidance import ObstacleAvoidance
from behaviours.colour_recognition import OptionGroundSensor
from behaviours.quorum_sensing import QuorumSensing 
from swarm_platform.controller.client import SwarmClient
from utils.communication import SwarmUDPManager 

class QuorumSensingExperiment:
    """ 
    Implement the quorum sensing algorithm.
    The robots move around the arena using the obstacle avoidance algorithm.
    They use the color recognition to know the patch they are on.
    The Optitrack system allows to get the position of the robots and their neighbours.
    The robots communicate using Wi-Fi to send their opinion to their neighbours.

    Requires `tracking: true` for this experiment in swarm_project.yaml.
    """
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
            self.quorum_sensing = QuorumSensing()

            self.radius = 0.3

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
        
            nearby_hostnames = await self.robot.get_neighbours(self.radius)
            my_pose = await self.robot.get_global_pose()

            received = self.udp.receive_messages()
            
            neighbours = {}
            for id, msg in received.items():
                if msg.get("id") in nearby_hostnames:
                    neighbours[id] = msg

            opinion = self.quorum_sensing.tick(patch, neighbours)

            self.udp.send_to_all(
                {"id": self.robot_id, 
                "tick": self.tick,  
                "opinion": opinion}, 
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
                        "neighbours": nearby_hostnames,
                        "position": my_pose.position if my_pose else None
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
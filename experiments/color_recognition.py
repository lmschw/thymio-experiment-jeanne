import asyncio

from behaviours.obstacle_avoidance import ObstacleAvoidance
from behaviours.colour_recognition import OptionGroundSensor

class ColorRecognitionExperiment:
    """
    Obstacle avoidance driving combined with floor patch color detection.
    """

    def __init__(self, robot, config=None, logger=None):
        self.robot = robot
        self.logger = logger
        self.config = config or {}

        self.running = True
        self.paused = False

        # Parameters
        self.wheel_velocity = self.config.get("wheel_velocity", 200)

        self.obstacle_avoidance = ObstacleAvoidance(wheel_velocity=self.wheel_velocity)
        #self.color_recognition = ColorRecognition()
        self.color_recognition = OptionGroundSensor()

    async def run(self):

        while self.running:

            if self.paused:
                await self.robot.stop()
                await asyncio.sleep(0.05)
                continue

            prox = await self.robot.proximity_horizontal()

            left, right = self.obstacle_avoidance.step_motion(prox)

            ground = await self.robot.proximity_ground_reflected()

            #_, color = self.color_recognition.filtered_color(ground)
            color, _ = self.color_recognition.detect_option(ground)

            await self.robot.drive(left, right)

            if self.logger:
                self.logger.log(
                    state={"proximity": prox,
                           "reflected_0": ground[0],
                           "reflected_1": ground[1],
                           "reflected_avg": (ground[0] + ground[1])/2},
                    command={
                        "left_motor": left,
                        "right_motor": right,
                        "color": color
                    },
                )

            await asyncio.sleep(0.05)

        await self.robot.stop()


    async def pause(self):
        self.paused = True

    async def resume(self):
        self.paused = False

    async def stop(self):
        self.running = False
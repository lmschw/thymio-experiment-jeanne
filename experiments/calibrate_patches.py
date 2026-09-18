import asyncio
from pathlib import Path

import yaml

PATCH_NAMES = ["black", "white", "brown"]

OUTPUT_FILE = (
    Path(__file__).resolve().parent.parent
    / "behaviours"
    / "config"
    / "patch_positions.yaml"
)

# LED shown on the ring while waiting for the button press for each step.
STEP_COLORS = {
    "center": (255, 255, 0),
    0: (0, 0, 139),
    1: (255, 255, 255),
    2: (50, 15, 0),
}
DONE_COLOR = (0, 255, 0)


class CalibratePatchesExperiment:
    """
    One-off calibration experiment: records the arena's (x, z) centre and
    the position of each of the 3 floor patches, using the robot's
    Optitrack-tracked global pose. The result is consumed by
    `behaviours.patch_location.PatchLocationChecker` to cross-check (and
    veto) ground-sensor readings in the `*_leds` experiments.

    Patches are squares, and not all the same size, so each one is
    calibrated by its axis-aligned bounding box rather than a centre +
    radius.

    Usage: start this experiment, then place the robot at the arena
    centre and press its centre button to record that point. The LED
    ring then shows the colour of the next patch to record (dark blue
    for black, white, then the dim brown) -- for each patch, place the
    robot at one CORNER of that square patch and press the button, then
    at the diagonally OPPOSITE corner and press the button again. Once
    all 3 patches are recorded, the resulting YAML is printed to stdout
    (visible over `journalctl -u swarm-daemon.service -f` if run on a
    robot) -- copy it into behaviours/config/patch_positions.yaml,
    commit, and redeploy so every robot picks it up. The experiment
    also makes a best-effort attempt to write the file directly.

    Requires `tracking: true` for this experiment in swarm_project.yaml.
    """

    def __init__(self, robot, config=None, logger=None):
        self.robot = robot
        self.logger = logger
        self.config = config or {}

        self.running = True
        self.paused = False
        self._was_pressed = False

    async def _wait_for_press(self):
        """Blocks (cooperatively) until the centre button is pressed, edge-triggered."""
        while self.running:
            if self.paused:
                await asyncio.sleep(0.05)
                continue

            pressed = (await self.robot.buttons())["center"]
            if pressed and not self._was_pressed:
                self._was_pressed = True
                return True
            if not pressed:
                self._was_pressed = False

            await asyncio.sleep(0.05)

        return False

    async def _record(self, label, colour):
        if self.robot.has_led_ring:
            await self.robot.led_ring_fill(*colour)

        print(f"[CALIBRATE] Place the robot on '{label}' and press the centre button...")
        if not await self._wait_for_press():
            return None

        pose = await self.robot.get_global_pose()
        if pose is None:
            print(
                f"[CALIBRATE] No tracked pose available for '{label}' "
                f"-- is tracking running? Try again."
            )
            return await self._record(label, colour)

        x, _, z = pose.position
        print(f"[CALIBRATE] Recorded '{label}' at x={x:.3f}, z={z:.3f}")
        return x, z

    async def run(self):
        center = await self._record("arena centre", STEP_COLORS["center"])
        if center is None:
            return

        patches = {}
        for index, name in enumerate(PATCH_NAMES):
            colour = STEP_COLORS[index]

            corner_a = await self._record(f"{name} patch -- one CORNER", colour)
            if corner_a is None:
                return

            corner_b = await self._record(f"{name} patch -- the OPPOSITE corner", colour)
            if corner_b is None:
                return

            x_min, x_max = sorted((corner_a[0], corner_b[0]))
            z_min, z_max = sorted((corner_a[1], corner_b[1]))

            patches[index] = {
                "x_min": x_min,
                "x_max": x_max,
                "z_min": z_min,
                "z_max": z_max,
                "name": name,
            }
            print(
                f"[CALIBRATE]   '{name}' bounds: "
                f"x=[{x_min:.3f}, {x_max:.3f}], z=[{z_min:.3f}, {z_max:.3f}]"
            )

        result = {
            "center": {"x": center[0], "z": center[1]},
            "patches": patches,
        }

        yaml_text = yaml.safe_dump(result, sort_keys=False)
        print(
            "[CALIBRATE] Done. Paste this into "
            "behaviours/config/patch_positions.yaml:\n"
        )
        print(yaml_text)

        try:
            OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
            OUTPUT_FILE.write_text(yaml_text)
            print(f"[CALIBRATE] Also wrote it to {OUTPUT_FILE}")
        except Exception as e:
            print(f"[CALIBRATE] Could not write {OUTPUT_FILE}: {e}")

        if self.robot.has_led_ring:
            await self.robot.led_ring_fill(*DONE_COLOR)

        self.running = False

    async def pause(self):
        self.paused = True

    async def resume(self):
        self.paused = False

    async def stop(self):
        self.running = False

import math
from pathlib import Path

import yaml

DEFAULT_RADIUS = 0.15  # metres; how close counts as "on" a patch

CONFIG_FILE = (
    Path(__file__).resolve().parent
    / "config"
    / "patch_positions.yaml"
)


class PatchLocationChecker:
    """
    Cross-checks a ground-sensor colour reading against the robot's
    tracked (Optitrack) position, using patch positions recorded by the
    `calibrate_patches` experiment.

    Used only to police/veto ground-sensor readings, never to invent an
    opinion the sensor itself didn't report: if the robot isn't actually
    near the patch the sensor claims, the reading is rejected to unknown
    (-1) for that tick.

    If no calibration file is found, policing is a no-op (every reading
    passes through unchanged) so behaviour is unaffected until the arena
    has been calibrated.
    """

    def __init__(self, config_file: Path = CONFIG_FILE):
        self.radius = DEFAULT_RADIUS
        self.patches = {}

        if not config_file.exists():
            print(
                f"[PATCH_LOCATION] No calibration file at {config_file}; "
                f"position-based policing is disabled. Run the "
                f"'calibrate_patches' experiment to generate one."
            )
            return

        with config_file.open() as f:
            config = yaml.safe_load(f) or {}

        self.radius = config.get("radius", DEFAULT_RADIUS)
        self.patches = {
            int(index): (patch["x"], patch["z"])
            for index, patch in config.get("patches", {}).items()
        }

    def expected_patch(self, position):
        """
        Args:
            position: (x, y, z) tracked position, or None.

        Returns:
            The index of the recorded patch whose centre is nearest to
            `position` and within `self.radius`, or -1 if `position` is
            None or isn't within range of any known patch.
        """
        if position is None:
            return -1

        x, _, z = position

        best_index = -1
        best_distance = self.radius
        for index, (px, pz) in self.patches.items():
            distance = math.hypot(x - px, z - pz)
            if distance <= best_distance:
                best_distance = distance
                best_index = index

        return best_index

    def check(self, patch, position):
        """
        Args:
            patch: The ground sensor's detected patch index (-1 if none).
            position: The robot's tracked (x, y, z) position, or None.

        Returns:
            `patch` unchanged if there's no calibration data to check
            against, if the sensor reported no patch, or if the robot's
            tracked position is within range of that same patch.
            Otherwise -1 (the reading is vetoed).
        """
        if patch == -1 or not self.patches:
            return patch

        if self.expected_patch(position) != patch:
            return -1

        return patch

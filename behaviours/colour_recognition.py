"""
Ground patch classifier for decision-making options, calibrated to real
Thymio hardware readings.

Same "nearest calibrated centre within an allowed offset" classification
as GroundColourSensor, generalised to N configurable option centres.

Unlike the earlier version, white is NOT treated as a "background /
no-option" sentinel here - it's a real decision option like black or
grey (matching a 3-option best-of-3 over black / grey / white patches).
UNKNOWN only means "this reading doesn't match any calibrated centre
closely enough", not "this is the empty floor".
"""

UNKNOWN = -1    # reading doesn't match any calibrated centre closely enough


class OptionGroundSensor:

    ALLOWED_OFFSET = 30
    ALLOWED_SENSOR_OFFSET = 50

    # Default centres, in index order: option 0, option 1, option 2
    # (black, grey, white - calibrated hardware values from
    # GroundColourSensor: BLACK_CENTER=51, GREY_CENTER=154, WHITE_CENTER=885).
    DEFAULT_OPTION_CENTERS = [39, 78, 620]
    DEFAULT_ALLOWED_OFFSETS = [15, 10, 55]

    def __init__(
        self,
        num_options=3,
        option_centers=None,
        allowed_offsets=None,
    ):
        self.num_options = num_options

        self.option_centers = (
            list(option_centers)
            if option_centers is not None
            else self.DEFAULT_OPTION_CENTERS[:num_options]
        )

        if len(self.option_centers) != num_options:
            raise ValueError(
                "option_centers length must match num_options "
                f"({len(self.option_centers)} != {num_options})"
            )

        self.allowed_offsets = (
            list(allowed_offsets)
            if allowed_offsets is not None
            else self.DEFAULT_ALLOWED_OFFSETS[:num_options]
        )

        if len(self.allowed_offsets) != num_options:
            raise ValueError(
                "allowed_offsets length must match num_options "
                f"({len(self.allowed_offsets)} != {num_options})"
            )

    def _classify(self, value: int) -> int:
        best_key = UNKNOWN
        best_distance = float("inf")

        for idx, centre in enumerate(self.option_centers):
            distance = abs(value - centre)

            if distance < best_distance:
                best_distance = distance
                best_key = idx

        if best_distance <= self.allowed_offsets[best_key]:
            return best_key

        return UNKNOWN

    def detect_option(self, reflected):
        """
        reflected: [left_reading, right_reading] raw ADC values from
        robot.proximity_ground_reflected().

        Returns (option_index, avg_reading):
          option_index is -1 only if the two sensors disagree on
          different options, or the reading matches no centre at all.
        """
        avg = 0.5 * ((reflected[0] if len(reflected) > 0 else 0)
                     + (reflected[1] if len(reflected) > 1 else 0))
        
        if abs(reflected[0] - reflected[1]) >= self.ALLOWED_SENSOR_OFFSET:
            return UNKNOWN, avg

        colour = self._classify(avg)

        return colour, avg

import time
import math
from oak9 import Robot


ROBOT_IP = "192.168.3.3"

# ---------------------------------------------------------
# Robot geometry
# ---------------------------------------------------------

R = 0.034       # wheel radius [m]
L = 0.1659      # wheel separation [m]
d = 0.05        # o1 -> o5 [m]


# ---------------------------------------------------------
# Goal for control point o5
# ---------------------------------------------------------

X_GOAL = 0.30
Y_GOAL = 0.00


# ---------------------------------------------------------
# Cartesian feedback gains
# ---------------------------------------------------------

KX = 0.8
KY = 0.8


# ---------------------------------------------------------
# Minimum-jerk trajectory limits
# ---------------------------------------------------------

MAX_REFERENCE_SPEED = 0.08   # m/s
MAX_REFERENCE_ACCEL = 0.12   # m/s^2
MAX_REFERENCE_JERK = 0.45    # m/s^3


# ---------------------------------------------------------
# Robot command limits
# ---------------------------------------------------------

MAX_CARTESIAN_SPEED = 0.10
MAX_OMEGA = 0.5

POSITION_TOLERANCE = 0.015

CONTROL_PERIOD = 0.10


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


class MinimumJerkProfile:
    """
    Quintic minimum-jerk trajectory:

        sigma(t) = 10 tau^3 - 15 tau^4 + 6 tau^5

    Provides scalar displacement s and scalar speed ds/dt.
    """

    def __init__(
        self,
        distance,
        max_speed,
        max_accel,
        max_jerk,
        minimum_time
    ):

        self.distance = max(
            0.0,
            float(distance)
        )

        if self.distance < 1e-12:
            self.duration = minimum_time
            return

        # Maximum normalized values for quintic minimum jerk:
        #
        # max sigma_dot   = 1.875
        # max sigma_ddot  = 5.7735
        # max sigma_dddot = 60

        time_speed = (
            1.875
            * self.distance
            / max_speed
        )

        time_accel = math.sqrt(
            5.7735026919
            * self.distance
            / max_accel
        )

        time_jerk = (
            60.0
            * self.distance
            / max_jerk
        ) ** (1.0 / 3.0)

        self.duration = max(
            minimum_time,
            time_speed,
            time_accel,
            time_jerk
        )

    def sample(self, time_s):

        if time_s <= 0.0:
            return 0.0, 0.0

        if time_s >= self.duration:
            return self.distance, 0.0

        tau = time_s / self.duration

        tau2 = tau * tau
        tau3 = tau2 * tau
        tau4 = tau3 * tau
        tau5 = tau4 * tau

        sigma = (
            10.0 * tau3
            - 15.0 * tau4
            + 6.0 * tau5
        )

        sigma_dot = (
            30.0 * tau2
            - 60.0 * tau3
            + 30.0 * tau4
        ) / self.duration

        displacement = (
            self.distance
            * sigma
        )

        speed = (
            self.distance
            * sigma_dot
        )

        return displacement, speed


robot = Robot(ROBOT_IP)

try:

    # ---------------------------------------------------------
    # Known-good robot calibration
    # ---------------------------------------------------------

    robot.motor.invert_set(
        left=False,
        right=False
    )

    robot.encoder.invert_set(
        left=True,
        right=False
    )

    robot.wheel_velocity.stop()

    time.sleep(0.5)


    # ---------------------------------------------------------
    # Reset current physical location to o1 = (0,0,0)
    # ---------------------------------------------------------

    robot.pose.set(
        x_mm=0,
        y_mm=0,
        theta_deg=0
    )

    time.sleep(0.5)


    # ---------------------------------------------------------
    # Read initial pose
    # ---------------------------------------------------------

    pose = robot.pose.get()

    x1 = pose["x_mm"] / 1000.0
    y1 = pose["y_mm"] / 1000.0
    theta = math.radians(
        pose["theta_deg"]
    )

    initial_x5 = (
        x1
        + d * math.cos(theta)
    )

    initial_y5 = (
        y1
        + d * math.sin(theta)
    )


    # ---------------------------------------------------------
    # Define straight-line trajectory of o5
    # ---------------------------------------------------------

    path_dx = (
        X_GOAL
        - initial_x5
    )

    path_dy = (
        Y_GOAL
        - initial_y5
    )

    path_length = math.hypot(
        path_dx,
        path_dy
    )

    direction_x = (
        path_dx / path_length
    )

    direction_y = (
        path_dy / path_length
    )


    # ---------------------------------------------------------
    # Build minimum-jerk profile
    # ---------------------------------------------------------

    profile = MinimumJerkProfile(
        distance=path_length,
        max_speed=MAX_REFERENCE_SPEED,
        max_accel=MAX_REFERENCE_ACCEL,
        max_jerk=MAX_REFERENCE_JERK,
        minimum_time=CONTROL_PERIOD
    )


    print("Initial pose:")
    print(pose)

    print(
        f"\nInitial o5 = "
        f"({initial_x5:.3f}, "
        f"{initial_y5:.3f}) m"
    )

    print(
        f"Goal o5 = "
        f"({X_GOAL:.3f}, "
        f"{Y_GOAL:.3f}) m"
    )

    print(
        f"Path length = "
        f"{path_length:.3f} m"
    )

    print(
        f"Minimum-jerk duration = "
        f"{profile.duration:.2f} s\n"
    )


    start_time = time.monotonic()


    while True:

        now = time.monotonic()

        elapsed = (
            now - start_time
        )


        # =====================================================
        # 1. Real pose of robot center o1
        # =====================================================

        pose = robot.pose.get()

        x1 = (
            pose["x_mm"]
            / 1000.0
        )

        y1 = (
            pose["y_mm"]
            / 1000.0
        )

        theta = math.radians(
            pose["theta_deg"]
        )


        # =====================================================
        # 2. Real position of o5
        # =====================================================

        x5 = (
            x1
            + d * math.cos(theta)
        )

        y5 = (
            y1
            + d * math.sin(theta)
        )


        # =====================================================
        # 3. Minimum-jerk moving reference
        # =====================================================

        reference_s, reference_speed = (
            profile.sample(elapsed)
        )

        x_reference = (
            initial_x5
            + direction_x
            * reference_s
        )

        y_reference = (
            initial_y5
            + direction_y
            * reference_s
        )


        # Feedforward velocity of moving reference
        vx_feedforward = (
            direction_x
            * reference_speed
        )

        vy_feedforward = (
            direction_y
            * reference_speed
        )


        # =====================================================
        # 4. Cartesian tracking error
        # =====================================================

        error_x = (
            x_reference
            - x5
        )

        error_y = (
            y_reference
            - y5
        )


        # =====================================================
        # 5. Cartesian controller + feedforward
        # =====================================================

        ux = (
            vx_feedforward
            + KX * error_x
        )

        uy = (
            vy_feedforward
            + KY * error_y
        )


        # =====================================================
        # Limit Cartesian command
        # =====================================================

        cartesian_speed = math.hypot(
            ux,
            uy
        )

        if (
            cartesian_speed
            > MAX_CARTESIAN_SPEED
        ):

            scale = (
                MAX_CARTESIAN_SPEED
                / cartesian_speed
            )

            ux *= scale
            uy *= scale


        # =====================================================
        # 6. o5 inverse kinematics
        # =====================================================

        v_command = (
            math.cos(theta) * ux
            +
            math.sin(theta) * uy
        )

        omega_command = (
            -math.sin(theta) * ux
            +
            math.cos(theta) * uy
        ) / d


        omega_command = clamp(
            omega_command,
            -MAX_OMEGA,
            MAX_OMEGA
        )


        # =====================================================
        # 7. Differential-drive inverse kinematics
        # =====================================================

        left_rad_s = (
            v_command
            - 0.5 * L * omega_command
        ) / R

        right_rad_s = (
            v_command
            + 0.5 * L * omega_command
        ) / R


        left_deg_s = math.degrees(
            left_rad_s
        )

        right_deg_s = math.degrees(
            right_rad_s
        )


        # =====================================================
        # 8. Send commands
        # =====================================================

        robot.wheel_velocity.set(
            left=left_deg_s,
            right=right_deg_s
        )


        # =====================================================
        # Final target error
        # =====================================================

        final_distance_error = math.hypot(
            X_GOAL - x5,
            Y_GOAL - y5
        )


        # =====================================================
        # Debug
        # =====================================================

        print(
            f"t={elapsed:5.2f} | "
            f"o5=({x5:.3f},{y5:.3f}) | "
            f"ref=({x_reference:.3f},"
            f"{y_reference:.3f}) | "
            f"etrack="
            f"({error_x:.3f},"
            f"{error_y:.3f}) | "
            f"efinal="
            f"{final_distance_error:.3f} | "
            f"v={v_command:.3f} "
            f"omega={omega_command:.3f} | "
            f"L={left_deg_s:6.1f} "
            f"R={right_deg_s:6.1f}"
        )


        # =====================================================
        # Stop condition
        # =====================================================

        profile_finished = (
            elapsed
            >= profile.duration
        )

        if (
            profile_finished
            and final_distance_error
            < POSITION_TOLERANCE
        ):

            robot.wheel_velocity.stop()

            print(
                "\nMINIMUM-JERK "
                "O5 GOAL REACHED"
            )

            print(
                f"o1 = "
                f"({x1:.3f}, "
                f"{y1:.3f})"
            )

            print(
                f"o5 = "
                f"({x5:.3f}, "
                f"{y5:.3f})"
            )

            print(
                f"theta = "
                f"{math.degrees(theta):.2f} deg"
            )

            break


        time.sleep(
            CONTROL_PERIOD
        )


finally:

    robot.wheel_velocity.stop()
    robot.close()
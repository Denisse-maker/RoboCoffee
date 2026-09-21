import time
import math
from oak9 import Robot


ROBOT_IP = "192.168.3.3"

# ---------------------------------------------------------
# Robot geometry
# ---------------------------------------------------------

R = 0.034       # wheel radius [m]
L = 0.1659      # wheel separation [m]

# Distance from robot center o1 to control point o5
d = 0.05        # [m]


# ---------------------------------------------------------
# Goal for CONTROL POINT o5
# ---------------------------------------------------------

X_GOAL = 0.30   # m
Y_GOAL = 0.00   # m


# ---------------------------------------------------------
# Cartesian controller gains
# Start conservative
# ---------------------------------------------------------

KX = 0.8
KY = 0.8


# ---------------------------------------------------------
# Limits
# ---------------------------------------------------------

MAX_CARTESIAN_SPEED = 0.05   # m/s
MAX_OMEGA = 0.5              # rad/s

POSITION_TOLERANCE = 0.015   # 1.5 cm

CONTROL_PERIOD = 0.10        # seconds


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


robot = Robot(ROBOT_IP)

try:

    # ---------------------------------------------------------
    # Known-good physical robot calibration
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
    # Establish current physical location as o1 = (0,0,0)
    # ---------------------------------------------------------

    robot.pose.set(
        x_mm=0,
        y_mm=0,
        theta_deg=0
    )

    time.sleep(0.5)

    print("Initial pose:")
    print(robot.pose.get())

    print(
        f"\nGoal for o5: "
        f"x={X_GOAL:.3f} m, "
        f"y={Y_GOAL:.3f} m\n"
    )

    while True:

        # =====================================================
        # 1. Read pose of robot center o1
        # =====================================================

        pose = robot.pose.get()

        x1 = pose["x_mm"] / 1000.0
        y1 = pose["y_mm"] / 1000.0
        theta = math.radians(
            pose["theta_deg"]
        )


        # =====================================================
        # 2. Calculate position of control point o5
        #
        #             o5
        #             *
        #             |
        #          d  |
        #             |
        #             o1 ---> robot heading
        #
        # =====================================================

        x5 = x1 + d * math.cos(theta)
        y5 = y1 + d * math.sin(theta)


        # =====================================================
        # 3. Cartesian error of o5
        # =====================================================

        error_x = X_GOAL - x5
        error_y = Y_GOAL - y5

        distance_error = math.hypot(
            error_x,
            error_y
        )


        # =====================================================
        # Stop when o5 reaches the target
        # =====================================================

        if distance_error < POSITION_TOLERANCE:

            robot.wheel_velocity.stop()

            print("\nO5 GOAL REACHED")

            print(
                f"o1 = ({x1:.3f}, {y1:.3f}) m"
            )

            print(
                f"o5 = ({x5:.3f}, {y5:.3f}) m"
            )

            print(
                f"theta = "
                f"{math.degrees(theta):.2f} deg"
            )

            break


        # =====================================================
        # 4. Cartesian controller
        #
        # ux, uy = desired velocity of point o5
        # =====================================================

        ux = KX * error_x
        uy = KY * error_y


        # =====================================================
        # Limit Cartesian velocity
        # =====================================================

        cartesian_speed = math.hypot(
            ux,
            uy
        )

        if cartesian_speed > MAX_CARTESIAN_SPEED:

            scale = (
                MAX_CARTESIAN_SPEED
                / cartesian_speed
            )

            ux *= scale
            uy *= scale


        # =====================================================
        # 5. Inverse kinematics of o5
        #
        # [ux]   [ cos(theta)  -d sin(theta)] [v    ]
        # [uy] = [ sin(theta)   d cos(theta)] [omega]
        #
        # Inverse:
        #
        # v =
        # cos(theta)*ux + sin(theta)*uy
        #
        # omega =
        # (-sin(theta)*ux + cos(theta)*uy) / d
        #
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


        # =====================================================
        # Limit angular speed
        # =====================================================

        omega_command = clamp(
            omega_command,
            -MAX_OMEGA,
            MAX_OMEGA
        )


        # =====================================================
        # 6. Differential-drive inverse kinematics
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
        # 7. Send command to real robot
        # =====================================================

        robot.wheel_velocity.set(
            left=left_deg_s,
            right=right_deg_s
        )


        # =====================================================
        # Debug
        # =====================================================

        print(
            f"o1=({x1: .3f},{y1: .3f}) "
            f"theta={math.degrees(theta):6.2f} | "
            f"o5=({x5: .3f},{y5: .3f}) | "
            f"err=({error_x: .3f},{error_y: .3f}) "
            f"dist={distance_error:.3f} | "
            f"ux={ux: .3f} "
            f"uy={uy: .3f} | "
            f"v={v_command: .3f} "
            f"omega={omega_command: .3f} | "
            f"L={left_deg_s:6.1f} "
            f"R={right_deg_s:6.1f}"
        )

        time.sleep(CONTROL_PERIOD)


finally:

    robot.wheel_velocity.stop()
    robot.close()
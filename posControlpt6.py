import time
import math
from oak9 import Robot

ROBOT_IP = "192.168.3.3"

# Robot geometry
R = 0.034      # wheel radius [m]
L = 0.1659     # wheel separation [m]

# Goal for robot center o1
X_GOAL = 0.400  # m
Y_GOAL = 0.00  # m

# Controller gains
K_V = 0.8
K_THETA = 1.0

# Limits
MAX_V = 0.05       # m/s
MAX_OMEGA = 0.5    # rad/s

# Stop tolerance
POSITION_TOLERANCE = 0.015  # 1.5 cm

CONTROL_PERIOD = 0.10  # s


def normalize_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


robot = Robot(ROBOT_IP)

try:
    # ---------------------------------------------------------
    # Known-good hardware calibration
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
    # Current physical location becomes (0,0,0)
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
        f"\nGoal: x={X_GOAL:.3f} m, "
        f"y={Y_GOAL:.3f} m\n"
    )

    while True:

        # -----------------------------------------------------
        # Read real robot pose
        # -----------------------------------------------------
        pose = robot.pose.get()

        x = pose["x_mm"] / 1000.0
        y = pose["y_mm"] / 1000.0
        theta = math.radians(pose["theta_deg"])

        # -----------------------------------------------------
        # Position error
        # -----------------------------------------------------
        error_x = X_GOAL - x
        error_y = Y_GOAL - y

        distance_error = math.hypot(
            error_x,
            error_y
        )

        # Stop once close enough
        if distance_error < POSITION_TOLERANCE:
            robot.wheel_velocity.stop()

            print("\nGOAL REACHED")
            print(robot.pose.get())
            break

        # -----------------------------------------------------
        # Desired heading toward goal
        # -----------------------------------------------------
        theta_goal = math.atan2(
            error_y,
            error_x
        )

        error_theta = normalize_angle(
            theta_goal - theta
        )

        # -----------------------------------------------------
        # Simple position controller
        # -----------------------------------------------------
        v_command = K_V * distance_error
        omega_command = K_THETA * error_theta

        v_command = clamp(
            v_command,
            0.0,
            MAX_V
        )

        omega_command = clamp(
            omega_command,
            -MAX_OMEGA,
            MAX_OMEGA
        )

        # -----------------------------------------------------
        # Optional safety behavior:
        # slow down if heading error is large
        # -----------------------------------------------------
        heading_scale = max(
            0.0,
            math.cos(error_theta)
        )

        v_command *= heading_scale

        # -----------------------------------------------------
        # Differential-drive inverse kinematics
        # -----------------------------------------------------
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

        # -----------------------------------------------------
        # Send commands
        # -----------------------------------------------------
        robot.wheel_velocity.set(
            left=left_deg_s,
            right=right_deg_s
        )

        # -----------------------------------------------------
        # Debug output
        # -----------------------------------------------------
        print(
            f"x={x: .3f} "
            f"y={y: .3f} "
            f"theta={math.degrees(theta): 6.2f} | "
            f"dist={distance_error:.3f} "
            f"theta_goal={math.degrees(theta_goal): 6.2f} "
            f"err_theta={math.degrees(error_theta): 6.2f} | "
            f"v={v_command:.3f} "
            f"omega={omega_command:.3f} | "
            f"L={left_deg_s:6.1f} "
            f"R={right_deg_s:6.1f}"
        )

        time.sleep(CONTROL_PERIOD)

finally:
    robot.wheel_velocity.stop()
    robot.close()
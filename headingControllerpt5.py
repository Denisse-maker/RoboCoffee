import time
import math
from oak9 import Robot


'''
a heading deviation causes a corrective 
turn in the correct direction, and theta tends back 
toward zero instead of running away.'''

ROBOT_IP = "192.168.3.3"

R = 0.034
L = 0.1659

V_COMMAND = 0.05       # m/s
THETA_REF = 0.0        # rad

K_THETA = 1.0
MAX_OMEGA = 0.5

TEST_TIME = 5.0

robot = Robot(ROBOT_IP)

try:
    # Known-good calibration
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

    # Current physical pose becomes our origin
    robot.pose.set(
        x_mm=0,
        y_mm=0,
        theta_deg=0
    )

    time.sleep(0.5)

    print("Initial pose:")
    print(robot.pose.get())

    start = time.monotonic()

    while time.monotonic() - start < TEST_TIME:

        pose = robot.pose.get()

        theta = math.radians(
            pose["theta_deg"]
        )

        # Heading error
        error_theta = THETA_REF - theta

        # Normalize to [-pi, pi]
        error_theta = (
            error_theta + math.pi
        ) % (2 * math.pi) - math.pi

        # Simple P heading controller
        omega_command = K_THETA * error_theta

        omega_command = max(
            -MAX_OMEGA,
            min(MAX_OMEGA, omega_command)
        )

        # Differential-drive inverse kinematics
        left_rad_s = (
            V_COMMAND
            - 0.5 * L * omega_command
        ) / R

        right_rad_s = (
            V_COMMAND
            + 0.5 * L * omega_command
        ) / R

        left_deg_s = math.degrees(left_rad_s)
        right_deg_s = math.degrees(right_rad_s)

        robot.wheel_velocity.set(
            left=left_deg_s,
            right=right_deg_s
        )

        print(
            f"x={pose['x_mm']:7.1f} "
            f"y={pose['y_mm']:7.1f} "
            f"theta={pose['theta_deg']:7.2f} "
            f"err_theta={math.degrees(error_theta):7.2f} "
            f"omega={omega_command:6.3f} "
            f"L={left_deg_s:6.1f} "
            f"R={right_deg_s:6.1f}"
        )

        time.sleep(0.1)

    robot.wheel_velocity.stop()

    print("\nFinal pose:")
    print(robot.pose.get())

finally:
    robot.wheel_velocity.stop()
    robot.close()
import time
import math
from oak9 import Robot

ROBOT_IP = "192.168.3.3"

R = 0.034      # m
L = 0.1659     # m

V_COMMAND = 0.05   # m/s
OMEGA_COMMAND = 0.25   # rad/s
TEST_TIME = 5.0       # seconds

robot = Robot(ROBOT_IP)

try:

    # ---------------------------------------------------------
    # Known-good calibration
    # ---------------------------------------------------------

    robot.motor.invert_set(
        left=False,
        right=False
    )

    robot.encoder.invert_set(
        left=True,
        right=False
    )

    # ---------------------------------------------------------
    # Establish current location as origin
    # ---------------------------------------------------------

    robot.wheel_velocity.stop()
    time.sleep(0.5)

    robot.pose.set(
        x_mm=0,
        y_mm=0,
        theta_deg=0
    )

    time.sleep(0.5)

    print("Initial pose:")
    print(robot.pose.get())

    # ---------------------------------------------------------
    # Differential-drive inverse kinematics
    # ---------------------------------------------------------

    left_rad_s = (
        V_COMMAND - 0.5 * L * OMEGA_COMMAND
    ) / R

    right_rad_s = (
        V_COMMAND + 0.5 * L * OMEGA_COMMAND
    ) / R

    left_deg_s = math.degrees(left_rad_s)
    right_deg_s = math.degrees(right_rad_s)

    print(
        f"\nCommand:"
        f"\nv={V_COMMAND:.3f} m/s"
        f"\nomega={OMEGA_COMMAND:.3f} rad/s"
        f"\nleft={left_deg_s:.1f} deg/s"
        f"\nright={right_deg_s:.1f} deg/s"
    )

    # ---------------------------------------------------------
    # Move
    # ---------------------------------------------------------

    robot.wheel_velocity.set(
        left=left_deg_s,
        right=right_deg_s
    )

    start = time.monotonic()

    while time.monotonic() - start < TEST_TIME:

        pose = robot.pose.get()

        print(
            f"x={pose['x_mm']:7.1f} "
            f"y={pose['y_mm']:7.1f} "
            f"theta={pose['theta_deg']:7.2f}"
        )

        time.sleep(0.1)

    robot.wheel_velocity.stop()

    print("\nFinal pose:")
    print(robot.pose.get())

finally:

    robot.wheel_velocity.stop()
    robot.close()
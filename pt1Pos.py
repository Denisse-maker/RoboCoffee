import time
from oak9 import Robot

ROBOT_IP = "192.168.3.3"

robot = Robot(ROBOT_IP)

try:
    # ------------------------------------------------------------
    # Known working hardware configuration
    # ------------------------------------------------------------

    robot.motor.invert_set(
        left=True,
        right=False
    )

    robot.encoder.invert_set(
        left=False,
        right=False
    )

    robot.pose.set(
        x_mm=0,
        y_mm=0,
        theta_deg=0
    )

    print("Motor:", robot.motor.invert_get())
    print("Encoder:", robot.encoder.invert_get())

    time.sleep(0.5)

    # ------------------------------------------------------------
    # Helper
    # ------------------------------------------------------------

    def show_pose(label):
        pose = robot.pose.get()

        print(
            f"{label}: "
            f"x={pose['x_mm']:.1f} mm, "
            f"y={pose['y_mm']:.1f} mm, "
            f"theta={pose['theta_deg']:.1f} deg"
        )

    # ------------------------------------------------------------
    # TEST 1
    # Both wheels +90 deg/s
    # Expected: straight forward
    # ------------------------------------------------------------

    print("\n--- TEST 1: straight ---")

    show_pose("Before")

    robot.wheel_velocity.set(
        left=90,
        right=90
    )

    time.sleep(2)

    robot.wheel_velocity.stop()

    show_pose("After")

    time.sleep(2)

    # ------------------------------------------------------------
    # Reset pose
    # ------------------------------------------------------------

    robot.pose.set(
        x_mm=0,
        y_mm=0,
        theta_deg=0
    )

    time.sleep(0.5)

    # ------------------------------------------------------------
    # TEST 2
    # Left slower, right faster
    # ------------------------------------------------------------

    print("\n--- TEST 2: left=45 right=90 ---")

    show_pose("Before")

    robot.wheel_velocity.set(
        left=45,
        right=90
    )

    time.sleep(2)

    robot.wheel_velocity.stop()

    show_pose("After")

    time.sleep(2)

    # ------------------------------------------------------------
    # Reset pose
    # ------------------------------------------------------------

    robot.pose.set(
        x_mm=0,
        y_mm=0,
        theta_deg=0
    )

    time.sleep(0.5)

    # ------------------------------------------------------------
    # TEST 3
    # Left faster, right slower
    # ------------------------------------------------------------

    print("\n--- TEST 3: left=90 right=45 ---")

    show_pose("Before")

    robot.wheel_velocity.set(
        left=90,
        right=45
    )

    time.sleep(2)

    robot.wheel_velocity.stop()

    show_pose("After")

finally:

    robot.wheel_velocity.stop()
    robot.close()
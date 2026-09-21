import time
from oak9 import Robot

robot = Robot("192.168.3.3")

try:

    # Robot-specific calibration
    robot.motor.invert_set(
        left=False,
        right=False
    )

    robot.encoder.invert_set(
        left=True,
        right=False
    )

    # Establish THIS physical location as our origin
    robot.pose.set(
        x_mm=0,
        y_mm=0,
        theta_deg=0
    )

    time.sleep(0.5)

    print("Initial pose:")
    print(robot.pose.get())

    # Known firmware-controlled movement
    robot.go2point.set(
        x=300,
        y=0
    )

    status = robot.go2point.status()

    while status["active"]:

        pose = robot.pose.get()

        print(
            f"x={pose['x_mm']:.1f}  "
            f"y={pose['y_mm']:.1f}  "
            f"theta={pose['theta_deg']:.1f}"
        )

        time.sleep(0.1)

        status = robot.go2point.status()

    print("\nFinal:")
    print(robot.pose.get())
    print(status)

finally:

    robot.go2point.stop()
    robot.close()
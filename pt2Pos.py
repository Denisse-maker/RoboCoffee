import time
from oak9 import Robot

robot = Robot("192.168.3.3")

try:
    # Configuration used by the known-good go2point example
    robot.motor.invert_set(left=True, right=False)
    robot.encoder.invert_set(left=True, right=False)

    print("Motor:", robot.motor.invert_get())
    print("Encoder:", robot.encoder.invert_get())

    # Same parameters from your working square
    robot.go2point.pid_set(
        v_max_giro_deg_s=90.0,
        a_max_giro_deg_s2=120.0,
        v_max_avance_mm_s=150.0,
        a_max_avance_mm_s2=100.0,
        ts_ms=25
    )

    robot.pose.set(
        x_mm=0,
        y_mm=0,
        theta_deg=0
    )

    time.sleep(0.5)

    print("\n--- GO2POINT STRAIGHT TEST ---")
    

    result = robot.go2point.set(
        x=300,
        y=0
    )

    print("go2point.set:", result)

    while True:
        go_status = robot.go2point.status()
        pose = robot.pose.get()
        vel = robot.wheel_velocity.status()

        print(
            f"x={pose['x_mm']:7.1f} "
            f"y={pose['y_mm']:7.1f} "
            f"theta={pose['theta_deg']:7.2f} | "
            f"Ltarget={vel.get('left_target')} "
            f"Rtarget={vel.get('right_target')} | "
            f"Lmeas={vel.get('left_measured')} "
            f"Rmeas={vel.get('right_measured')}"
        )

        if not go_status["active"]:
            break

        time.sleep(0.1)

    print("\nFINAL:")
    print(robot.pose.get())
    print(robot.go2point.status())

finally:
    robot.go2point.stop()
    robot.close()
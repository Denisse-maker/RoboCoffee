import math
import time
from oak9 import Robot

ROBOT_IP = "192.168.1.219"
R = 0.035
L = 0.1659
d = 0.05
NOSE_OFFSET = 0.10

KX = 0.8
KY = 0.8
MAX_REFERENCE_SPEED = 0.08
MAX_REFERENCE_ACCEL = 0.12
MAX_REFERENCE_JERK = 0.15
USE_ZVD = True
SLOSH_FREQUENCY_HZ = 1.00  # Current placeholder, not experimentally characterized
DAMPING_RATIO = 0.05
MAX_CARTESIAN_SPEED = 0.10
MAX_OMEGA = 0.6
POSITION_TOLERANCE = 0.005
CONTROL_PERIOD = 0.10
MAX_EXTRA_TIME = 25.0


# Only sound event: after POS2 arrival, before the Enter prompt.
MUSIC_TEMPO_BPM = 160
MUSIC_GAP_MS = 35

MUSIC_FREQUENCIES_HZ = {
    "G3": 196, "C4": 262, "D4": 294, "E4": 330,
    "F4": 349, "G4": 392, "A4": 440, "B4": 494,
    "C5": 523, "D5": 587,
}

# Short original four-measure version, through "imagination".
MUSIC_MEASURES = [
    [("E4", 2.5), ("G3", .5), ("C4", .5), ("E4", .5)],
    [("F4", 2.5), ("F4", .5), ("G4", .5), ("A4", .5)],
    [("D4", 2.5), ("D4", .5), ("G4", .5), ("D5", .5)],
    [("D5", 1), ("C5", .5), ("B4", .5), ("A4", 1), ("G4", 1)],
]


def play_arrival_music(robot):
    """Play short melody once at POS2 after wheels have stopped."""
    robot.wheel_velocity.stop()
    print("\nARRIVED AT POS2: playing short melody once...")
    beat_s = 60.0 / MUSIC_TEMPO_BPM
    try:
        for measure in MUSIC_MEASURES:
            for pitch, beats in measure:
                length_s = beats * beat_s
                tone_ms = max(40, round(length_s * 1000) - MUSIC_GAP_MS)
                response = robot.send("buzzer.tone", hz=MUSIC_FREQUENCIES_HZ[pitch], ms=tone_ms)
                if isinstance(response, dict) and not response.get("ok", True):
                    raise RuntimeError(f"Buzzer command failed: {response}")
                time.sleep(length_s)
        time.sleep(1.0)  # Allow last note to settle before Enter prompt.
        print("Short arrival melody finished. Robot remains stationary.")
    except Exception as exc:
        print(f"Arrival music stopped: {exc}. Robot remains stationary.")


# Nose positions, in meters, relative to its initial HOME location.
POSITIONS = {"HOME": (0.0, 0.0), "POS2": (1.20, -0.60)}


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class MinimumJerkProfile:
    def __init__(self, distance, max_speed, max_accel, max_jerk, minimum_time):
        self.distance = max(0.0, float(distance))
        if self.distance < 1e-12:
            self.duration = minimum_time
            return
        self.duration = max(
            minimum_time,
            1.875 * self.distance / max_speed,
            math.sqrt(5.7735026919 * self.distance / max_accel),
            (60.0 * self.distance / max_jerk) ** (1.0 / 3.0),
        )

    def sample(self, time_s):
        if time_s <= 0.0:
            return 0.0, 0.0
        if time_s >= self.duration:
            return self.distance, 0.0
        tau = time_s / self.duration
        sigma = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        sigma_dot = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / self.duration
        return self.distance * sigma, self.distance * sigma_dot


class ZVDShaper:
    def __init__(self, base_profile, natural_frequency_hz, damping_ratio, enabled=True):
        self.base_profile = base_profile
        if not enabled:
            self.impulses = [(1.0, 0.0)]
            self.delay = 0.0
            return
        if natural_frequency_hz <= 0:
            raise ValueError("ZVD natural frequency must be positive")
        if not 0 <= damping_ratio < 1:
            raise ValueError("ZVD damping ratio must be in [0, 1)")
        omega_n = 2 * math.pi * natural_frequency_hz
        root = math.sqrt(1 - damping_ratio * damping_ratio)
        omega_d = omega_n * root
        self.delay = math.pi / omega_d
        decay = math.exp(-damping_ratio * math.pi / root)
        denominator = (1 + decay)**2
        self.impulses = [
            (1 / denominator, 0.0),
            (2 * decay / denominator, self.delay),
            (decay**2 / denominator, 2 * self.delay),
        ]

    @property
    def total_duration(self):
        return self.base_profile.duration + self.impulses[-1][1]

    def sample(self, time_s):
        displacement, speed = 0.0, 0.0
        for amplitude, delay in self.impulses:
            s, v = self.base_profile.sample(time_s - delay)
            displacement += amplitude * s
            speed += amplitude * v
        return displacement, speed


def get_pose(robot):
    p = robot.pose.get()
    if not p.get("ok", False):
        raise RuntimeError(f"Could not read pose: {p}")
    return p["x_mm"] / 1000.0, p["y_mm"] / 1000.0, math.radians(p["theta_deg"])


def nose_displacement(x1, y1, theta):
    # At initial HOME, nose global coordinates are (NOSE_OFFSET, 0),
    # because pose.set(0, 0, 0) resets o1, NOT the physical nose.
    return (x1 + NOSE_OFFSET * (math.cos(theta) - 1.0),
            y1 + NOSE_OFFSET * math.sin(theta))


def set_wheels(robot, linear, angular):
    left_rad_s = (linear - 0.5 * L * angular) / R
    right_rad_s = (linear + 0.5 * L * angular) / R
    robot.wheel_velocity.set(left=math.degrees(left_rad_s),
                             right=math.degrees(right_rad_s))


def rotate_before_return(robot, heading_goal):
    """Turn to a specified heading, with stall/timeout diagnostics.

    This is a separate test phase: one wheel may reverse during a pivot,
    but the chassis should not intentionally drive backward.
    """
    print("  Pivoting in place toward the destination...")
    start = time.monotonic()
    last_report = -1.0
    last_progress_time = start
    _, _, initial_theta = get_pose(robot)
    progress_heading = initial_theta
    tolerance = math.radians(5.0)
    # Conservatively allow more time for physical turning and communications,
    # but do not spin indefinitely if the odometry is not progressing.
    PIVOT_TIMEOUT = 35.0
    STALL_TIMEOUT = 5.0
    PROGRESS_THRESHOLD = math.radians(2.0)
    try:
        while True:
            now = time.monotonic()
            _, _, theta = get_pose(robot)
            error = wrap_angle(heading_goal - theta)
            elapsed = now - start
            if elapsed - last_report >= 0.5:
                print(f"  pivot t={elapsed:5.1f}s | heading={math.degrees(theta):7.2f}deg | "
                      f"target={math.degrees(heading_goal):7.2f}deg | "
                      f"error={math.degrees(error):+7.2f}deg")
                last_report = elapsed
            if abs(error) <= tolerance:
                print("  Pivot completed within 5 deg tolerance.")
                return
            if abs(wrap_angle(theta - progress_heading)) >= PROGRESS_THRESHOLD:
                last_progress_time = now
                progress_heading = theta
            if now - last_progress_time > STALL_TIMEOUT:
                raise TimeoutError(
                    f"Pivot stalled: heading={math.degrees(theta):.1f} deg, "
                    f"target={math.degrees(heading_goal):.1f} deg. "
                    "Check whether the robot actually rotates and whether pose.theta_deg changes."
                )
            if elapsed > PIVOT_TIMEOUT:
                raise TimeoutError(
                    f"Pivot timeout: heading={math.degrees(theta):.1f} deg, "
                    f"target={math.degrees(heading_goal):.1f} deg, "
                    f"remaining error={math.degrees(error):.1f} deg."
                )
            omega = math.copysign(clamp(1.0 * abs(error), 0.20, 0.40), error)
            wheel_deg_s = math.degrees((L * omega / 2.0) / R)
            if elapsed - last_report < CONTROL_PERIOD + 0.05:
                print(f"           pivot wheel commands: L={-wheel_deg_s:+.1f}, R={wheel_deg_s:+.1f} deg/s")
            set_wheels(robot, 0.0, omega)
            time.sleep(CONTROL_PERIOD)
    finally:
        robot.wheel_velocity.stop()
        time.sleep(0.4)


def navigate_to(robot, name):
    target_x, target_y = POSITIONS[name]
    x1, y1, theta = get_pose(robot)
    current_nx, current_ny = nose_displacement(x1, y1, theta)
    dx, dy = target_x - current_nx, target_y - current_ny
    travel_distance = math.hypot(dx, dy)
    print(f"\n=== NAVIGATE TO {name} (nose target {target_x:.3f}, {target_y:.3f} m) ===")
    print(f"Nose now (global): ({current_nx:.3f}, {current_ny:.3f}) m")
    print(f"Pose heading: {math.degrees(theta):.1f} deg")
    if travel_distance < 0.01:
        print("Already near the goal; no movement commanded.")
        return

    heading_estimated = math.atan2(dy, dx)

    # A goal behind the chassis should not start with a long backing motion.
    # Instead pivot first. The outward leg retains its proven smooth-arc o5 control.
    if abs(wrap_angle(heading_estimated - theta)) > math.radians(90.0):
        rotate_before_return(robot, heading_estimated)
        # The nose moves along an arc around o1 during a pivot. Recalculate
        # the direction using its NEW position before setting the o5 target.
        x1, y1, theta = get_pose(robot)
        current_nx, current_ny = nose_displacement(x1, y1, theta)
        heading_estimated = math.atan2(target_y - current_ny,
                                       target_x - current_nx)

    # Fixed GLOBAL target for the o5 point. The final angle is estimated from
    # this leg's direction; the original o5 controller does not regulate it.
    goal_x5 = target_x + NOSE_OFFSET - (NOSE_OFFSET - d) * math.cos(heading_estimated)
    goal_y5 = target_y - (NOSE_OFFSET - d) * math.sin(heading_estimated)

    # Critical: recompute the starting o5 AFTER any pivot. Never reset pose.
    x1, y1, theta = get_pose(robot)
    initial_x5 = x1 + d * math.cos(theta)
    initial_y5 = y1 + d * math.sin(theta)
    path_dx = goal_x5 - initial_x5
    path_dy = goal_y5 - initial_y5
    path_length = math.hypot(path_dx, path_dy)
    if path_length < 1e-8:
        print("o5 already at goal")
        return
    direction_x, direction_y = path_dx / path_length, path_dy / path_length
    base = MinimumJerkProfile(path_length, MAX_REFERENCE_SPEED,
                              MAX_REFERENCE_ACCEL, MAX_REFERENCE_JERK,
                              CONTROL_PERIOD)
    profile = ZVDShaper(base, SLOSH_FREQUENCY_HZ, DAMPING_RATIO, USE_ZVD)
    print(f"Initial o5: ({initial_x5:.3f}, {initial_y5:.3f})")
    print(f"Goal o5:    ({goal_x5:.3f}, {goal_y5:.3f})")
    print(f"Reference duration: {profile.total_duration:.1f} s")

    start = time.monotonic()
    last_print = -10.0
    try:
        while True:
            elapsed = time.monotonic() - start
            x1, y1, theta = get_pose(robot)
            x5 = x1 + d * math.cos(theta)
            y5 = y1 + d * math.sin(theta)

            ref_s, ref_speed = profile.sample(elapsed)
            x_ref = initial_x5 + direction_x * ref_s
            y_ref = initial_y5 + direction_y * ref_s
            ux = direction_x * ref_speed + KX * (x_ref - x5)
            uy = direction_y * ref_speed + KY * (y_ref - y5)
            speed = math.hypot(ux, uy)
            if speed > MAX_CARTESIAN_SPEED:
                scale = MAX_CARTESIAN_SPEED / speed
                ux *= scale
                uy *= scale

            # Original o5 inverse-kinematics feedback, unchanged.
            v_command = math.cos(theta) * ux + math.sin(theta) * uy
            omega_command = (-math.sin(theta) * ux + math.cos(theta) * uy) / d
            omega_command = clamp(omega_command, -MAX_OMEGA, MAX_OMEGA)
            set_wheels(robot, v_command, omega_command)

            final_error = math.hypot(goal_x5 - x5, goal_y5 - y5)
            if elapsed - last_print > 0.7:
                nx, ny = nose_displacement(x1, y1, theta)
                print(f"t={elapsed:5.1f}  nose=({nx:.3f},{ny:.3f}) "
                      f"o5=({x5:.3f},{y5:.3f}) "
                      f"theta={math.degrees(theta):.1f}deg "
                      f"o5_err={100*final_error:.1f}cm")
                last_print = elapsed

            if elapsed >= profile.total_duration and final_error < POSITION_TOLERANCE:
                print(f"Reached o5 goal for {name}")
                break
            if elapsed > profile.total_duration + MAX_EXTRA_TIME:
                raise TimeoutError(f"{name} movement timed out; o5 error={final_error:.3f} m")
            time.sleep(CONTROL_PERIOD)
    finally:
        robot.wheel_velocity.stop()
        time.sleep(0.4)

    x1, y1, theta = get_pose(robot)
    nx, ny = nose_displacement(x1, y1, theta)
    print(f"FINISHED {name}: nose displacement=({nx:.3f}, {ny:.3f}) m; "
          f"heading={math.degrees(theta):.1f} deg")
    print(f"Estimated nose error: X={100*(nx-target_x):+.1f}cm, "
          f"Y={100*(ny-target_y):+.1f}cm")


def main():
    robot = Robot(ROBOT_IP)
    try:
        robot.motor.invert_set(left=False, right=False)
        robot.encoder.invert_set(left=True, right=False)
        robot.wheel_velocity.stop()
        time.sleep(0.5)
        # This is the ONLY pose reset in the entire experiment.
        robot.pose.set(x_mm=0, y_mm=0, theta_deg=0)
        time.sleep(0.5)
        print("\nStarting from physical HOME (nose on origin), facing +X")
        print("Initial pose:", robot.pose.get())

        navigate_to(robot, "POS2")
        print("\nRobot stopped at POS2. Measure the nose-to-mark error.")
        play_arrival_music(robot)
        input("Press ENTER to initiate the return HOME (Ctrl+C to abort): ")
        # Coordinate correction: pose.set(0,0,0) defines the axle CENTER
        # as (0,0) in odometry, even though the nose is physically at the
        # HOME floor mark at startup. After rotating back to theta=0,
        # the nose lands at the original HOME mark when axle center=(0,0).
        # While facing backwards, the nose must stage ~20 cm behind HOME
        # in the floor-mark-relative nose displacement coordinates.
        x1, y1, theta = get_pose(robot)
        home_center_x = 0.0
        home_center_y = 0.0
        approach_heading = math.atan2(home_center_y - y1,
                                      home_center_x - x1)
        # Nose's target immediately BEFORE its final pivot; pivot does not
        # translate o1, so this makes the nose land near (0, 0) afterward.
        staging_nose_x = home_center_x + NOSE_OFFSET * (math.cos(approach_heading) - 1.0)
        staging_nose_y = home_center_y + NOSE_OFFSET * math.sin(approach_heading)
        POSITIONS["HOME_STAGING"] = (staging_nose_x, staging_nose_y)
        print("\nHOME approach: aiming for final CENTER at "
              f"({home_center_x:.3f}, {home_center_y:.3f}) m, "
              "so the nose can finish at HOME after reorientation.")
        print(f"Pre-pivot nose target: ({staging_nose_x:.3f}, {staging_nose_y:.3f}) m")
        navigate_to(robot, "HOME_STAGING")
        print("\nSTAGING COMPLETE. Confirm clear space for final pivot.")
        input("Press ENTER to start the final 0 deg rotation (Ctrl+C to abort): ")
        print("\nFinal rotation to 0 deg at HOME...")
        rotate_before_return(robot, 0.0)
        x1, y1, theta = get_pose(robot)
        nx, ny = nose_displacement(x1, y1, theta)
        print(f"\nFINAL HOME: nose=({nx:.3f}, {ny:.3f}) m; "
              f"heading={math.degrees(theta):.1f} deg")
        print(f"Final estimated nose error: X={100*nx:+.1f}cm, "
              f"Y={100*ny:+.1f}cm")
        print("Round trip complete. Verify nose against original HOME floor mark.")
    finally:
        robot.wheel_velocity.stop()
        robot.close()


if __name__ == "__main__":
    main()

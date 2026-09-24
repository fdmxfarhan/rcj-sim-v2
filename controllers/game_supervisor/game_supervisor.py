from controller import Supervisor
import math
import random

TIME_STEP = 32
STUCK_TIMEOUT = 5.0
MOVE_THRESHOLD = 0.001  # متر

# ۵ نقطه استاندارد برای اسپاون مجدد توپ در صورت گیر کردن
RESPAWN_POINTS = [
    [0.0, 0.0, 0.03],
    [0.6, 0.4, 0.03],
    [-0.6, 0.4, 0.03],
    [-0.6, -0.4, 0.03],
    [0.6, -0.4, 0.03],
]

# ---- ابعاد زمین بازی (محدوده داخلی زمین) ----
FIELD_X_MIN = -1.22
FIELD_X_MAX =  1.22
FIELD_Y_MIN = -0.91
FIELD_Y_MAX =  0.91

# ---- ابعاد دهانه دروازه‌ها ----
GOAL_Y_MIN = -0.28
GOAL_Y_MAX =  0.28

BALL_Z = 0.03         # ارتفاع قرارگیری توپ روی چمن
ROBOT_Z = 0.04        # ارتفاع ربات‌ها روی زمین (متناسب با پروتوی شما)
INWARD_MARGIN = 0.08  # فاصله ایمن بعد از اوت شدن توپ


# ---- تعاریف موقعیت و زاویه اولیه کیک‌آف (Kickoff Configurations) ----
# فرمت زاویه محور-زاویه در وباتس: [x, y, z, angle_rad]
# برای ربات‌های زرد (نگاه به سمت +X یعنی دروازه راست): [0, 0, 1, 0]
# برای ربات‌های آبی (نگاه به سمت -X یعنی دروازه چپ): [0, 0, 1, math.pi]

KICKOFF_POSITIONS = {
    # حالتی که تیم آبی گل خورده و شروع‌کننده (Kickoff) است:
    "BLUE_KICKOFF": {
        "Y1": {"pos": [-0.4,  0.00, 0.04], "rot": [0, 0, 1, 0]},         # دروازه‌بان زرد
        "Y2": {"pos": [-0.72,  0.00, 0.04], "rot": [0, 0, 1, 0]},         # مهاجم زرد (عقب‌تر در نیمه خود)
        "B1": {"pos": [ 0.14,  0.00, 0.04], "rot": [0, 0, 1, math.pi]},   # دروازه‌بان آبی
        "B2": {"pos": [ 0.72,  0.00, 0.04], "rot": [0, 0, 1, math.pi]},   # مهاجم آبی (نزدیک توپ و روبروی آن)
    },
    # حالتی که تیم زرد گل خورده و شروع‌کننده (Kickoff) است:
    "YELLOW_KICKOFF": {
        "Y1": {"pos": [-0.14,  0.00, 0.04], "rot": [0, 0, 1, 0]},         # دروازه‌بان زرد
        "Y2": {"pos": [-0.72,  0.00, 0.04], "rot": [0, 0, 1, 0]},         # مهاجم زرد (نزدیک توپ و روبروی آن)
        "B1": {"pos": [ 0.4,  0.00, 0.04], "rot": [0, 0, 1, math.pi]},   # دروازه‌بان آبی
        "B2": {"pos": [ 0.72,  0.00, 0.04], "rot": [0, 0, 1, math.pi]},   # مهاجم آبی (عقب‌تر در نیمه خود)
    }
}

# ---- Wall Touch Penalty ----
WALL_PENALTY_TIME = 30.0       # seconds
WALL_TOUCH_MARGIN = 0.09       # approximate robot radius / wall-touch distance

# Robot is moved here while serving a penalty.
# These positions are deliberately outside the playing field.
PENALTY_POSITIONS = {
    "Y1": [ -1.5, -0.15, ROBOT_Z ],
    "Y2": [ -1.5, 0.15, ROBOT_Z ],
    "B1": [ 1.5, -0.15, ROBOT_Z ],
    "B2": [ 1.5, 0.15, ROBOT_Z ]
}
PENALTY_ROTATIONS = {
    # Yellow goal is at +X, so face toward -X
    "Y1": [0, 0, 1, math.pi],
    "Y2": [0, 0, 1, math.pi],

    # Blue goal is at -X, so face toward +X
    "B1": [0, 0, 1, 0],
    "B2": [0, 0, 1, 0],
}
# Neutral positions available after a wall penalty.
# All of them are safely inside the field.
NEUTRAL_POSITIONS = [
    [-0.75, -0.55, ROBOT_Z],
    [-0.75,  0.55, ROBOT_Z],
    [ 0.00, -0.55, ROBOT_Z],
    [ 0.00,  0.55, ROBOT_Z],
    [ 0.75, -0.55, ROBOT_Z],
    [ 0.75,  0.55, ROBOT_Z],
]
NEUTRAL_OCCUPANCY_DISTANCE = 0.25
BALL_SAFE_Z = 0.02

# ---- Match Timer ----
MATCH_TIME = 10.0 * 60.0   # 10 minutes in seconds
match_start_time = 0.0
match_finished = False

# ---- راه‌اندازی سوپروایزر ----
robot = Supervisor()

# توپ
ball_node = robot.getFromDef("BALL")
if ball_node is None:
    raise RuntimeError("Ball node with DEF 'BALL' not found.")
ball_translation = ball_node.getField("translation")
last_position = ball_translation.getSFVec3f()
stuck_time = 0.0

# ربات‌ها (Y1, Y2, B1, B2)
# مطمئن شوید در درخت صحنه (Scene Tree) فیلد DEF ربات‌ها دقیقاً یکی از این نام‌ها باشد
robot_defs = ["Y1", "Y2", "B1", "B2"]
robots_data = {}
# ---- Wall penalty state ----
# Example:
# {
#   "Y1": {
#       "active": True,
#       "until": 123.45,
#       "penalty_position": [...]
#   }
# }
wall_penalties = {}

for r_def in robot_defs:
    wall_penalties[r_def] = {
        "active": False,
        "until": 0.0,
        "penalty_position": None
    }

for r_def in robot_defs:
    node = robot.getFromDef(r_def)
    if node is not None:
        robots_data[r_def] = {
            "node": node,
            "translation": node.getField("translation"),
            "rotation": node.getField("rotation")
        }
    else:
        print(f"[Warning] Robot with DEF '{r_def}' not found in WorldInfo/Scene Tree.")



# متغیرهای امتیاز و وضعیت
score_yellow = 0
score_blue = 0
goal_cooldown = 0  # فریم‌های مکث پس از گل


def get_wall_touch(robot_pos):
    """
    Determine whether a robot is touching a field wall.

    Returns:
        "left"
        "right"
        "top"
        "bottom"
        None
    """

    x, y, _ = robot_pos

    # Left wall
    if x <= -1.11:
        return "left"

    # Right wall
    if x >= 1.11:
        return "right"

    # Bottom wall
    if y <= -0.8:
        return "bottom"

    # Top wall
    if y >= 0.8:
        return "top"

    return None

def check_ball_below_field(position):
    """
    Check whether the ball has fallen below the field.

    If it has, return a corrected position with Z = 0.02.
    """

    x, y, z = position

    if z < -BALL_SAFE_Z:
        return True, [x, y, BALL_SAFE_Z]

    return False, position

def get_penalty_position(robot_pos):
    """
    Put the robot outside the field according to
    the wall it touched.
    """

    wall = get_wall_touch(robot_pos)

    if wall is None:
        # Fallback position
        return PENALTY_POSITIONS["right"][:]

    return PENALTY_POSITIONS[wall][:]

def start_wall_penalty(robot_name):
    """
    Send a robot behind its own team's goal and start
    a 60-second wall penalty.

    The robot's rotation is also reset so that it faces
    toward the field.
    """

    if robot_name not in robots_data:
        return

    # Do not penalize an already penalized robot
    if wall_penalties[robot_name]["active"]:
        return

    r_info = robots_data[robot_name]

    penalty_pos = PENALTY_POSITIONS[robot_name][:]
    penalty_rot = PENALTY_ROTATIONS[robot_name][:]

    now = robot.getTime()

    # ---------------------------------------------
    # Move robot behind its team's goal
    # ---------------------------------------------
    r_info["translation"].setSFVec3f(penalty_pos)

    # ---------------------------------------------
    # Reset rotation
    # ---------------------------------------------
    if r_info["rotation"]:
        r_info["rotation"].setSFRotation(penalty_rot)

    # ---------------------------------------------
    # Reset physics
    # ---------------------------------------------
    r_info["node"].resetPhysics()

    # ---------------------------------------------
    # Store penalty state
    # ---------------------------------------------
    wall_penalties[robot_name]["active"] = True
    wall_penalties[robot_name]["until"] = (
        now + WALL_PENALTY_TIME
    )
    wall_penalties[robot_name]["penalty_position"] = penalty_pos

    team = "YELLOW" if robot_name.startswith("Y") else "BLUE"

    # print(
    #     f"[WALL PENALTY] {robot_name} ({team}) "
    #     f"sent behind goal for "
    #     f"{WALL_PENALTY_TIME:.0f} seconds."
    # )

def maintain_wall_penalty(robot_name):
    """
    Keep a penalized robot at its designated position
    behind its team's goal.
    """

    if robot_name not in robots_data:
        return

    penalty = wall_penalties[robot_name]

    if not penalty["active"]:
        return

    r_info = robots_data[robot_name]

    r_info["translation"].setSFVec3f(
        penalty["penalty_position"]
    )

    r_info["node"].resetPhysics()

def find_farthest_neutral_position(ball_position, returning_robot_name):
    """
    Find the farthest neutral position from the ball that
    is not occupied by another robot.

    Neutral positions are checked from farthest to nearest.
    The first available position is selected.
    """

    candidates = []

    # ---------------------------------------------
    # Calculate distance of every neutral position
    # from the ball
    # ---------------------------------------------
    for position in NEUTRAL_POSITIONS:
        distance = distance2d(position, ball_position)

        candidates.append({
            "position": position,
            "distance": distance
        })

    # ---------------------------------------------
    # Sort from farthest to nearest
    # ---------------------------------------------
    candidates.sort(
        key=lambda item: item["distance"],
        reverse=True
    )

    # ---------------------------------------------
    # Check each position
    # ---------------------------------------------
    for candidate in candidates:

        position = candidate["position"]

        occupied = False

        for robot_name, r_info in robots_data.items():

            # Don't compare the robot with itself
            if robot_name == returning_robot_name:
                continue

            # Ignore robots currently serving penalties
            if wall_penalties[robot_name]["active"]:
                continue

            other_position = r_info["translation"].getSFVec3f()

            distance = distance2d(
                position,
                other_position
            )

            if distance < NEUTRAL_OCCUPANCY_DISTANCE:
                occupied = True
                break

        # -----------------------------------------
        # This position is free
        # -----------------------------------------
        if not occupied:
            # print(
            #     f"[NEUTRAL POSITION] "
            #     f"{returning_robot_name} -> "
            #     f"{position[:2]} "
            #     f"(distance from ball: "
            #     f"{candidate['distance']:.3f}m)"
            # )

            return position[:]

        # print(
        #     f"[NEUTRAL POSITION] "
        #     f"{position[:2]} occupied, "
        #     f"trying next farthest position."
        # )

    # ---------------------------------------------
    # No free neutral position
    # ---------------------------------------------
    # print(
    #     f"[WARNING] No free neutral position for "
    #     f"{returning_robot_name}!"
    # )

    return None

def release_wall_penalty(robot_name, ball_position):
    """
    Release a robot after its 60-second penalty.

    The robot is placed at the farthest available
    neutral position from the current ball.
    """

    if robot_name not in robots_data:
        return

    penalty = wall_penalties[robot_name]

    if not penalty["active"]:
        return

    target_position = find_farthest_neutral_position(
        ball_position,
        robot_name
    )

    # No available position yet.
    # Keep the robot behind the goal and continue waiting.
    if target_position is None:
        return

    r_info = robots_data[robot_name]

    # ---------------------------------------------
    # Return robot to field
    # ---------------------------------------------
    r_info["translation"].setSFVec3f(
        target_position
    )

    r_info["node"].resetPhysics()

    # ---------------------------------------------
    # Clear penalty
    # ---------------------------------------------
    penalty["active"] = False
    penalty["until"] = 0.0
    penalty["penalty_position"] = None

    # print(
    #     f"[WALL PENALTY END] {robot_name} returned "
    #     f"to {target_position[:2]}"
    # )

def update_wall_penalties(ball_position):
    """
    Update all wall penalties.

    1. Keep penalized robots outside.
    2. Release robots whose 60 seconds have expired.
    3. Detect new wall touches for active robots.
    """

    now = robot.getTime()

    for robot_name in robots_data:

        penalty = wall_penalties[robot_name]

        # -------------------------------------------------
        # Robot is currently serving a penalty
        # -------------------------------------------------
        if penalty["active"]:

            # Keep robot outside
            maintain_wall_penalty(robot_name)

            # Has the 60 seconds expired?
            if now >= penalty["until"]:
                release_wall_penalty(
                    robot_name,
                    ball_position
                )

            continue

        # -------------------------------------------------
        # Robot is active -> detect wall touch
        # -------------------------------------------------

        r_info = robots_data[robot_name]

        robot_position = r_info["translation"].getSFVec3f()

        wall = get_wall_touch(robot_position)

        if wall is not None:
            start_wall_penalty(robot_name)

def distance2d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def nearest_point(position, points):
    return random.choice(points)

def check_goal(pos):
    """
    بررسی گل شدن توپ برای دو تیم
    """
    x, y, _ = pos
    if GOAL_Y_MIN <= y <= GOAL_Y_MAX:
        if x > 1.105:
            return "YELLOW"  # ورود به دروازه آبی -> گل زرد
        elif x < -1.105:
            return "BLUE"    # ورود به دروازه زرد -> گل آبی
    return None

def check_out_of_bounds(pos):
    """
    بررسی اوت شدن توپ
    """
    x, y, _ = pos
    is_out = False
    new_x, new_y = x, y

    # دیواره‌های راست / چپ
    if x > FIELD_X_MAX:
        new_x = FIELD_X_MAX - INWARD_MARGIN
        is_out = True
    elif x < FIELD_X_MIN:
        new_x = FIELD_X_MIN + INWARD_MARGIN
        is_out = True

    # دیواره‌های بالا / پایین
    if y > FIELD_Y_MAX:
        new_y = FIELD_Y_MAX - INWARD_MARGIN
        is_out = True
    elif y < FIELD_Y_MIN:
        new_y = FIELD_Y_MIN + INWARD_MARGIN
        is_out = True

    new_x = max(FIELD_X_MIN + INWARD_MARGIN, min(FIELD_X_MAX - INWARD_MARGIN, new_x))
    new_y = max(FIELD_Y_MIN + INWARD_MARGIN, min(FIELD_Y_MAX - INWARD_MARGIN, new_y))

    return is_out, [new_x, new_y, BALL_Z]

def respawn_robots(kickoff_team):
    """
    Reset all 4 robots for kickoff.

    Any active wall penalty is cancelled.
    All robots return to their kickoff positions.
    """

    config_key = f"{kickoff_team}_KICKOFF"
    setup = KICKOFF_POSITIONS[config_key]

    for r_name, r_info in robots_data.items():

        if r_name not in setup:
            continue

        target_pos = setup[r_name]["pos"]
        target_rot = setup[r_name]["rot"]

        # ---------------------------------------------
        # Cancel wall penalty
        # ---------------------------------------------
        if r_name in wall_penalties:
            wall_penalties[r_name]["active"] = False
            wall_penalties[r_name]["until"] = 0.0
            wall_penalties[r_name]["penalty_position"] = None

        # ---------------------------------------------
        # Move robot back to kickoff position
        # ---------------------------------------------
        r_info["translation"].setSFVec3f(target_pos)

        if r_info["rotation"]:
            r_info["rotation"].setSFRotation(target_rot)

        # ---------------------------------------------
        # Reset physics
        # ---------------------------------------------
        r_info["node"].resetPhysics()

    # print(
    #     f"[KICKOFF] All robots returned to field. "
    #     f"{kickoff_team} will kick off."
    # )

def update_match_timer():
    """
    Update the 10-minute match countdown.
    """

    global match_finished

    if match_finished:
        remaining = 0.0
    else:
        elapsed = robot.getTime() - match_start_time
        remaining = max(0.0, MATCH_TIME - elapsed)

    minutes = int(remaining // 60)
    seconds = int(remaining % 60)

    timer_text = f"{minutes:02d}:{seconds:02d}"

    # Display timer at the top center
    robot.setLabel(
        14,
        timer_text,
        0.47,
        0.01,
        0.12,
        0xFFFFFF,
        0.0,
        "Arial"
    )

    # Match finished
    if remaining <= 0.0 and not match_finished:
        match_finished = True

        # print("[MATCH] 10 minutes reached. Match finished.")

        # Make sure display shows exactly 00:00
        robot.setLabel(
            14,
            "00:00",
            0.47,
            0.01,
            0.12,
            0xFFFFFF,
            0.0,
            "Arial"
        )

        # Pause Webots simulation
        robot.simulationSetMode(
            Supervisor.SIMULATION_MODE_PAUSE
        )

def update_scoreboard(status_text="", status_color=0x00FF66):
    # Yellow score - top left
    robot.setLabel(
        10,
        f"YELLOW: {score_yellow}",
        0.02, 0.01, 0.10,
        0xFFFF00,
        0.0,
        "Arial"
    )

    # Blue score - top right
    robot.setLabel(
        11,
        f"{score_blue} :BLUE",
        0.85, 0.01, 0.10,
        0x3399FF,
        0.0,
        "Arial"
    )

    # Status message - center
    if status_text:
        robot.setLabel(
            13,
            status_text,
            0.41, 0.5, 0.11,
            status_color,
            0.0,
            "Arial"
        )
    else:
        robot.setLabel(
            13,
            "",
            0.41, 0.5, 0.11,
            0x000000,
            0.0,
            "Arial"
        )

update_scoreboard()
match_start_time = robot.getTime()
update_match_timer()

while robot.step(TIME_STEP) != -1:
    current_position = ball_translation.getSFVec3f()
    
    update_match_timer()

    ball_below_field, corrected_position = check_ball_below_field(
        current_position
    )

    if ball_below_field:
        ball_translation.setSFVec3f(corrected_position)
        ball_node.resetPhysics()

        current_position = corrected_position[:]

        last_position = corrected_position[:]
        stuck_time = 0.0

        # print(
        #     f"[BALL SAFETY] Ball was below field. "
        #     f"Returned to Z={BALL_SAFE_Z:.2f}"
        # )

        continue
    
    update_wall_penalties(current_position)

    # ۱. مدیریت زمان بعد از گل (مکث و نمایش اعلان)
    if goal_cooldown > 0:
        goal_cooldown -= 1
        if goal_cooldown == 0:
            update_scoreboard("")
        continue

    # ۲. بررسی به ثمر رسیدن گل
    scoring_team = check_goal(current_position)
    if scoring_team:
        if scoring_team == "YELLOW":
            score_yellow += 1
            kickoff_team = "BLUE"  # زرد گل زده -> آبی شروع‌کننده است
            # print(f"[GOAL!] Team YELLOW scored! ({score_yellow} - {score_blue})")
            update_scoreboard("GOAL FOR YELLOW!", 0xFFFF00)
        else:
            score_blue += 1
            kickoff_team = "YELLOW"  # آبی گل زده -> زرد شروع‌کننده است
            # print(f"[GOAL!] Team BLUE scored! ({score_yellow} - {score_blue})")
            update_scoreboard("GOAL FOR BLUE!", 0x3399FF)

        # ریست توپ به مرکز زمین
        center_pos = [0.0, 0.0, BALL_Z]
        ball_translation.setSFVec3f(center_pos)
        ball_node.resetPhysics()

        # ریست و بازچینی ۴ ربات با آرایش کیک‌آف
        respawn_robots(kickoff_team)
        
        last_position = center_pos[:]
        stuck_time = 0.0
        goal_cooldown = int(2.0 / (TIME_STEP / 1000.0))  # ۲ ثانیه مکث
        continue

    # ۳. بررسی خروج توپ از محدوده بازی (اوت)
    is_out, respawn_pos = check_out_of_bounds(current_position)
    if is_out:
        ball_translation.setSFVec3f(respawn_pos)
        ball_node.resetPhysics()
        last_position = respawn_pos[:]
        stuck_time = 0.0
        # print(f"[Out of Bounds] Ball returned: {respawn_pos[:2]}")
        continue

    # ۴. بررسی گیر کردن توپ (Stuck)
    moved = distance2d(current_position, last_position)
    if moved < MOVE_THRESHOLD:
        stuck_time += TIME_STEP / 1000.0
    else:
        stuck_time = 0.0

    if stuck_time >= STUCK_TIMEOUT:
        target = nearest_point(current_position, RESPAWN_POINTS)
        ball_translation.setSFVec3f(target)
        ball_node.resetPhysics()

        # print(f"[Stuck] Ball respawned: {target}")
        last_position = target[:]
        stuck_time = 0.0
        continue

    last_position = current_position[:]
    

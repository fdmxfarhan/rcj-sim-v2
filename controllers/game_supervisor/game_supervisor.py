from controller import Supervisor
import math
import random

TIME_STEP = 32
STUCK_TIMEOUT = 6.0
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
ROBOT_Z = 0.08        # ارتفاع ربات‌ها روی زمین (متناسب با پروتوی شما)
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


def respawn_robots(kickoff_team):
    """
    ریست کردن موقعیت و فیزیک ۴ ربات براساس تیمی که گل خورده است
    kickoff_team: 'BLUE' یا 'YELLOW'
    """
    config_key = f"{kickoff_team}_KICKOFF"
    setup = KICKOFF_POSITIONS[config_key]

    for r_name, r_info in robots_data.items():
        if r_name in setup:
            target_pos = setup[r_name]["pos"]
            target_rot = setup[r_name]["rot"]
            
            # تغییر موقعیت و چرخش
            r_info["translation"].setSFVec3f(target_pos)
            if r_info["rotation"]:
                r_info["rotation"].setSFRotation(target_rot)
            
            # صفر کردن اینرسی و سرعت ربات
            r_info["node"].resetPhysics()


# متغیرهای امتیاز و وضعیت
score_yellow = 0
score_blue = 0
goal_cooldown = 0  # فریم‌های مکث پس از گل

def update_scoreboard(status_text="", status_color=0x00FF66):
    """
    نمایش متن‌های چند رنگ روی پنجره Webots
    """
    robot.setLabel(10, f"YELLOW: {score_yellow}", 0.32, 0.03, 0.10, 0xFFFF00, 0.0, "Arial")
    robot.setLabel(11, " | ", 0.49, 0.03, 0.10, 0xFFFFFF, 0.0, "Arial")
    robot.setLabel(12, f"{score_blue} :BLUE", 0.53, 0.03, 0.10, 0x3399FF, 0.0, "Arial")

    if status_text:
        robot.setLabel(13, status_text, 0.41, 0.09, 0.11, status_color, 0.0, "Arial")
    else:
        robot.setLabel(13, "", 0.41, 0.09, 0.11, 0x000000, 0.0, "Arial")


# نمایش اولیه اسکوربورد
update_scoreboard()

while robot.step(TIME_STEP) != -1:
    current_position = ball_translation.getSFVec3f()

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
            print(f"[GOAL!] Team YELLOW scored! ({score_yellow} - {score_blue})")
            update_scoreboard("GOAL FOR YELLOW!", 0xFFFF00)
        else:
            score_blue += 1
            kickoff_team = "YELLOW"  # آبی گل زده -> زرد شروع‌کننده است
            print(f"[GOAL!] Team BLUE scored! ({score_yellow} - {score_blue})")
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
        print(f"[Out of Bounds] Ball returned: {respawn_pos[:2]}")
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

        print(f"[Stuck] Ball respawned: {target}")
        last_position = target[:]
        stuck_time = 0.0
        continue

    last_position = current_position[:]

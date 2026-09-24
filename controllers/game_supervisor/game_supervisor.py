from controller import Supervisor
import math, random

TIME_STEP = 32
STUCK_TIMEOUT = 3.0
MOVE_THRESHOLD = 0.01  # متر

# 5 predefined respawn points
RESPAWN_POINTS = [
    [0.0, 0.0, 0.03],
    [0.6, 0.4, 0.03],
    [-0.6, 0.4, 0.03],
    [-0.6, -0.4, 0.03],
    [0.6, -0.4, 0.03],
]

# ---- ابعاد زمین بازی (محدوده داخلی زمین) ----
# این مقادیر را مطابق ابعاد دقیق زمین خود (خطوط اوت/دیواره‌ها) تنظیم کنید
FIELD_X_MIN = -1.22
FIELD_X_MAX =  1.22
FIELD_Y_MIN = -0.91
FIELD_Y_MAX =  0.91

BALL_Z = 0.03       # ارتفاع قرارگیری توپ روی چمن
INWARD_MARGIN = 0.08  # فاصله‌ای که توپ از دیواره به داخل زمین بازگردانده می‌شود


def distance2d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def nearest_point(position, points):
    return random.choice(points)


def check_out_of_bounds(pos):
    """
    بررسی خروج توپ از کادر زمین و محاسبه موقعیت برگشت چسبیده به همان دیواره
    """
    x, y, _ = pos
    is_out = False
    new_x, new_y = x, y

    # بررسی دیواره‌های شرقی / غربی (محور X)
    if x > FIELD_X_MAX:
        new_x = FIELD_X_MAX - INWARD_MARGIN
        is_out = True
    elif x < FIELD_X_MIN:
        new_x = FIELD_X_MIN + INWARD_MARGIN
        is_out = True

    # بررسی دیواره‌های شمالی / جنوبی (محور Y)
    if y > FIELD_Y_MAX:
        new_y = FIELD_Y_MAX - INWARD_MARGIN
        is_out = True
    elif y < FIELD_Y_MIN:
        new_y = FIELD_Y_MIN + INWARD_MARGIN
        is_out = True

    # محدود نگه داشتن مختصات دیگر در بازه مجاز در صورت پرتاب به گوشه‌ها
    new_x = max(FIELD_X_MIN + INWARD_MARGIN, min(FIELD_X_MAX - INWARD_MARGIN, new_x))
    new_y = max(FIELD_Y_MIN + INWARD_MARGIN, min(FIELD_Y_MAX - INWARD_MARGIN, new_y))

    return is_out, [new_x, new_y, BALL_Z]


robot = Supervisor()

ball_node = robot.getFromDef("BALL")
if ball_node is None:
    raise RuntimeError("Ball node with DEF BALL not found.")

ball_translation = ball_node.getField("translation")

last_position = ball_translation.getSFVec3f()
stuck_time = 0.0

while robot.step(TIME_STEP) != -1:
    current_position = ball_translation.getSFVec3f()

    # ۱. بررسی فوری خروج توپ از زمین
    is_out, respawn_pos = check_out_of_bounds(current_position)
    if is_out:
        ball_translation.setSFVec3f(respawn_pos)
        ball_node.resetPhysics()  # حذف تکانه و سرعت قبلی برای توقف در محل جدید
        last_position = respawn_pos[:]
        stuck_time = 0.0
        print(f"[Out of Bounds] Ball returned to field boundary: {respawn_pos[:2]}")
        continue

    # ۲. بررسی گیر کردن توپ (Stuck Detection)
    moved = distance2d(current_position, last_position)
    if moved < MOVE_THRESHOLD:
        stuck_time += TIME_STEP / 1000.0
    else:
        stuck_time = 0.0

    if stuck_time >= STUCK_TIMEOUT:
        target = nearest_point(current_position, RESPAWN_POINTS)
        ball_translation.setSFVec3f(target)
        ball_node.resetPhysics()

        print(f"[Stuck] Ball reset to center/respawn point: {target}")

        last_position = target[:]
        stuck_time = 0.0
        continue

    last_position = current_position[:]

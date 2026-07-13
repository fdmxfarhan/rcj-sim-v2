from controller import Supervisor
import math, random

TIME_STEP = 32
STUCK_TIMEOUT = 3.0
MOVE_THRESHOLD = 0.01  # meters; treat smaller movement as no progress

# 5 predefined respawn points
RESPAWN_POINTS = [
    [0.0, 0.0, 0.03],
    [0.6, 0.4, 0.03],
    [-0.6, 0.4, 0.03],
    [-0.6, -0.4, 0.03],
    [0.6, -0.4, 0.03],
]

def distance2d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def nearest_point(position, points):
    return random.choice(points)
    # return min(points, key=lambda p: distance2d(position, p))

robot = Supervisor()

# Replace BALL with your ball node DEF name in the world
ball_node = robot.getFromDef("BALL")
if ball_node is None:
    raise RuntimeError("Ball node with DEF BALL not found.")

ball_translation = ball_node.getField("translation")

last_position = ball_translation.getSFVec3f()
stuck_time = 0.0

while robot.step(TIME_STEP) != -1:
    current_position = ball_translation.getSFVec3f()
    moved = distance2d(current_position, last_position)

    if moved < MOVE_THRESHOLD:
        stuck_time += TIME_STEP / 1000.0
    else:
        stuck_time = 0.0

    if stuck_time >= STUCK_TIMEOUT:
        target = nearest_point(current_position, RESPAWN_POINTS)
        ball_translation.setSFVec3f(target)
        ball_node.resetPhysics()

        print(f"Ball was stuck. Moved to nearest point: {target}")

        last_position = target[:]
        stuck_time = 0.0
        continue

    last_position = current_position[:]

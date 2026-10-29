import math
import random

# Ball memory / search
BALL_MEMORY_TIME = 3.0     # seconds a lost ball's last position is trusted
REACHED_DIST = 0.1         # metres

# Stuck detection
STUCK_WINDOW = 0.8         # seconds of trying without progress before reacting
STUCK_DIST = 0.03          # metres the robot or the ball must move to count as progress
MOVING_CMD = 6             # translation command that counts as "trying to move"
ESCAPE_SPEED = 15
REPEAT_TIME = 5.0          # getting stuck again this soon...
REPEAT_DIST = 0.3          # ...and this close to the last place counts as a repeat
WALL_X = 0.95              # escapes never drive further out than this
WALL_Y = 0.65
GUARD_X = 1.0              # no robot drives further towards a wall than this,
GUARD_Y = 0.7              # and past it gets nudged back in
GUARD_PUSH = 6             # (the supervisor penalises |x| >= 1.11 or |y| >= 0.8)
GUARD_X_GOAL = 1.05        # in front of a goal robots may go deeper, to push the ball in or out
SCRUM_DIST = 0.2           # ball this close...
SCRUM_TIME = 1.5           # ...for this long...
SCRUM_BALL_MOVE = 0.08     # ...while moving less than this is a scrum worth breaking up

# Each robot runs its own controller process, so module state is per robot.
_nav = {
    "cmd": (0.0, 0.0),
    "window_t": None,
    "window_pos": None,
    "window_ball": None,
    "plan": [],            # escape steps: (until_time, wx, wy, dribbler)
    "stuck_t": -100.0,
    "stuck_pos": (0.0, 0.0),
    "repeats": 0,
    "side": 1,
    "scrum_t": None,
    "scrum_ball": None,
    "ball": None,
    "ball_t": -1.0,
    "search_idx": None,
}

def _wallGuard(robot, vx, vy):
    """Drop the part of a move that heads into a nearby wall (touching one is a 30 s penalty)
    and nudge the robot back in, in case it drifted or got pushed there.

    vx is to the robot's right, vy straight ahead; sliding along the wall is still allowed.
    """
    pos = robot.get_position()
    guard_x = GUARD_X_GOAL if abs(pos[1]) < 0.25 else GUARD_X
    if abs(pos[0]) < guard_x and abs(pos[1]) < GUARD_Y:
        return vx, vy
    h = robot.get_heading()
    wx = vy * math.cos(h) + vx * math.sin(h)
    wy = vy * math.sin(h) - vx * math.cos(h)
    if abs(pos[0]) >= guard_x:
        wx = -math.copysign(GUARD_PUSH, pos[0]) if wx * pos[0] > 0 else wx - math.copysign(GUARD_PUSH, pos[0])
    if abs(pos[1]) >= GUARD_Y:
        wy = -math.copysign(GUARD_PUSH, pos[1]) if wy * pos[1] > 0 else wy - math.copysign(GUARD_PUSH, pos[1])
    return wx * math.sin(h) - wy * math.cos(h), wx * math.cos(h) + wy * math.sin(h)

def moveXY(robot, vx, vy, w=0):
    vx, vy = _wallGuard(robot, vx, vy)
    _nav["cmd"] = (vx, vy)
    v1 = vx + vy + w
    v2 = -vx + vy + w
    v3 = -vx - vy + w
    v4 = vx - vy + w
    robot.motor(v1, v2, v3, v4)

def moveAngle(robot, angle_deg, speed, heading):
    rad = math.radians(angle_deg)
    vx = -speed * math.sin(rad)
    vy = speed * math.cos(rad)
    moveXY(robot, vx, vy, heading*20)

def moveTo(robot, target_x, target_y, target_w=0):
    pos = robot.get_position()
    heading = robot.get_heading()

    err_x = (pos[0] - target_x) * 50
    err_y = (pos[1] - target_y) * 50
    err_w = (heading - target_w) * 20
    moveXY(robot, err_y, -err_x, err_w)

def getAngle(robot, pos1):
    abs_angle = math.atan2(pos1[0], pos1[1])
    relative_angle_deg = math.degrees(abs_angle)
    if relative_angle_deg > 180: relative_angle_deg -= 360
    if relative_angle_deg < -180: relative_angle_deg += 360
    return relative_angle_deg

def angleBetween(robot, pos1, pos2):
    heading = robot.get_heading()
    dx = pos1[0] - pos2[0]
    dy = pos1[1] - pos2[1]
    abs_angle = math.atan2(dy, dx)
    relative_angle_rad = abs_angle - heading
    relative_angle_deg = math.degrees(relative_angle_rad)
    if relative_angle_deg > 180: relative_angle_deg -= 360
    if relative_angle_deg < -180: relative_angle_deg += 360
    return relative_angle_deg

def driveWorld(robot, wx, wy, speed):
    """Drive in a field direction (wx, wy) while keeping heading at 0."""
    if wx == 0 and wy == 0:
        moveXY(robot, 0, 0, robot.get_heading()*20)
        return
    heading = robot.get_heading()
    rel = math.atan2(wy, wx) - heading
    moveXY(robot, -speed * math.sin(rel), speed * math.cos(rel), heading*20)

def goTo(robot, x, y, speed=15):
    """Like moveTo, but with a capped speed so it doesn't saturate the wheels."""
    pos = robot.get_position()
    dx = x - pos[0]
    dy = y - pos[1]
    dist = math.hypot(dx, dy)
    driveWorld(robot, dx, dy, speed * min(1.0, dist / 0.15))

def rememberBall(robot, ball_pos):
    """Store where the ball is on the field so it can be found again after losing sight of it."""
    if not ball_pos:
        return
    pos = robot.get_position()
    dist = math.hypot(ball_pos[0], ball_pos[1])
    angle = robot.get_heading() + math.atan2(ball_pos[0], ball_pos[1])
    _nav["ball"] = (pos[0] + dist * math.cos(angle), pos[1] + dist * math.sin(angle))
    _nav["ball_t"] = robot.robot.getTime()
    _nav["search_idx"] = None

def lastBallPosition(robot):
    """Last seen field position of the ball, or None if it is too old."""
    if _nav["ball"] and robot.robot.getTime() - _nav["ball_t"] < BALL_MEMORY_TIME:
        return _nav["ball"]
    return None

def teammateOnField(robot):
    """False while our teammate is off the field serving a wall penalty.

    Robots can't message each other, so this reads the teammate's position through the
    Supervisor API (the team controllers already run as Supervisors).
    """
    team, number = robot.name[-2], robot.name[-1]
    mate = robot.robot.getFromDef(team + ("2" if number == "1" else "1"))
    if mate is None:
        return True
    x, y = mate.getPosition()[:2]
    return abs(x) < 1.25 and abs(y) < 0.95

def clearBall(robot, ball, goal=(-1.1, 0.0), speed=20):
    """Push the ball away from our own goal, going round it first if we're on the wrong side.

    The push always points away from our goal centre, so a clearance can't turn into an
    own goal, and it leans upfield so the ball doesn't just end up in our corner.
    """
    pos = robot.get_position()
    if abs(ball[1]) > 0.6:
        dx, dy = 1.0, -math.copysign(0.3, ball[1])    # by a side wall: push along it, upfield
    else:
        dx, dy = ball[0] - goal[0], ball[1] - goal[1]
        norm = math.hypot(dx, dy) or 1.0
        dx, dy = dx / norm + 0.8, dy / norm
    norm = math.hypot(dx, dy)
    dx, dy = dx / norm, dy / norm                     # direction to push the ball in
    behind = (ball[0] - dx * 0.13, ball[1] - dy * 0.13)

    # How far we are behind the ball along the push direction (positive = behind it).
    depth = (ball[0] - pos[0]) * dx + (ball[1] - pos[1]) * dy
    if depth < 0.06:
        # Wrong side: swing round the ball on whichever side we're already on.
        side = 1 if (pos[0] - ball[0]) * -dy + (pos[1] - ball[1]) * dx >= 0 else -1
        target = (behind[0] - dy * side * 0.15, behind[1] + dx * side * 0.15)
    elif math.hypot(behind[0] - pos[0], behind[1] - pos[1]) > 0.05:
        target = behind
    else:
        driveWorld(robot, dx, dy, speed)               # lined up: push it out
        return
    goTo(robot, target[0], target[1], speed)

def searchBall(robot, search_points, speed=15):
    """Go to where the ball was last seen, then patrol search_points until it shows up again."""
    pos = robot.get_position()
    last = lastBallPosition(robot)
    if last and math.hypot(last[0] - pos[0], last[1] - pos[1]) > REACHED_DIST:
        goTo(robot, last[0], last[1], speed)
        return
    _nav["ball"] = None

    if _nav["search_idx"] is None:
        # Start from the nearest patrol point.
        _nav["search_idx"] = min(range(len(search_points)),
                                 key=lambda i: math.hypot(search_points[i][0] - pos[0], search_points[i][1] - pos[1]))
    target = search_points[_nav["search_idx"]]
    if math.hypot(target[0] - pos[0], target[1] - pos[1]) < REACHED_DIST:
        _nav["search_idx"] = (_nav["search_idx"] + 1) % len(search_points)
        target = search_points[_nav["search_idx"]]
    goTo(robot, target[0], target[1], speed)

def _resetStuckWindow(t, pos):
    _nav["window_t"] = t
    _nav["window_pos"] = pos
    _nav["window_ball"] = _nav["ball"]

def _awayFromWalls(pos, wx, wy):
    """Stop an escape direction from driving the robot into a wall (and a wall penalty)."""
    if abs(pos[0]) > WALL_X and wx * pos[0] > 0:
        wx = -math.copysign(0.5, pos[0])
    if abs(pos[1]) > WALL_Y and wy * pos[1] > 0:
        wy = -math.copysign(0.5, pos[1])
    return wx, wy

def _ballIfClose(pos):
    ball = _nav["ball"]
    if ball and math.hypot(ball[0] - pos[0], ball[1] - pos[1]) < SCRUM_DIST:
        return ball
    return None

def _inScrum(t, pos):
    """True when the ball has stayed next to us without getting anywhere for SCRUM_TIME,
    e.g. two strikers pushing it against each other."""
    ball = _ballIfClose(pos)
    if ball is None:
        _nav["scrum_t"] = None
        return False
    start = _nav["scrum_ball"]
    if _nav["scrum_t"] is None or math.hypot(ball[0] - start[0], ball[1] - start[1]) >= SCRUM_BALL_MOVE:
        _nav["scrum_t"] = t
        _nav["scrum_ball"] = ball
        return False
    return t - _nav["scrum_t"] >= SCRUM_TIME

def escapeIfStuck(robot, tackle=False):
    """Get out of jams: pushing into another robot, or a ball squeezed between two robots.

    The robot counts as stuck when it is trying to move (or holding the ball) but has not
    moved, and neither has the ball next to it, for STUCK_WINDOW. It then reverses out,
    steps to one side and, with tackle=True (strikers), hits the ball from that side.
    Tackling strikers also break up scrums where the ball stays put between robots.

    Call at the start of loop(); when it returns True it has already set the motors.
    """
    t = robot.robot.getTime()
    pos = robot.get_position()

    while _nav["plan"] and t >= _nav["plan"][0][0]:
        _nav["plan"].pop(0)
        _resetStuckWindow(t, pos)
        _nav["scrum_t"] = None
    if _nav["plan"]:
        _, wx, wy, dribble = _nav["plan"][0]
        robot.set_dribbler(dribble)
        wx, wy = _awayFromWalls(pos, wx, wy)
        driveWorld(robot, wx, wy, ESCAPE_SPEED)
        return True

    # Off the field means the supervisor is holding us for a wall penalty.
    if abs(pos[0]) > 1.25 or abs(pos[1]) > 0.95:
        _resetStuckWindow(t, pos)
        _nav["scrum_t"] = None
        return False

    if tackle and _inScrum(t, pos):
        _nav["scrum_t"] = None
        _planEscape(robot, t, pos, tackle)
        return escapeIfStuck(robot, tackle)

    vx, vy = _nav["cmd"]
    trying = robot.is_ball_touched() or math.hypot(vx, vy) >= MOVING_CMD
    if not trying or _nav["window_t"] is None:
        _resetStuckWindow(t, pos)
        return False
    if t - _nav["window_t"] < STUCK_WINDOW:
        return False

    moved = math.hypot(pos[0] - _nav["window_pos"][0], pos[1] - _nav["window_pos"][1])
    # Only a ball right next to us counts as our progress, not one rolling around elsewhere.
    ball, old_ball = _ballIfClose(pos), _nav["window_ball"]
    ball_moved = math.hypot(ball[0] - old_ball[0], ball[1] - old_ball[1]) if ball and old_ball else 0.0
    _resetStuckWindow(t, pos)
    if moved >= STUCK_DIST or ball_moved >= STUCK_DIST:
        return False

    _planEscape(robot, t, pos, tackle)
    return escapeIfStuck(robot, tackle)

def _countRepeat(t, pos):
    """Track jams in the same place; returns how much bigger the escape should be."""
    repeat = (t - _nav["stuck_t"] < REPEAT_TIME and
              math.hypot(pos[0] - _nav["stuck_pos"][0], pos[1] - _nav["stuck_pos"][1]) < REPEAT_DIST)
    if repeat:
        _nav["repeats"] += 1
        if _nav["repeats"] % 3 == 0:
            _nav["side"] = -_nav["side"]   # this side isn't working, try the other one
    else:
        _nav["repeats"] = 0
    _nav["stuck_t"] = t
    _nav["stuck_pos"] = pos
    return min(1.0 + 0.5 * _nav["repeats"], 2.5)

def _planEscape(robot, t, pos, tackle):
    """Step to one side, then back off, and for strikers hit the ball from that side.

    The sidestep comes first: when two robots meet head-on, a wheel (which sticks out past
    the body) can catch on the other robot's skirt, and backing straight out is exactly
    the direction that catch blocks. Without the ball each robot steps to its own left, so
    two robots facing each other shear apart in opposite directions instead of sliding
    along together. A tackling striker picks its side and timing at random; both teams run
    the same code, so otherwise two strikers would mirror each other into the same jam.
    """
    heading = robot.get_heading()
    hx, hy = math.cos(heading), math.sin(heading)   # the way the robot faces, on the field
    scale = _countRepeat(t, pos)
    ball = _ballIfClose(pos) if tackle else None

    if ball:
        fx, fy = ball[0] - pos[0], ball[1] - pos[1]
        norm = math.hypot(fx, fy) or 1.0
        fx, fy = fx / norm, fy / norm
        _nav["side"] = random.choice((-1, 1))
    else:
        fx, fy = hx, hy
        if not _nav["repeats"]:
            _nav["side"] = 1
    if abs(pos[1]) > 0.45:
        _nav["side"] = 1 if fx * -pos[1] >= 0 else -1   # near a side wall, step towards the middle

    sx, sy = -fy * _nav["side"], fx * _nav["side"]   # perpendicular to the way we were going
    step = t + 0.35 * scale
    back = step + random.uniform(0.2, 0.4) * scale
    _nav["plan"] = [(step, sx, sy, False), (back, -hx, -hy, False)]
    if ball:
        _nav["plan"].append((back + 0.5, fx - 0.8 * sx, fy - 0.8 * sy, False))

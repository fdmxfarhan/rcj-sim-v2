import math
from utils import *

Goaler_x = 0.9

# Ball inside this box in front of our goal is the goalie's to clear.
BOX_X = -0.55
BOX_Y = 0.6
# While our striker is off the field the goalie plays as a defender.
DEFENDER_X = -0.7        # guard position, a step out of the goal
DEFENDER_ZONE_X = 0.0    # clears any ball in our own half
LEAVE_MARGIN = 0.15      # ball must get this far past the zone before the goalie goes back

_clearing = False

def loop(rcj):
    global _clearing
    ball_pos = rcj.get_ball_position()
    robot_pos = rcj.get_position()
    ball_in_kicker = rcj.is_ball_touched()

    rememberBall(rcj, ball_pos)
    if escapeIfStuck(rcj):
        return

    defending = not teammateOnField(rcj)
    home_x = DEFENDER_X if defending else -Goaler_x
    ball = lastBallPosition(rcj)

    if ball:
        zone_x = DEFENDER_ZONE_X if defending else BOX_X
        zone_y = 0.8 if defending else BOX_Y
        if _clearing:
            zone_x += LEAVE_MARGIN
        in_zone = ball[0] < zone_x and abs(ball[1]) < zone_y
        behind_us = ball[0] < robot_pos[0] - 0.02
        # Past the goal line beside the goal: can't be reached without touching the wall.
        unreachable = ball[0] < -1.05 and abs(ball[1]) > 0.25
        _clearing = (in_zone or behind_us) and not unreachable
    else:
        _clearing = False

    if ball_in_kicker:
        rcj.set_dribbler(False)
        if not rcj.kick():
            clearBall(rcj, ball)
    elif _clearing:
        clearBall(rcj, ball)
    else:
        # Guard the goal: stay between the ball and the goal, lined up with the ball.
        target_y = max(-0.4, min(ball[1], 0.4)) if ball else 0
        goTo(rcj, home_x, target_y, 20)

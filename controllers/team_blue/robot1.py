import math
from utils import *

# Patrol points used to look for the ball when it hasn't been seen for a while.
SEARCH_POINTS = [(-0.5, 0.45), (0.6, 0.45), (0.6, -0.45), (-0.5, -0.45)]

def loop(rcj):
    ball_pos = rcj.get_ball_position()
    robot_heading = rcj.get_heading()
    robot_pos = rcj.get_position()
    ball_in_kicker = rcj.is_ball_touched()

    rememberBall(rcj, ball_pos)
    if escapeIfStuck(rcj, tackle=True):
        return

    if ball_in_kicker:
        target_angle = angleBetween(rcj, [1.1, 0], robot_pos)
        if abs(target_angle) < 4:
            rcj.set_dribbler(False)
            if not rcj.kick():
                # Kick already used on this touch: carry the ball forward instead of standing still,
                # or near the end wall back off so the ball leaves the kicker and the kick re-arms.
                if robot_pos[0] < 0.85:
                    moveXY(rcj, 0, 15, 0)
                else:
                    moveXY(rcj, 0, -10, 0)
        else:
            rcj.set_dribbler(True)
            moveXY(rcj, 0, 0, -max(-10, min(10, target_angle)))
    elif ball_pos:
        ball_field = lastBallPosition(rcj)
        if ball_field[0] < -0.65 and abs(ball_field[1]) < 0.5:
            # Ball is in front of our own goal: leave it to the goalie instead of crashing into it.
            goTo(rcj, -0.5, max(-0.4, min(ball_field[1], 0.4)))
        elif robot_pos[0] > 1 or robot_pos[0] < -1:
            moveTo(rcj, -0.5, 0, 0)
        elif robot_pos[1] > 0.7 or robot_pos[1] < -0.7:
            moveTo(rcj, -0.5, 0, 0)
        else:
            ball_angle = getAngle(rcj, ball_pos)
            shift = max(-60, min(ball_angle * 1.5, 60))
            moveAngle(rcj, ball_angle + shift, 20, robot_heading)
            # moveXY(rcj, -ball_pos[0]*500, ball_pos[1]*500 , -ball_angle*2)
    else:
        searchBall(rcj, SEARCH_POINTS)

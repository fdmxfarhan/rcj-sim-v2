import math
from utils import *

def run_robot(rcj):
    # استراتژی روبات شماره ۱ (مثلاً مهاجم)
    ball_pos = rcj.get_ball_position()
    robot_heading = rcj.get_heading()
    robot_pos = rcj.get_position()
    ball_in_kicker = rcj.is_ball_touched()

    if ball_in_kicker:
        ball_angle = angleBetween(rcj, [1.1, 0], robot_pos)
        if abs(ball_angle) < 4:
            rcj.set_dribbler(False)
            rcj.kick()
        else:
            rcj.set_dribbler(True)
            moveXY(rcj, 5, 0, -ball_angle/2)
    elif ball_pos:
        ball_angle = angleBetween(rcj, ball_pos, robot_pos)
        shift = max(-60, min(ball_angle * 1.5, 60))
        if ball_pos[1] > 0.8:
            moveTo(rcj, max(-1, min(ball_pos[0], 1)), 0.4, 0)
        elif ball_pos[1] < -0.8:
            moveTo(rcj, max(-1, min(ball_pos[0], 1)), -0.4, 0)
        elif ball_pos[0] > 1.06:
            moveTo(rcj, 0.8, max(-0.8, min(ball_pos[1], 0.8)), 0)
        elif ball_pos[0] < -1.06:
            moveTo(rcj, -0.8, max(-0.8, min(ball_pos[1], 0.8)), 0)
        else:
            moveAngle(rcj, ball_angle + shift, 20, robot_heading)
    else:
        rcj.motor(0, 0, 0, 0)

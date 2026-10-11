import math
from utils import *

def loop(rcj):
    ball_pos = rcj.get_ball_position()
    robot_heading = rcj.get_heading()
    robot_pos = rcj.get_position()
    ball_in_kicker = rcj.is_ball_touched()

    if ball_in_kicker:
        target_angle = angleBetween(rcj, [1.1, 0], robot_pos)
        if abs(target_angle) < 4:
            rcj.set_dribbler(False)
            rcj.kick()
        else:
            rcj.set_dribbler(True)
            moveXY(rcj, 0, 0, -max(-10, min(10, target_angle)))
    elif ball_pos:
        ball_angle = getAngle(rcj, ball_pos)
        shift = max(-60, min(ball_angle * 1.5, 60))
        # moveAngle(rcj, ball_angle + shift, 20, robot_heading)
        moveXY(rcj, -ball_pos[0]*200, ball_pos[1]*200 , -ball_angle)
    else:
        moveTo(rcj, -0.5, 0, 0)

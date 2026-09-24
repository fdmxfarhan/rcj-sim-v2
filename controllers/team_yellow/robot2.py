import math
from utils import *

Goaler_x = 0.9

def run_robot(rcj):
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
        if ball_pos[1] > 0.4:
            moveTo(rcj, -Goaler_x, 0.4)
        elif ball_pos[1] < -0.4:
            moveTo(rcj, -Goaler_x, -0.4)
        else:
            moveTo(rcj, -Goaler_x, ball_pos[1])
    else:
        moveTo(rcj, -Goaler_x, 0)

import math
from utils import *

def run_robot(rcj):
    # استراتژی روبات شماره ۱ (مثلاً مهاجم)
    ball_pos = rcj.get_ball_position()
    robot_heading = rcj.get_heading()
    robot_pos = rcj.get_position()

    if ball_pos:
        dx = ball_pos[0] - robot_pos[0]
        dy = ball_pos[1] - robot_pos[1]
        abs_angle = math.atan2(dy, dx)
        relative_angle_rad = abs_angle - robot_heading
        relative_angle_deg = math.degrees(relative_angle_rad)
        shift = max(-60, min(relative_angle_deg * 0.8, 60))
        moveAngle(rcj, relative_angle_deg + shift, 20, robot_heading)
    else:
        rcj.motor(0, 0, 0, 0)

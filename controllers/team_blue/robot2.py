import math
from utils import *

def run_robot(rcj):
    # استراتژی روبات شماره ۱ (مثلاً مهاجم)
    ball_pos = rcj.get_ball_position()
    robot_heading = rcj.get_heading()
    robot_pos = rcj.get_position()

    rcj.motor(10, -10, -10, 10)

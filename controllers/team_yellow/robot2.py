import math
from utils import *


def run_robot(rcj):
    # استراتژی روبات شماره ۲ (مثلاً دروازه‌بان)
    my_pos = rcj.get_position()
    # همیشه در یک ایکس ثابت بمان و فقط وای را با توپ تنظیم کن
    ball_pos = rcj.get_ball_position()
    
    if ball_pos:
        moveTo(rcj, -0.7, ball_pos[1])
    else:
        moveTo(rcj, -0.7, 0)

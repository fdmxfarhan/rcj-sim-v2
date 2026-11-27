import math


def loop(rcj):
    ball = rcj.get_ball_position()       # Ball position relative to this robot
    heading = rcj.get_heading()           # Compass: robot direction in radians
    position = rcj.get_position()         # GPS: robot position on the field
    ball_touched = rcj.is_ball_touched()  # True when the ball is in the kicker

    if ball_touched:
        rcj.kick()
    elif ball:
        # 0 degrees is forward; positive angles are to the left.
        angle = math.degrees(math.atan2(ball[0], ball[1]))

        if -22.5 <= angle < 22.5:      # Forward
            rcj.motor(10, 10, -10, -10)
        elif 22.5 <= angle < 67.5:     # Forward-left
            rcj.motor(0, 20, 0, -20)
        elif 67.5 <= angle < 112.5:    # Left
            rcj.motor(-10, 10, 10, -10)
        elif 112.5 <= angle < 157.5:   # Backward-left
            rcj.motor(-20, 0, 20, 0)
        elif angle >= 157.5 or angle < -157.5:  # Backward
            rcj.motor(-10, -10, 10, 10)
        elif -157.5 <= angle < -112.5:  # Backward-right
            rcj.motor(0, -20, 0, 20)
        elif -112.5 <= angle < -67.5:   # Right
            rcj.motor(10, -10, -10, 10)
        elif -67.5 <= angle < -22.5:    # Forward-right
            rcj.motor(20, 0, -20, 0)
    else:
        rcj.motor(0, 0, 0, 0)

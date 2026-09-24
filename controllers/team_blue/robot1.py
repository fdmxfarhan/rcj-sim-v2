def run_robot(rcj):
    # استراتژی روبات شماره ۱ (مثلاً مهاجم)
    ball_pos = rcj.get_ball_position()
    
    if ball_pos:
        # اگر توپ را دید به سمتش برو
        rcj.moveTo(-0.3, -0.5)
    else:
        # اگر ندید بایست
        rcj.motor(0, 0, 0, 0)

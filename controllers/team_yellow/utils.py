import math

def moveXY(robot, vx, vy, w=0):
    v1 = vx + vy + w
    v2 = -vx + vy + w
    v3 = -vx - vy + w
    v4 = vx - vy + w
    robot.motor(v1, v2, v3, v4)

def moveAngle(robot, angle_deg, speed, heading):
    rad = math.radians(angle_deg)
    vx = -speed * math.sin(rad)
    vy = speed * math.cos(rad)
    moveXY(robot, vx, vy, heading*20)

def moveTo(robot, target_x, target_y, target_w=0):
    pos = robot.get_position()
    heading = robot.get_heading()
    
    err_x = (pos[0] - target_x) * 50
    err_y = (pos[1] - target_y) * 50
    err_w = (heading - target_w) * 20
    moveXY(robot, err_y, -err_x, err_w)

def angleBetween(robot, pos1, pos2):
    heading = robot.get_heading()
    dx = pos1[0] - pos2[0]
    dy = pos1[1] - pos2[1]
    abs_angle = math.atan2(dy, dx)
    relative_angle_rad = abs_angle - heading
    relative_angle_deg = math.degrees(relative_angle_rad)
    if relative_angle_deg > 180: relative_angle_deg -= 360
    if relative_angle_deg < -180: relative_angle_deg += 360
    return relative_angle_deg
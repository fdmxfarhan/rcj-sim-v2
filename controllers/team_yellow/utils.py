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

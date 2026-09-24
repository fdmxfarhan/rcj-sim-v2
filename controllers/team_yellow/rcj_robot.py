import math
import struct
from controller import Robot

class RCJRobot:
    def __init__(self, robot):
        self.robot = robot
        self.time_step = int(robot.getBasicTimeStep())
        
        # شناسایی نام ربات
        self.name = robot.getName()

        # راه‌اندازی موتورها
        self.wheel_names = ["wheel1_45_motor", "wheel2_135_motor", "wheel3_225_motor", "wheel4_315_motor"]
        self.motors = []
        for name in self.wheel_names:
            m = robot.getDevice(name)
            if m is not None:
                m.setPosition(float("inf"))
                m.setVelocity(0.0)
            self.motors.append(m)

        # راه‌اندازی امن سنسورها (اگر روی ربات نبود، None برمی‌گرداند تا کرش نکند)
        self.gps = robot.getDevice("gps")
        if self.gps:
            self.gps.enable(self.time_step)
        
        self.compass = robot.getDevice("compass")
        if self.compass:
            self.compass.enable(self.time_step)
        
        self.receiver = robot.getDevice("receiver")
        if self.receiver:
            self.receiver.enable(self.time_step)
        
        self.kicker_touch = robot.getDevice("touch_kicker")
        if self.kicker_touch:
            self.kicker_touch.enable(self.time_step)

    def get_position(self):
        if self.gps:
            vals = self.gps.getValues()
            return [vals[0], vals[1]] # x, y
        return [0.0, 0.0]

    def get_heading(self):
        if self.compass:
            c = self.compass.getValues()
            rad = math.atan2(c[0], c[1])
            return math.degrees(rad)
        return 0.0

    def get_ball_position(self):
        if self.receiver and self.receiver.getQueueLength() > 0:
            packet = self.receiver.getBytes()
            ball_data = struct.unpack("ddd", packet)
            self.receiver.nextPacket()
            return [ball_data[0], ball_data[1]]
        return None

    def is_ball_touched(self):
        if self.kicker_touch:
            return self.kicker_touch.getValue() > 0
        return False

    def motor(self, v1, v2, v3, v4):
        speeds = [v1, v2, v3, v4]
        for i in range(4):
            if self.motors[i] is not None:
                val = max(-20, min(speeds[i], 20))
                if i % 2 == 1: 
                    val = -val 
                self.motors[i].setVelocity(val)

    def moveXY(self, vx, vy, w=0):
        v1 = vx + vy + w
        v2 = -vx + vy + w
        v3 = -vx - vy + w
        v4 = vx - vy + w
        self.motor(v1, v2, v3, v4)

    def moveAngle(self, angle_deg, speed):
        rad = math.radians(angle_deg)
        vx = -speed * math.sin(rad)
        vy = speed * math.cos(rad)
        self.moveXY(vx, vy, 0)

    def moveTo(self, target_x, target_y, target_w=0):
        pos = self.get_position()
        heading = math.radians(self.get_heading())
        
        err_x = (pos[0] - target_x) * 50
        err_y = (pos[1] - target_y) * 50
        err_w = (heading - target_w) * 20
        self.moveXY(err_y, -err_x, err_w)

    def step(self):
        return self.robot.step(self.time_step) != -1

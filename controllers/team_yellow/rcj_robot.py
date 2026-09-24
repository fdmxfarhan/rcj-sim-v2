import math
import struct
from controller import Robot

class RCJRobot:
    def __init__(self, robot):
        self.robot = robot
        self.time_step = int(robot.getBasicTimeStep())
        
        # شناسایی نام ربات و تیم
        self.name = robot.getName()
        
        # تشخیص خودکار تیم بر اساس نام ربات (یا می‌توانید دستی مقداردهی کنید)
        # اگر نام ربات با 'B' شروع شود یعنی تیم آبی است و باید مختصات را معکوس کند
        self.is_blue_team = self.name.startswith("Robot_B")

        # راه‌اندازی موتورها
        self.wheel_names = ["wheel1_45_motor", "wheel2_135_motor", "wheel3_225_motor", "wheel4_315_motor"]
        self.motors = []
        for name in self.wheel_names:
            m = robot.getDevice(name)
            if m is not None:
                m.setPosition(float("inf"))
                m.setVelocity(0.0)
            self.motors.append(m)

        # راه‌اندازی امن سنسورها
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
            x, y = vals[0], vals[1]
            # اگر تیم آبی است، مختصات زمین را ۱۸۰ درجه برعکس می‌کنیم (-1 در X و Y)
            if self.is_blue_team:
                return [-x, -y]
            return [x, y]
        return [0.0, 0.0]

    def get_heading(self):
        if self.compass:
            c = self.compass.getValues()
            rad = math.atan2(c[0], c[1])
            heading = math.degrees(rad)
            # برای تیم آبی زاویه سر ربات نیز ۱۸۰ درجه معکوس می‌شود
            if self.is_blue_team:
                heading += 180
                if heading > 180:
                    heading -= 360
            return math.radians(heading)
        return 0.0

    def get_ball_position(self):
        if self.receiver and self.receiver.getQueueLength() > 0:
            packet = self.receiver.getBytes()
            ball_data = struct.unpack("ddd", packet)
            self.receiver.nextPacket()
            bx, by = ball_data[0], ball_data[1]
            # موقعیت توپ هم برای تیم آبی باید قرینه شود تا در دیدگاه آن‌ها درست محاسبه شود
            if self.is_blue_team:
                return [-bx, -by]
            return [bx, by]
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

    def step(self):
        return self.robot.step(self.time_step) != -1

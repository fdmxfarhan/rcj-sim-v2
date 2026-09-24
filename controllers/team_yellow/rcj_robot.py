import math
import struct
from controller import Supervisor, Robot

class RCJRobot:
    def __init__(self, robot):
        self.robot = robot
        self.time_step = int(robot.getBasicTimeStep())
        
        # شناسایی نام ربات و تیم
        self.name = robot.getName()
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

        # کش برای نگه‌داشتن آخرین موقعیت توپ
        self.last_ball_pos = None

        # دسترسی به نود توپ برای شوت زدن و دریبلر مجازی
        self.ball_node = None
        if hasattr(self.robot, "getFromDef"):
            self.ball_node = self.robot.getFromDef("BALL")

        # متغیر وضعیت برای جلوگیری از شلیک متوالی در یک برخورد
        self.has_kicked = False

        # متغیر وضعیت دریبلر مجازی
        self.dribbler_active = False

    def set_dribbler(self, state: bool):
        """روشن یا خاموش کردن دریبلر مجازی"""
        self.dribbler_active = state

    def apply_dribbler(self, force=1.0):
        """
        اعمال نیرو به توپ برای کشیدن آن دقیقاً به سمت مرکز ربات
        هنگامی که دریبلر روشن است و توپ در محوطه کیکر قرار دارد.
        """
        if self.dribbler_active and self.ball_node is not None:
            if self.is_ball_in_kicker():
                robot_pos = self.get_position()
                ball_pos = self.get_ball_position()

                if robot_pos is None or ball_pos is None:
                    returnf

                # بردار از مرکز توپ به سمت مرکز ربات
                dx = robot_pos[0] - ball_pos[0]
                dy = robot_pos[1] - ball_pos[1]
                distance = math.hypot(dx, dy)

                if distance > 0.001:
                    # بردار واحد به سمت مرکز ربات ضرب‌در شدت نیرو
                    fx = (dx / distance) * force
                    fy = (dy / distance) * force

                    # تبدیل بردار به سیستم مختصات جهانی Webots (Global) برای تیم آبی
                    if self.is_blue_team:
                        fx = -fx
                        fy = -fy

                    # اعمال نیرو به مرکز توپ
                    self.ball_node.addForce([fx, fy, 0.0], False)

    def apply_dribbler_pd(self, hold_dist=0.075, kp=100.0, kd=10.0):
        """
        نگه‌داشتن پایدار توپ با فیدبک موقعیت و سرعت
        - hold_dist: فاصله نقطه هدف توپ از مرکز ربات (متر)
        """
        if not (self.dribbler_active and self.ball_node and self.is_ball_in_kicker()):
            return

        # زاویه فیزیکی واقعی ربات در جهان
        raw_heading = self.get_raw_heading()
        
        # موقعیت جهانی واقعی ربات
        if self.gps:
            rx, ry = self.gps.getValues()[0], self.gps.getValues()[1]
        else:
            return

        # نقطه هدف قرارگیری توپ در جلوی ربات (در فضای سراسری Webots)
        target_x = rx + math.cos(raw_heading) * hold_dist
        target_y = ry + math.sin(raw_heading) * hold_dist

        # موقعیت و سرعت واقعی توپ
        ball_pos = self.ball_node.getPosition()
        ball_vel = self.ball_node.getVelocity()  # [vx, vy, vz, wx, wy, wz]

        # خطای موقعیت
        err_x = target_x - ball_pos[0]
        err_y = target_y - ball_pos[1]

        # نیروی فنر + دمپر برای جلوگیری از لرزش و پرتاب
        fx = kp * err_x - kd * ball_vel[0]
        fy = kp * err_y - kd * ball_vel[1]

        self.ball_node.addForce([fx, fy, 0.0], False)

    def get_position(self):
        if self.gps:
            vals = self.gps.getValues()
            x, y = vals[0], vals[1]
            if self.is_blue_team:
                return [-x, -y]
            return [x, y]
        return [0.0, 0.0]

    def get_raw_heading(self):
        """زاویه واقعی ربات در دستگاه مختصات سراسری زمین (Global) برحسب رادیان"""
        if self.compass:
            c = self.compass.getValues()
            return math.atan2(c[0], c[1])
        return 0.0

    def get_heading(self):
        if self.compass:
            rad = self.get_raw_heading()
            heading = math.degrees(rad)
            if self.is_blue_team:
                heading += 180
                if heading > 180:
                    heading -= 360
            return math.radians(heading)
        return 0.0

    def get_ball_position(self):
        # خواندن همه پکت‌های صف و نگه داشتن تازه‌ترین داده
        if self.receiver and self.receiver.getQueueLength() > 0:
            while self.receiver.getQueueLength() > 0:
                packet = self.receiver.getBytes()
                ball_data = struct.unpack("ddd", packet)
                self.receiver.nextPacket()
            
            bx, by = ball_data[0], ball_data[1]
            if self.is_blue_team:
                self.last_ball_pos = [-bx, -by]
            else:
                self.last_ball_pos = [bx, by]

        return self.last_ball_pos

    def is_ball_in_kicker(self, min_dist=0.06, max_dist=0.11, lateral_tolerance=0.045):
        """
        بررسی قرارگیری توپ در دهانه کیکر به جای سنسور تاچ
        - min_dist / max_dist: بازه فاصله طولی توپ نسبت به مرکز ربات به سمت جلو (متر)
        - lateral_tolerance: حداکثر انحراف مجاز به چپ یا راست از خط تقارن کیکر (متر)
        """
        robot_pos = self.get_position()
        ball_pos = self.get_ball_position()

        if ball_pos is None or robot_pos is None:
            return False

        # بردار فاصله از ربات به توپ در مختصات زمین تیم
        dx = ball_pos[0] - robot_pos[0]
        dy = ball_pos[1] - robot_pos[1]

        # تبدیل بردار به دستگاه مختصات محلی خود ربات
        heading = self.get_heading()
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)

        # forward: امتداد دید رو به جلوی ربات
        # lateral: انحراف عرضی نسبت به مرکز کیکر
        forward = dx * cos_h + dy * sin_h
        lateral = -dx * sin_h + dy * cos_h

        return (min_dist <= forward <= max_dist) and (abs(lateral) <= lateral_tolerance)

    # برای حفظ سازگاری با کدهای موجود شما (مثل robot1.py)
    def is_ball_touched(self):
        return self.is_ball_in_kicker()

    def kick(self):
        force = 20.0
        if not self.is_ball_in_kicker():
            self.has_kicked = False
            return False

        if self.has_kicked:
            return False

        if self.ball_node is not None:
            heading = self.get_heading()
            
            fx = math.cos(heading) * force
            fy = math.sin(heading) * force

            # اگر تیم آبی باشد، چون مختصات محلی ۱۸۰ درجه شیفت خورده، بردار اعمالی در Webots معکوس می‌شود
            if self.is_blue_team:
                fx = -fx
                fy = -fy
            
            self.ball_node.addForce([fx, fy, 0.0], False)
            
            self.has_kicked = True
            return True

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
        # بررسی موقعیت برای ریست کردن خودکار قفل شوت پس از دور شدن توپ
        if not self.is_ball_in_kicker():
            self.has_kicked = False

        # اعمال کشش دریبلر در صورت فعال بودن
        self.apply_dribbler_pd()

        return self.robot.step(self.time_step) != -1

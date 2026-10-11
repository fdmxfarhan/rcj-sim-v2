import math
import struct
from controller import Supervisor, Robot


ROBOT_DEFS = ["Y1", "Y2", "B1", "B2"]
ROBOT_BLOCK_RADIUS = 0.09
LINE_END_MARGIN = 0.05

class RCJRobot:
    def __init__(self, robot):
        self.robot = robot
        self.time_step = int(robot.getBasicTimeStep())
        
        self.name = robot.getName()
        self.is_blue_team = self.name.startswith("Robot_B")

        self.wheel_names = ["wheel1_45_motor", "wheel2_135_motor", "wheel3_225_motor", "wheel4_315_motor"]
        self.motors = []
        for name in self.wheel_names:
            m = robot.getDevice(name)
            if m is not None:
                m.setPosition(float("inf"))
                m.setVelocity(0.0)
            self.motors.append(m)

        self.gps = robot.getDevice("gps")
        if self.gps:
            self.gps.enable(self.time_step)
        
        self.compass = robot.getDevice("compass")
        if self.compass:
            self.compass.enable(self.time_step)
        
        self.receiver = robot.getDevice("receiver")
        if self.receiver:
            self.receiver.enable(self.time_step)

        self.last_ball_pos = None

        self.ball_node = None
        if hasattr(self.robot, "getFromDef"):
            self.ball_node = self.robot.getFromDef("BALL")

        self.has_kicked = False

        self.dribbler_active = False
    def set_dribbler(self, state: bool):
        self.dribbler_active = state
    def apply_dribbler(self, force=1.0):
        if self.dribbler_active and self.ball_node is not None:
            if self.is_ball_in_kicker():
                robot_pos = self.get_position()
                ball_pos = self.get_ball_position()

                if robot_pos is None or ball_pos is None:
                    return

                dx = robot_pos[0] - ball_pos[0]
                dy = robot_pos[1] - ball_pos[1]
                distance = math.hypot(dx, dy)

                if distance > 0.001:
                    fx = (dx / distance) * force
                    fy = (dy / distance) * force

                    if self.is_blue_team:
                        fx = -fx
                        fy = -fy

                    self.ball_node.addForce([fx, fy, 0.0], False)
    def apply_dribbler_pd(self, hold_dist=0.075, kp=150.0, kd=20.0):
        if not (self.dribbler_active and self.ball_node and self.is_ball_in_kicker()):
            return

        raw_heading = self.get_raw_heading()
        
        if self.gps:
            rx, ry = self.gps.getValues()[0], self.gps.getValues()[1]
        else:
            return

        target_x = rx + math.cos(raw_heading) * hold_dist
        target_y = ry + math.sin(raw_heading) * hold_dist

        ball_pos = self.ball_node.getPosition()
        ball_vel = self.ball_node.getVelocity()  # [vx, vy, vz, wx, wy, wz]

        err_x = target_x - ball_pos[0]
        err_y = target_y - ball_pos[1]

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
    def is_ball_blocked(self):
        """Return True if another robot blocks the ball's line of sight."""

        if not isinstance(self.robot, Supervisor):
            return False

        ball_node = self.robot.getFromDef("BALL")
        receiver_node = self.robot.getSelf()

        if ball_node is None or receiver_node is None:
            return False

        ball_pos = ball_node.getPosition()
        receiver_pos = receiver_node.getPosition()

        # Use world X/Y coordinates.
        bx, by = ball_pos[0], ball_pos[1]
        rx, ry = receiver_pos[0], receiver_pos[1]

        dx = rx - bx
        dy = ry - by
        length_sq = dx * dx + dy * dy

        if length_sq < 1e-8:
            return False

        length = math.sqrt(length_sq)

        for def_name in ROBOT_DEFS:
            # The receiving robot cannot block its own signal.
            if def_name == self.name:
                continue

            node = self.robot.getFromDef(def_name)
            if node is None:
                continue

            obstacle_pos = node.getPosition()
            ox, oy = obstacle_pos[0], obstacle_pos[1]

            # Project the obstacle position onto the ball-to-receiver line.
            t = ((ox - bx) * dx + (oy - by) * dy) / length_sq

            # Ignore obstacles outside the segment and near its endpoints.
            margin_t = LINE_END_MARGIN / length
            if t <= margin_t or t >= 1.0 - margin_t:
                continue

            # Find the closest point on the line segment.
            closest_x = bx + t * dx
            closest_y = by + t * dy

            distance = math.hypot(
                ox - closest_x,
                oy - closest_y
            )

            if distance <= ROBOT_BLOCK_RADIUS:
                return True

        return False
    def get_ball_position(self):
        if self.receiver and self.receiver.getQueueLength() > 0:
            while self.receiver.getQueueLength() > 0:
                packet = self.receiver.getBytes()
                ball_data = struct.unpack("ddd", packet)
                self.receiver.nextPacket()

            bx, by = ball_data[0], ball_data[1]

            if self.is_blue_team:
                bx = -bx
                by = -by

            robot_pos = self.get_position()
            heading = self.get_heading()

            dx = bx - robot_pos[0]
            dy = by - robot_pos[1]

            ball_angle = math.atan2(dy, dx)
            ball_dist = math.hypot(dx, dy)

            if ball_dist > 0.8:
                self.last_ball_pos = None
                return None

            # Reject the ball if another robot blocks the signal.
            if self.is_ball_blocked():
                self.last_ball_pos = None
                return None

            relative_angle_rad = ball_angle - heading

            global_bx = ball_dist * math.sin(relative_angle_rad)
            global_by = ball_dist * math.cos(relative_angle_rad)

            self.last_ball_pos = [global_bx, global_by]

        # Recheck the line of sight even when no new packet arrives.
        if self.last_ball_pos is not None and self.is_ball_blocked():
            self.last_ball_pos = None

        return self.last_ball_pos
    def is_ball_in_kicker(self, min_dist=0.06, max_dist=0.11, lateral_tolerance=0.045):
        ball_pos = self.get_ball_position()

        if ball_pos is None:
            return False

        dx = ball_pos[0]
        dy = ball_pos[1]


        return (min_dist <= dy <= max_dist) and (abs(dx) <= lateral_tolerance)
    def is_ball_touched(self):
        return self.is_ball_in_kicker()
    def kick(self):
        force = 100.0
        if not self.is_ball_in_kicker():
            self.has_kicked = False
            return False

        if self.has_kicked:
            return False

        if self.ball_node is not None:
            heading = self.get_heading()
            
            fx = math.cos(heading) * force
            fy = math.sin(heading) * force

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
        if not self.is_ball_in_kicker():
            self.has_kicked = False

        self.apply_dribbler_pd()

        return self.robot.step(self.time_step) != -1

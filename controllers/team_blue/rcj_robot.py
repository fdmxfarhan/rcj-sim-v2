import math
import struct

from controller import Supervisor


CONTROL_STEP_MS = 32
ROBOT_DEFS = ("Y1", "Y2", "B1", "B2")
ROBOT_BLOCK_RADIUS = 0.09
LINE_END_MARGIN = 0.05
KICK_FORCE = 100.0


class RCJRobot:
    def __init__(self, robot):
        self.robot = robot
        basic_time_step = int(robot.getBasicTimeStep())
        self.time_step = self._control_step_for(basic_time_step)

        self.name = robot.getName()
        self.is_blue_team = self.name.startswith("Robot_B")
        team = "B" if self.is_blue_team else "Y"
        number = self.name[-1] if self.name[-1:].isdigit() else "1"
        self.def_name = team + number
        self.teammate_def = team + ("2" if number == "1" else "1")

        self.wheel_names = [
            "wheel1_45_motor", "wheel2_135_motor",
            "wheel3_225_motor", "wheel4_315_motor",
        ]
        self.motors = []
        self.motor_speeds = [None] * len(self.wheel_names)
        for name in self.wheel_names:
            motor = robot.getDevice(name)
            if motor is not None:
                motor.setPosition(float("inf"))
                motor.setVelocity(0.0)
            self.motors.append(motor)

        self.gps = robot.getDevice("gps")
        if self.gps:
            self.gps.enable(self.time_step)
        self.compass = robot.getDevice("compass")
        if self.compass:
            self.compass.enable(self.time_step)
        self.receiver = robot.getDevice("receiver")
        if self.receiver:
            self.receiver.enable(self.time_step)

        self.is_supervisor = isinstance(robot, Supervisor)
        self.self_node = robot.getSelf() if self.is_supervisor else None
        self.ball_node = robot.getFromDef("BALL") if self.is_supervisor else None
        self.robot_nodes = {}
        if self.is_supervisor:
            self.robot_nodes = {
                def_name: robot.getFromDef(def_name) for def_name in ROBOT_DEFS
            }
        self.teammate_node = self.robot_nodes.get(self.teammate_def)

        self.last_ball_pos = None
        self.last_ball_packet = None
        self.ball_world_position = None
        self.teammate_world_position = None
        self.raw_position = [0.0, 0.0, 0.0]
        self.position = [0.0, 0.0]
        self.raw_heading = 0.0
        self.heading = 0.0
        self.ball_blocked = False
        self.ball_in_kicker = False
        self.has_kicked = False
        self.dribbler_active = False

    @staticmethod
    def _control_step_for(basic_time_step):
        steps_per_control = max(1, math.ceil(CONTROL_STEP_MS / basic_time_step))
        return basic_time_step * steps_per_control

    def _refresh_pose(self):
        if self.gps:
            values = self.gps.getValues()
            self.raw_position = [values[0], values[1], values[2]]
            self.position = [-values[0], -values[1]] if self.is_blue_team else [values[0], values[1]]
        if self.compass:
            compass = self.compass.getValues()
            self.raw_heading = math.atan2(compass[0], compass[1])
            heading_degrees = math.degrees(self.raw_heading)
            if self.is_blue_team:
                heading_degrees += 180
                if heading_degrees > 180:
                    heading_degrees -= 360
            self.heading = math.radians(heading_degrees)

    def _consume_ball_packets(self):
        if not self.receiver:
            return
        while self.receiver.getQueueLength() > 0:
            packet = self.receiver.getBytes()
            self.receiver.nextPacket()
            if len(packet) == struct.calcsize("ddd"):
                self.last_ball_packet = struct.unpack("ddd", packet)

    def _is_ball_blocked(self):
        if not self.is_supervisor or self.ball_world_position is None:
            return False
        bx, by = self.ball_world_position[0], self.ball_world_position[1]
        rx, ry = self.raw_position[0], self.raw_position[1]
        dx, dy = rx - bx, ry - by
        length_squared = dx * dx + dy * dy
        if length_squared < 1e-8:
            return False

        margin_t = LINE_END_MARGIN / math.sqrt(length_squared)
        for def_name, node in self.robot_nodes.items():
            if def_name == self.def_name or node is None:
                continue
            ox, oy = node.getPosition()[:2]
            t = ((ox - bx) * dx + (oy - by) * dy) / length_squared
            if t <= margin_t or t >= 1.0 - margin_t:
                continue
            closest_x, closest_y = bx + t * dx, by + t * dy
            if math.hypot(ox - closest_x, oy - closest_y) <= ROBOT_BLOCK_RADIUS:
                return True
        return False

    def _refresh_ball_state(self):
        self.ball_world_position = self.ball_node.getPosition() if self.ball_node else None
        self._consume_ball_packets()
        self.ball_blocked = self._is_ball_blocked()
        self.last_ball_pos = None
        if self.last_ball_packet is None or self.ball_blocked:
            return

        bx, by = self.last_ball_packet[:2]
        if self.is_blue_team:
            bx, by = -bx, -by
        dx, dy = bx - self.position[0], by - self.position[1]
        distance = math.hypot(dx, dy)
        if distance > 0.8:
            return
        relative_angle = math.atan2(dy, dx) - self.heading
        self.last_ball_pos = [
            distance * math.sin(relative_angle),
            distance * math.cos(relative_angle),
        ]

    def _refresh_snapshot(self):
        self._refresh_pose()
        self._refresh_ball_state()
        self.teammate_world_position = (
            self.teammate_node.getPosition() if self.teammate_node else None
        )
        if self.last_ball_pos is None:
            self.ball_in_kicker = False
            return
        dx, dy = self.last_ball_pos
        self.ball_in_kicker = 0.06 <= dy <= 0.11 and abs(dx) <= 0.045

    def set_dribbler(self, state: bool):
        self.dribbler_active = state

    def apply_dribbler(self, force=1.0):
        if not (
            self.dribbler_active and self.ball_node and self.ball_world_position
            and self.ball_in_kicker
        ):
            return
        dx = self.raw_position[0] - self.ball_world_position[0]
        dy = self.raw_position[1] - self.ball_world_position[1]
        distance = math.hypot(dx, dy)
        if distance > 0.001:
            self.ball_node.addForce([dx * force / distance, dy * force / distance, 0.0], False)

    def apply_dribbler_pd(self, hold_dist=0.075, kp=150.0, kd=20.0):
        if not (
            self.dribbler_active and self.ball_node and self.self_node
            and self.ball_world_position and self.ball_in_kicker
        ):
            return
        rx, ry = self.raw_position[:2]
        target_x = rx + math.cos(self.raw_heading) * hold_dist
        target_y = ry + math.sin(self.raw_heading) * hold_dist
        ball_velocity = self.ball_node.getVelocity()
        robot_velocity = self.self_node.getVelocity()
        target_vx = robot_velocity[0] - robot_velocity[5] * (target_y - ry)
        target_vy = robot_velocity[1] + robot_velocity[5] * (target_x - rx)
        force_x = kp * (target_x - self.ball_world_position[0]) + kd * (target_vx - ball_velocity[0])
        force_y = kp * (target_y - self.ball_world_position[1]) + kd * (target_vy - ball_velocity[1])
        self.ball_node.addForce([force_x, force_y, 0.0], False)

    def get_position(self):
        return self.position[:]

    def get_raw_heading(self):
        return self.raw_heading

    def get_heading(self):
        return self.heading

    def get_ball_position(self):
        return self.last_ball_pos[:] if self.last_ball_pos is not None else None

    def is_ball_blocked(self):
        return self.ball_blocked

    def is_ball_in_kicker(self, min_dist=0.06, max_dist=0.11, lateral_tolerance=0.045):
        if self.last_ball_pos is None:
            return False
        dx, dy = self.last_ball_pos
        return min_dist <= dy <= max_dist and abs(dx) <= lateral_tolerance

    def is_ball_touched(self):
        return self.ball_in_kicker

    def is_teammate_on_field(self):
        if self.teammate_world_position is None:
            return True
        x, y = self.teammate_world_position[:2]
        return abs(x) < 1.25 and abs(y) < 0.95

    def kick(self):
        if not self.ball_in_kicker:
            self.has_kicked = False
            return False
        if self.has_kicked or self.ball_node is None:
            return False
        force_x = math.cos(self.heading) * KICK_FORCE
        force_y = math.sin(self.heading) * KICK_FORCE
        if self.is_blue_team:
            force_x, force_y = -force_x, -force_y
        self.ball_node.addForce([force_x, force_y, 0.0], False)
        self.has_kicked = True
        return True

    def motor(self, v1, v2, v3, v4):
        for index, speed in enumerate((v1, v2, v3, v4)):
            motor = self.motors[index]
            if motor is None:
                continue
            velocity = max(-20, min(speed, 20))
            if index % 2 == 1:
                velocity = -velocity
            if velocity != self.motor_speeds[index]:
                motor.setVelocity(velocity)
                self.motor_speeds[index] = velocity

    def step(self):
        if not self.ball_in_kicker:
            self.has_kicked = False
        self.apply_dribbler_pd()
        if self.robot.step(self.time_step) == -1:
            return False
        self._refresh_snapshot()
        return True

import math
import struct
from controller import Robot

# ---- physical constants ----
WHEEL_RADIUS = 0.03   # m
MOUNT_RADIUS = 0.08   # m

WHEEL_ANGLES_DEG = [45, 135, 225, 315]
WHEEL_MOTOR_NAMES = [
    "wheel1_45_motor",
    "wheel2_135_motor",
    "wheel3_225_motor",
    "wheel4_315_motor",
]

TIME_STEP = 32


class OmniDrive:
    def __init__(self, robot):
        self.motors = []
        for name in WHEEL_MOTOR_NAMES:
            motor = robot.getDevice(name)
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)
            self.heading = 0
            self.motors.append(motor)

    def setHeading(self, heading):
        self.heading = heading

    def motor(self, v1, v2, v3, v4):
        v1 += self.heading * 0.1
        v2 += self.heading * 0.1
        v3 += self.heading * 0.1
        v4 += self.heading * 0.1
        v1 = max(-20, min(v1, 20))
        v2 = max(-20, min(v2, 20))
        v3 = max(-20, min(v3, 20))
        v4 = max(-20, min(v4, 20))
        self.motors[0].setVelocity(v1)
        self.motors[1].setVelocity(-v2)
        self.motors[2].setVelocity(v3)
        self.motors[3].setVelocity(-v4)

    def move(self, Vx, Vy, w):
        Vl1 = Vx + Vy + w
        Vl2 = -Vx + Vy + w
        Vr2 = -Vx - Vy + w
        Vr1 = Vx - Vy + w
        self.motor(Vl1, Vl2, Vr2, Vr1)

    def moveAngle(self, a, v):
        x = -v * math.sin(math.radians(a))
        y = v * math.cos(math.radians(a))
        self.move(x, y, 0)

def main():
    robot = Robot()
    drive = OmniDrive(robot)

    # ---- Initialize Sensors ----
    gps = robot.getDevice("gps")
    gps.enable(TIME_STEP)

    compass = robot.getDevice("compass")
    compass.enable(TIME_STEP)

    receiver = robot.getDevice("receiver")
    receiver.enable(TIME_STEP)

    while robot.step(TIME_STEP) != -1:
        # Move forward slowly
        
        
        # 1. Get our own position and heading
        self_pos = gps.getValues()
        self_x, self_y = self_pos[0], self_pos[1]
        
        compass_val = compass.getValues()
        # Compute heading angle: 0 rad is pointing along +X (forward) in Webots coordinate frame
        self_heading = math.atan2(-compass_val[0], -compass_val[1])
        drive.setHeading(math.degrees(self_heading))
        # print(math.degrees(self_heading))
        # 2. Check if a packet from the ball has arrived
        ball_detected = False
        while receiver.getQueueLength() > 0:
            packet = receiver.getBytes()
            # Unpack the 3 coordinates sent by the ball
            ball_x, ball_y, ball_z = struct.unpack("ddd", packet)
            ball_detected = True
            receiver.nextPacket()  # Go to next packet if any
            
        if ball_detected:
            # 3. Calculate distance
            dx = ball_x - self_x
            dy = ball_y - self_y
            distance = math.sqrt(dx**2 + dy**2)
            
            # 4. Calculate absolute angle to ball, then make it relative to our heading
            abs_angle = math.atan2(-dy, -dx)
            relative_angle_rad = abs_angle - self_heading
            
            # Normalize angle to range [-pi, pi]
            relative_angle_rad = (relative_angle_rad + math.pi) % (2 * math.pi) - math.pi
            relative_angle_deg = math.degrees(relative_angle_rad)
            
            shift = max(-60, min(relative_angle_deg * 0.8, 60))
            drive.moveAngle(relative_angle_deg + shift, 20)
            # print(f"Ball detected! Distance: {distance:.2f} m | Angle: {relative_angle_deg:.1f}°")
        else:
            # print("No signal from ball.")
            drive.motor(0, 0, 0, 0)
        # drive.moveAngle(45, 10)

if __name__ == "__main__":
    main()

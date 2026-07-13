import struct
from controller import Robot

TIME_STEP = 32

def main():
    robot = Robot()
    
    # Initialize devices
    gps = robot.getDevice("gps")
    gps.enable(TIME_STEP)
    
    emitter = robot.getDevice("emitter")
    
    while robot.step(TIME_STEP) != -1:
        # Get absolute coordinates [X, Y, Z]
        x, y, z = gps.getValues()
        
        # Pack coordinates as 3 doubles (24 bytes)
        message = struct.pack("ddd", x, y, z)
        
        # Broadcast the data
        emitter.send(message)

if __name__ == "__main__":
    main()

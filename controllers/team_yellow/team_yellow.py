from controller import Robot
from rcj_robot import RCJRobot
import robot1
import robot2

def main():
    webot_robot = Robot()
    rcj = RCJRobot(webot_robot)
    
    # تشخیص اینکه این کنترلر روی کدام روبات در حال اجراست
    # فرض بر این است که نام روبات‌ها در وباتس Robot_Y1 و Robot_Y2 است
    robot_name = rcj.name
    
    while rcj.step():
        if "1" in robot_name:
            robot1.run_robot(rcj)
        elif "2" in robot_name:
            robot2.run_robot(rcj)

if __name__ == "__main__":
    main()

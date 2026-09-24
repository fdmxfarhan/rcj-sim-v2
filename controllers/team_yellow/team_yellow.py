from controller import Supervisor  # به جای Robot
from rcj_robot import RCJRobot
import robot1
import robot2

def main():
    webot_robot = Supervisor()  # ایجاد آبجکت Supervisor
    rcj = RCJRobot(webot_robot)
    
    robot_name = rcj.name
    
    while rcj.step():
        if "1" in robot_name:
            robot1.run_robot(rcj)
        elif "2" in robot_name:
            robot2.run_robot(rcj)

if __name__ == "__main__":
    main()

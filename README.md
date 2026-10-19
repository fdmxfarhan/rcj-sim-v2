<p align="right" dir="rtl">
  برای مشاهده <a href="README-fa.md">نسخه ترجمه شده به فارسی</a> کلیک کنید.
</p>


<p align="right">
  <a href="README-fa.md">
    <img src="https://img.shields.io/badge/🇮🇷%20Read%20in%20Persian-238636?style=for-the-badge&labelColor=238636" alt="Read in Persian">
  </a>
</p>

---
<br>

# Advanced Soccer Robot Simulator

A soccer simulation featuring four robots, built with Webots. This project includes a custom field and ball, two teams (blue and yellow), Python-based robot controllers, and a game supervisor for tracking scores and managing gameplay.

> This project uses **Webots R2025a**. Make sure you use a compatible version of Webots to open and run the world.

<br>

![Webots simulation world](docs/pic/world.png)

<br>

## Features

- Two teams, yellow and blue, each consisting of two robots.
- Yellow team robots: `Y1`, `Y2`
- Blue team robots: `B1`, `B2`
- Four-wheeled robot models with an omnidirectional drive system.
- Each robot is equipped with GPS, a compass, and a ball sensor.
- Wall-contact penalty system that temporarily moves the offending robot outside the field.

<br>

## Project Structure

The simulation project has the following file structure:

<div dir="ltr">

```text
.
├── controllers/             # Robot controllers
│   ├── ball_controller/     # Ball controller code
│   ├── game_supervisor/     # Game supervisor code
│   ├── team_blue/           # Blue team controller code
│   └── team_yellow/         # Yellow team controller code
├── protos/
└── worlds/
    ├── world.wbt            # Webots simulation world
    └── assets/
```

</div>

<br>

## Required Software

| Software | Description | Download |
|---|---|---|
| 🐍 Python | Python programming language. | [Download](https://www.python.org/downloads/) |
| 🤖 Webots | Robot simulation software. | [Download](https://cyberbotics.com/) |
| 💻 Visual Studio Code | Code editor. | [Download](https://code.visualstudio.com/download) |
| 📦 RCJ sim v2 | Webots soccer simulation project. | [Download](https://github.com/fdmxfarhan/rcj-sim-v2/archive/refs/heads/main.zip) |

<br>

## Running the Simulation

1. After installing the required software and downloading the `rcj-sim-v2` project, extract the downloaded ZIP file.
2. Open `worlds/world.wbt` in Webots.

<p align="center" style="background-color: #212121">
    <img src="./docs/pic/world.gif" alt="Webots simulation world">
</p>

## Robot Controllers

Each team has a controller directory (`team_yellow` or `team_blue`) containing five Python files:

- `rcj_robot.py`
- `team_blue.py`
- `robot1.py`
- `robot2.py`
- `utils.py`

**Important:** Do not modify `rcj_robot.py` or `team_blue.py`. In the simulation competition, only the following files are submitted for evaluation:

- `robot1.py`
- `robot2.py`
- `utils.py`

Each robot has its own Python file, either `robot1.py` or `robot2.py`, containing its algorithms and control commands. Commands, functions, and variables shared by both robots on the same team are defined in `utils.py`.

For example, consider the controller for Robot 1:

<div dir="ltr">

```python
from utils import *

def loop(rcj):
    ball_pos = rcj.get_ball_position()
    robot_heading = rcj.get_heading()
    robot_pos = rcj.get_position()
    ball_in_kicker = rcj.is_ball_touched()

    if ball_in_kicker:
        rcj.kick()
    if ball_pos:
        ball_angle = getAngle(rcj, ball_pos)
        moveAngle(rcj, ball_angle, 5, robot_heading)
    else:
        rcj.motor(0, 0, 0, 0)
```

</div>

> In this program, all functions defined in `utils.py` are imported into Robot 1's controller. The following sections explain the commands and sensors used in this program.

<br>

### 1. The `loop` Function

The `loop` function is repeatedly called in an infinite loop after the game starts. It executes the robot's gameplay algorithm.

<div dir="ltr">

```python
def loop(rcj):
```

</div>

### 2. Retrieving the Ball Position

The following function retrieves the ball's position **relative to the robot** and stores it in a variable. It returns a two-element array `[x, y]` in metres, where `y` points straight ahead of the robot and `x` points to the robot's left. It returns `None` when the ball is more than 0.8 m away or another robot blocks the line of sight.

<div dir="ltr">

```python
ball_pos = rcj.get_ball_position()
```

</div>

To get the ball's direction in degrees (0 is straight ahead, positive is to the left), use `getAngle` from `utils.py`:

<div dir="ltr">

```python
ball_angle = getAngle(rcj, ball_pos)
```

</div>

### 3. Compass Sensor

The compass sensor provides the robot's heading angle in radians.

<div dir="ltr">

```python
robot_heading = rcj.get_heading()
```

</div>

### 4. GPS Sensor

The following function retrieves the robot's position on the field and stores it in a variable. It returns a two-element array in the format `[x, y]`.

<div dir="ltr">

```python
robot_pos = rcj.get_position()
```

</div>

The field's coordinate system is shown below. The center of the field is the origin of the coordinate system. Coordinates are mirrored for the blue team, so both teams attack toward +X.

![Field coordinate axes](docs/pic/axis.png)

### 5. Ball Contact Sensor

This sensor detects whether the ball is inside the robot's front opening. The ball must be in this position for the robot to kick the ball or use its dribbler.

<div dir="ltr">

```python
ball_in_kicker = rcj.is_ball_touched()
```

</div>

## Omnidirectional Drive System

The soccer robots use an **omnidirectional drive system**, which allows them to move in any direction without first turning to face that direction.

Each robot uses four omni wheels. The speed of each motor can be controlled using the following function:

`rcj.motor(ML1, ML2, MR2, MR1)`

![Omnidirectional drive system](docs/pic/omni.jpg)

### Moving Forward

To move the robot forward, use the following commands:

<div dir="ltr">

```python
from utils import *

def loop(rcj):
    rcj.motor(20, 20, -20, -20)
```

</div>

> **Note:** The maximum motor speed is `20`.

<p align="center" style="background-color: #212121">
    <img src="./docs/pic/forward.gif" alt="Robot moving forward">
</p>

### Moving Right

To move the robot to the right, use the following commands:

<div dir="ltr">

```python
from utils import *

def loop(rcj):
    rcj.motor(10, -10, -10, 10)
```

</div>

<p align="center" style="background-color: #212121">
    <img src="./docs/pic/right.gif" alt="Robot moving right">
</p>

### Moving Diagonally

To move the robot diagonally, use the following commands:

<div dir="ltr">

```python
from utils import *

def loop(rcj):
    rcj.motor(10, 0, -10, 0)
```

</div>

<p align="center" style="background-color: #212121">
    <img src="./docs/pic/orib.gif" alt="Robot moving diagonally">
</p>

### Rotating in Place

To rotate the robot around its own center, use the following commands:

<div dir="ltr">

```python
from utils import *

def loop(rcj):
    rcj.motor(5, 5, 5, 5)
```

</div>

<p align="center" style="background-color: #212121">
    <img src="./docs/pic/turn.gif" alt="Robot rotating in place">
</p>
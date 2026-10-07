# RCJ Soccer Simulation v2

A four-robot RoboCup Junior soccer simulation built with **Webots**. The project includes a custom field and ball, two robot teams (blue and yellow), Python robot controllers, and a match supervisor for scoring and match flow.

> The world file uses the Webots **R2025a** format. Use a compatible Webots release to open and run it.

## Features

- Two teams with two robots each: `Y1`, `Y2`, `B1`, and `B2`.
- Four-wheel omni-drive robot models with GPS, compass, and radio receiver devices.
- Ball-position broadcasts from the ball controller to the robot controllers.
- Separate role logic for robot 1 and robot 2 on each team.
- Match scoreboard and a ten-minute countdown.
- Goal detection, kickoff repositioning, and a short pause after a goal.
- Ball recovery when it leaves the playable area, falls below the field, or remains stationary.
- Wall-touch penalties that temporarily move a robot outside the field.

## Repository layout

```text
.
├── controllers/
│   ├── ball_controller/       # Broadcasts ball GPS coordinates
│   ├── game_supervisor/       # Match clock, score, kickoff, and game rules
│   ├── team_blue/             # Blue team entry point, robot roles, and helpers
│   └── team_yellow/           # Yellow team entry point, robot roles, and helpers
├── protos/                    # Four robot PROTO definitions and mesh assets
└── worlds/
    ├── world.wbt              # Webots world and match setup
    └── assets/                # Field and ball textures
```

## Requirements

- **Webots R2025a** (or a compatible newer release that can load the R2025a world format).
- A Webots installation with its Python controller runtime available.
- Internet access the first time the world is opened if Webots needs to retrieve the background PROTOs referenced from the Webots R2025a GitHub release.

There is no separate Python dependency or package manifest in this repository. The controllers use Python's standard library and Webots' `controller` API.

## Run the simulation

1. Clone or download this repository.
2. Open `worlds/world.wbt` in Webots. You can use **File → Open World…** or, from the repository root, launch it with:

   ```bash
   webots worlds/world.wbt
   ```

3. Start or reset the simulation in Webots. The world assigns the controllers to the ball, teams, and match supervisor automatically.

If Webots reports that it cannot locate a controller, confirm that the repository's `controllers/` directory is at the project root and that the world is opened from this checkout. If an external PROTO cannot be loaded, check network access and that the Webots release matches the URL version in `worlds/world.wbt`.

## How it works

### World and teams

`worlds/world.wbt` defines the field, goals, ball, team robots, and a separate supervisor robot. The four team robots are instantiated from `protos/Robot_*.proto`. Their PROTOs name the controllers `team_blue` or `team_yellow`; the team controller dispatches to `robot1.py` or `robot2.py` according to the robot's name.

The blue and yellow controller directories have parallel APIs and role files. Their shared `rcj_robot.py` wrapper handles sensing, team-relative coordinates, wheel commands, ball possession, dribbling, and kicking. `utils.py` provides movement and aiming helpers.

### Ball tracking and kicking

`controllers/ball_controller/ball_controller.py` reads the ball's GPS position and broadcasts three double-precision values (`x`, `y`, and `z`) through a Webots emitter. Each robot receives the latest packet through its receiver. The `RCJRobot` wrapper exposes the ball position in team-relative coordinates.

Ball possession is estimated geometrically: the ball must be in front of the robot within the kicker's distance and lateral bounds. The controller wrapper applies the kick force directly to the ball node and prevents repeated kicks while the ball remains in the kicker area. The dribbler is implemented as a controller-side force, not as a separate physical motor.

### Match supervisor

`controllers/game_supervisor/game_supervisor.py` runs the match logic. The current settings include:

- A **10-minute** match clock; the simulation pauses when time expires.
- Goal detection at either end of the field, within the configured goal-mouth range.
- Scoreboard updates and a two-second goal pause before play resumes.
- Ball repositioning after an out-of-bounds event, a fall below the field, or approximately three seconds without meaningful movement.
- A **30-second** wall-touch penalty, after which a penalized robot returns to an available neutral position.

The supervisor uses the scene-tree `DEF` names `BALL`, `Y1`, `Y2`, `B1`, and `B2`. Keep these names in sync if editing the world or supervisor.

## Controller development

### Editing a team strategy

Each team entry point delegates behavior based on the robot's name:

- `robot1.py` implements the first robot's strategy.
- `robot2.py` implements the second robot's strategy.

To change a strategy, edit the corresponding file in `controllers/team_blue/` or `controllers/team_yellow/`. Keep the `run_robot(rcj)` entry-point signature, or update the team's dispatcher if you intentionally change it.

The shared robot wrapper exposes these commonly used methods:

| Method | Purpose |
| --- | --- |
| `get_position()` | Robot position in the team's coordinate frame |
| `get_heading()` | Robot heading in radians in the team's coordinate frame |
| `get_ball_position()` | Most recently received ball position, or `None` before the first packet |
| `is_ball_in_kicker()` / `is_ball_touched()` | Checks whether the ball is within the kicker region |
| `motor(v1, v2, v3, v4)` | Sets the four wheel speeds (clamped by the wrapper) |
| `kick()` | Kicks when the ball is within the kicker region |
| `set_dribbler(state)` | Enables or disables the controller-side dribbler |

`utils.py` also defines `moveXY`, `moveAngle`, `moveTo`, and `angleBetween` for movement and relative-angle calculations.

### Changing the world or match rules

- Edit `worlds/world.wbt` to change field geometry, textures, initial placements, or scene-tree assignments.
- Edit the robot definitions under `protos/` to change robot geometry, sensors, devices, or default controllers.
- Edit `controllers/game_supervisor/game_supervisor.py` to change match timing, scoring bounds, kickoff positions, ball recovery, or penalties.
- Edit `controllers/ball_controller/ball_controller.py` and the robot receiver code together if changing the ball broadcast format.

When changing controller device names, update both the PROTO device definitions and the corresponding Python lookups.

## Checks

The controller files can be checked for Python syntax from the repository root with:

```bash
python3 -m compileall -q controllers
```

A successful syntax check does not validate the Webots runtime; run the world in Webots to verify simulation behavior and device configuration.

## License and attribution

No license file is currently included. Until a license is added, reuse and redistribution rights are not explicitly granted. The world also references background PROTOs hosted in the [Cyberbotics Webots repository](https://github.com/cyberbotics/webots) at the R2025a release.

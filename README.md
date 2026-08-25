# Automated Chemical Synthesis with a 7-DOF Robotic Arm

Diploma thesis — Electrical and Computer Engineering, University of Patras.
Division of Systems, Control and Robotics.

A robotic system that automates liquid-handling in a chemistry lab: it locates
test tubes visually, picks them up, carries them to a mixing vessel and pours,
keeping the tube upright throughout so nothing spills.

The system does not work from taught positions. Tube and vessel locations
come from ArUco markers detected at runtime, so the rack can be moved between
runs and the arm adapts.

---

## The problem

Three problems that a simple pick-and-place demo does not have:

1. The tube must stay upright.
2. Positions are not known in advance.
3. A 7-DOF arm is redundant.

---

## Hardware

| | |
| --- | --- |
| Arm | Elephant Robotics MyArm 300 Pi (7 DOF) |
| Gripper | Custom, Thorgripper |
| Camera | Creative Livecam HD |
| Markers | ArUco, 30 mm |
| Compute | Raspberry Pi on the arm + external PC for planning |

---

## How it works

```
camera ──► ArUco detection ──► TF frames (marker_1 … marker_6)
                                     │
                                     ▼
                       planning scene (tubes + mixing vessel)
                                     │
                                     ▼
                           MoveIt 2 ──► trajectory
                                     │
                                     ▼
                                MyArm 300 Pi
```

**Vision.** ArUco markers on the rack and the mixing vessel are detected and
published as TF frames.
Camera-to-arm calibration in aruco/calibrate_camera.py .

**Planning.** Approaches are planned backwards: IK solution found at the
grasp pose first, and the standoff above it is seeded from that solution, so
both ends share an IK branch. Each candidate standoff is then verified by
dry-running the actual descent from it before the arm commits to moving there.

**Keeping the tube upright.** Rather than asking OMPL to respect an orientation
constraint in the full 7-DOF space, transits search the 5-DOF manifold on which
the constraint is already satisfied — joints 1–4 plus the roll about the
vertical, with the wrist recovered in closed form.

---

## Repository layout

| Path | What it is |
| --- | --- |
| [`aruco/aruco_anchor.py`](aruco) | Marker detection, TF publishing |
| [`aruco/calibrate_camera.py/`](camera_calibration) | Intrinsics and camera-to-arm calibration |
| [`robot_initialisation/`](MyArm%20300%20Pi) | Arm driver and hardware bridge |
| [`robot_initialisation/`](robot_initialisation) | Startup and homing |
| [`launch/`](launch) | Launch files — fake robot, Gazebo, real arm |
| [`config/`](config) | Kinematics, joint limits, OMPL settings |
| [`GUI/`](GUI) | Tkinter operator interface |
| [`3d_prints/`](3d_prints) | Custom bases and flange (STL files) |
| `simple_move.cpp` | Motion node |

---

## Running

Built and tested on **Ubuntu 22.04 / ROS 2 Humble**, with MoveIt 2 and
MoveIt Task Constructor built from source.

Three modes: fake robot (no hardware), Gazebo, and the real
arm.

### Commands

The GUI publishes on `/gui_commands`
| Command | Action |
| --- | --- |
| `TASK <1-5>` | Full pipeline for each tube |
| `m<0-6>` | Manual move to a marker |
| `o` / `c` | Open / close gripper |
| `p` | Pour |
| `h` | Return home |

## Related

The final packaged ROS 2 workspace is in
[**thesis-repo**](https://github.com/vkaratrantos/thesis-repo)

## Status

Thesis defence scheduled for September 2026. The system runs on the physical
arm; known limitations are documented in the workspace README.

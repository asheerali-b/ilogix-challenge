# iLogix Innovation Challenges

This repository contains three Python programs developed for the iLogix innovation challenges. Each program uses a camera and a robot arm to perform a vision-guided task.

## Challenges

- **Cap color sorting** (`cap_color_sort.py`): Detects blue and green caps and sorts them into their corresponding boxes. HSV color ranges are configurable, with optional YOLO-based detection.
- **Cube rotation** (`cube_rotation.py`): Detects ArUco-marked cubes and flips the leftmost visible cube so the face that was toward the robot base is on top.
- **Cube stacking** (`cube_stack.py`): Detects ArUco-marked cubes and stacks them at a left or right target location. It supports automatic operation and an optional command mode.

## Requirements

These programs are intended to run in a compatible OLO robot runtime. They need access to a robot arm, gripper, and the camera topics configured in each script. The cube challenges use ArUco markers and require camera calibration data. The scripts also use OpenCV and NumPy; YOLO detection in `cap_color_sort.py` additionally requires Ultralytics and a configured model.

Install the Python dependencies in the environment provided for your robot setup. Robot runtime packages and camera configuration are environment-specific and are not included here.

## Running

Run the desired script from the configured robot environment, for example:

```powershell
python .\cap_color_sort.py
python .\cube_rotation.py
python .\cube_stack.py
```

The scripts start their routines when executed and do not provide a shared command-line interface. Review each script's configuration constants before running it on a robot. In particular, `cube_stack.py` defaults to automatic operation and can continue indefinitely; stop it with `Ctrl+C`.
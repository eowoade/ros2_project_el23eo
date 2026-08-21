# Autonomous TurtleBot Navigation & Colour-Reactive Control (ROS2/Nav2)

A ROS2 package that moves a TurtleBot through a mapped environment using Nav2 waypoint navigation and uses a live camera feed to watch for a specific coloured target. On detecting it, the robot cancels its current navigation goal, switches to closed-loop visual servoing to approach and centre the target, then stops at a safe distance.


## What this demonstrates

- **Hybrid control architecture** - goal-directed autonomous navigation (Nav2 `NavigateToPose` action client) with a reactive visual-servoing override that interrupts navigation when needed, rather than treating the two as separate modes bolted together.
- **Classical computer vision** - HSV colour-space thresholding and contour detection (OpenCV) to identify and track coloured objects in real time from a live ROS `Image` topic.
- **Concurrent ROS2 design** - `rclpy.spin()` run on a background thread so the main control loop can poll robot state and camera detections independently of incoming callbacks.
- **SLAM-based mapping** - the environment (`map/map.pgm`, `map/map.yaml`) was mapped beforehand and is used by Nav2 for localisation during navigation.

## How it works

`project_code.py` runs a single `Robot` node that:

1. Subscribes to `/camera/image_raw`, converts each frame with `cv_bridge`, and thresholds it in HSV space to detect red, green, and blue regions via contour area.
2. Normally, sends the robot through three predefined waypoints in sequence using Nav2's `NavigateToPose` action - waiting for goal acceptance, then goal completion, before moving to the next.
3. If blue has been seen within the last 2 seconds, it **cancels the active Nav2 goal** and switches to proportional visual servoing: it computes the horizontal offset of the blue blob's centroid from the image centre and turns to correct it, driving forward until the blob's contour area passes a threshold (i.e. it's close enough), then stops.
4. Once stopped at the target, it holds position.

## Project structure

This package was built up across four lab exercises, each adding a capability, before being combined into the final project:

| File | What it adds |
|---|---|
| `first_step.py` | HSV colour filtering - isolate a single colour in the camera feed |
| `second_step.py` | Multi-colour detection - filter and mask several colours at once |
| `third_step.py` | Contour-based detection with a publish/subscribe event (`chatter` topic) on detection |
| `fourth_step.py` | Closed-loop behaviour - follow one colour, emergency-stop on sight of another |
| `project_code.py` | **Final project** - combines the above with Nav2 waypoint navigation into the hybrid controller described above |


## Built with

ROS2 | Nav2 | OpenCV | `cv_bridge` | NumPy | Python

## Running it

```bash
# from your ROS2 workspace src/ directory
git clone https://github.com/eowoade/ros2_project_el23eo.git
cd ..
colcon build --packages-select ros2_project_el23eo
source install/setup.bash

# have open TurtleBot simulation/hardware + Nav2 stack with the saved map first, then:
ros2 run ros2_project_el23eo project_code
```
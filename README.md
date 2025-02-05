# spatial-robot

This project contains a MATLAB script for simulating and controlling a 2-DOF planar robot interactively. The robot can be moved by clicking on the screen, and the script will display the robot's position, speed, and acceleration.

## Getting Started

### Prerequisites

- MATLAB R2021a or later

### Installation

1. Clone the repository to your local machine:
    ```sh
    git clone https://github.com/yourusername/spatial-robot.git
    cd spatial-robot
    ```

2. Open MATLAB and navigate to the cloned repository directory.

### Usage

1. Add the `spatial_v2` folder and its subfolders to the MATLAB path:
    ```matlab
    addpath(genpath('spatial_v2'));
    ```

2. Open the `interactive_robot.m` script in MATLAB.

3. Run the script by clicking the "Run" button or typing `interactive_robot` in the MATLAB command window.

4. A figure window will open displaying the robot. Click anywhere on the screen to move the robot's end-effector to the clicked position.

5. The robot will move to the new position, and the speed and acceleration of each joint will be displayed on the screen.

### Files

- `interactive_robot.m`: Main script for running the interactive robot simulation.
- `Planar2DOFRobot.m`: Class definition for the 2-DOF planar robot.
- `README.md`: This file.

### Example

```matlab
% Add the spatial_v2 folder and its subfolders to the MATLAB path
addpath(genpath('spatial_v2'));

% Run the interactive robot script
interactive_robot
```

### Visualization

![Robot Visualization](https://i.imgur.com/Zx3nvhw.png)
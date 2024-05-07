# hand-eye & robot calibration
This project is a hand-eye robot calibration algorithm that aims to implement the calibration of a robot's hand-eye coordination using the POE Adjoint and EMDH models. The algorithm is designed for the UR5 robot and developed using Matlab 2021.

# Installation
Before using this project, make sure you have the following software and tools installed:

- Matlab 2021
- Matlab Robotics Toolbox 2017

# Usage
Run "main_calibration_POE.m" and "main_calibration_EMDH.m" in main directory

# Notes
- The data in the 'data' folder is generated through simulation. You can adjust the kinematic parameters and hand-eye parameters inside to adapt to different robots and scenarios. You can also adjust the noise within to modify the measurement noise of the camera.
- The camera only needs to measure the position.
- Rotation angles should be consistently represented in degrees or radians. Please check the relevant functions in the robotics toolbox.
- For detailed explanations of POE and EMDH, please refer to the referenced literature.

# Reference
- C. Li, Y. Wu, H. Loewe, and Z. Li, “POE-Based Robot Kinematic Calibration Using Axis Configuration Space and the Adjoint Error Model,” Ieee Transactions on Robotics, vol. 32, no. 5, pp. 1264-1279, Oct, 2016.
- M. Dehghani, R. A. McKenzie, R. A. Irani, and M. Ahmadi, “Robot-mounted sensing and local calibration for high-accuracy manufacturing,” Robotics and Computer-Integrated Manufacturing, vol. 79, 2023.



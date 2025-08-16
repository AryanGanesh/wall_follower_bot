# wall_follower_bot

Wall Follower Bot (ROS 2 Humble – TurtleBot3 Simulation)










This project implements a Wall Follower robot in ROS 2 Humble using the TurtleBot3 platform. The robot moves forward, detects obstacles, and performs precise 90° turns by integrating odometry feedback with timer-based approximations.

The project was first tested in the TurtleBot World simulation and then extended to the more challenging Café World, where careful tuning led to smoother and more reliable navigation.

![WhatsApp Image 2025-08-16 at 5 54 39 PM](https://github.com/user-attachments/assets/ecce6159-ee1f-4d3e-bdda-3b63dd295f3f)

✨ Features

Custom Python ROS 2 node for autonomous wall following

Odometry + Timer Approximation for precise 90° turns

Laser scan tuning (wider angles & thresholds) for earlier obstacle detection

Real-time sensor data logging for debugging and feedback

Tested in TurtleBot World and Café World simulations

📂 Repository Structure
wall_follower_bot/
├── launch/                 # Launch files for simulation
├── src/                    # Source code (Python node)
│   └── wall_follower.py
├── worlds/                 # Gazebo world files (TurtleBot & Café)
├── CMakeLists.txt
├── package.xml
└── README.md

📊 Demo

TurtleBot World (Initial Testing): Early trials to test obstacle detection and turning.

![Testing Phase](https://github.com/user-attachments/assets/b573c6bb-7fe9-4613-b2d0-7d59873c0ce8)

Café World (Refined Simulation): Improved performance with tuned thresholds and scan coverage.

![Refined Simulation](https://github.com/user-attachments/assets/c04d0bd4-0b5d-4865-ab07-b09ab4ffc1c6)

🧠 Learnings

Importance of sensor fusion in autonomous navigation

How iteration and tuning improve robustness and adaptability

Practical application of ROS 2 fundamentals: message subscriptions, velocity command publishing, and odometry integration

🔮 Future Work

Integrate IMU for accurate turnings and distance sensor for accurate distance

Extend to real TurtleBot3 hardware testing

Add PID control for smoother wall-following behavior

Incorporate mapping and SLAM for enhanced autonomy

# Pegasus Robot Arm with Gesture Teleoperation
![alt text](/images/image.png)
This is a ROS 2 system for observing and controlling a Pegasus-based robot arm using hand gesture teleoperation. This project integrates computer vision through Google mediapipe and a Kinect V2 camera to track hand movements and map them to robot arm joint trajectories in real-time.

## 🚀 Features

*   **Gesture Teleoperation**: Control the robot arm using natural hand movements and gestures.
*   **Computer Vision Stack**: Uses `handcv` with MediaPipe to track hand landmarks in 3D space.
*   **Depth Camera Support**: capabilities for Microsoft Kinect v2.
*   **Hybrid Control**: Supports both GUI-based control and handtracking mode for gesture control.
*   **Simulation & Real Hardware**: Includes an Arduino bridge simulator for testing without the physical arm.
*   **Path Planning**: Integrated Inverse Kinematics (IK) for calculating joint angles from cartesian coordinates.

## 📦 Project Structure

```text
├── src/
│   ├── pegasus_control/          # Core control logic and IK solvers
│   ├── handcv/                   # Computer vision hand tracking (MediaPipe)
│   ├── cv_pegasus_bridge/        # Bridges CV data to robot commands
│   ├── gripper_control/          # End-effector control
│   ├── update_pegasus_description/ # URDF models and robot description
│   ├── kinect2_bridge/           # Kinect v2 hardware interface
│   └── ...
├── scripts/
│   ├── run_full_stack.sh         # Main entry point to launch the system
│   ├── arduino_bridge_simulator.sh # Simulates the microcontroller interface
│   └── ...
└── build/ & install/             # Colcon build artifacts
```

## 🛠️ Prerequisites

*   **OS**: Linux (Ubuntu 20.04/22.04 recommended)
*   **ROS 2**: (Humble or Foxy)
*   **Python**: 3.8+
*   **Hardware**: 
    *   Pegasus-based Robot Arm
      ![WhatsApp Image 2026-02-16 at 12 52 47](https://github.com/user-attachments/assets/09bd02b3-5a66-427e-bae4-f0d338b38c4d)
      
    *   Depth Camera (Microsoft Kinect v2)
      ![kinect](https://github.com/user-attachments/assets/baa6efb7-2c3a-42d5-b462-c2a1ff67cb66)


### Dependencies

Install the required Python libraries:

```bash
pip3 install opencv-python mediapipe numpy scipy pyyaml pillow
sudo apt-get install python3-tk
```

Ensure standard ROS 2 packages are installed:

```bash
sudo apt install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-tf2-ros
```

## 🔨 Installation

1.  **Clone the repository**:
    ```bash
    cd ~/ros2_ws/src
    git clone <repo_url>
    cd ..
    ```

2.  **Build the workspace**:
    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```

## 🤖 Architecture

1.  **Vision Layer**: The
 `kinect2_bridge` package captures depth and color images through Kinect v2 camera, and sends them to `handcv` package to processes them with MediaPipe to find hand landmarks, and publishes the hand's 3D position.
<img width="893" height="425" alt="Screenshot from 2026-02-16 12-54-53" src="https://github.com/user-attachments/assets/1697d095-15cc-4ab0-bbc7-fa603b4e5428" />
3.  **Bridge Layer**: `cv_pegasus_bridge` subscribes to hand coordinates and translates them into a coordinate frame relative to the robot base.
4.  **Control Layer**: `pegasus_control` receives target coordinates, performs Inverse Kinematics (IK), and sends joint angle commands to the hardware interface.
5.  **Hardware Layer**: ST3215 Servo Driver integrated with ESP32 receives joint angles and drives the physical servos motors.
![WhatsApp Image 2026-02-16 at 12 57 39](https://github.com/user-attachments/assets/bac34f31-440d-46e1-b909-d27c6255978f)
![WhatsApp Image 2026-02-16 at 12 56 53](https://github.com/user-attachments/assets/4af149b3-0b45-41fa-8240-355105b96f62)




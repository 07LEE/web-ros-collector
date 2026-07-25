# Camera Bridge

A ROS2 pipeline project that utilizes a smartphone's web browser as a camera sensor to stream video frames to a PC. It captures camera feeds via standard web APIs, compresses them into 60% quality JPEG images, and publishes them as ROS2 `/image_raw/compressed` topics over a local HTTPS connection without requiring any mobile app installation.

## Prerequisites

- OS: Ubuntu 24.04 LTS
- ROS2: Jazzy Jalisco (Desktop version recommended)
- Network: Both the PC and the smartphone must be connected to the same Wi-Fi network.

## Installation & Build

1. Install ROS2 Jazzy (run only if ROS2 is not installed):

   ```bash
   chmod +x scripts/install_ros2.sh
   ./scripts/install_ros2.sh
   source ~/.bashrc
   ```

2. Build the Package:

   ```bash
   colcon build
   ```

## Running the Package

1. Launch the Camera Bridge Node:

   ```bash
   source install/setup.bash
   ros2 run camera_bridge camera_bridge_node
   ```

2. Stream from Smartphone:
   - Open a web browser on your smartphone and navigate to the address displayed in the execution log (e.g., `https://<PC_IP>:8443`).
   - Bypass the SSL certificate warning page by clicking 'Advanced' -> 'Proceed to website (unsafe)'.
   - Accept the camera permission request, and tap 'Start Streaming'.

3. Verify Data Reception:
   Open a new terminal window and run:

   ```bash
   source install/setup.bash
   ros2 topic hz /image_raw/compressed
   ```

## File Structure

- package.xml: Package metadata, including license, maintainer details, and ROS 2 dependencies (rclpy, sensor_msgs).
- setup.py: Build and installation configuration script for the Python package, defining executable entry points and installing resource files.
- setup.cfg: Installation script path configuration.
- resource/camera_bridge: Empty marker file used by the ament index to register the package and allow ROS 2 to locate it.
- web/index.html: Client-side web interface served to the smartphone browser to capture camera frames and upload them via HTTPS POST.
- camera_bridge/camera_bridge_node.py: Main ROS 2 executable node. It hosts the HTTPS server, receives JPEG frame data from the mobile client, and publishes sensor_msgs/msg/CompressedImage messages.

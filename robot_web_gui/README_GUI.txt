================================================================================
TURTLEBOT3 LIVE SLAM & AUTONOMOUS WEB GUI — TEAM SETUP & OPERATION GUIDE
================================================================================

1. OVERVIEW
--------------------------------------------------------------------------------
This folder contains a browser-based ROS 2 control and mapping interface for the
TurtleBot3 Burger:
  - index.html : Dashboard layout, controls, and HTML5 map/LiDAR canvas
  - app.js     : ROSBridge WebSocket client, live SLAM renderer, Auto-Explore
                 frontier scanner, .yaml/.pgm map exporter/loader, and A*
                 path planner with LiDAR obstacle avoidance

Features included:
  - Live WebSocket connection indicator with 3-second sensor watchdog
  - Real-time 3S LiPo battery voltage, percentage, and estimated runtime
  - Standby ACTIVE / OFF toggle (immobilizes motors via /motor_power & /cmd_vel)
  - Manual Keyboard Teleop (Arrow keys / WASD) + Autonomous Auto-Explore Scan
  - One-click Save Scan Map (.yaml + .pgm) and Upload Map (.yaml + .pgm)
  - Interactive Robot Pose calibration (/initialpose) and Waypoint Autopilot
  - Spacebar Emergency Stop (halts autonomous motion immediately)


2. ONE-TIME SETUP ON YOUR PERSONAL PC (TO RUN AS LOCALHOST)
--------------------------------------------------------------------------------
Prerequisites: Ubuntu 24.04, ROS 2 Jazzy, and the TurtleBot3 Zenoh client setup.

Step A — Install rosbridge_suite (if not already installed):
  sudo apt update
  sudo apt install -y ros-jazzy-rosbridge-suite ros-jazzy-rmw-zenoh-cpp

Step B — Copy the GUI folder into your ROS 2 workspace:
  mkdir -p ~/turtlebot3_ws/src/robot_web_gui
  cp index.html app.js ~/turtlebot3_ws/src/robot_web_gui/

Step C — Verify your ~/.bashrc has the Zenoh & Domain ID exports:
  Ensure these lines exist in ~/.bashrc (run `source ~/.bashrc` after editing):
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=30
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_SESSION_CONFIG_URI="$HOME/.config/zenoh/client.json5"

  (Your ~/.config/zenoh/client.json5 must contain:
   { mode: "client", connect: { endpoints: [ "tcp/turtlebot3.local:7447" ] } })

Step D — Allow ports 8000 and 9090 through firewall (only if UFW is active):
  sudo ufw allow 8000/tcp
  sudo ufw allow 9090/tcp


3. STEP-BY-STEP STARTUP PROCEDURE (HOSTING ON LOCALHOST)
--------------------------------------------------------------------------------
Note: Only ONE person should run slam_toolbox and rosbridge_server at a time
to avoid conflicting /map or /cmd_vel commands on the robot.

Step 1 — Connect to the Robot Wi-Fi Hotspot:
  - Turn on the TurtleBot3 (battery + OpenCR motor switch).
  - Connect your PC to the 2.4 GHz Wi-Fi hotspot:
      SSID     : TurtleBot
      Password : turtlebot
  - Disable any active VPN on your PC.

Step 2 — Verify Zenoh & Robot Communication (Terminal 1):
  - Check that the TurtleBot3 Zenoh router is reachable:
      nc -vz turtlebot3.local 7447
      (Expected: "Connection to turtlebot3.local (...) 7447 port [tcp/*] succeeded!")

  - Check that all 13 base robot topics are visible:
      ros2 topic list --no-daemon

  - Confirm all 13 topics appear:
      /battery_state, /cmd_vel, /imu, /joint_states, /magnetic_field, /odom,
      /parameter_events, /robot_description, /rosout, /scan, /sensor_state,
      /tf, /tf_static

  *(If topics do not appear, SSH into `ssh ubuntu@turtlebot3.local` and run:
    `export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_bringup robot.launch.py`)*

Step 3 — Launch Live SLAM & Verify /map Rate (Terminal 1 & Terminal 2):
  - In Terminal 1, start slam_toolbox for real-world hardware time:
      ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false

  - In Terminal 2, verify that /map is now being published:
      ros2 topic hz /map
      (Press Ctrl+C once you see the average rate printed.)

Step 4 — Start the ROS WebSocket Bridge (Terminal 2):
  - Launch rosbridge bound to all network interfaces (port 9090):
      ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0

Step 5 — Start the Local Web GUI Server (Terminal 3):
  - Check your PC's hotspot IP address (if sharing with teammates):
      hostname -I

  - Start the HTTP server locked strictly to the robot_web_gui folder:
      python3 -m http.server 8000 --bind 0.0.0.0 --directory ~/turtlebot3_ws/src/robot_web_gui

Step 6 — Open the Web GUI in Your Browser:
  - On your own PC (the host), open Chrome and go to:
      http://localhost:8000/

  - For teammates on the SAME "TurtleBot" hotspot (no local setup needed):
      Open Chrome and go to: http://<HOST_PC_IP>:8000/
      (Replace <HOST_PC_IP> with the first IP printed by `hostname -I` in Step 5.
       If Chrome blocks the local IP, click the tune/warning icon left of the
       URL -> Site settings -> set "Local network access" to Allow and reload.)


4. USING THE WEB GUI WORKFLOW
--------------------------------------------------------------------------------
A. Verify Connection & Message Type:
   - Confirm the top-left light is GREEN ("Connected") and battery voltage shows.
   - Keep "Msg Type" set to "TwistStamped (Physical / Gazebo Jazzy)".

B. Mapping the Maze (Manual or Auto-Explore):
   - Manual Scan: Click the webpage background and drive with Arrow keys or WASD.
   - Auto-Explore Scan: Click "Scan Mode: MANUAL" to toggle "AUTO-EXPLORE".
     The robot will autonomously explore frontiers until the maze is mapped.
   - Click "Save Scan Map", enter a map name, and save both .yaml and .pgm files.

C. Autonomous Navigation on a Saved Map:
   1. Click "Upload Map" and select BOTH your saved .yaml and .pgm files together.
   2. Click "Set Robot Pose", then click and drag on the map to match the
      robot's physical position and facing direction.
   3. Click open floor space on the map to place waypoints and click "Set Target"
      on your final destination.
   4. Click "Autopilot: OFF" to engage "Autopilot: ON" (press SPACEBAR anytime
      for an immediate Emergency Stop).
================================================================================

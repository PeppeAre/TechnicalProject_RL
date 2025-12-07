# Cooperative Search & Support System - ROS 2 & PX4

This project simulates a cooperative mission between a drone (PX4 Autopilot) and ground rovers (ROS 2) within a Gazebo simulation environment. The drone flies to target coordinates to scan an area, while ground rovers navigate to the target location avoiding obstacles. The system includes a fault tolerance mechanism with an emergency rover.

**Authors:** Nicholas Ruggiero, Giuseppe Arena

---

## 🐳 Docker Setup (Host Machine)

Before starting, ensure you are inside the project folder.

### Build the Docker image:

```bash
cd docker
./build.sh
```

### Run the Container:

```bash
./run.sh
```

### Join the Container (for new terminals)

To open additional terminals attached to the running container, use:

```bash
cd docker
./join.sh
```

---

## ⚙️ Environment Setup & Installation (Inside Docker)

Once inside the Docker container, execute the following steps to set up the workspace and dependencies.

### 1. Install Utilities and Python Dependencies

```bash
apt-get update && apt-get install -y mesa-utils
pip install "numpy<1.24"
```

Optional: Verify graphics support:

```bash
glxinfo -B
```

### 2. Clone Repositories

Inside the `workspace/src/` folder:

```bash
cd ~/workspace/src
git clone https://github.com/PX4/px4_msgs.git
git clone -b humble https://github.com/gazebosim/ros_gz.git
```

Inside the root (`/root/`) folder:

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

### 3. Install ROS Dependencies

From the workspace root:

```bash
cd ~/workspace
rosdep install -r --from-paths src -i -y --rosdistro humble
```

### 4. Setup Custom Drone Model

Copy the custom model SDF (with the project logo/physics) to the PX4 directory:

```bash
# Backup original model
cp ~/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf ~/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model_backup.sdf

# Remove the model.sdf
rm ~/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf

# Copy project model (assuming model.sdf is in workspace root)
cp ~/workspace/model.sdf ~/PX4-Autopilot/Tools/simulation/gz/models/x500_base/
```

### 5. Build the Workspace

```bash
cd ~/workspace
colcon build --symlink-install
```

---

## 🚀 Running the Simulation

Open separate terminals (using `./join.sh`) and run the following commands in this specific order:

**Terminal 1: Start PX4 SITL (Simulation)**

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

**Terminal 2: Spawn World and Rovers**

```bash
cd ~/workspace
source install/setup.bash
ros2 launch my_ground_robot spawn_rovers.launch.py
```

**Terminal 3: Start Micro XRCE-DDS Agent**  
(Required for communication between PX4 and ROS 2)

```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 4: Start Rover Controller Logic**

```bash
cd ~/workspace
source install/setup.bash
ros2 run project_logic rover_controller
```

**Terminal 5: Start Drone Mission Logic**

```bash
cd ~/workspace
source install/setup.bash
ros2 run project_logic drone_mission
```

Follow the on-screen prompts to input coordinates (e.g., X: 7, Y: -0.15).

**Terminal 6: Camera View**

```bash
ros2 run rqt_image_view rqt_image_view
```

---

## 🛠 Fault Injection & Repair

You can simulate failures and repairs using ROS 2 services.

### Rover 1 (Main Rover)

**Break Rover 1:**

```bash
ros2 service call /rover_1/set_health std_srvs/srv/SetBool "{data: false}"
```

**Repair Rover 1:**

```bash
ros2 service call /rover_1/set_health std_srvs/srv/SetBool "{data: true}"
```

### Emergency Rover

**Break Emergency Rover:**

```bash
ros2 service call /rover_emergency/set_health std_srvs/srv/SetBool "{data: false}"
```

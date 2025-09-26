# RobotLab Cloud - Standalone Package

🤖 **Transform your robot URDF files into complete, production-ready ROS workspaces in seconds!**

RobotLab Cloud is a powerful platform that automatically generates complete robot workspaces from URDF files or robot_description packages. It creates all necessary ROS packages, launch files, Docker configurations, and integrates with the Robot Capability Manifest (RCM) system.

## ✨ Features

- **📁 Complete Workspace Generation**: Automatically creates ROS packages, launch files, and configuration files from your URDF
- **🐳 Docker Ready**: Generates Dockerfile and docker-compose.yml for easy containerization and deployment
- **🧠 Smart Package Detection**: Intelligently detects and adds MoveIt, navigation, and other packages based on your robot's capabilities
- **⚡ One-Click Download**: Download your complete workspace as a ZIP file ready for immediate use
- **🔧 RCM Integration**: Creates Robot Capability Manifests for LLM-based robot control
- **🌐 Cross-Platform**: Works on Linux, macOS, and Windows (via Docker Desktop)
- **🔍 Kinematics Analysis**: Uses Pinocchio for advanced robot kinematics and dynamics analysis
- **📦 Package Management**: Automatically handles ROS dependencies and package relationships
- **🎯 Multi-Robot Support**: Generate workspaces for different robot types (manipulators, mobile robots, etc.)
- **⚙️ Customizable**: Add custom ROS packages and configurations to generated workspaces

## 🚀 Quick Start

### Prerequisites (for running locally)

1. **Python 3.8+** with pip
2. **Pinocchio** (for robot kinematics/dynamics analysis)
3. **ROS 2 Humble** (for generated workspaces only - not needed when using Docker)

### Installation

1. **Clone or download this package**
   ```bash
   # If you have the files, just navigate to the directory
   cd robotlab_cloud_standalone
   ```

2. **Install Pinocchio**
   
   **Option A: Using pip (recommended)**
   ```bash
   # Install Pinocchio with dependencies
   python -m pip install pin
   ```

   **Option B: Using apt (Ubuntu 20.04, 22.04, 24.04)**
   ```bash
   # Add robotpkg repository
   sudo apt install -qqy lsb-release curl
   sudo mkdir -p /etc/apt/keyrings
   curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc
   echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
   sudo apt update
   
   # Install Pinocchio
   sudo apt install -qqy robotpkg-py3*-pinocchio
   
   # Configure environment variables
   export PATH=/opt/openrobots/bin:$PATH
   export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
   export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
   export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH
   export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
   ```

   **Option C: From source**
   ```bash
   # Clone and build from source
   git clone --recursive https://github.com/stack-of-tasks/pinocchio
   cd pinocchio && mkdir build && cd build
   cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
   make -j4 && sudo make install
   ```

3. **Install other dependencies**
   ```bash
   pip install -r requirements.txt
   ```

4. **Start the server**
   
   **Option A: Direct Python execution**
   ```bash
   python3 rcm_server.py
   ```
   
   **Option B: Using the start script**
   ```bash
   chmod +x start_server.sh
   ./start_server.sh
   ```

5. **Open your browser**
   ```
   http://localhost:8000
   ```

### Docker Usage (No ROS Required)

When using Docker, you don't need to install ROS or Pinocchio locally:

```bash
# The generated workspaces include Docker configuration
# No local ROS installation required
# Windows compatible through Docker Desktop
```

## 📖 How to Use

### ⚠️ **IMPORTANT: Robot ID Naming Convention**

**🔴 CRITICAL: Your Robot ID must match your URDF filename exactly!**

- If your URDF file is named `my_robot.urdf`, your Robot ID must be `my_robot`
- If your URDF file is named `panda_arm.urdf`, your Robot ID must be `panda_arm`
- This ensures proper file references and launch file compatibility
- **Mismatched names will cause launch failures in Docker containers**

### 1. Upload Your Robot Description

**Option A: Single URDF File**
- Upload a `.urdf` or `.xacro` file
- **Robot ID must match the URDF filename** (without extension)
- The system will automatically search for associated mesh files
- Best for: Simple robots with basic descriptions

**Option B: Complete Robot Description Package**
- Upload a ZIP file containing a complete robot_description package
- **Robot ID should match the main URDF filename** in the package
- Includes all meshes, materials, and configurations
- Best for: Complex robots with detailed visual/collision models

### 2. Configure Your Workspace

- **Robot ID**: A unique identifier (used in package names and file paths)
- **Robot Type**: Select from predefined types or choose "Custom"
- **Additional Packages**: Specify extra ROS packages (comma-separated)
  - Example: `ros-humble-navigation2, ros-humble-moveit`

### 3. Generate and Download

- Click "Generate Robot Workspace"
- Wait for the system to process your robot (usually 30-60 seconds)
- Download the complete workspace as a ZIP file

## 🏗️ What Gets Generated

Your generated workspace includes:

```
your_robot_workspace/
├── src/
│   ├── your_robot_description/     # Robot description package
│   ├── your_robot_launch/          # Launch files
│   ├── your_robot_rviz/            # RViz configurations
│   ├── your_robot_rcm/             # RCM configuration
│   ├── moveit/                     # MoveIt configuration (if applicable)
│   └── moveit-servo/               # MoveIt Servo (if applicable)
├── Dockerfile                      # Docker configuration
├── docker-compose.yml              # Docker Compose setup
├── build_workspace.sh              # Build script
└── README.md                       # Generated documentation
```

## 🔧 Advanced Usage

### Custom Robot Types

The system automatically detects robot capabilities and adds relevant packages:

- **Manipulators**: Automatically adds MoveIt, MoveIt Servo, gripper controllers
- **Mobile Robots**: Automatically adds Navigation2, SLAM toolbox
- **Articulated Arms**: Adds joint state publisher, robot state publisher

### Docker Deployment

Each generated workspace includes Docker configuration for easy deployment:

```bash
# Build the Docker image
docker build -t your_robot_workspace .

# Run with Docker Compose
docker-compose up
```

**Windows Compatibility**: Generated workspaces work seamlessly on Windows through Docker Desktop. No local ROS installation required - everything runs in containers.

### ROS 2 Integration

The generated workspaces are fully compatible with ROS 2 Humble:

```bash
# Build the workspace
cd your_robot_workspace
./build_workspace.sh

# Source and run
source install/setup.bash
ros2 launch your_robot_launch your_robot_launch.py
```

## 🛠️ Troubleshooting

### Common Issues

1. **"Launch file not found" or "Simulation launch failed"**
   - **Most common cause**: Robot ID doesn't match URDF filename
   - Ensure Robot ID exactly matches your URDF filename (without .urdf extension)
   - Example: If URDF is `my_robot.urdf`, Robot ID must be `my_robot`
   - Check the Docker logs for "Available launch files" to see what was generated

2. **"Pinocchio not found" error**
   - Install Pinocchio using pip: `python -m pip install pin`
   - Or follow the apt installation instructions above
   - Ensure environment variables are set correctly if using apt installation

3. **"URDF parsing error"**
   - Check that your URDF file is valid XML
   - Ensure the root element is `<robot>`
   - Verify file is not empty

4. **"Mesh files not found" warnings**
   - These are warnings, not errors
   - The workspace will still generate successfully
   - For complete visual models, use robot_description packages (ZIP)

5. **Port 8000 already in use**
   - Kill existing processes: `pkill -f rcm_server.py`
   - Or change the port in `rcm_server.py`

### Getting Help

- Check the server logs for detailed error messages
- Ensure all dependencies are installed correctly
- Verify your URDF file is valid

## 📁 File Structure

```
robotlab_cloud_standalone/
├── README.md                    # This file
├── requirements.txt             # Python dependencies
├── rcm_server.py               # Main FastAPI server
├── robotlab_web_ui.html        # Web interface
├── robot_config_generator.py   # Workspace generation logic
├── urdf_to_rcm.py             # URDF to RCM conversion
└── generated_workspaces/       # Generated workspaces (created at runtime)
```

## 🔬 Technical Details

### RCM (Robot Capability Manifest)

The system generates RCM files that describe your robot's capabilities:

- **Kinematics**: Joint limits, forward/inverse kinematics
- **Dynamics**: Mass properties, center of mass
- **Sensors**: Camera, LiDAR, IMU configurations
- **End Effectors**: Gripper types and capabilities
- **Workspaces**: Reachable space analysis

### Smart Package Detection

The system analyzes your URDF to automatically detect:

- **Manipulation capabilities** → MoveIt packages
- **Mobile base** → Navigation2 packages
- **Grippers** → Gripper controller packages
- **Sensors** → Sensor-specific packages

## 🤝 Contributing

This is a standalone package. For development or modifications:

1. Edit the Python files as needed
2. Test with your robot URDFs
3. The web UI can be customized by editing `robotlab_web_ui.html`

## 📄 License

This project is part of the RobotLab Cloud initiative for simplifying robot development and deployment.

---

**Happy robot development! 🤖✨**

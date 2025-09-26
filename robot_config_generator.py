#!/usr/bin/env python3
"""
Robot Configuration Generator - Creates complete robot workspaces from URDFs
Solves the #1 pain point: Robot Setup & Configuration Hell
"""

import os
import json
import logging
import shutil
import tempfile
from pathlib import Path
from typing import Dict, Any, List, Optional
from dataclasses import dataclass
import subprocess
import zipfile
import time

from urdf_to_rcm import build_rcm_with_pinocchio

logger = logging.getLogger(__name__)

@dataclass
class RobotWorkspace:
    """Represents a complete robot workspace"""
    robot_id: str
    rcm: Dict[str, Any]
    workspace_path: str
    docker_config: Dict[str, Any]
    launch_files: List[str]
    config_files: List[str]
    mesh_files: List[str] = None

class RobotConfigGenerator:
    """Generates complete robot workspaces from URDFs"""
    
    def __init__(self, output_dir: str = "generated_workspaces"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        # URDF to RCM conversion is handled by build_rcm_with_pinocchio function
        
        # Robot templates for common configurations
        self.robot_templates = {
            "turtlebot3": {
                "name": "TurtleBot3",
                "description": "Popular mobile robot for education and research",
                "ros_packages": ["turtlebot3", "turtlebot3_simulations", "navigation2"],
                "launch_files": ["turtlebot3_gazebo.launch.py", "turtlebot3_navigation2.launch.py"],
                "dependencies": [
                    "ros-humble-turtlebot3-msgs",
                    "ros-humble-turtlebot3",
                    "ros-humble-turtlebot3-simulations",
                    "ros-humble-navigation2",
                    "ros-humble-nav2-bringup",
                    "ros-humble-slam-toolbox"
                ],
                "urdf_url": "https://github.com/ROBOTIS-GIT/turtlebot3/raw/humble-devel/turtlebot3_description/urdf/turtlebot3_burger.urdf.xacro",
                "type": "mobile_robot"
            },
            "ur5": {
                "name": "Universal Robots UR5",
                "description": "Industrial 6-DOF collaborative robot arm",
                "ros_packages": ["ur_robot_driver", "moveit", "ur_description"],
                "launch_files": ["ur5_bringup.launch.py", "ur5_moveit_planning_execution.launch.py"],
                "dependencies": [
                    "ros-humble-ur-robot-driver",
                    "ros-humble-ur-description",
                    "ros-humble-moveit",
                    "ros-humble-moveit-servo"
                ],
                "urdf_url": "https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/raw/main/urdf/ur.urdf.xacro",
                "type": "manipulator"
            },
            "panda": {
                "name": "Franka Panda",
                "description": "7-DOF collaborative robot arm with gripper",
                "ros_packages": ["franka_ros", "moveit", "panda_moveit_config"],
                "launch_files": ["panda_bringup.launch.py", "panda_moveit_planning_execution.launch.py"],
                "dependencies": [
                    "ros-humble-franka-ros",
                    "ros-humble-moveit",
                    "ros-humble-moveit-servo",
                    "ros-humble-gripper-controllers"
                ],
                "urdf_url": "https://github.com/frankaemika/franka_ros2/raw/develop/franka_description/robots/panda_arm_hand.urdf.xacro",
                "type": "manipulator"
            },
            "kuka_kr210": {
                "name": "KUKA KR210",
                "description": "Industrial 6-DOF robot arm",
                "ros_packages": ["kuka_kr210_support", "moveit"],
                "launch_files": ["kr210_bringup.launch.py", "kr210_moveit_planning_execution.launch.py"],
                "dependencies": [
                    "ros-humble-kuka-kr210-support",
                    "ros-humble-moveit",
                    "ros-humble-moveit-servo"
                ],
                "type": "manipulator"
            },
            "pr2": {
                "name": "PR2",
                "description": "Personal Robot 2 - mobile manipulator with dual arms",
                "ros_packages": ["pr2_robot", "moveit", "navigation2"],
                "launch_files": ["pr2_bringup.launch.py", "pr2_moveit_planning_execution.launch.py"],
                "dependencies": [
                    "ros-humble-pr2-robot",
                    "ros-humble-moveit",
                    "ros-humble-navigation2",
                    "ros-humble-gripper-controllers"
                ],
                "type": "mobile_manipulator"
            },
            "pioneer": {
                "name": "Pioneer 3-DX",
                "description": "Mobile robot with differential drive",
                "ros_packages": ["pioneer_bringup", "navigation2"],
                "launch_files": ["pioneer_bringup.launch.py", "pioneer_navigation.launch.py"],
                "dependencies": [
                    "ros-humble-navigation2",
                    "ros-humble-nav2-bringup",
                    "ros-humble-slam-toolbox"
                ],
                "type": "mobile_robot"
            },
            "custom": {
                "name": "Custom Robot",
                "description": "Upload your own URDF or robot description package",
                "dependencies": [],
                "type": "custom"
            }
        }
    
    def generate_workspace(self, urdf_path: str, robot_id: str, 
                          robot_type: str = "custom", 
                          additional_packages: List[str] = None,
                          robot_description_path: str = None) -> RobotWorkspace:
        """Generate complete robot workspace from URDF"""
        
        logger.info(f"Generating workspace for robot {robot_id} from {urdf_path}")
        logger.info(f"Robot type: {robot_type}, Additional packages: {additional_packages}")
        logger.info(f"Robot description path: {robot_description_path}")
        
        # Validate URDF file exists and is readable (only if not using robot_description_path)
        if not robot_description_path:
            urdf_file_path = Path(urdf_path)
            if not urdf_file_path.exists():
                raise FileNotFoundError(f"URDF file not found: {urdf_path}")
            if not urdf_file_path.is_file():
                raise ValueError(f"URDF path is not a file: {urdf_path}")
            logger.info(f"URDF file validated: {urdf_file_path} (size: {urdf_file_path.stat().st_size} bytes)")
        else:
            logger.info(f"Using robot description package: {robot_description_path}")
        
        # Create workspace directory
        workspace_path = self.output_dir / f"{robot_id}_workspace"
        
        # Clean up existing workspace if it exists
        if workspace_path.exists():
            logger.info(f"Cleaning up existing workspace: {workspace_path}")
            import shutil
            shutil.rmtree(workspace_path)
        
        workspace_path.mkdir(exist_ok=True)
        
        try:
            # Generate RCM from URDF
            logger.info("Step 1: Generating RCM from URDF")
            if robot_description_path:
                # Find URDF file in robot description package
                urdf_file = Path(robot_description_path) / "urdf" / f"{robot_id}.urdf"
                if not urdf_file.exists():
                    # Try alternative naming
                    urdf_file = Path(robot_description_path) / "urdf" / f"{Path(robot_description_path).name.replace('_description', '')}.urdf"
                if not urdf_file.exists():
                    raise FileNotFoundError(f"URDF file not found in robot description package: {robot_description_path}")
                rcm = self._generate_rcm(str(urdf_file), robot_id)
            else:
                rcm = self._generate_rcm(urdf_path, robot_id)
            
            # Detect smart packages based on robot capabilities
            smart_packages = self._detect_smart_packages(rcm)
            if smart_packages:
                logger.info(f"Detected smart packages: {smart_packages}")
                if additional_packages:
                    additional_packages.extend(smart_packages)
                else:
                    additional_packages = smart_packages
            
            # Create ROS workspace structure
            logger.info("Step 2: Creating ROS workspace structure")
            if robot_description_path:
                # Robot description package already exists, just create additional structure
                self._create_ros_workspace_with_existing_package(workspace_path, robot_id, robot_description_path)
            else:
                # Create standard ROS workspace structure
                self._create_ros_workspace(workspace_path, robot_id, urdf_path)
                
                # Create additional packages needed for launch files
                self._create_rcm_package(workspace_path, robot_id)
                self._create_launch_package(workspace_path, robot_id)
                self._create_rviz_package(workspace_path, robot_id, urdf_path)
                self._create_rcm_config_package(workspace_path, robot_id)
            
            # Generate launch files
            logger.info("Step 3: Generating launch files")
            if robot_description_path:
                # For robot_description packages, launch files go in the launch package
                launch_files = self._generate_launch_files_for_packages(workspace_path, robot_id, robot_type)
            else:
                # For single URDF files, also generate launch files in the launch package
                launch_files = self._generate_launch_files_for_packages(workspace_path, robot_id, robot_type)
            
            # Collect mesh file information (only for single URDF files)
            if not robot_description_path:
                logger.info("Step 4: Collecting mesh file information")
                mesh_files = self._collect_mesh_files(urdf_path, workspace_path, robot_id)
            else:
                # For robot_description packages, meshes are already in the package
                mesh_files = []
            
            # Generate configuration files
            logger.info("Step 5: Generating configuration files")
            # For both single URDF files and robot_description packages, use the same approach
            config_files = self._generate_config_files(workspace_path, robot_id, rcm)
            
            # Generate Docker configuration
            logger.info("Step 6: Generating Docker configuration")
            docker_config = self._generate_docker_config(workspace_path, robot_id, robot_type, additional_packages)
            
            # Create workspace manifest
            logger.info("Step 7: Creating workspace manifest")
            self._create_workspace_manifest(workspace_path, robot_id, rcm, launch_files, config_files)
            
            # Create additional packages
            if additional_packages:
                logger.info("Step 8: Creating additional packages")
                additional_package_paths = self._create_additional_packages(workspace_path, robot_id, additional_packages)
                logger.info(f"Created additional packages: {additional_package_paths}")
            
            # Create comprehensive README
            logger.info("Step 9: Creating comprehensive README")
            self._create_workspace_readme(workspace_path, robot_id, robot_type, rcm, mesh_files)
            
            # Create build script
            logger.info("Step 10: Creating build script")
            self._create_build_script(workspace_path, robot_id)
        
        except Exception as e:
            logger.error(f"Error in workspace generation step: {str(e)}")
            import traceback
            logger.error(f"Full traceback: {traceback.format_exc()}")
            raise
        
        return RobotWorkspace(
            robot_id=robot_id,
            rcm=rcm,
            workspace_path=str(workspace_path),
            docker_config=docker_config,
            launch_files=launch_files,
            config_files=config_files,
            mesh_files=mesh_files
        )
    
    def _generate_rcm(self, urdf_path: str, robot_id: str) -> Dict[str, Any]:
        """Generate RCM from URDF using existing converter"""
        try:
            rcm = build_rcm_with_pinocchio(urdf_path, samples=400)
            
            # Enhance RCM with metadata
            rcm["metadata"] = {
                "robot_id": robot_id,
                "generated_by": "RobotLab Cloud",
                "generation_timestamp": time.time(),
                "urdf_source": urdf_path
            }
            
            return rcm
        except Exception as e:
            logger.error(f"Failed to generate RCM: {e}")
            raise
    
    def _detect_smart_packages(self, rcm: Dict[str, Any]) -> List[str]:
        """Detect smart packages based on robot capabilities"""
        smart_packages = []
        
        # Check for mobile base
        has_mobile_base = rcm.get("has_mobile_base", False)
        if has_mobile_base:
            smart_packages.extend([
                "ros-humble-navigation2",
                "ros-humble-nav2-bringup",
                "ros-humble-cartographer",
                "ros-humble-slam-toolbox"
            ])
        
        # Check for manipulator arms
        joints = rcm.get("joints", {})
        manipulator_joints = 0
        for joint_name, joint_data in joints.items():
            if joint_data.get("type") in ["revolute", "prismatic"] and not any(kw in joint_name.lower() for kw in ["wheel", "caster", "base"]):
                manipulator_joints += 1
        
        if manipulator_joints >= 2:  # Likely a manipulator (even simple grippers)
            smart_packages.extend([
                "ros-humble-moveit",
                "ros-humble-moveit-servo"
            ])
        
        # Check for gripper
        has_gripper = rcm.get("has_gripper", False)
        if has_gripper:
            smart_packages.append("ros-humble-gripper-controllers")
        
        # Check for sensors
        sensors = rcm.get("sensors", [])
        for sensor in sensors:
            sensor_type = sensor.get("type", "")
            if "camera" in sensor_type or "rgbd" in sensor_type:
                smart_packages.extend([
                    "ros-humble-image-transport",
                    "ros-humble-cv-bridge",
                    "ros-humble-vision-msgs"
                ])
            elif "lidar" in sensor_type:
                smart_packages.extend([
                    "ros-humble-laser-filters",
                    "ros-humble-pointcloud-to-laserscan"
                ])
        
        # Remove duplicates and return
        return list(set(smart_packages))
    
    def _create_ros_workspace(self, workspace_path: Path, robot_id: str, urdf_path: str):
        """Create standard ROS workspace structure"""
        
        # Create workspace directories
        directories = [
            "src",
            "launch", 
            "config",
            "worlds",
            "maps",
            "rviz",
            "scripts",
            "urdf"
        ]
        
        for dir_name in directories:
            (workspace_path / dir_name).mkdir(exist_ok=True)
        
        # Create package.xml
        package_xml = f"""<?xml version="1.0"?>
<package format="3">
  <name>{robot_id}_robot</name>
  <version>1.0.0</version>
  <description>Auto-generated robot package for {robot_id}</description>
  <maintainer email="robotlab@example.com">RobotLab Cloud</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>"""
        
        with open(workspace_path / "package.xml", "w") as f:
            f.write(package_xml)
        
        # Create CMakeLists.txt
        cmake_lists = f"""cmake_minimum_required(VERSION 3.8)
project({robot_id}_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${{PROJECT_NAME}}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${{PROJECT_NAME}}/
)

# Install RViz configs
install(DIRECTORY
  rviz
  DESTINATION share/${{PROJECT_NAME}}/
)

# Install scripts
install(DIRECTORY
  scripts
  DESTINATION share/${{PROJECT_NAME}}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()"""
        
        with open(workspace_path / "CMakeLists.txt", "w") as f:
            f.write(cmake_lists)
        
        # Create robot description package for single URDF
        self._create_robot_description_package(workspace_path, robot_id, urdf_path)
        
        # Create RCM integration package
        self._create_rcm_package(workspace_path, robot_id)
    
    def _generate_launch_files(self, workspace_path: Path, robot_id: str, robot_type: str) -> List[str]:
        """Generate launch files for the robot"""
        
        launch_files = []
        
        # Basic robot launch file
        basic_launch = f"""#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='{robot_id}',
        description='Robot ID for namespacing'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{{
            'robot_description': open('{workspace_path}/urdf/{robot_id}.urdf', 'r').read(),
            'use_sim_time': True
        }}]
    )
    
    # Joint state publisher GUI for manual control
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{{'use_sim_time': True}}]
    )
    
    return LaunchDescription([
        robot_id_arg,
        robot_state_publisher,
        joint_state_publisher_gui
    ])"""
        
        launch_file_path = workspace_path / "launch" / f"{robot_id}_basic.launch.py"
        with open(launch_file_path, "w") as f:
            f.write(basic_launch)
        launch_files.append(str(launch_file_path))
        
        # RCM integration launch file
        rcm_launch = f"""#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='{robot_id}',
        description='Robot ID for namespacing'
    )
    
    # RCM ROS Bridge
    rcm_bridge = Node(
        package='robotlab_rcm',
        executable='ros_bridge',
        name='rcm_ros_bridge',
        parameters=[{{
            'robot_namespace': LaunchConfiguration('robot_id'),
            'rcm_file': '{workspace_path}/rcm/{robot_id}_rcm.json'
        }}]
    )
    
    # RCM Tool Generator
    rcm_tools = Node(
        package='robotlab_rcm',
        executable='ros_tool_generator',
        name='rcm_tool_generator',
        parameters=[{{
            'robot_namespace': LaunchConfiguration('robot_id'),
            'rcm_file': '{workspace_path}/rcm/{robot_id}_rcm.json'
        }}]
    )
    
    return LaunchDescription([
        robot_id_arg,
        rcm_bridge,
        rcm_tools
    ])"""
        
        rcm_launch_path = workspace_path / "launch" / f"{robot_id}_rcm.launch.py"
        with open(rcm_launch_path, "w") as f:
            f.write(rcm_launch)
        launch_files.append(str(rcm_launch_path))
        
        return launch_files
    
    def _generate_launch_files_for_packages(self, workspace_path: Path, robot_id: str, robot_type: str) -> List[str]:
        """Generate launch files for robot_description packages (in src/ packages)"""
        
        launch_files = []
        launch_package_dir = workspace_path / "src" / f"{robot_id}_launch"
        
        # Basic robot launch file
        basic_launch = f"""#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='{robot_id}',
        description='Robot ID for namespacing'
    )
    
    # Get package directories
    robot_desc_pkg_dir = get_package_share_directory('{robot_id}_description')
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{{
            'robot_description': open(os.path.join(robot_desc_pkg_dir, 'urdf', '{robot_id}.urdf'), 'r').read(),
            'use_sim_time': True
        }}]
    )
    
    # Joint state publisher GUI for manual control
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{{'use_sim_time': True}}]
    )
    
    return LaunchDescription([
        robot_id_arg,
        robot_state_publisher,
        joint_state_publisher_gui
    ])"""
        
        launch_file_path = launch_package_dir / "launch" / f"{robot_id}_basic.launch.py"
        with open(launch_file_path, "w") as f:
            f.write(basic_launch)
        launch_files.append(str(launch_file_path))
        
        # RCM integration launch file
        rcm_launch = f"""#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='{robot_id}',
        description='Robot ID for namespacing'
    )
    
    # Get package directories
    robot_desc_pkg_dir = get_package_share_directory('{robot_id}_description')
    rcm_pkg_dir = get_package_share_directory('{robot_id}_rcm')
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{{
            'robot_description': open(os.path.join(robot_desc_pkg_dir, 'urdf', '{robot_id}.urdf'), 'r').read(),
            'use_sim_time': True
        }}]
    )
    
    # Joint state publisher GUI for manual control
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{{'use_sim_time': True}}]
    )
    
    # RCM ROS Bridge
    rcm_bridge = Node(
        package='robotlab_rcm',
        executable='ros_bridge',
        name='rcm_ros_bridge',
        parameters=[{{
            'robot_namespace': LaunchConfiguration('robot_id'),
            'rcm_file': os.path.join(rcm_pkg_dir, 'rcm', '{robot_id}_rcm.json')
        }}]
    )
    
    return LaunchDescription([
        robot_id_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rcm_bridge
    ])"""
        
        rcm_launch_path = launch_package_dir / "launch" / f"{robot_id}_rcm.launch.py"
        with open(rcm_launch_path, "w") as f:
            f.write(rcm_launch)
        launch_files.append(str(rcm_launch_path))
        
        # Simple simulation launch file (RViz only)
        simulation_launch = f"""#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # Declare launch arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='{robot_id}',
        description='Robot ID for namespacing'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz visualization?'
    )
    
    # Get package directories
    robot_desc_pkg_dir = get_package_share_directory('{robot_id}_description')
    rviz_pkg_dir = get_package_share_directory('{robot_id}_rviz')
    
    # RViz config file
    rviz_config_file = os.path.join(rviz_pkg_dir, 'rviz', '{robot_id}_config.rviz')
    
    # Robot description
    robot_description_content = open(os.path.join(robot_desc_pkg_dir, 'urdf', '{robot_id}.urdf'), 'r').read()
    robot_description = {{"robot_description": robot_description_content}}
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description],
        output='both'
    )
    
    # Joint state publisher GUI for manual control
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='both'
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        output='log'
    )
    
    return LaunchDescription([
        robot_id_arg,
        launch_rviz_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])"""
        
        simulation_launch_path = launch_package_dir / "launch" / f"{robot_id}_simulation.launch.py"
        with open(simulation_launch_path, "w") as f:
            f.write(simulation_launch)
        launch_files.append(str(simulation_launch_path))
        
        # RViz launch file
        rviz_launch = f"""#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='{robot_id}',
        description='Robot ID for namespacing'
    )
    
    # Get package directories
    robot_desc_pkg_dir = get_package_share_directory('{robot_id}_description')
    rviz_pkg_dir = get_package_share_directory('{robot_id}_rviz')
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{{
            'robot_description': open(os.path.join(robot_desc_pkg_dir, 'urdf', '{robot_id}.urdf'), 'r').read(),
            'use_sim_time': True
        }}]
    )
    
    # Joint state publisher GUI for manual control
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{{'use_sim_time': True}}]
    )
    
    # RViz node with config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(rviz_pkg_dir, 'rviz', '{robot_id}_config.rviz')],
        output='screen'
    )
    
    return LaunchDescription([
        robot_id_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])"""
        
        rviz_launch_path = launch_package_dir / "launch" / f"{robot_id}_rviz.launch.py"
        with open(rviz_launch_path, "w") as f:
            f.write(rviz_launch)
        launch_files.append(str(rviz_launch_path))
        
        
        return launch_files
    
    def _create_rcm_package(self, workspace_path: Path, robot_id: str) -> None:
        """Create RCM integration package for ROS workspace"""
        
        rcm_package_dir = workspace_path / "src" / "robotlab_rcm"
        rcm_package_dir.mkdir(parents=True, exist_ok=True)
        
        # Create package.xml
        package_xml = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robotlab_rcm</name>
  <version>1.0.0</version>
  <description>RobotLab Cloud RCM integration package</description>
  <maintainer email="robotlab@example.com">RobotLab Cloud</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_python</buildtool_depend>
  
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>trajectory_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>"""
        
        with open(rcm_package_dir / "package.xml", "w") as f:
            f.write(package_xml)
        
        # Create setup.py
        setup_py = f"""from setuptools import setup

package_name = 'robotlab_rcm'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rcm_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RobotLab Cloud',
    maintainer_email='robotlab@example.com',
    description='RobotLab Cloud RCM integration package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={{
        'console_scripts': [
            'ros_bridge = robotlab_rcm.ros_bridge:main',
            'ros_tool_generator = robotlab_rcm.ros_tool_generator:main',
        ],
    }},
)"""
        
        with open(rcm_package_dir / "setup.py", "w") as f:
            f.write(setup_py)
        
        # Create resource directory
        (rcm_package_dir / "resource").mkdir(exist_ok=True)
        (rcm_package_dir / "resource" / "robotlab_rcm").touch()
        
        # Create Python package directory
        (rcm_package_dir / "robotlab_rcm").mkdir(exist_ok=True)
        
        # Create __init__.py
        (rcm_package_dir / "robotlab_rcm" / "__init__.py").touch()
        
        # Create launch directory
        (rcm_package_dir / "launch").mkdir(exist_ok=True)
        
        # Create basic RCM bridge launch file
        rcm_launch = f"""from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='{robot_id}',
        description='Robot ID for namespacing'
    )
    
    rcm_file_arg = DeclareLaunchArgument(
        'rcm_file',
        default_value='install/robotlab_rcm/share/robotlab_rcm/rcm/{robot_id}_rcm.json',
        description='Path to RCM file'
    )
    
    # RCM ROS Bridge
    rcm_bridge = Node(
        package='robotlab_rcm',
        executable='ros_bridge',
        name='rcm_ros_bridge',
        parameters=[{{
            'robot_namespace': LaunchConfiguration('robot_id'),
            'rcm_file': LaunchConfiguration('rcm_file')
        }}]
    )
    
    return LaunchDescription([
        robot_id_arg,
        rcm_file_arg,
        rcm_bridge
    ])"""
        
        with open(rcm_package_dir / "launch" / "rcm_bridge.launch.py", "w") as f:
            f.write(rcm_launch)
    
    def _create_launch_package(self, workspace_path: Path, robot_id: str) -> None:
        """Create launch package in src/"""
        
        launch_package_dir = workspace_path / "src" / f"{robot_id}_launch"
        launch_package_dir.mkdir(parents=True, exist_ok=True)
        
        # Create package.xml
        package_xml = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{robot_id}_launch</name>
  <version>1.0.0</version>
  <description>Launch files for {robot_id} robot</description>
  <maintainer email="robotlab@example.com">RobotLab Cloud</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>robot_state_publisher</depend>
  <depend>joint_state_publisher_gui</depend>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>"""
        
        with open(launch_package_dir / "package.xml", "w") as f:
            f.write(package_xml)
        
        # Create CMakeLists.txt
        cmake_lists = f"""cmake_minimum_required(VERSION 3.8)
project({robot_id}_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${{PROJECT_NAME}}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()"""
        
        with open(launch_package_dir / "CMakeLists.txt", "w") as f:
            f.write(cmake_lists)
        
        # Create launch directory
        (launch_package_dir / "launch").mkdir(exist_ok=True)
    
    def _get_robot_base_link(self, urdf_path: str) -> str:
        """Extract the base link from URDF file"""
        try:
            import xml.etree.ElementTree as ET
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            
            # Find the first link that is not a child of any joint
            all_links = set()
            child_links = set()
            
            for joint in root.findall('joint'):
                child_link = joint.find('child')
                if child_link is not None:
                    child_links.add(child_link.get('link'))
            
            for link in root.findall('link'):
                link_name = link.get('name')
                if link_name:
                    all_links.add(link_name)
            
            # Base link is the one that's not a child of any joint
            base_links = all_links - child_links
            
            if base_links:
                return list(base_links)[0]  # Return first base link found
            else:
                # Fallback: return first link found
                first_link = root.find('link')
                if first_link is not None:
                    return first_link.get('name', 'base_link')
                else:
                    return 'base_link'
        except Exception as e:
            logger.warning(f"Could not parse URDF to find base link: {e}")
            return 'base_link'

    def _create_rviz_package(self, workspace_path: Path, robot_id: str, urdf_path: str = None) -> None:
        """Create rviz package in src/"""
        
        rviz_package_dir = workspace_path / "src" / f"{robot_id}_rviz"
        rviz_package_dir.mkdir(parents=True, exist_ok=True)
        
        # Get the robot's base link for proper fixed frame
        base_link = 'base_link'  # default
        if urdf_path and Path(urdf_path).exists():
            base_link = self._get_robot_base_link(urdf_path)
        
        # Create package.xml
        package_xml = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{robot_id}_rviz</name>
  <version>1.0.0</version>
  <description>RViz configurations for {robot_id} robot</description>
  <maintainer email="robotlab@example.com">RobotLab Cloud</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>"""
        
        with open(rviz_package_dir / "package.xml", "w") as f:
            f.write(package_xml)
        
        # Create CMakeLists.txt
        cmake_lists = f"""cmake_minimum_required(VERSION 3.8)
project({robot_id}_rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install rviz configs
install(DIRECTORY
  rviz
  DESTINATION share/${{PROJECT_NAME}}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()"""
        
        with open(rviz_package_dir / "CMakeLists.txt", "w") as f:
            f.write(cmake_lists)
        
        # Create rviz directory
        rviz_dir = rviz_package_dir / "rviz"
        rviz_dir.mkdir(exist_ok=True)
        
        # Create RViz configuration file - optimized for robot visualization
        rviz_config = f"""Panels:
  - Class: rviz_common/Displays
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /RobotModel1
        - /TF1
      Splitter Ratio: 0.25
    Tree Height: 300
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Name: Tool Properties
    Splitter Ratio: 0.25
  - Class: rviz_common/Views
    Name: Views
    Splitter Ratio: 0.25
  - Class: rviz_common/Time
    Name: Time
Preferences:
  PromptSaveOnExit: true
Toolbars:
  toolButtonStyle: 2
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 0.5
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 30
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description File: ""
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
      Mass Properties:
        Inertia: false
        Mass: false
      Name: RobotModel
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        {{}}
      Update Interval: 0
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: {base_link}
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 3.2
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0.5.5
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Far Clip Distance: 1000
      Pitch: 0.65
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz_default_plugins)
      Yaw: 0.55
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 950
  Width: 1450
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd000000040000000000000156000002b0fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000001e2000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500670065007400730100000041000000e70000000000000000fb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f000002b0fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730000000041000000680000000000000000fb0000000a00560069006500770073010000003d000002b0000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004b00000003efc0100000002fb0000000800540069006d00650100000000000004b0000002eb00fffffffb0000000800540069006d006501000000000000045000000000000000000000023f000002b000000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Time:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1400
  X: 60
  Y: 60
"""
        
        with open(rviz_dir / f"{robot_id}_config.rviz", "w") as f:
            f.write(rviz_config)
    
    def _generate_config_files(self, workspace_path: Path, robot_id: str, rcm: Dict[str, Any]) -> List[str]:
        """Generate configuration files for the robot workspace"""
        config_files = []
        
        # Create RCM config package which contains the main config files
        self._create_rcm_config_package(workspace_path, robot_id)
        
        # Add the RCM config files to the list
        rcm_config_dir = workspace_path / "src" / f"{robot_id}_rcm_config"
        if rcm_config_dir.exists():
            for config_file in rcm_config_dir.rglob("*.yaml"):
                config_files.append(str(config_file.relative_to(workspace_path)))
            for config_file in rcm_config_dir.rglob("*.json"):
                config_files.append(str(config_file.relative_to(workspace_path)))
        
        logger.info(f"Generated {len(config_files)} configuration files")
        return config_files
    
    def _generate_docker_config(self, workspace_path: Path, robot_id: str, robot_type: str, additional_packages: List[str] = None) -> Dict[str, Any]:
        """Generate Docker configuration for the workspace"""
        docker_config = {
            "robot_id": robot_id,
            "robot_type": robot_type,
            "base_image": "ros:humble-desktop-full",
            "additional_packages": additional_packages or [],
            "workspace_path": str(workspace_path),
            "docker_ready": True
        }
        
        # Create Dockerfile
        dockerfile_content = f"""FROM ros:humble-desktop-full

# Install additional packages
RUN apt-get update && apt-get install -y \\
    python3-pip \\
    python3-colcon-common-extensions \\
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install pinocchio

# Set up workspace
WORKDIR /workspace
COPY . /workspace/

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Source the workspace
RUN echo "source /workspace/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
"""
        
        with open(workspace_path / "Dockerfile", "w") as f:
            f.write(dockerfile_content)
        
        # Create docker-compose.yml
        compose_content = f"""version: '3.8'

services:
  {robot_id}_robot:
    build: .
    container_name: {robot_id}_workspace
    environment:
      - ROS_DOMAIN_ID=0
    volumes:
      - .:/workspace
    ports:
      - "8080:8080"
    command: /bin/bash -c "source /workspace/install/setup.bash && ros2 launch {robot_id}_launch {robot_id}_launch.py"
"""
        
        with open(workspace_path / "docker-compose.yml", "w") as f:
            f.write(compose_content)
        
        logger.info("Generated Docker configuration")
        return docker_config
    
    def package_workspace(self, workspace_path: str, output_format: str = "zip") -> str:
        """Package the workspace into a downloadable format"""
        workspace_path = Path(workspace_path)
        robot_id = workspace_path.name.replace("_workspace", "")
        
        if output_format == "zip":
            zip_path = workspace_path.parent / f"{robot_id}_workspace.zip"
            
            # Remove existing zip if it exists
            if zip_path.exists():
                zip_path.unlink()
            
            # Create zip file
            with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
                for file_path in workspace_path.rglob('*'):
                    if file_path.is_file():
                        arcname = file_path.relative_to(workspace_path.parent)
                        zipf.write(file_path, arcname)
            
            logger.info(f"Packaged workspace as {zip_path}")
            return str(zip_path)
        else:
            raise ValueError(f"Unsupported output format: {output_format}")
    
    def _create_rcm_config_package(self, workspace_path: Path, robot_id: str) -> None:
        """Create rcm config package in src/"""
        
        rcm_package_dir = workspace_path / "src" / f"{robot_id}_rcm"
        rcm_package_dir.mkdir(parents=True, exist_ok=True)
        
        # Create package.xml
        package_xml = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{robot_id}_rcm</name>
  <version>1.0.0</version>
  <description>RCM configuration for {robot_id} robot</description>
  <maintainer email="robotlab@example.com">RobotLab Cloud</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>"""
        
        with open(rcm_package_dir / "package.xml", "w") as f:
            f.write(package_xml)
        
        # Create CMakeLists.txt
        cmake_lists = f"""cmake_minimum_required(VERSION 3.8)
project({robot_id}_rcm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install rcm configs
install(DIRECTORY
  rcm
  DESTINATION share/${{PROJECT_NAME}}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()"""
        
        with open(rcm_package_dir / "CMakeLists.txt", "w") as f:
            f.write(cmake_lists)
        
        # Create rcm directory
        (rcm_package_dir / "rcm").mkdir(exist_ok=True)
    
    
    def _create_additional_packages(self, workspace_path: Path, robot_id: str, additional_packages: List[str]) -> List[str]:
        """Create ROS packages for additional packages"""
        created_packages = []
        
        if not additional_packages:
            return created_packages
        
        for package_name in additional_packages:
            # Extract package name from ros-humble-package-name format
            if package_name.startswith("ros-humble-"):
                clean_name = package_name.replace("ros-humble-", "")
            else:
                clean_name = package_name
            
            # Create package directory
            package_dir = workspace_path / "src" / clean_name
            package_dir.mkdir(parents=True, exist_ok=True)
            
            # Create package.xml
            package_xml = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{clean_name}</name>
  <version>1.0.0</version>
  <description>Additional package for {robot_id} robot</description>
  <maintainer email="robotlab@example.com">RobotLab Cloud</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>"""
            
            with open(package_dir / "package.xml", "w") as f:
                f.write(package_xml)
            
            # Create CMakeLists.txt
            cmake_lists = f"""cmake_minimum_required(VERSION 3.8)
project({clean_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()"""
            
            with open(package_dir / "CMakeLists.txt", "w") as f:
                f.write(cmake_lists)
            
            # Create launch directory for navigation/moveit packages
            if any(keyword in clean_name for keyword in ["navigation", "nav2", "moveit"]):
                (package_dir / "launch").mkdir(exist_ok=True)
                (package_dir / "config").mkdir(exist_ok=True)
                
                # Create a basic launch file
                launch_content = f"""#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Add {clean_name} specific launch configuration here
        Node(
            package='{clean_name}',
            executable='{clean_name}_node',
            name='{clean_name}_node',
            output='screen'
        )
    ])"""
                
                with open(package_dir / "launch" / f"{clean_name}.launch.py", "w") as f:
                    f.write(launch_content)
            
            created_packages.append(str(package_dir))
            
            # Special handling for MoveIt packages
            if "moveit" in clean_name and clean_name != "moveit":
                # Get RCM data for accurate MoveIt configuration
                # Try multiple possible RCM file locations
                rcm_data = None
                possible_rcm_paths = [
                    # For robot_description packages
                    workspace_path / "src" / f"{robot_id}_rcm" / "rcm" / f"{robot_id}_rcm.json",
                    # For single URDF uploads
                    workspace_path / "rcm" / f"{robot_id}_rcm.json",
                    # Alternative naming patterns
                    workspace_path / "src" / "rcm" / f"{robot_id}_rcm.json",
                ]
                
                for rcm_path in possible_rcm_paths:
                    if rcm_path.exists():
                        try:
                            import json
                            with open(rcm_path, 'r') as f:
                                rcm_data = json.load(f)
                            break
                        except Exception as e:
                            print(f"Warning: Could not load RCM from {rcm_path}: {e}")
                            continue
                
                self._create_moveit_package(package_dir, clean_name, robot_id, rcm_data)
        
        return created_packages
    
    def _create_moveit_package(self, package_path: Path, package_name: str, robot_id: str, rcm: dict = None) -> None:
        """Create a proper MoveIt configuration package with all necessary files based on actual robot capabilities."""
        
        # Create config directory
        config_path = package_path / "config"
        config_path.mkdir(exist_ok=True)
        
        # Get actual robot joints and capabilities from RCM
        joints = rcm.get("joints", {}) if rcm else {}
        has_gripper = rcm.get("has_gripper", False) if rcm else False
        end_effectors = rcm.get("end_effectors", []) if rcm else []
        
        print(f"Creating MoveIt package for robot: {robot_id}")
        print(f"RCM data available: {rcm is not None}")
        print(f"Joints found: {len(joints)}")
        print(f"Has gripper: {has_gripper}")
        
        # Extract revolute and prismatic joints (excluding fixed joints)
        manipulator_joints = []
        joint_limits_data = {}
        
        for joint_name, joint_data in joints.items():
            if joint_data.get("type") in ["revolute", "prismatic"]:
                manipulator_joints.append(joint_name)
                
                # Extract joint limits from RCM
                limits = joint_data.get("limits", {})
                max_vel = limits.get("velocity", 1.0) if limits.get("velocity") else 1.0
                max_acc = 1.0  # Default acceleration limit
                
                joint_limits_data[joint_name] = {
                    "has_velocity_limits": True,
                    "max_velocity": max_vel,
                    "has_acceleration_limits": True,
                    "max_acceleration": max_acc
                }
        
        print(f"Manipulator joints: {manipulator_joints}")
        
        # If no RCM data available, create a basic MoveIt configuration
        if not rcm or not manipulator_joints:
            print(f"Warning: No RCM data or manipulator joints found for {robot_id}. Creating basic MoveIt configuration.")
            manipulator_joints = ["joint1", "joint2", "joint3"]  # Default fallback
            joint_limits_data = {
                joint: {
                    "has_velocity_limits": True,
                    "max_velocity": 1.0,
                    "has_acceleration_limits": True,
                    "max_acceleration": 1.0
                } for joint in manipulator_joints
            }
        
        # Create moveit_controllers.yaml
        moveit_controllers = f"""# MoveIt uses this configuration for controller management
trajectory_execution:
  allowed_execution_duration_scaling: 1.2
  allowed_goal_duration_margin: 0.5
  allowed_start_tolerance: 0.01
  trajectory_duration_monitoring: true

moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - {robot_id}_arm_controller"""

        if has_gripper:
            moveit_controllers += f"""
    - {robot_id}_gripper_controller"""
        
        moveit_controllers += f"""

  {robot_id}_arm_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:"""

        for joint in manipulator_joints:
            moveit_controllers += f"""
      - {joint}"""
        
        if has_gripper:
            moveit_controllers += f"""

  {robot_id}_gripper_controller:
    action_ns: gripper_action
    type: GripperCommand
    default: true
    parallel: true
"""
        
        with open(config_path / "moveit_controllers.yaml", "w") as f:
            f.write(moveit_controllers)
        
        # Create ros2_controllers.yaml (essential for actual robot control)
        ros2_controllers = f"""# This config file is used by ros2_control
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    {robot_id}_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController"""

        if has_gripper:
            ros2_controllers += f"""

    {robot_id}_gripper_controller:
      type: position_controllers/GripperActionController"""
        
        ros2_controllers += f"""

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster


{robot_id}_arm_controller:
  ros__parameters:
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    joints:"""

        for joint in manipulator_joints:
            ros2_controllers += f"""
      - {joint}"""
        
        if has_gripper:
            ros2_controllers += f"""

{robot_id}_gripper_controller:
  ros__parameters:
    joint: gripper_joint
"""
        
        with open(config_path / "ros2_controllers.yaml", "w") as f:
            f.write(ros2_controllers)
        
        # Create kinematics.yaml
        kinematics = f"""{robot_id}_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
"""
        
        with open(config_path / "kinematics.yaml", "w") as f:
            f.write(kinematics)
        
        # Create joint_limits.yaml with actual joint data
        joint_limits = f"""# joint_limits.yaml allows the dynamics properties specified in the URDF to be overwritten or augmented as needed
# Specific joint properties can be changed with the keys [max_position, min_position, max_velocity, max_acceleration]
# Joint limits can be turned off with [has_velocity_limits, has_acceleration_limits]

joint_limits:"""

        for joint_name, limits_data in joint_limits_data.items():
            joint_limits += f"""
  {joint_name}:
    has_velocity_limits: {str(limits_data['has_velocity_limits']).lower()}
    max_velocity: {limits_data['max_velocity']}
    has_acceleration_limits: {str(limits_data['has_acceleration_limits']).lower()}
    max_acceleration: {limits_data['max_acceleration']}"""
        
        with open(config_path / "joint_limits.yaml", "w") as f:
            f.write(joint_limits)
        
        # Create ompl_planning.yaml
        ompl_planning = f"""planning_plugin: ompl_interface/OMPLPlanner
request_adapters: >-
    default_planner_request_adapters/AddTimeOptimalParameterization
    default_planner_request_adapters/FixWorkspaceBounds
    default_planner_request_adapters/FixStartStateBounds
    default_planner_request_adapters/FixStartStateCollision
    default_planner_request_adapters/FixStartStatePathConstraints
response_adapters: >-
    default_planner_response_adapters/AddTimeOptimalParameterization
    default_planner_response_adapters/ValidateSolution

planner_configs:
  SBLkConfigDefault:
    type: geometric::SBL
    range: 0.0
  ESTkConfigDefault:
    type: geometric::EST
    range: 0.0
    goal_bias: 0.05
  LBKPIECEkConfigDefault:
    type: geometric::LBKPIECE
    range: 0.0
    border_fraction: 0.9
    min_valid_path_fraction: 0.5
  BKPIECEkConfigDefault:
    type: geometric::BKPIECE
    range: 0.0
    border_fraction: 0.9
    min_valid_path_fraction: 0.5
  KPIECEkConfigDefault:
    type: geometric::KPIECE
    range: 0.0
    goal_bias: 0.05
    border_fraction: 0.9
    min_valid_path_fraction: 0.5
  RRTkConfigDefault:
    type: geometric::RRT
    range: 0.0
    goal_bias: 0.05
  RRTConnectkConfigDefault:
    type: geometric::RRTConnect
    range: 0.0
  RRTstarkConfigDefault:
    type: geometric::RRTstar
    range: 0.0
    goal_bias: 0.05
    delay_collision_checking: 1
  TRRTkConfigDefault:
    type: geometric::TRRT
    range: 0.0
    goal_bias: 0.05
    max_states_failed: 10
    temp_change_factor: 2.0
    min_temperature: 10e-10
    init_temperature: 10e-6
    frountier_threshold: 0.0
    k_constant: 0.0
  PRMkConfigDefault:
    type: geometric::PRM
    max_nearest_neighbors: 10
  PRMstarkConfigDefault:
    type: geometric::PRMstar
"""
        
        with open(config_path / "ompl_planning.yaml", "w") as f:
            f.write(ompl_planning)
        
        # Create sensors_3d.yaml (optional, for depth sensors)
        sensors_3d = f"""sensors:
  head_camera:
    type: depth_camera
    parent_link: base_link
    origin:
      xyz: [0.0, 0.0, 0.0]
      rpy: [0.0, 0.0, 0.0]
    spec:
      image_topic: /camera/depth/image_raw
      info_topic: /camera/depth/camera_info
      queue_size: 5
      use_message_range: true
"""
        
        with open(config_path / "sensors_3d.yaml", "w") as f:
            f.write(sensors_3d)
        
        # Update CMakeLists.txt to install config files
        cmake_lists_path = package_path / "CMakeLists.txt"
        if cmake_lists_path.exists():
            cmake_content = cmake_lists_path.read_text()
            cmake_content = cmake_content.replace(
                "# Install launch files\ninstall(DIRECTORY\n  launch\n  DESTINATION share/${{PROJECT_NAME}}/\n)",
                "# Install launch files\ninstall(DIRECTORY\n  launch\n  DESTINATION share/${{PROJECT_NAME}}/\n)\n\n# Install config files\ninstall(DIRECTORY\n  config\n  DESTINATION share/${{PROJECT_NAME}}/\n)"
            )
            cmake_lists_path.write_text(cmake_content)
    
    def _create_ros_workspace_with_existing_package(self, workspace_path: Path, robot_id: str, robot_description_path: str) -> None:
        """Create ROS workspace structure when robot_description package already exists"""
        
        # Create standard ROS 2 workspace structure
        (workspace_path / "src").mkdir(exist_ok=True)
        (workspace_path / "build").mkdir(exist_ok=True)
        (workspace_path / "install").mkdir(exist_ok=True)
        (workspace_path / "log").mkdir(exist_ok=True)
        
        # Copy robot_description package to src/ with standardized name
        import shutil
        robot_desc_source = Path(robot_description_path)
        robot_desc_dest = workspace_path / "src" / f"{robot_id}_description"
        
        # Remove existing directory if it exists to avoid conflicts
        if robot_desc_dest.exists():
            shutil.rmtree(robot_desc_dest)
        
        shutil.copytree(robot_desc_source, robot_desc_dest)
        
        # Update package.xml to use standardized name
        self._update_description_package_xml(robot_desc_dest, robot_id)
        
        # Update URDF mesh paths to use the new package name
        urdf_files = list((robot_desc_dest / "urdf").glob("*.urdf"))
        for urdf_file in urdf_files:
            self._update_urdf_mesh_paths(urdf_file, robot_id)
        
        # Create RCM integration package in src/
        self._create_rcm_package(workspace_path, robot_id)
        
        # Create launch package in src/
        self._create_launch_package(workspace_path, robot_id)
        
        # Create rviz package in src/
        # Find URDF path for base link detection
        urdf_path = None
        if robot_description_path:
            # Look for URDF in the copied package
            urdf_files = list(Path(robot_description_path).glob("**/*.urdf"))
            if urdf_files:
                urdf_path = str(urdf_files[0])
        else:
            # Single URDF case - find the URDF in the generated package
            urdf_files = list(workspace_path.glob("src/*_description/urdf/*.urdf"))
            if urdf_files:
                urdf_path = str(urdf_files[0])
        
        self._create_rviz_package(workspace_path, robot_id, urdf_path)
        
        # Create rcm package in src/
        self._create_rcm_config_package(workspace_path, robot_id)
        
        # Create Gazebo world file
    
    def _update_description_package_xml(self, package_dir: Path, robot_id: str) -> None:
        """Update package.xml to use standardized robot_id naming"""
        
        package_xml_path = package_dir / "package.xml"
        if not package_xml_path.exists():
            logger.warning(f"package.xml not found in {package_dir}")
            return
        
        # Read existing package.xml
        with open(package_xml_path, 'r') as f:
            content = f.read()
        
        # Update package name to use robot_id
        import re
        # Replace the package name in the XML
        content = re.sub(r'<name>[^<]+</name>', f'<name>{robot_id}_description</name>', content)
        
        # Write updated package.xml
        with open(package_xml_path, 'w') as f:
            f.write(content)
        
        # Also update CMakeLists.txt if it exists
        cmake_path = package_dir / "CMakeLists.txt"
        if cmake_path.exists():
            with open(cmake_path, 'r') as f:
                cmake_content = f.read()
            
            # Update project name in CMakeLists.txt
            cmake_content = re.sub(r'project\([^)]+\)', f'project({robot_id}_description)', cmake_content)
            
            with open(cmake_path, 'w') as f:
                f.write(cmake_content)
        
        logger.info(f"Updated package.xml and CMakeLists.txt for {robot_id}_description")
    
    def _create_robot_description_package(self, workspace_path: Path, robot_id: str, urdf_path: str) -> None:
        """Create robot description package for single URDF files"""
        
        # Create description package directory
        desc_package_dir = workspace_path / "src" / f"{robot_id}_description"
        desc_package_dir.mkdir(parents=True, exist_ok=True)
        
        # Create package.xml
        package_xml = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{robot_id}_description</name>
  <version>1.0.0</version>
  <description>Robot description package for {robot_id}</description>
  <maintainer email="robotlab@example.com">RobotLab Cloud</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <depend>urdf</depend>
  <depend>xacro</depend>
  <depend>ros2_control</depend>
  <depend>ros2_controllers</depend>
  <depend>gazebo_ros2_control</depend>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>"""
        
        with open(desc_package_dir / "package.xml", "w") as f:
            f.write(package_xml)
        
        # Create CMakeLists.txt
        cmake_lists = f"""cmake_minimum_required(VERSION 3.8)
project({robot_id}_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(gazebo_ros2_control REQUIRED)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${{PROJECT_NAME}}/
)

# Install mesh files
install(DIRECTORY
  meshes
  DESTINATION share/${{PROJECT_NAME}}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${{PROJECT_NAME}}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()"""
        
        with open(desc_package_dir / "CMakeLists.txt", "w") as f:
            f.write(cmake_lists)
        
        # Create urdf directory and copy URDF file
        urdf_dir = desc_package_dir / "urdf"
        urdf_dir.mkdir(exist_ok=True)
        
        import shutil
        urdf_dest = urdf_dir / f"{robot_id}.urdf"
        shutil.copy2(urdf_path, urdf_dest)
        
        # Update URDF mesh paths to use package:// format
        self._update_urdf_mesh_paths(urdf_dest, robot_id)
        
        # Create meshes directory
        (desc_package_dir / "meshes").mkdir(exist_ok=True)
        
        # Create config directory and controller configuration
        config_dir = desc_package_dir / "config"
        config_dir.mkdir(exist_ok=True)
        
        # Create controller configuration file
        joints = self._get_joints_from_urdf(urdf_path)
        if not joints:
            # Fallback joints if none found
            joints = ["joint1", "joint2", "joint3"]
        
        controller_config = f"""controller_manager:
  ros__parameters:
    update_rate: 50  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
      joints:
{chr(10).join([f"        - {joint}" for joint in joints])}
      command_interfaces:
        - position
      state_interfaces:
        - position
        - velocity
      action_monitor_rate: 20.0
      allow_partial_joints_goal: true
      open_loop_control: true
      constraints:
        stopped_velocity_tolerance: 0.01
        goal_time: 0.6
        joint:
{chr(10).join([f"          {joint}:" for joint in joints])}
{chr(10).join([f"            goal: 0.1" for joint in joints])}
"""
        
        with open(config_dir / f"{robot_id}_controllers.yaml", 'w') as f:
            f.write(controller_config)
        
        logger.info(f"Created robot description package for {robot_id}")
    
    def _update_urdf_mesh_paths(self, urdf_path: Path, robot_id: str) -> None:
        """Update URDF mesh paths to use package:// format"""
        try:
            with open(urdf_path, 'r') as f:
                urdf_content = f.read()
            
            # Update mesh paths to use package:// format
            import re
            
            # First, handle existing package:// paths (most common case)
            # Replace any package://old_package_name/meshes/ with package://robot_id_description/meshes/
            urdf_content = re.sub(
                r'filename="package://[^/]+/meshes/([^"]*)"',
                rf'filename="package://{robot_id}_description/meshes/\1"',
                urdf_content
            )
            
            # Then handle relative paths (but only if they don't already have package://)
            urdf_content = re.sub(
                r'filename="(?!package://)([^"]*\.(stl|dae|obj))"',
                rf'filename="package://{robot_id}_description/meshes/\1"',
                urdf_content
            )
            
            with open(urdf_path, 'w') as f:
                f.write(urdf_content)
                
            logger.info(f"Updated URDF mesh paths for {robot_id}")
            
        except Exception as e:
            logger.warning(f"Failed to update URDF mesh paths: {e}")
    
    def _get_joints_from_urdf(self, urdf_path: str) -> List[str]:
        """Extract joint names from URDF file"""
        try:
            import xml.etree.ElementTree as ET
            with open(urdf_path, 'r') as f:
                urdf_content = f.read()
            
            root = ET.fromstring(urdf_content)
            joints = []
            for joint in root.findall('.//joint'):
                joint_name = joint.get('name')
                joint_type = joint.get('type')
                if joint_name and joint_type and joint_type in ['revolute', 'continuous', 'prismatic']:
                    joints.append(joint_name)
            return joints
        except Exception as e:
            logger.warning(f"Failed to extract joints from URDF: {e}")
            return []
    
    def _collect_mesh_files(self, urdf_path: str, workspace_path: Path, robot_id: str) -> List[str]:
        """Extract mesh file paths from URDF and copy them to workspace"""
        mesh_files = []
        urdf_dir = Path(urdf_path).parent
        
        try:
            import xml.etree.ElementTree as ET
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            
            # Create meshes directory in description package
            meshes_dir = workspace_path / "src" / f"{robot_id}_description" / "meshes"
            meshes_dir.mkdir(parents=True, exist_ok=True)
            
            # Find all mesh references
            for mesh in root.findall('.//mesh'):
                filename = mesh.get('filename')
                if filename:
                    # Convert package:// URIs to actual file paths
                    if filename.startswith('package://'):
                        # Extract package name and relative path
                        parts = filename.replace('package://', '').split('/', 1)
                        if len(parts) == 2:
                            package_name, rel_path = parts
                            # Try to find the mesh file in the source directory
                            source_mesh = urdf_dir / rel_path
                            if source_mesh.exists():
                                # Copy mesh to workspace
                                dest_mesh = meshes_dir / source_mesh.name
                                import shutil
                                shutil.copy2(source_mesh, dest_mesh)
                                mesh_files.append(str(dest_mesh))
                                logger.info(f"Copied mesh: {source_mesh} -> {dest_mesh}")
                            else:
                                logger.warning(f"Mesh file not found: {source_mesh}")
                    else:
                        # Direct file path - try relative to URDF
                        source_mesh = urdf_dir / filename
                        if source_mesh.exists():
                            # Copy mesh to workspace
                            dest_mesh = meshes_dir / source_mesh.name
                            import shutil
                            shutil.copy2(source_mesh, dest_mesh)
                            mesh_files.append(str(dest_mesh))
                            logger.info(f"Copied mesh: {source_mesh} -> {dest_mesh}")
                        else:
                            logger.warning(f"Mesh file not found: {source_mesh}")
            
            # Write mesh info file
            mesh_info = {
                'robot_id': robot_id,
                'mesh_files': mesh_files,
                'instructions': [
                    "Mesh files have been copied to the description package",
                    "If you still see boxes in Gazebo, check:",
                    "1. URDF mesh paths point to package://robot_description/meshes/filename",
                    "2. The description package is built: colcon build --packages-select robot_description",
                    "3. Gazebo can find the package: source install/setup.bash"
                ]
            }
            
            with open(meshes_dir / "mesh_info.json", "w") as f:
                json.dump(mesh_info, f, indent=2)
                
        except Exception as e:
            logger.warning(f"Could not process mesh files: {e}")
            
        return mesh_files
    
    def _create_workspace_readme(self, workspace_path: Path, robot_id: str, robot_type: str, rcm: Dict[str, Any], mesh_files: List[str]) -> None:
        """Create comprehensive README for the workspace"""
        
        # Extract robot capabilities from RCM
        capabilities = []
        if rcm.get("has_gripper", False):
            capabilities.append(f"Gripper: {rcm.get('gripper_type', 'unknown')}")
        if rcm.get("locomotion", {}).get("type"):
            capabilities.append(f"Locomotion: {rcm['locomotion']['type']}")
        if rcm.get("sensors"):
            sensor_types = [s.get("type", "unknown") for s in rcm["sensors"]]
            capabilities.append(f"Sensors: {', '.join(set(sensor_types))}")
        
        # Generate primitives summary
        primitives = rcm.get("primitives", [])
        primitive_descriptions = []
        for prim in primitives[:5]:  # Show first 5 primitives
            primitive_descriptions.append(f"- **{prim.get('primitive_id', 'unknown')}**: {prim.get('signature', 'no signature')}")
        
        # Check if MoveIt is available (not used in current implementation)
        has_moveit = False
        
        # Generate joint information
        joints = rcm.get("joints", {})
        joint_names = list(joints.keys())
        joint_info = []
        for joint_name, joint_data in list(joints.items())[:5]:  # Show first 5 joints
            joint_type = joint_data.get("type", "unknown")
            joint_info.append(f"- **{joint_name}**: {joint_type}")
        
        # Generate topic information
        topics_info = [
            "## 📡 ROS 2 Topics",
            "",
            "### Core Topics",
            "- `/robot_description` - Robot URDF description",
            "- `/joint_states` - Current joint positions and velocities",
            "- `/tf` - Transform tree for robot links",
            "- `/tf_static` - Static transforms",
            "",
            "### Robot-Specific Topics",
        ]
        
        
        if rcm.get("has_gripper", False):
            topics_info.extend([
                "- `/gripper_action` - Gripper control actions",
                "- `/gripper_state` - Current gripper state",
            ])
        
        topics_info.extend([
            "",
            "### Controller Topics",
            "- `/joint_trajectory_controller/follow_joint_trajectory` - Joint trajectory action",
            "- `/joint_trajectory_controller/state` - Controller state",
            "- `/joint_trajectory_controller/joint_trajectory` - Trajectory commands",
            "- `/controller_manager/list_controllers` - Available controllers",
        ])
        
        # Generate mesh instructions
        mesh_instructions = []
        if mesh_files:
            mesh_instructions.append("### 🎨 Adding Visual Meshes (Optional)")
            mesh_instructions.append("To get realistic robot visualization:")
            mesh_instructions.append("1. Copy your robot's mesh files (.stl, .dae, .obj) to the `src/{robot_id}_description/meshes/` directory")
            mesh_instructions.append("2. Update the URDF file to use relative paths: `meshes/robot.stl`")
            mesh_instructions.append("3. Rebuild and relaunch the workspace")
            mesh_instructions.append("")
            mesh_instructions.append("**Current mesh references found:**")
            for mesh in mesh_files[:3]:  # Show first 3
                if isinstance(mesh, dict):
                    mesh_instructions.append(f"- {mesh.get('original_path', 'unknown')}")
        else:
            mesh_instructions.append("### 🎨 Visual Representation")
            mesh_instructions.append("No mesh files detected. The robot will appear as basic geometric shapes in RViz.")
            mesh_instructions.append("This is perfectly fine for development and testing!")
        
        readme_content = f"""# {robot_id.title()} Robot Workspace

> **Generated by RobotLab Cloud** - Complete ROS 2 workspace with RCM integration

## 🚀 Quick Start

### Prerequisites
- **ROS 2 Humble**: Install ROS 2 Humble on Ubuntu 22.04
- **Docker** (optional): For containerized deployment (works on Windows, Mac, Linux)

### 1. Build the Workspace

#### Local Build
```bash
# Navigate to workspace
cd {workspace_path.name}

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

#### Docker Build (Recommended for Windows/Mac)
```bash
# Build and run with Docker
docker-compose up --build

# Or build manually
docker build -t {robot_id}_robot .
docker run -it --rm \\
  --network host \\
  --env DISPLAY=$DISPLAY \\
  --volume /tmp/.X11-unix:/tmp/.X11-unix \\
  {robot_id}_robot
```

### 2. Launch Options

#### Basic Visualization (No RViz)
```bash
ros2 launch {robot_id}_launch {robot_id}_basic.launch.py
```
**What this does:**
- Launches robot state publisher
- Starts joint state publisher GUI for manual control
- **No RViz** - just the core robot nodes
- Perfect for headless operation or when you want to control RViz separately

#### RCM Integration (LLM Control)
```bash
ros2 launch {robot_id}_launch {robot_id}_rcm.launch.py
```
**What this does:**
- Launches robot state publisher
- Starts joint state publisher GUI for manual control
- Starts RCM bridge for LLM integration
- **No RViz** - enables programmatic robot control via RCM tools

#### RViz Only (Pre-configured)
```bash
ros2 launch {robot_id}_launch {robot_id}_rviz.launch.py
```
**What this does:**
- Launches robot state publisher
- Starts joint state publisher GUI for manual control
- Launches RViz with pre-configured robot visualization
- Perfect for visualization and debugging

#### Simulation (RViz + Joint Control)
```bash
ros2 launch {robot_id}_launch {robot_id}_simulation.launch.py
```
**What this does:**
- Launches robot state publisher
- Starts joint state publisher GUI for manual control
- Launches RViz with pre-configured robot visualization
- Includes launch argument to disable RViz if needed
- Perfect for testing robot visualization and joint control

## 🐳 Docker Usage (Cross-Platform)

### Windows/Mac/Linux
```bash
# Build and run with Docker (includes GUI support)
docker-compose up --build

# Access container shell
docker-compose exec {robot_id}_robot bash

# Run specific launch file
docker-compose exec {robot_id}_robot bash -c "source install/setup.bash && ros2 launch {robot_id}_launch {robot_id}_simulation.launch.py"
```

## 🤖 Robot Information

- **Robot ID**: `{robot_id}`
- **Type**: {robot_type}
- **Capabilities**: {', '.join(capabilities) if capabilities else 'Basic robot with kinematic model'}
- **Generated**: {time.strftime('%Y-%m-%d %H:%M:%S')}

### Robot Joints
{chr(10).join(joint_info) if joint_info else "- **No joint information available**"}

## 📁 Workspace Structure

```
{workspace_path.name}/
├── src/                              # ROS 2 packages
│   ├── {robot_id}_description/            # Robot description package
│   │   ├── urdf/
│   │   │   └── {robot_id}.urdf           # Robot kinematic description
│   │   └── meshes/                  # Visual mesh files
│   ├── {robot_id}_launch/                # Launch files package
│   │   └── launch/
│   │       ├── {robot_id}_basic.launch.py      # Basic robot visualization
│   │       ├── {robot_id}_rcm.launch.py        # RCM integration
│   │       ├── {robot_id}_rviz.launch.py       # RViz visualization
│   │       └── {robot_id}_simulation.launch.py # Simulation
│   ├── {robot_id}_rcm/                   # RCM configuration package
│   │   └── rcm/
│   │       └── {robot_id}_rcm.json       # Robot capability manifest
│   └── {robot_id}_rviz/                  # RViz configuration package
│       └── rviz/
│           └── {robot_id}_config.rviz    # RViz visualization config
├── Dockerfile                       # Container configuration
├── docker-compose.yml              # Multi-container setup
└── README.md                       # This file
```

## 🔧 Usage Examples

### Manual Joint Control
```bash
# Build and launch
colcon build --symlink-install
source install/setup.bash
ros2 launch {robot_id}_launch {robot_id}_simulation.launch.py

# In RViz:
# 1. Use Joint State Publisher GUI panel to move joints
# 2. Adjust joint sliders to see robot movement
# 3. Use Interactive Markers for 3D manipulation
```

### Programmatic Control via RCM
```python
# Launch RCM integration first
# ros2 launch {robot_id}_launch {robot_id}_rcm.launch.py

# Then use RCM tools
from rcm_tools import move_joint, get_robot_state, plan_motion

# Move individual joints
move_joint("joint_1", 1.57)  # Move to 90 degrees
move_joint("joint_2", -0.5)  # Move to -0.5 radians

# Get current robot state
state = get_robot_state()
print(f"Current joint positions: {{state['joint_positions']}}")
```

### Monitor Robot State
```bash
# Monitor joint states
ros2 topic echo /joint_states

# List all topics
ros2 topic list

# View transform tree
ros2 run tf2_tools view_frames
```
{chr(10).join(mesh_instructions)}

## 🆘 Troubleshooting

### Common Issues
- **RViz not opening**: Make sure X11 forwarding is enabled for Docker
- **Joint State Publisher GUI not showing**: Check that `joint_state_publisher_gui` package is installed
- **Robot not visible**: Verify URDF is valid and mesh paths are correct

### Getting Help
- **RobotLab Cloud Documentation**: [Add your docs link]
- **ROS 2 Tutorials**: https://docs.ros.org/en/humble/Tutorials.html
- **RCM Framework**: See your existing RCM documentation

---

**Generated by RobotLab Cloud** - Making robot development accessible to everyone! 🤖✨
"""
        
        readme_path = workspace_path / "README.md"
        with open(readme_path, "w") as f:
            f.write(readme_content)
    
    def _create_build_script(self, workspace_path: Path, robot_id: str) -> None:
        """Create a build script to make workspace setup easier"""
        
        build_script = f"""#!/bin/bash
# Build script for {robot_id} workspace
# This script builds the workspace and sources it for immediate use

set -e  # Exit on any error

echo "🔨 Building {robot_id} workspace..."

# Build the workspace
colcon build --symlink-install

if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    echo ""
    echo "📦 Sourcing workspace..."
    source install/setup.bash
    echo "✅ Workspace sourced!"
    echo ""
    echo "🚀 Ready to launch! Try:"
    echo "  ros2 launch {robot_id}_launch {robot_id}_simulation.launch.py"
    echo "  ros2 launch {robot_id}_launch {robot_id}_basic.launch.py"
else
    echo "❌ Build failed! Check the error messages above."
    exit 1
fi
"""
        
        # Write build script
        build_script_path = workspace_path / "build_workspace.sh"
        with open(build_script_path, "w") as f:
            f.write(build_script)
        
        # Make it executable
        import os
        os.chmod(build_script_path, 0o755)
    
    def _create_workspace_manifest(self, workspace_path: Path, robot_id: str, 
                                  rcm: Dict[str, Any], launch_files: List[str], config_files: List[str]):
        """Create workspace manifest for version control"""
        
        manifest = {
            "robot_id": robot_id,
            "generated_by": "RobotLab Cloud",
            "generation_timestamp": time.time(),
            "workspace_version": "1.0.0",
            "rcm": rcm,
            "launch_files": launch_files,
            "config_files": config_files,
            "dependencies": {
                "ros_distro": "humble",
                "python_version": "3.10",
                "required_packages": []
            },
            "usage_instructions": {
                "local_setup": [
                    f"cd {workspace_path}",
                    "source /opt/ros/humble/setup.bash",
                    "colcon build",
                    "source install/setup.bash",
                    f"ros2 launch {robot_id}_launch {robot_id}_basic.launch.py"
                ],
                "docker_setup": [
                    f"cd {workspace_path}",
                    "docker-compose up --build"
                ]
            }
        }
        
        manifest_path = workspace_path / "workspace_manifest.json"
        with open(manifest_path, "w") as f:
            json.dump(manifest, f, indent=2)

# Example usage
def main():
    generator = RobotConfigGenerator()
    
    # Generate workspace from URDF
    workspace = generator.generate_workspace(
        urdf_path="turtlebot3_burger.urdf",
        robot_id="my_turtlebot",
        robot_type="turtlebot3"
    )
    
    print(f"Generated workspace at: {workspace.workspace_path}")
    print(f"Launch files: {workspace.launch_files}")
    print(f"Config files: {workspace.config_files}")

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
RCM Server - FastAPI service that serves Robot Capability Manifests
"""

import json
import logging
import shutil
from pathlib import Path
from typing import Dict, Any, Optional, List
import tempfile
import zipfile
import time

from fastapi import FastAPI, HTTPException, UploadFile, File, Form
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

from robot_config_generator import RobotConfigGenerator

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RCMServer:
    def __init__(self):
        self.app = FastAPI(
            title="RobotLab Cloud - RCM Server",
            description="Robot Capability Manifest Server with Workspace Generation",
            version="2.0.0"
        )
        
        # Enable CORS for web clients
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        
        # Mount static files for web UI
        self.app.mount("/static", StaticFiles(directory="."), name="static")
        
        # In-memory storage for RCMs
        self.rcms: Dict[str, Dict[str, Any]] = {}
        
        # Initialize robot config generator
        self.config_generator = RobotConfigGenerator()
        
        # Storage for generated workspaces metadata
        self.generated_workspaces: Dict[str, Dict[str, Any]] = {}
        
        # Check dependencies
        self._check_dependencies()
        
        # Setup routes
        self._setup_routes()
    
    def _check_dependencies(self):
        """Check if required dependencies are available"""
        try:
            import pinocchio
            logger.info(f"Pinocchio version: {pinocchio.__version__}")
        except ImportError as e:
            logger.error(f"Pinocchio not available: {e}")
            logger.error("Please ensure pin_env conda environment is activated")
        
        try:
            from urdf_to_rcm import build_rcm_with_pinocchio
            logger.info("URDF to RCM converter available")
        except ImportError as e:
            logger.error(f"URDF to RCM converter not available: {e}")
    
    def _setup_routes(self):
        @self.app.get("/")
        async def root():
            return {
                "message": "RobotLab Cloud Server is running", 
                "robots": list(self.rcms.keys()),
                "generated_workspaces": list(self.generated_workspaces.keys()),
                "features": [
                    "RCM Management",
                    "Robot Workspace Generation",
                    "URDF to RCM Conversion",
                    "Docker Containerization",
                    "One-click Deployment"
                ],
                "web_ui": "http://localhost:8000/ui"
            }
        
        @self.app.get("/ui")
        async def web_ui():
            """Serve the web UI"""
            return FileResponse("robotlab_web_ui.html")
        
        @self.app.get("/robots")
        async def list_robots():
            """List all available robots"""
            return {"robots": list(self.rcms.keys())}
        
        @self.app.get("/robots/{robot_id}/rcm")
        async def get_rcm(robot_id: str):
            """Get complete RCM for a robot"""
            if robot_id not in self.rcms:
                raise HTTPException(status_code=404, detail=f"Robot {robot_id} not found")
            return self.rcms[robot_id]
        
        @self.app.get("/robots/{robot_id}/capabilities")
        async def get_capabilities(robot_id: str):
            """Get just the capabilities/primitives for a robot"""
            if robot_id not in self.rcms:
                raise HTTPException(status_code=404, detail=f"Robot {robot_id} not found")
            
            rcm = self.rcms[robot_id]
            return {
                "robot_id": robot_id,
                "primitives": rcm.get("primitives", []),
                "locomotion": rcm.get("locomotion", {}),
                "end_effectors": rcm.get("end_effectors", []),
                "has_gripper": rcm.get("has_gripper", False),
                "gripper_type": rcm.get("gripper_type", "none"),
                "joints": rcm.get("joints", {}),
                "sensors": rcm.get("sensors", [])
            }
        
        @self.app.get("/robots/{robot_id}/constraints")
        async def get_constraints(robot_id: str):
            """Get safety constraints for a robot"""
            if robot_id not in self.rcms:
                raise HTTPException(status_code=404, detail=f"Robot {robot_id} not found")
            
            rcm = self.rcms[robot_id]
            constraints = {}
            
            # Joint limits
            joints = rcm.get("joints", {})
            for joint_name, joint_data in joints.items():
                limits = joint_data.get("limits", {})
                if any(v is not None for v in limits.values()):
                    constraints[f"joint_{joint_name}"] = limits
            
            # Dynamics constraints
            dynamics = rcm.get("dynamics", {})
            if dynamics.get("mass_kg"):
                constraints["payload_capacity_kg"] = max(0, dynamics["mass_kg"] * 0.1)  # 10% of robot mass
            
            # Safety profile
            safety = rcm.get("safety_profile", {})
            constraints["safety"] = safety
            
            return {"robot_id": robot_id, "constraints": constraints}
        
        @self.app.post("/robots/{robot_id}/rcm")
        async def upload_rcm(robot_id: str, rcm: Dict[str, Any]):
            """Upload/update RCM for a robot"""
            self.rcms[robot_id] = rcm
            logger.info(f"Updated RCM for robot: {robot_id}")
            return {"message": f"RCM updated for robot {robot_id}"}
        
        @self.app.delete("/robots/{robot_id}")
        async def delete_robot(robot_id: str):
            """Remove a robot's RCM"""
            if robot_id not in self.rcms:
                raise HTTPException(status_code=404, detail=f"Robot {robot_id} not found")
            
            del self.rcms[robot_id]
            return {"message": f"Robot {robot_id} removed"}
        
        # New endpoints for robot workspace generation
        @self.app.get("/workspaces")
        async def list_workspaces():
            """List all generated workspaces"""
            return {"workspaces": list(self.generated_workspaces.keys())}
        
        @self.app.get("/workspaces/{robot_id}")
        async def get_workspace_info(robot_id: str):
            """Get information about a generated workspace"""
            if robot_id not in self.generated_workspaces:
                raise HTTPException(status_code=404, detail=f"Workspace for robot {robot_id} not found")
            return self.generated_workspaces[robot_id]
        
        @self.app.post("/robots/{robot_id}/generate-workspace")
        async def generate_robot_workspace(
            robot_id: str,
            robot_file: UploadFile = File(...),
            upload_type: str = Form("urdf"),
            robot_type: str = Form("custom"),
            additional_packages: str = Form(None)
        ):
            """Generate complete robot workspace from uploaded URDF or robot_description package"""
            try:
                content = await robot_file.read()
                
                if upload_type == "package":
                    # Handle robot_description package (ZIP file)
                    import zipfile
                    
                    # Create temporary directory for extraction (keep alive during entire process)
                    temp_dir = tempfile.mkdtemp()
                    temp_dir_path = Path(temp_dir)
                    
                    try:
                        # Save and extract ZIP file
                        zip_path = temp_dir_path / f"{robot_file.filename}"
                        with open(zip_path, "wb") as f:
                            f.write(content)
                        
                        # Extract ZIP file
                        with zipfile.ZipFile(zip_path, 'r') as zip_ref:
                            zip_ref.extractall(temp_dir_path)
                        
                        # Find robot_description package (look for any description package)
                        robot_desc_dir = None
                        description_packages = []
                        
                        for item in temp_dir_path.iterdir():
                            if item.is_dir() and "description" in item.name.lower():
                                description_packages.append(item)
                                # Prefer packages with "robot_description" in the name
                                if "robot_description" in item.name.lower():
                                    robot_desc_dir = item
                                    break
                        
                        # If no exact match, use the first description package found
                        if not robot_desc_dir and description_packages:
                            robot_desc_dir = description_packages[0]
                        
                        if not robot_desc_dir:
                            raise HTTPException(
                                status_code=400, 
                                detail=f"No robot description package found in uploaded ZIP file. Found directories: {[d.name for d in temp_dir_path.iterdir() if d.is_dir()]}"
                            )
                        
                        # Find URDF file in the extracted package first
                        urdf_files = list(robot_desc_dir.glob("**/*.urdf"))
                        xacro_files = list(robot_desc_dir.glob("**/*.xacro"))
                        
                        if xacro_files:
                            # Prefer xacro files
                            urdf_path = str(xacro_files[0])
                        elif urdf_files:
                            urdf_path = str(urdf_files[0])
                        else:
                            raise HTTPException(
                                status_code=400,
                                detail="No URDF or XACRO file found in robot_description package"
                            )
                        
                        # Store robot_description path for later use (will be copied during workspace generation)
                        robot_desc_path = str(robot_desc_dir)
                        
                    except Exception as e:
                        # Clean up temp directory on error
                        shutil.rmtree(temp_dir, ignore_errors=True)
                        raise
                
                else:
                    # Handle single URDF/XACRO file
                    file_ext = Path(robot_file.filename).suffix
                    
                    # Validate file content before saving
                    if not content:
                        raise HTTPException(status_code=400, detail="Empty file uploaded")
                    
                    # For URDF files, validate XML structure
                    if file_ext.lower() in ['.urdf', '.xacro']:
                        try:
                            import xml.etree.ElementTree as ET
                            root = ET.fromstring(content)
                            if root.tag != 'robot':
                                raise HTTPException(status_code=400, detail="Invalid URDF file: root element must be 'robot'")
                        except ET.ParseError as e:
                            raise HTTPException(status_code=400, detail=f"Invalid URDF file: XML parsing error - {str(e)}")
                        except Exception as e:
                            raise HTTPException(status_code=400, detail=f"Invalid file format: {str(e)}")
                    
                    with tempfile.NamedTemporaryFile(delete=False, suffix=file_ext) as tmp_file:
                        tmp_file.write(content)
                        urdf_path = tmp_file.name
                    
                    robot_desc_path = None
                
                # Parse additional packages
                extra_packages = []
                if additional_packages:
                    extra_packages = [p.strip() for p in additional_packages.split(",")]
                
                # File validation already done above
                
                # Generate workspace
                try:
                    workspace = self.config_generator.generate_workspace(
                        urdf_path=urdf_path,
                        robot_id=robot_id,
                        robot_type=robot_type,
                        additional_packages=extra_packages,
                        robot_description_path=robot_desc_path if upload_type == "package" else None
                    )
                except Exception as e:
                    # Clean up any partially created workspace
                    workspace_dir = Path(self.config_generator.output_dir) / f"{robot_id}_workspace"
                    if workspace_dir.exists():
                        shutil.rmtree(workspace_dir)
                    
                    # Clean up temporary directory if it was created
                    if upload_type == "package" and 'temp_dir' in locals():
                        shutil.rmtree(temp_dir, ignore_errors=True)
                    
                    # Log the full error for debugging
                    import traceback
                    error_traceback = traceback.format_exc()
                    logger.error(f"Workspace generation failed for {robot_id}: {str(e)}")
                    logger.error(f"Full traceback: {error_traceback}")
                    
                    # Provide more detailed error information to user
                    error_detail = str(e) if str(e) else "Unknown error occurred"
                    logger.error(f"Detailed error: {error_detail}")
                    logger.error(f"Error type: {type(e).__name__}")
                    
                    if "No module named 'pinocchio'" in error_detail:
                        error_detail = "Pinocchio library not found. Please ensure the pin_env conda environment is activated and Pinocchio is installed."
                    elif "FileNotFoundError" in error_detail:
                        error_detail = f"File not found: {error_detail}"
                    elif "URDF" in error_detail and "parse" in error_detail.lower():
                        error_detail = f"URDF parsing error: {error_detail}"
                    elif "PermissionError" in error_detail:
                        error_detail = f"Permission error: {error_detail}"
                    elif "does not contain a valid URDF model" in error_detail:
                        error_detail = f"Invalid URDF file: {error_detail}"
                        raise HTTPException(status_code=400, detail=f"Invalid URDF file: {error_detail}")
                    elif "URDF" in error_detail and ("parse" in error_detail.lower() or "model" in error_detail.lower()):
                        error_detail = f"URDF parsing error: {error_detail}"
                        raise HTTPException(status_code=400, detail=f"URDF parsing error: {error_detail}")
                    elif not error_detail or error_detail == "Unknown error occurred":
                        error_detail = f"Workspace generation failed: {type(e).__name__} - {error_detail}"
                    
                    raise HTTPException(status_code=500, detail=f"Workspace generation failed: {error_detail}")
                
                finally:
                    # Clean up temporary directory after successful generation
                    if upload_type == "package" and 'temp_dir' in locals():
                        shutil.rmtree(temp_dir, ignore_errors=True)
                
                # Package workspace for download
                zip_path = self.config_generator.package_workspace(
                    workspace.workspace_path, 
                    output_format="zip"
                )
                
                # Store workspace metadata
                self.generated_workspaces[robot_id] = {
                    "robot_id": robot_id,
                    "robot_type": robot_type,
                    "generated_at": time.time(),
                    "workspace_path": workspace.workspace_path,
                    "zip_path": zip_path,
                    "launch_files": workspace.launch_files,
                    "config_files": workspace.config_files,
                    "docker_config": workspace.docker_config,
                    "rcm": workspace.rcm,
                    "additional_packages": extra_packages
                }
                
                # Store RCM in robot registry
                self.rcms[robot_id] = workspace.rcm
                
                logger.info(f"Generated workspace for robot {robot_id} at {workspace.workspace_path}")
                
                return {
                    "status": "success",
                    "message": f"Workspace generated successfully for robot {robot_id}",
                    "robot_id": robot_id,
                    "workspace_path": workspace.workspace_path,
                    "download_url": f"/workspaces/{robot_id}/download",
                    "launch_files": workspace.launch_files,
                    "config_files": workspace.config_files,
                    "docker_ready": True
                }
                
            except Exception as e:
                logger.error(f"Failed to generate workspace for robot {robot_id}: {e}")
                raise HTTPException(status_code=500, detail=f"Workspace generation failed: {str(e)}")
        
        @self.app.get("/workspaces/{robot_id}/download")
        async def download_workspace(robot_id: str):
            """Download generated workspace as ZIP file"""
            if robot_id not in self.generated_workspaces:
                raise HTTPException(status_code=404, detail=f"Workspace for robot {robot_id} not found")
            
            workspace_info = self.generated_workspaces[robot_id]
            zip_path = workspace_info["zip_path"]
            
            if not Path(zip_path).exists():
                raise HTTPException(status_code=404, detail="Workspace ZIP file not found")
            
            return FileResponse(
                path=zip_path,
                filename=f"{robot_id}_workspace.zip",
                media_type="application/zip"
            )
        
        @self.app.get("/robot-templates")
        async def get_robot_templates():
            """Get available robot templates for workspace generation"""
            return {
                "templates": self.config_generator.robot_templates,
                "description": "Available robot templates for auto-configuration"
            }
        
        @self.app.delete("/workspaces/{robot_id}")
        async def delete_workspace(robot_id: str):
            """Delete a generated workspace"""
            if robot_id not in self.generated_workspaces:
                raise HTTPException(status_code=404, detail=f"Workspace for robot {robot_id} not found")
            
            workspace_info = self.generated_workspaces[robot_id]
            
            # Remove files
            try:
                import shutil
                if Path(workspace_info["workspace_path"]).exists():
                    shutil.rmtree(workspace_info["workspace_path"])
                if Path(workspace_info["zip_path"]).exists():
                    Path(workspace_info["zip_path"]).unlink()
            except Exception as e:
                logger.warning(f"Failed to delete workspace files: {e}")
            
            # Remove from registry
            del self.generated_workspaces[robot_id]
            if robot_id in self.rcms:
                del self.rcms[robot_id]
            
            return {"message": f"Workspace for robot {robot_id} deleted successfully"}
    
    def load_rcm_from_file(self, robot_id: str, rcm_path: str):
        """Load RCM from JSON file"""
        try:
            with open(rcm_path, 'r') as f:
                rcm = json.load(f)
            self.rcms[robot_id] = rcm
            logger.info(f"Loaded RCM for robot {robot_id} from {rcm_path}")
        except Exception as e:
            logger.error(f"Failed to load RCM from {rcm_path}: {e}")
            raise
    
    def run(self, host: str = "0.0.0.0", port: int = 8000):
        """Run the server"""
        logger.info(f"Starting RobotLab Cloud server on {host}:{port}")
        logger.info("Available features:")
        logger.info("  - RCM Management")
        logger.info("  - Robot Workspace Generation")
        logger.info("  - URDF to RCM Conversion")
        logger.info("  - Docker Containerization")
        logger.info("  - One-click Deployment")
        uvicorn.run(self.app, host=host, port=port)

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="RobotLab Cloud - RCM Server with Workspace Generation")
    parser.add_argument("--host", default="0.0.0.0", help="Host to bind to")
    parser.add_argument("--port", type=int, default=8000, help="Port to bind to")
    parser.add_argument("--rcm", help="Path to RCM JSON file to load")
    parser.add_argument("--robot-id", default="default", help="Robot ID for loaded RCM")
    
    args = parser.parse_args()
    
    server = RCMServer()
    
    # Load RCM if provided
    if args.rcm:
        server.load_rcm_from_file(args.robot_id, args.rcm)
    
    server.run(host=args.host, port=args.port)

if __name__ == "__main__":
    main()

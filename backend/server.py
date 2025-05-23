import os
import asyncio
import subprocess
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode

# Initialize FastAPI
app = FastAPI()

# Enable CORS for development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Serve UI files from /static
@app.get("/")
async def root():
    return FileResponse("static/index.html")

app.mount("/static", StaticFiles(directory="static"), name="static")

# Initialize ROS node
rclpy.init()

class MAVROSControlNode(Node):
    def __init__(self):
        super().__init__('mavros_control_panel')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.get_logger().info("Waiting for services...")
        self.arm_client.wait_for_service()
        self.mode_client.wait_for_service()

node = MAVROSControlNode()
launch_processes = {}

# Arm endpoint
@app.post("/arm")
async def arm():
    req = CommandBool.Request()
    req.value = True
    future = node.arm_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    if not result:
        raise HTTPException(status_code=500, detail="Failed to arm")
    return {"success": result.success}

# Disarm endpoint
@app.post("/disarm")
async def disarm():
    req = CommandBool.Request()
    req.value = False
    future = node.arm_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    if not result:
        raise HTTPException(status_code=500, detail="Failed to disarm")
    return {"success": result.success}

# Set mode endpoint
@app.post("/mode/{mode_name}")
async def set_mode(mode_name: str):
    req = SetMode.Request()
    req.custom_mode = mode_name.upper()
    future = node.mode_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    if not result:
        raise HTTPException(status_code=500, detail="Failed to set mode")
    return {"mode_set": result.mode_sent}

# Launch MAVROS
@app.post("/launch/mavros")
async def launch_mavros():
    key = 'mavros'
    if key in launch_processes and launch_processes[key].poll() is None:
        raise HTTPException(status_code=400, detail="MAVROS already running")
    cmd = ['ros2', 'launch', 'mavros', 'apm.launch', f"fcu_url:={os.getenv('FCU_URL', '/dev/ttyACM1:57600')}"]
    proc = subprocess.Popen(cmd)
    launch_processes[key] = proc
    return {"launched": key, "pid": proc.pid}

# Launch AGV bringup
@app.post("/launch/bringup")
async def launch_bringup():
    key = 'bringup'
    if key in launch_processes and launch_processes[key].poll() is None:
        raise HTTPException(status_code=400, detail="Bringup already running")
    cmd = ['ros2', 'launch', 'agv_bringup', 'AGV.launch.xml']
    proc = subprocess.Popen(cmd)
    launch_processes[key] = proc
    return {"launched": key, "pid": proc.pid}

# Stop service
@app.post("/stop/{service}")
async def stop_service(service: str):
    if service not in launch_processes or launch_processes[service].poll() is not None:
        raise HTTPException(status_code=400, detail=f"{service} not running")
    proc = launch_processes.pop(service)
    proc.terminate()
    return {"stopped": service}

# Graceful shutdown of rclpy
@app.on_event("shutdown")
def shutdown_event():
    rclpy.shutdown()

# If running directly
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)

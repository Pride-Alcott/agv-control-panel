import subprocess
from fastapi import FastAPI, HTTPException
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode

app = FastAPI()
rclpy.init()

class MAVROSControlNode(Node):
    def __init__(self):
        super().__init__('mavros_control_panel')
        # Clients for arming/disarming and mode setting
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Wait for the services to become available
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/cmd/arming...')
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/set_mode...')

node = MAVROSControlNode()
launch_processes = {}

@app.post("/arm")
async def arm():
    """
    Arms the vehicle.
    Equivalent to:
    ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
    """
    req = CommandBool.Request()
    req.value = True
    future = node.arm_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return {"success": future.result().success}

@app.post("/disarm")
async def disarm():
    """
    Disarms the vehicle.
    Equivalent to:
    ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
    """
    req = CommandBool.Request()
    req.value = False
    future = node.arm_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return {"success": future.result().success}

@app.post("/mode/{mode_name}")
async def set_mode(mode_name: str):
    """
    Sets the MAVROS mode.
    Equivalent to:
    ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: '<MODE>'}"
    Where <MODE> is e.g. 'AUTO', 'MANUAL', 'HOLD', 'RTL'
    """
    req = SetMode.Request()
    req.custom_mode = mode_name.upper()
    future = node.mode_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return {"mode_set": future.result().mode_sent}

@app.post("/launch/mavros")
async def launch_mavros():
    """
    Launches MAVROS with the APM launch file:
    ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM1:57600
    """
    key = 'mavros'
    if key in launch_processes and launch_processes[key].poll() is None:
        raise HTTPException(status_code=400, detail="MAVROS is already running")
    
    cmd = [
        'ros2', 'launch', 'mavros', 'apm.launch',
        'fcu_url:=/dev/ttyACM1:57600'
    ]
    proc = subprocess.Popen(cmd)
    launch_processes[key] = proc
    return {"launched": "mavros", "pid": proc.pid}

@app.post("/launch/bringup")
async def launch_bringup():
    """
    Launches the AGV bringup:
    ros2 launch agv_bringup AGV.launch.xml
    """
    key = 'bringup'
    if key in launch_processes and launch_processes[key].poll() is None:
        raise HTTPException(status_code=400, detail="AGV bringup is already running")
    
    cmd = ['ros2', 'launch', 'agv_bringup', 'AGV.launch.xml']
    proc = subprocess.Popen(cmd)
    launch_processes[key] = proc
    return {"launched": "agv_bringup", "pid": proc.pid}

@app.post("/stop/{service}")
async def stop_service(service: str):
    """
    Stops a running launch subprocess ('mavros' or 'bringup').
    """
    if service not in launch_processes or launch_processes[service].poll() is not None:
        raise HTTPException(status_code=400, detail=f"{service} is not running")
    
    proc = launch_processes.pop(service)
    proc.terminate()
    return {"stopped": service}

# If running this file directly
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)

from fastapi import FastAPI
from fastapi.responses import FileResponse
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandBool

app = FastAPI()

rclpy.init()
node = rclpy.create_node('agv_web_control')

mode_client = node.create_client(SetMode, '/mavros/set_mode')
arm_client = node.create_client(CommandBool, '/mavros/cmd/arming')

while not mode_client.wait_for_service(timeout_sec=1.0):
    node.get_logger().info('Waiting for /mavros/set_mode service...')
while not arm_client.wait_for_service(timeout_sec=1.0):
    node.get_logger().info('Waiting for /mavros/cmd/arming service...')

@app.get("/")
def serve_ui():
    return FileResponse("control.html")

@app.post("/set_mode/{mode}")
def set_mode(mode: str):
    req = SetMode.Request()
    req.base_mode = 0
    req.custom_mode = mode
    future = mode_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    res = future.result()
    return {"mode": mode, "success": res.mode_sent}

@app.post("/arm")
def arm():
    req = CommandBool.Request()
    req.value = True
    future = arm_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return {"status": "armed", "success": future.result().success}

@app.post("/disarm")
def disarm():
    req = CommandBool.Request()
    req.value = False
    future = arm_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return {"status": "disarmed", "success": future.result().success}

from omni.isaac.kit import SimulationApp

# Only if running outside Isaac Sim script editor
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.ros2_bridge.scripts.ros2_camera_helper import ROS2CameraHelper

# Initialize world and camera
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

camera_prim_path = "/World/Camera"

# Add a camera if not already there
world.scene.add_camera(
    prim_path=camera_prim_path,
    position=(2.0, 0.0, 1.0),
    orientation=(0.0, 0.0, 0.0, 1.0),
)

# Initialize the ROS2CameraHelper
camera_helper = ROS2CameraHelper()
camera_helper.initialize(
    camera_prim_path=camera_prim_path,
    image_type="rgb",
    image_topic="/camera/image_raw",
    camera_info_topic="/camera/camera_info",
    frame_id="camera_frame",
)

world.reset()
while simulation_app.is_running():
    world.step(render=True)

# Clean up (only if you created SimulationApp above)
simulation_app.close()

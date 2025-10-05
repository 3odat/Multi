# Sensors and Visual Camera and Map 

### Activate the environment
```
source ~/px4/bin/activate
```
### Run the codes
```
cd ~/agent
python3 multi_drone_visual_api_complete_v3.py --port 8088
python3 sensors_service_multi.py
multi_drone_map_server.py

```
Check the detection
### Detection API
```
http://localhost:8088/drone2/scene
http://localhost:8088/drone1/scene
```
### Video
```
http://localhost:8088/drone1/video.mjpg
http://localhost:8088/drone2/video.mjpg
```
### Sensors API
```
http://localhost:8001/sensors
http://localhost:8001/sensors/1
http://localhost:8001/sensors/2
```
### MAP
Multi:
```
http://localhost:8002/fleet?ids=1,2
```
Singles:
```
http://localhost:8002/map1
http://localhost:8002/map1
```
============================================================================


# Running both drones

- Drone # 1:
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4 -i 0
- Drone # 2:
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-126.22,3.86,0,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4 -i 1

============================================================================

# Sensor code for both drones

Running the sensors:
sensors_service_multi.py
python sensors_service_multi.py --vehicle id=1,url=udp://:14540,grpc=50051 --vehicle id=2,url=udp://:14541,grpc=50052 --hz 1.0 --port 8001

python sensors_service_multi.py \
  --vehicle id=1,url=udp://:14540,grpc=50051 \
  --vehicle id=2,url=udp://:14541,grpc=50052 \
  --hz 1.0 --port 8001
  
  
http://localhost:8001/sensors/1
http://localhost:8001/sensors/1

Endpoints you can hit

- http://localhost:8001/sensors/1 → Drone 1 (UDP 14540 ↔ gRPC 50051)
- http://localhost:8001/sensors/2 → Drone 2 (UDP 14541 ↔ gRPC 50052)
- http://localhost:8001/sensors → both snapshots in one JSON
- http://localhost:8001/sensors?drone=1 → single snapshot (compat-style)

This keeps your architecture clean: each drone has a dedicated MAVSDK gRPC server (System(port=...)) and a separate UDP link, and the web API exposes both neatly. If you later add more drones, just append more --vehicle flags.


-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
python3 multi_drone_visual_api_complete_v3.py --port 8088

(px4) (base) px4@UAV-Research:~/Downloads$ python3 multi_drone_visual_api_complete.py --port 8088




============================================================================
Run the drone bridge ros2 with gazebo.
cd ~/Micro-XRCE-DDS-Agent
MicroXRCEAgent udp4 -p 8888



-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

# Drone 1  (UDP 14540  -> gRPC 50051)
drone1 = System(port=50051)
await drone1.connect(system_address="udp://:14540")

# Drone 2  (UDP 14541  -> gRPC 50052)
drone2 = System(port=50052)
await drone2.connect(system_address="udp://:14541")





============================================================================
Try the endpoints

Live video:
http://localhost:8088/drone1/video.mjpg
http://localhost:8088/drone2/video.mjpg

JSON:
http://localhost:8088/drone1/scene | /drone1/history
http://localhost:8088/drone2/scene | /drone2/history

Snapshot:
http://localhost:8088/drone1/take_photo (saved as images/drone1_photo_*.jpg), same for drone2.




(px4) (base) px4@UAV-Research:~/Downloads$ python3 multi_drone_visual_api_complete.py --port 8088




============================================================================

 # Multi-Drone 
cd ~/agent/tmp
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:=$HOME/agent/tmp/bridge2.yaml

============================================================================

 # For Single drone bridge the topi
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock /camera@sensor_msgs/msg/Image@gz.msgs.Image /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked /world/default/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU

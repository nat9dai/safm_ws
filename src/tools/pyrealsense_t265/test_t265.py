
import pyrealsense2 as rs

# 1. Initialize pipeline and configuration
pipe = rs.pipeline()
cfg = rs.config()

# 2. Enable the pose stream (6DOF)
cfg.enable_stream(rs.stream.pose)

# 3. Start streaming
pipe.start(cfg)

try:
    while True:
        # Wait for the next set of frames
        frames = pipe.wait_for_frames()
        
        # Fetch the pose frame
        pose = frames.get_pose_frame()
        if pose:
            # Extract pose data
            data = pose.get_pose_data()
            
            # Translation (x, y, z) in meters
            print(f"Position: {data.translation}")
            
            # Rotation as a quaternion (x, y, z, w)
            print(f"Rotation: {data.rotation}")
            
            # Optional: Velocity and Acceleration
            # print(f"Velocity: {data.velocity}")
finally:
    pipe.stop()

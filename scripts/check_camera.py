import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy
import message_filters

class CameraDoctor(Node):
    def __init__(self):
        super().__init__('camera_doctor')
        
        # 1. Define the MoveIt-friendly Reliable QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )

        # 2. Set up subscribers
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/rgb/image_raw', qos_profile=qos)
        self.info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/rgb/camera_info', qos_profile=qos)

        # 3. The TimeSynchronizer - This ONLY triggers if timestamps match EXACTLY
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.info_sub], 10)
        self.ts.registerCallback(self.sync_callback)

        self.get_logger().info("Doctor started. Waiting for synchronized Image and CameraInfo...")
        self.get_logger().info("If nothing happens, your timestamps do NOT match.")

    def sync_callback(self, image_msg, info_msg):
        self.get_logger().info("-------------------------------------------")
        self.get_logger().info("SUCCESS: Messages are Synchronized!")
        self.get_logger().info(f"Timestamp: {image_msg.header.stamp.sec}.{image_msg.header.stamp.nanosec}")
        self.get_logger().info(f"Image Frame ID: {image_msg.header.frame_id}")
        self.get_logger().info(f"Info Frame ID:  {info_msg.header.frame_id}")
        
        if image_msg.header.frame_id != info_msg.header.frame_id:
            self.get_logger().error("ERROR: Frame IDs do not match!")
        else:
            self.get_logger().info("Frame IDs match. MoveIt should see this.")

def main():
    rclpy.init()
    node = CameraDoctor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import math

import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class RobotTFBroadcaster(Node):
    """
    Publishes dynamic and static transforms for the Crazyflie robot.
    
    Dynamic transforms: world -> robot base_link (based on current pose)
    Static transforms: base_link -> sensor frames (fixed mounting positions)
    """
    
    def __init__(self):
        super().__init__('robot_tf_broadcaster')
        
        # Transform broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Subscribe to robot pose for dynamic transforms
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/cf_1/pose',
            self.pose_callback,
            qos_profile_sensor_data
        )
        
        # Publish static transforms once
        self.publish_static_transforms()
        
        self.get_logger().info('Robot TF broadcaster initialized.')
    
    def pose_callback(self, msg: PoseStamped):
        """
        Callback for robot pose messages.
        Publishes dynamic transform from world to robot base_link.
        """
        try:
            # Create dynamic transform: world -> robot base_link
            transform = TransformStamped()
            
            transform.header.stamp = msg.header.stamp
            transform.header.frame_id = 'world'
            transform.child_frame_id = 'crazyflie_0/base_link'
            
            # Copy position
            transform.transform.translation.x = msg.pose.position.x
            transform.transform.translation.y = msg.pose.position.y
            transform.transform.translation.z = msg.pose.position.z
            
            # Copy orientation
            transform.transform.rotation = msg.pose.orientation
            
            # Broadcast the transform
            self.tf_broadcaster.sendTransform(transform)
            
        except Exception as e:
            self.get_logger().error(f'Error in pose_callback: {str(e)}')
    
    def publish_static_transforms(self):
        """
        Publishes static transforms for sensor mounting positions.
        These transforms don't change relative to the robot base.
        """
        try:
            # Static transform: base_link -> lidar_sensor  
            # From SDF: lidar sensor has pose="0 0 0.4 0 0 0" within lidar link
            # lidar link is fixed to base_link, so total offset is 0 0 0.4
            lidar_transform = TransformStamped()
            
            lidar_transform.header.stamp = self.get_clock().now().to_msg()
            lidar_transform.header.frame_id = 'crazyflie_0/base_link'
            lidar_transform.child_frame_id = 'crazyflie_0/lidar/lidar_sensor'
            
            # LiDAR sensor offset and rotation from SDF file: 
            # Translation: 0.4m above base_link
            # Rotation: 52 degrees forward tilt (0.9076 rad around Y-axis)
            lidar_transform.transform.translation.x = 0.0
            lidar_transform.transform.translation.y = 0.0
            lidar_transform.transform.translation.z = 0.4
            
            # 52-degree forward tilt: quaternion for rotation around Y-axis
            # q = [0, sin(θ/2), 0, cos(θ/2)] where θ = 0.9076 rad
            half_angle = 0.9076 / 2.0  # 52 degrees / 2 in radians
            lidar_transform.transform.rotation.x = 0.0
            lidar_transform.transform.rotation.y = math.sin(half_angle)  # ≈ 0.4383
            lidar_transform.transform.rotation.z = 0.0
            lidar_transform.transform.rotation.w = math.cos(half_angle)  # ≈ 0.8988
            
            # Broadcast static transform
            self.static_tf_broadcaster.sendTransform(lidar_transform)
            
            self.get_logger().info('Published static transforms.')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing static transforms: {str(e)}')


def main(args=None):
    """Main function to initialize and run the TF broadcaster node."""
    rclpy.init(args=args)
    
    try:
        node = RobotTFBroadcaster()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

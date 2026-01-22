import rclpy
from rclpy.node import Node
import numpy as np

# from ros2_ws.build.tutorial_interfaces.ament_cmake_python.tutorial_interfaces.tutorial_interfaces import msg
from sensor_msgs.msg import PointCloud2, PointField, Imu
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import serialize_message, deserialize_message
from builtin_interfaces.msg import Time as Ros2Time
from tutorial_interfaces.msg import CustomMsg, CustomPoint
import rosbag2_py

from std_msgs.msg import Header

LIDAR_TOPIC = "/cf_0/lidar/points"
IMU_TOPICS = ["/livox/imu", "/livox/loc"]
LIDAR_FREQ = 20.0

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        #   Declare variables to store PointCloud2 message data
        self.header = Header()
        self.timebase = 0  # Initialize to 0, will set to first timestamp
        self.point_num = 0
        self.lidar_id = 0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.reflectivity = 0
        self.tag = 0
        self.line = 0

        # Subscription to PointCloud2 messages
        self.subscription = self.create_subscription(
            PointCloud2,                                               # CHANGE
            '/cf_0/lidar/points',
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning

        # Publish the converted CustomMsg
        self.publisher_ = self.create_publisher(CustomMsg, '/converted_CustomMsg', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counters for debugging (cumulative received messages)
        self.original_lidar = 0
        self.original_imu = 0
        self.total_msgs = 0
        self.processed_msgs = 0
        self.lidar_count = 0
        self.offset_time = 0

    def timer_callback(self):
        msg = CustomMsg()  
        msg.header = self.header
        msg.timebase = int(self.timebase) #this is just t_nanosec
        msg.point_num = self.point_num
        msg.lidar_id = self.lidar_id
        msg.rsvd = [0, 0, 0]

        pt = CustomPoint()
        pt.offset_time = self.offset_time
        pt.x = self.x
        pt.y = self.y
        pt.z = self.z
        pt.reflectivity = self.reflectivity
        pt.tag = self.tag
        pt.line = self.line
        # msg.timebase = 0 # placeholder
        # msg.point_num = 0 # placeholder
        # msg.lidar_id = 0 # placeholder
        # msg.rsvd = [0, 0, 0] # placeholder
        # msg.points = [0, 0, 0, 0, 0, 0, 0] # placeholder

        # Publish the whole CustomMsg
        # self.publisher_.publish(msg)
        # self.get_logger().info(f"\nPublishing header stamp: {msg.header}")
        # self.get_logger().info(f"Publishing timebase: {msg.timebase}")
        # self.get_logger().info(f"Publishing point_num: {msg.point_num}")
        # self.get_logger().info(f"Publishing lidar_id: {msg.lidar_id}")
        # self.get_logger().info(f"Publishing rsvd: {msg.rsvd}")
        # self.get_logger().info(f"Publishing offset_time: {pt.offset_time}")
        # self.get_logger().info(f"Publishing x: {pt.x}")
        # self.get_logger().info(f"Publishing y: {pt.y}")
        # self.get_logger().info(f"Publishing z: {pt.z}")
        # self.get_logger().info(f"Publishing reflectivity: {pt.reflectivity}")
        # self.get_logger().info(f"Publishing tag: {pt.tag}")
        # self.get_logger().info(f"Publishing line: {pt.line}")
        # self.get_logger().info("\n")

    def stamp_to_float_seconds(self, stamp: Ros2Time) -> float:
        """builtin_interfaces/Time -> float seconds"""
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def float_seconds_to_time(self, tsec: float) -> Ros2Time:
        """float seconds -> builtin_interfaces/Time"""
        ros_time = Ros2Time()
        ros_time.sec = int(tsec)
        ros_time.nanosec = int((tsec - ros_time.sec) * 1e9)
        return ros_time

    def manual_parse_pc2_(self, node: Node, msg: PointCloud2):
        """
        Parse ROS 2 sensor_msgs/PointCloud2 to numpy arrays for x/y/z (+ reflectivity if present).
        Returns dict or None on failure.
        """
        try:
            # 验证必要属性（ROS 2）
            required_attrs = ['fields', 'data', 'point_step', 'width', 'height', 'header']
            for attr in required_attrs:
                if not hasattr(msg, attr):
                    node.get_logger().error(f"缺失属性：{attr}，跳过")
                    return None

            # 解析字段（映射为小写）
            field_map = {}  # key: name(lower), value: (offset, dtype, bytes)
            for field in msg.fields:
                name = field.name.lower()
                if field.datatype == PointField.FLOAT64:
                    dtype = np.float64
                    nbytes = 8
                elif field.datatype == PointField.FLOAT32:
                    dtype = np.float32
                    nbytes = 4
                elif field.datatype == PointField.UINT8:
                    dtype = np.uint8
                    nbytes = 1
                elif field.datatype == PointField.INT8:
                    dtype = np.int8
                    nbytes = 1
                elif field.datatype == PointField.UINT16:
                    dtype = np.uint16
                    nbytes = 2
                elif field.datatype == PointField.INT16:
                    dtype = np.int16
                    nbytes = 2
                elif field.datatype == PointField.UINT32:
                    dtype = np.uint32
                    nbytes = 4
                elif field.datatype == PointField.INT32:
                    dtype = np.int32
                    nbytes = 4
                else:
                    node.get_logger().warn(f"不支持字段：{field.name} (datatype={field.datatype})，跳过")
                    continue
                field_map[name] = (field.offset, dtype, nbytes)

            # 必需字段
            for req in ['x', 'y', 'z']:
                if req not in field_map:
                    node.get_logger().error(f"缺失必需字段：{req}，跳过")
                    return None

            # 点数量与二进制缓冲
            point_step = msg.point_step
            num_points = msg.width * msg.height
            if num_points == 0:
                node.get_logger().warn("空点云帧，跳过")
                return None

            data_uint8 = np.frombuffer(msg.data, dtype=np.uint8, count=num_points * point_step)
            if not data_uint8.flags['C_CONTIGUOUS']:
                data_uint8 = np.ascontiguousarray(data_uint8)

            # 提取一个字段的向量
            def extract_field(field_name: str):
                offset, dtype, nbytes = field_map[field_name]
                base = np.arange(num_points, dtype=np.int64) * point_step + offset
                # gather bytes for each point, then view as dtype
                field_bytes = data_uint8[base[:, None] + np.arange(nbytes, dtype=np.int64)]
                return field_bytes.view(dtype).flatten()

            x = extract_field('x').astype(np.float32)
            y = extract_field('y').astype(np.float32)
            z = extract_field('z').astype(np.float32)

            # 反射率（如无 intensity 则 0）
            reflectivity = np.zeros(num_points, dtype=np.uint8)
            if 'intensity' in field_map:
                intensity = extract_field('intensity')
                reflectivity = np.clip(intensity, 0, 255).astype(np.uint8)

            # 帧时间 & frame_id
            frame_stamp_sec = self.stamp_to_float_seconds(msg.header.stamp)
            frame_id = msg.header.frame_id

            return {
                'x': x, 'y': y, 'z': z,
                'reflectivity': reflectivity,
                'frame_stamp_sec': frame_stamp_sec,
                'num_points': num_points,
                'frame_id': frame_id,
            }

        except Exception as e:
            node.get_logger().error(f"解析错误：{str(e)}，跳过该帧")
            return None
        
    def calculate_offset_time(self, num_points: int) -> np.ndarray:
        """计算 offset_time（逐点相对于帧起始的微秒偏移）"""
        frame_duration_us = int(1e6 / LIDAR_FREQ)  # 一帧持续时间（微秒）
        # Use endpoint=False so last point < frame_duration
        return np.linspace(0, frame_duration_us, num_points, endpoint=False, dtype=np.uint32)
        
    def convert_to_custommsg(self, pc_data: dict) -> CustomMsg:
        """严格匹配 CustomPoint 字段（无 timestamp）"""
        custom_msg = CustomMsg()

        # header
        custom_msg.header.stamp = self.float_seconds_to_time(pc_data['frame_stamp_sec'])
        custom_msg.header.frame_id = pc_data['frame_id']

        # points
        custom_msg.points = []
        offset_times = self.calculate_offset_time(pc_data['num_points'])
        for i in range(pc_data['num_points']):
            pt = CustomPoint()
            pt.offset_time   = int(offset_times[i])      # uint32 微秒
            self.offset_time = int(offset_times[i])  # Store for publishing
            pt.x             = float(pc_data['x'][i])
            # self.x = pt.x # Store for publishing
            pt.y             = float(pc_data['y'][i])
            # self.y = pt.y # Store for publishing
            pt.z             = float(pc_data['z'][i])
            # self.z = pt.z # Store for publishing
            pt.reflectivity  = int(pc_data['reflectivity'][i])  # uint8
            self.reflectivity = pt.reflectivity # Store for publishing
            pt.tag           = 0
            self.tag = pt.tag # Store for publishing
            pt.line          = 0
            self.line = pt.line # Store for publishing
            custom_msg.points.append(pt)

        # meta
        custom_msg.lidar_id  = 1
        self.lidar_id = custom_msg.lidar_id # store for publishing
        custom_msg.point_num = int(pc_data['num_points'])
        custom_msg.timebase  = int(pc_data['frame_stamp_sec'] * 1e6)  # 微秒
        custom_msg.rsvd = [0, 0, 0]

        return custom_msg
    
    def listener_callback(self, msg):
        # self.get_logger().info(f'\n\nI heard header: {msg.header}')
        # self.get_logger().info(f'\n\nI heard header: {msg.header.stamp}')
        # ... (other commented logs)

        self.header = msg.header

        created_topics = set()

        # Topic type mapping
        # topics_and_types = reader.get_all_topics_and_types()
        # topic_type_map = {t.name: t.type for t in topics_and_types}
        topic_type_map = {LIDAR_TOPIC: "sensor_msgs/msg/PointCloud2"} # sensor_msgs/msg/PointCloud2

        # The main conversion loop
        # topic_name, serialized_data, t_nanosec = reader.read_next()
        topic_name, serialized_data = LIDAR_TOPIC, serialize_message(msg)
        t_nanosec = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.total_msgs += 1
        
        # self.get_logger().info(f"\nDEBUG I heard serialized_data: {serialized_data}")

        # self.get_logger().info(f'\nI heard timebase: {t_nanosec}') #debug
        if self.timebase == 0:  # Set timebase only once, to the first message's timestamp
            self.timebase = t_nanosec

        # Tally originals on the fly (for debugging - counts received messages)
        if topic_name == LIDAR_TOPIC:
            self.original_lidar += 1
        elif topic_name in IMU_TOPICS:
            self.original_imu += 1

        # Deserialize the message
        msg_type = topic_type_map.get(topic_name)
        if not msg_type:
            self.get_logger().warn(f"未知话题类型：{topic_name}，跳过")
            return

        py_cls = get_message(msg_type)
        msg = deserialize_message(serialized_data, py_cls)

        # self.get_logger().info(f"\nDEBUG I heard msg: {msg}\n")

        # LIDAR: convert PointCloud2 -> CustomMsg
        if topic_name == LIDAR_TOPIC:
            if not isinstance(msg, PointCloud2):
                self.get_logger().warn(f"{LIDAR_TOPIC} 实际类型为 {msg_type} 非 PointCloud2，跳过")
                return

            pc_data = self.manual_parse_pc2_(self, msg)
            if pc_data is None:
                return

            # Set point_num from the current message
            self.point_num = pc_data['num_points']

            custom_msg = self.convert_to_custommsg(pc_data)
            
            self.publisher_.publish(custom_msg)
            self.get_logger().info(f"\n\nPublished CustomMsg with {custom_msg} points\n\n")

            self.x, self.y, self.z = custom_msg.points[0].x, custom_msg.points[0].y, custom_msg.points[0].z

            # Create the output topic (same name) but CustomMsg type
            out_topic = LIDAR_TOPIC
            out_type = 'tutorial_interfaces/msg/CustomMsg'
            # if out_topic not in created_topics:
            #     writer.create_topic(
            #         rosbag2_py.TopicMetadata(
            #             name=out_topic,
            #             type=out_type,
            #             serialization_format='cdr'
            #         )
            #     )
            #     created_topics.add(out_topic)

            # writer.write(out_topic, serialize_message(custom_msg), t_nanosec)
            self.lidar_count += 1

            if self.lidar_count % 50 == 0:
                self.get_logger().info(f"已转换 {self.lidar_count} 帧点云")

        # IMU topics: pass through unchanged
        # elif topic_name in IMU_TOPICS and isinstance(msg, Imu):
        #     # Ensure topic exists in writer with original type
        #     if topic_name not in created_topics:
        #         writer.create_topic(
        #             rosbag2_py.TopicMetadata(
        #                 name=topic_name,
        #                 type=msg_type,
        #                 serialization_format='cdr'
        #             )
        #         )
        #         created_topics.add(topic_name)

        #     writer.write(topic_name, serialized_data, t_nanosec)
        #     imu_count += 1

        # Other topics: ignore by design
        self.processed_msgs += 1
        if self.processed_msgs % 1000 == 0:
            self.get_logger().info(f"已处理 {self.processed_msgs} 条消息")

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# class MinimalSubscriber(Node):

#     def __init__(self):
#         super().__init__('minimal_subscriber')
#         self.subscription = self.create_subscription(
#             CustomMsg,                                               # CHANGE
#             'topic',
#             self.listener_callback,
#             10)
#         self.subscription

#     def listener_callback(self, msg):
#         self.get_logger().info('I heard: "%d"' % msg.point_num)  # CHANGE


# def main(args=None):
#     rclpy.init(args=args)

#     minimal_subscriber = MinimalSubscriber()

#     rclpy.spin(minimal_subscriber)

#     minimal_subscriber.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
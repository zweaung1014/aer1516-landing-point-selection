#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message, deserialize_message

from builtin_interfaces.msg import Time as Ros2Time
from sensor_msgs.msg import PointCloud2, PointField, Imu
from tutorial_interfaces.msg import CustomMsg, CustomPoint
from rosidl_runtime_py.utilities import get_message

# rosbag2 (Python) API
try:
    import rosbag2_py
except Exception as e:
    print("ERROR: rosbag2_py is not available. Install ROS 2 rosbag2 python bindings.", file=sys.stderr)
    raise

# -------------------------- 配置参数 --------------------------
# Note: For rosbag2, INPUT_BAG_PATH and OUTPUT_BAG_PATH should point to directories
INPUT_BAG_PATH = "/home/zweminhtetaung/CrazySim/ros2_ws/rosbag2_2025_10_15-14_28_18"   # directory containing *.db3
OUTPUT_BAG_PATH = "/home/zweminhtetaung/CrazySim/ros2_ws/converted_bag_final"          # directory to be created
LIDAR_TOPIC = "/cf_0/lidar/points"
IMU_TOPICS = ["/livox/imu", "/livox/loc"]
LIDAR_FREQ = 20.0  # 按雷达型号调整
# ----------------------------------------------------------------


def stamp_to_float_seconds(stamp: Ros2Time) -> float:
    """builtin_interfaces/Time -> float seconds"""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def float_seconds_to_time(tsec: float) -> Ros2Time:
    """float seconds -> builtin_interfaces/Time"""
    ros_time = Ros2Time()
    ros_time.sec = int(tsec)
    ros_time.nanosec = int((tsec - ros_time.sec) * 1e9)
    return ros_time


def manual_parse_pc2(node: Node, msg: PointCloud2):
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
        frame_stamp_sec = stamp_to_float_seconds(msg.header.stamp)
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


def calculate_offset_time(num_points: int) -> np.ndarray:
    """计算 offset_time（逐点相对于帧起始的微秒偏移）"""
    frame_duration_us = int(1e6 / LIDAR_FREQ)  # 一帧持续时间（微秒）
    # Use endpoint=False so last point < frame_duration
    return np.linspace(0, frame_duration_us, num_points, endpoint=False, dtype=np.uint32)


def convert_to_custommsg(pc_data: dict) -> CustomMsg:
    """严格匹配 CustomPoint 字段（无 timestamp）"""
    custom_msg = CustomMsg()

    # header
    custom_msg.header.stamp = float_seconds_to_time(pc_data['frame_stamp_sec'])
    custom_msg.header.frame_id = pc_data['frame_id']

    # points
    custom_msg.points = []
    offset_times = calculate_offset_time(pc_data['num_points'])
    for i in range(pc_data['num_points']):
        pt = CustomPoint()
        pt.offset_time   = int(offset_times[i])      # uint32 微秒
        pt.x             = float(pc_data['x'][i])
        pt.y             = float(pc_data['y'][i])
        pt.z             = float(pc_data['z'][i])
        pt.reflectivity  = int(pc_data['reflectivity'][i])  # uint8
        pt.tag           = 0
        pt.line          = 0
        custom_msg.points.append(pt)

    # meta
    custom_msg.lidar_id  = 1
    custom_msg.point_num = int(pc_data['num_points'])
    custom_msg.timebase  = int(pc_data['frame_stamp_sec'] * 1e6)  # 微秒

    return custom_msg


def main():
    rclpy.init(args=sys.argv)
    node = Node("pc2_to_custommsg_final")
    log = node.get_logger()

    log.info(f"开始转换：{INPUT_BAG_PATH}")

    # ---- Open reader (rosbag2) ----
    storage_id = 'sqlite3'  # db3
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=INPUT_BAG_PATH, storage_id=storage_id),
        rosbag2_py.ConverterOptions('', '')  # no conversion
    )

    # Topic type mapping
    topics_and_types = reader.get_all_topics_and_types()
    # print(f"\nDEBUG PRINT, variable topics_and_types: {topics_and_types}\n")  # Debug print")
    topic_type_map = {t.name: t.type for t in topics_and_types}
    print(f"\nDEBUG PRINT, variable topic_type_map: {topic_type_map}\n")  # Debug print

    # ---- Open writer (rosbag2) ----
    if os.path.exists(OUTPUT_BAG_PATH):
        log.warn(f"输出目录已存在，将覆盖写入: {OUTPUT_BAG_PATH}")
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=OUTPUT_BAG_PATH, storage_id=storage_id),
        rosbag2_py.ConverterOptions('', '')
    )

    # We will lazily create topics on first write
    created_topics = set()

    # Counters
    total_msgs = 0
    processed_msgs = 0
    lidar_count = 0
    imu_count = 0
    original_lidar = 0
    original_imu = 0

    # (Optional) estimate original counts by one pass over metadata if available
    # Not all distros expose per-topic counts here; we’ll just tally as we go.

    # ---- Read loop ----
    while reader.has_next():
        topic_name, serialized_data, t_nanosec = reader.read_next()
        # print(f"\nDEBUG PRINT, variable topic_name: {topic_name}\n")  # Debug print
        # print(f"\nDEBUG PRINT, variable serialized_data: {serialized_data}\n")  # Debug print
        # print(f"\nDEBUG PRINT, variable t_nanosec: {t_nanosec}\n")  # Debug print
        total_msgs += 1

        # print(f"\nDEBUG PRINT, variable topic_name: {topic_name}\n")  # Debug print

        # Tally originals on the fly
        if topic_name == LIDAR_TOPIC:
            original_lidar += 1
        elif topic_name in IMU_TOPICS:
            original_imu += 1

        # Deserialize the message
        msg_type = topic_type_map.get(topic_name)
        if not msg_type:
            log.warn(f"未知话题类型：{topic_name}，跳过")
            continue

        py_cls = get_message(msg_type)
        msg = deserialize_message(serialized_data, py_cls)

        # print(f"\nDEBUG PRINT, variable msg: {msg}\n")  # Debug print

        # LIDAR: convert PointCloud2 -> CustomMsg
        if topic_name == LIDAR_TOPIC:
            if not isinstance(msg, PointCloud2):
                log.warn(f"{LIDAR_TOPIC} 实际类型为 {msg_type} 非 PointCloud2，跳过")
                continue

            pc_data = manual_parse_pc2(node, msg)
            if pc_data is None:
                continue

            custom_msg = convert_to_custommsg(pc_data)

            # Create the output topic (same name) but CustomMsg type
            out_topic = LIDAR_TOPIC
            out_type = 'tutorial_interfaces/msg/CustomMsg'
            if out_topic not in created_topics:
                writer.create_topic(
                    rosbag2_py.TopicMetadata(
                        name=out_topic,
                        type=out_type,
                        serialization_format='cdr'
                    )
                )
                created_topics.add(out_topic)

            writer.write(out_topic, serialize_message(custom_msg), t_nanosec)
            # Debug: Print key details of what's being written
            # print(f"DEBUG: Writing CustomMsg to {out_topic} at t_nanosec={t_nanosec}")
            # print(f"DEBUG: CustomMsg header: stamp={custom_msg.header.stamp}, frame_id={custom_msg.header.frame_id}")
            # print(f"DEBUG: CustomMsg meta: lidar_id={custom_msg.lidar_id}, point_num={custom_msg.point_num}, timebase={custom_msg.timebase}")
            if custom_msg.points:
                print(f"DEBUG: First point: x={custom_msg.points[0].x}, y={custom_msg.points[0].y}, z={custom_msg.points[0].z}, reflectivity={custom_msg.points[0].reflectivity}")
            lidar_count += 1

            if lidar_count % 50 == 0:
                log.info(f"已转换 {lidar_count} 帧点云")

        # IMU topics: pass through unchanged
        elif topic_name in IMU_TOPICS and isinstance(msg, Imu):
            # Ensure topic exists in writer with original type
            if topic_name not in created_topics:
                writer.create_topic(
                    rosbag2_py.TopicMetadata(
                        name=topic_name,
                        type=msg_type,
                        serialization_format='cdr'
                    )
                )
                created_topics.add(topic_name)

            writer.write(topic_name, serialized_data, t_nanosec)
            imu_count += 1

        # Other topics: ignore by design
        processed_msgs += 1
        if processed_msgs % 1000 == 0:
            log.info(f"已处理 {processed_msgs} 条消息")

    # ---- Summary ----
    log.info("=" * 60)
    log.info(f"✅ 转换完成！输出路径：{OUTPUT_BAG_PATH}")
    log.info(f"📊 点云：{lidar_count}/{original_lidar} 帧")
    log.info(f"📊 IMU：{imu_count}/{original_imu} 条")
    log.info("=" * 60)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("转换被中断")
    except Exception as e:
        print(f"❌ 致命错误：{str(e)}", file=sys.stderr)
        raise

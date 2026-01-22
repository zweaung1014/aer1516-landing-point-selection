#!/usr/bin/env python3
import rospy
import rosbag
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField, Imu
from livox_ros_driver.msg import CustomMsg, CustomPoint
 
# -------------------------- 配置参数 --------------------------
INPUT_BAG_PATH = "/home/xd/datasets/2023-11-02-14-44-03.bag"
OUTPUT_BAG_PATH = "/home/xd/datasets/converted_bag_final.bag"
LIDAR_TOPIC = "/livox/lidar"
IMU_TOPICS = ["/livox/imu", "/livox/loc"]
LIDAR_FREQ = 10.0  # 按雷达型号调整
# ----------------------------------------------------------------
 
def manual_parse_pc2(msg):
    try:
        # 验证消息类型
        if not hasattr(msg, '_type') or msg._type != 'sensor_msgs/PointCloud2':
            rospy.logerr(f"非PointCloud2（类型：{getattr(msg, '_type', '未知')}），跳过")
            return None
 
        # 验证必要属性
        required_attrs = ['fields', 'data', 'point_step', 'width', 'height', 'header']
        for attr in required_attrs:
            if not hasattr(msg, attr):
                rospy.logerr(f"缺失属性：{attr}，跳过")
                return None
 
        # 解析字段（映射为小写）
        field_map = {}  # key:小写字段名, value:(offset, dtype, 字节数)
        for field in msg.fields:
            field_lower = field.name.lower()
            if field.datatype == PointField.FLOAT64:
                dtype = np.float64
                byte_size = 8
            elif field.datatype == PointField.FLOAT32:
                dtype = np.float32
                byte_size = 4
            elif field.datatype == PointField.UINT8:
                dtype = np.uint8
                byte_size = 1
            else:
                rospy.logwarn(f"不支持字段：{field.name}，跳过")
                continue
            field_map[field_lower] = (field.offset, dtype, byte_size)
 
        # 检查必需字段（x/y/z）
        for req in ['x', 'y', 'z']:
            if req not in field_map:
                rospy.logerr(f"缺失必需字段：{req}，跳过")
                return None
 
        # 解析二进制数据（一维数组）
        point_step = msg.point_step
        num_points = msg.width * msg.height
        if num_points == 0:
            rospy.logwarn("空点云帧，跳过")
            return None
 
        data_uint8 = np.frombuffer(msg.data, dtype=np.uint8, count=num_points * point_step)
        if not data_uint8.flags['C_CONTIGUOUS']:
            data_uint8 = np.ascontiguousarray(data_uint8)
 
        # 提取x/y/z
        pc_data = {}
        for field_name in ['x', 'y', 'z']:
            offset, dtype, byte_size = field_map[field_name]
            point_indices = np.arange(num_points) * point_step + offset
            field_bytes = data_uint8[point_indices[:, None] + np.arange(byte_size)]
            pc_data[field_name] = field_bytes.view(dtype).flatten().astype(np.float32)
 
        # 提取强度（映射为reflectivity）
        pc_data['reflectivity'] = np.zeros(num_points, dtype=np.uint8)
        if 'intensity' in field_map:
            offset, dtype, byte_size = field_map['intensity']
            point_indices = np.arange(num_points) * point_step + offset
            field_bytes = data_uint8[point_indices[:, None] + np.arange(byte_size)]
            intensity_data = field_bytes.view(dtype).flatten()
            pc_data['reflectivity'] = np.clip(intensity_data, 0, 255).astype(np.uint8)
 
        # 记录帧时间和点数
        pc_data['frame_stamp_sec'] = msg.header.stamp.to_sec()
        pc_data['num_points'] = num_points
        pc_data['frame_id'] = msg.header.frame_id  # 保留原始坐标系
 
        rospy.logdebug(f"成功解析 {num_points} 点（x：{pc_data['x'].min():.2f}~{pc_data['x'].max():.2f}）")
        return pc_data
 
    except Exception as e:
        rospy.logerr(f"解析错误：{str(e)}，跳过该帧")
        return None
 
def calculate_offset_time(num_points):
    """计算offset_time（逐点相对于帧起始的微秒偏移）"""
    frame_duration_us = int(1e6 / LIDAR_FREQ)  # 一帧持续时间（微秒）
    return np.linspace(0, frame_duration_us, num_points, dtype=np.uint32)
 
def convert_to_custommsg(pc_data):
    """严格匹配CustomPoint字段（无timestamp）"""
    custom_msg = CustomMsg()
 
    # 填充header
    custom_msg.header.stamp = rospy.Time.from_sec(pc_data['frame_stamp_sec'])
    custom_msg.header.frame_id = pc_data['frame_id']  # 继承原始坐标系
 
    # 填充点数据（仅使用存在的字段）
    custom_msg.points = []
    offset_times = calculate_offset_time(pc_data['num_points'])
    for i in range(pc_data['num_points']):
        pt = CustomPoint()
        pt.offset_time = offset_times[i]    # 正确时间字段：offset_time（uint32，微秒）
        pt.x = pc_data['x'][i]              # x坐标
        pt.y = pc_data['y'][i]              # y坐标
        pt.z = pc_data['z'][i]              # z坐标
        pt.reflectivity = pc_data['reflectivity'][i]  # 反射率（替代强度）
        pt.tag = 0                          # 标签（0=正常点）
        pt.line = 0                         # 激光线号（非多线雷达填0）
        # 已删除：pt.timestamp（CustomPoint无此字段）
        custom_msg.points.append(pt)
 
    # 补充CustomMsg其他字段
    custom_msg.lidar_id = 1
    custom_msg.point_num = pc_data['num_points']
    custom_msg.timebase = int(pc_data['frame_stamp_sec'] * 1e6)  # 帧时间基准（微秒）
 
    return custom_msg
 
def main():
    rospy.init_node("pc2_to_custommsg_final", log_level=rospy.DEBUG)
    rospy.loginfo(f"开始转换：{INPUT_BAG_PATH}")
 
    lidar_count = 0
    imu_count = 0
 
    with rosbag.Bag(OUTPUT_BAG_PATH, "w") as out_bag:
        with rosbag.Bag(INPUT_BAG_PATH, "r") as in_bag:
            total_msgs = in_bag.get_message_count()
            processed_msgs = 0
            for topic, msg, t in in_bag.read_messages():
                processed_msgs += 1
                if processed_msgs % 1000 == 0:
                    rospy.loginfo(f"已处理 {processed_msgs}/{total_msgs} 条消息")
 
                # 处理点云
                if topic == LIDAR_TOPIC:
                    pc_data = manual_parse_pc2(msg)
                    if pc_data is not None:
                        custom_msg = convert_to_custommsg(pc_data)
                        out_bag.write(LIDAR_TOPIC, custom_msg, t)
                        lidar_count += 1
                        if lidar_count % 50 == 0:
                            rospy.loginfo(f"已转换 {lidar_count} 帧点云")
 
                # 处理IMU
                elif topic in IMU_TOPICS and hasattr(msg, '_type') and msg._type == 'sensor_msgs/Imu':
                    out_bag.write(topic, msg, t)
                    imu_count += 1
 
    # 统计结果
    topic_info = in_bag.get_type_and_topic_info()[1]
    original_lidar = topic_info[LIDAR_TOPIC].message_count if LIDAR_TOPIC in topic_info else 0
    original_imu = sum(topic_info[t].message_count for t in IMU_TOPICS if t in topic_info)
 
    rospy.loginfo("="*60)
    rospy.loginfo(f"✅ 转换完成！输出路径：{OUTPUT_BAG_PATH}")
    rospy.loginfo(f"📊 点云：{lidar_count}/{original_lidar} 帧")
    rospy.loginfo(f"📊 IMU：{imu_count}/{original_imu} 条")
    rospy.loginfo("="*60)
 
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("转换被中断")
    except Exception as e:
        rospy.logerr(f"❌ 致命错误：{str(e)}")
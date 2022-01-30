#!/usr/bin/python
import rosbag
import yaml
import numpy as np

from geometry_msgs.msg import Twist, TwistStamped, Pose, PoseStamped, PointStamped, WrenchStamped
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState

# dir = '/home/zhanjet/moying_ws/src/mobile_manipulator_controller/whole_body_control/record/'
dir = '/home/zhanjet/moying_ws/src/mobile_manipulator_controller/admittance_control/record/'
bag_files = ['obst_avoid_2022-01-20-12-26-20.bag',
            'obst_avoid_line_2022-01-22-02-44-37.bag',
            'hri_avoid_2022-01-23-01-24-08.bag',
            'obst_avoid_line_2022-01-23-01-24-14.bag',
            'polish_2022-01-24-00-37-51.bag']

msg_types = ['nav_msgs/Odometry', 
            'geometry_msgs/PoseStamped', 'geometry_msgs/TwistStamped', 'geometry_msgs/PointStamped', 'geometry_msgs/WrenchStamped',
            'std_msgs/Float64', 'std_msgs/Float64MultiArray',
            'sensor_msgs/JointState']

topic_type_dict = {}

def read_bag(bagfile):
    global inbag
    inbag = rosbag.Bag(bagfile, 'r')
    return read_topic_type()

def read_topic_type():
    # topic_type_dict = {}
    info_dict = yaml.load(inbag._get_yaml_info(), Loader=yaml.FullLoader)
    for x in info_dict['topics']:
         topic_type_dict[x['topic']] = x['type']
    return topic_type_dict

def read_msg(topics):
    data = {}
    if len(topics) > 0:
        for topic, msg, t in inbag.read_messages():
         if topics.count(topic):
            if topic_type_dict[topic] == 'nav_msgs/Odometry':
                data = update_odometry(data, topic, msg, t)
            elif topic_type_dict[topic] == 'geometry_msgs/PoseStamped':
                data = update_pose(data, topic, msg, t)
            elif topic_type_dict[topic] == 'geometry_msgs/TwistStamped':
                data = update_twist(data, topic, msg.twist, t)
            elif topic_type_dict[topic] == 'geometry_msgs/Twist':
                data = update_twist(data, topic, msg, t)
            elif topic_type_dict[topic] == 'geometry_msgs/PointStamped':
                data = update_point(data, topic, msg, t)
            elif topic_type_dict[topic] == 'geometry_msgs/WrenchStamped':
                data = update_wrench(data, topic, msg, t)
            elif topic_type_dict[topic] == 'std_msgs/Float64':
                data = update_float64(data, topic, msg, t)
            elif topic_type_dict[topic] == 'std_msgs/Float64MultiArray':
                data = update_float64arr(data, topic, msg, t)
            elif topic_type_dict[topic] == 'sensor_msgs/JointState':
                data = update_jntstate(data, topic, msg, t)

    return data

def update_odometry(data, topic, msg, t):
    quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    [r, p, y] = euler_from_quaternion(quat)

    if topic in data:
        data[topic]['x'] = np.append(data[topic]['x'], msg.pose.pose.position.x)
        data[topic]['y'] = np.append(data[topic]['y'], msg.pose.pose.position.y)
        data[topic]['z'] = np.append(data[topic]['z'], msg.pose.pose.position.z)
        data[topic]['vx'] = np.append(data[topic]['vx'], msg.twist.twist.linear.x)
        data[topic]['vy'] = np.append(data[topic]['vy'], msg.twist.twist.linear.y)
        data[topic]['vz'] = np.append(data[topic]['vz'], msg.twist.twist.linear.z)
        data[topic]['roll'] = np.append(data[topic]['roll'], r)
        data[topic]['pitch'] = np.append(data[topic]['pitch'], p)
        data[topic]['yaw'] = np.append(data[topic]['yaw'], y)
        data[topic]['wx'] = np.append(data[topic]['wx'], msg.twist.twist.angular.x)
        data[topic]['wy'] = np.append(data[topic]['wy'], msg.twist.twist.angular.y)
        data[topic]['wz'] = np.append(data[topic]['wz'], msg.twist.twist.angular.z)
        data[topic]['t'] = np.append(data[topic]['t'], t.to_sec())
    else:
        data[topic] = {}
        data[topic]['x'] = np.array([msg.pose.pose.position.x])
        data[topic]['y'] = np.array([msg.pose.pose.position.y])
        data[topic]['z'] = np.array([msg.pose.pose.position.z])
        data[topic]['vx'] = np.array([msg.twist.twist.linear.x])
        data[topic]['vy'] = np.array([msg.twist.twist.linear.y])
        data[topic]['vz'] = np.array([msg.twist.twist.linear.z])
        data[topic]['wx'] = np.array([msg.twist.twist.angular.x])
        data[topic]['wy'] = np.array([msg.twist.twist.angular.y])
        data[topic]['wz'] = np.array([msg.twist.twist.angular.z])
        data[topic]['roll'] = np.array([r])
        data[topic]['pitch'] = np.array([p])
        data[topic]['yaw'] = np.array([y])
        data[topic]['t'] = np.array([t.to_sec()])
    return data

def update_pose(data, topic, msg, t):
    quat = [msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w]
    [r, p, y] = euler_from_quaternion(quat)

    if topic in data:
        data[topic]['x'] = np.append(data[topic]['x'], msg.pose.position.x)
        data[topic]['y'] = np.append(data[topic]['y'], msg.pose.position.y)
        data[topic]['z'] = np.append(data[topic]['z'], msg.pose.position.z)
        data[topic]['roll'] = np.append(data[topic]['roll'], r)
        data[topic]['pitch'] = np.append(data[topic]['pitch'], p)
        data[topic]['yaw'] = np.append(data[topic]['yaw'], y)
        data[topic]['t'] = np.append(data[topic]['t'],t.to_sec())
    else:
        data[topic] = {}
        data[topic]['x'] = np.array([msg.pose.position.x])
        data[topic]['y'] = np.array([msg.pose.position.y])
        data[topic]['z'] = np.array([msg.pose.position.z])
        data[topic]['roll'] = np.array([r])
        data[topic]['pitch'] = np.array([p])
        data[topic]['yaw'] = np.array([y])
        data[topic]['t'] = np.array([t.to_sec()])

    return data

def update_twist(data, topic, msg, t):
    if topic in data:
        data[topic]['vx'] = np.append(data[topic]['vx'], msg.linear.x)
        data[topic]['vy'] = np.append(data[topic]['vy'], msg.linear.y)
        data[topic]['vz'] = np.append(data[topic]['vz'], msg.linear.z)
        data[topic]['wx'] = np.append(data[topic]['wx'], msg.angular.x)
        data[topic]['wy'] = np.append(data[topic]['wy'], msg.angular.y)
        data[topic]['wz'] = np.append(data[topic]['wz'], msg.angular.z)
        data[topic]['t'] = np.append(data[topic]['t'], t.to_sec())
    else:
        data[topic] = {}
        data[topic]['vx'] = np.array([msg.linear.x])
        data[topic]['vy'] = np.array([msg.linear.y])
        data[topic]['vz'] = np.array([msg.linear.z])
        data[topic]['wx'] = np.array([msg.angular.x])
        data[topic]['wy'] = np.array([msg.angular.y])
        data[topic]['wz'] = np.array([msg.angular.z])
        data[topic]['t'] = np.array([t.to_sec()])

    return data

def update_point(data, topic, msg, t):
    if topic in data:
        data[topic]['x'] = np.append(data[topic]['x'], msg.point.x)
        data[topic]['y'] = np.append(data[topic]['y'], msg.point.y)
        data[topic]['z'] = np.append(data[topic]['z'], msg.point.z)
        data[topic]['t'] = np.append(data[topic]['t'], t.to_sec())
    else:
        data[topic] = {}
        data[topic]['x'] = np.array([msg.point.x])
        data[topic]['y'] = np.array([msg.point.y])
        data[topic]['z'] = np.array([msg.point.z])
        data[topic]['t'] = np.array([t.to_sec()])

    return data

def update_wrench(data, topic, msg, t):
    if topic in data:
        data[topic]['fx'] = np.append(data[topic]['fx'], msg.wrench.force.x)
        data[topic]['fy'] = np.append(data[topic]['fy'], msg.wrench.force.y)
        data[topic]['fz'] = np.append(data[topic]['fz'], msg.wrench.force.z)
        data[topic]['tx'] = np.append(data[topic]['tx'], msg.wrench.torque.x)
        data[topic]['ty'] = np.append(data[topic]['ty'], msg.wrench.torque.y)
        data[topic]['tz'] = np.append(data[topic]['tz'], msg.wrench.torque.z)
        data[topic]['t'] = np.append(data[topic]['t'], t.to_sec())
    else:
        data[topic] = {}
        data[topic]['fx'] = np.array([msg.wrench.force.x])
        data[topic]['fy'] = np.array([msg.wrench.force.y])
        data[topic]['fz'] = np.array([msg.wrench.force.z])
        data[topic]['tx'] = np.array([msg.wrench.torque.x])
        data[topic]['ty'] = np.array([msg.wrench.torque.y])
        data[topic]['tz'] = np.array([msg.wrench.torque.z])
        data[topic]['t'] = np.array([t.to_sec()])

    return data

def update_float64(data, topic, msg, t):
    if topic in data:
        data[topic]['data'] = np.append(data[topic]['data'], msg.data)
        data[topic]['t'] = np.append(data[topic]['t'], t.to_sec())
    else:
        data[topic] = {}
        data[topic]['data'] = np.array([msg.data])
        data[topic]['t'] = np.array([t.to_sec()])

    return data

def update_float64arr(data, topic, msg, t):
    if topic in data:
        data[topic]['data'] = np.append(data[topic]['data'], msg.data)
        data[topic]['t'] = np.append(data[topic]['t'], t.to_sec())
    else:
        data[topic] = {}
        data[topic]['data'] = np.array([msg.data])
        data[topic]['t'] = np.array([t.to_sec()])
    return data

def update_jntstate(data, topic, msg, t):
    jnt_num = 4
    if topic in data:
        data[topic]['q'] = np.append(data[topic]['q'], msg.position[jnt_num:jnt_num+6])
        data[topic]['t'] = np.append(data[topic]['t'], t.to_sec())
    else:
        data[topic] = {}
        data[topic]['q'] = np.array([msg.position[jnt_num:jnt_num+6]])
        data[topic]['t'] = np.array([t.to_sec()])

    return data


if __name__ == "__main__":
    # for test
    topic_type_dict = read_bag(bagfile=dir+bag_files[4])
    # print(topic_type_dict)
    data = read_msg(topic_type_dict.keys())
    print(data[topic_type_dict.keys()[0]]['t'])

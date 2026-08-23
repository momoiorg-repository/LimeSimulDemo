from typing import List
from math import radians
import numpy as np
import time
import os
from operator import add

import pickle

from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from tf2_ros.buffer import Buffer 
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image, CameraInfo, JointState
from geometry_msgs.msg import Twist
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates, LinkStates

import transforms3d
from pymoveit2 import MoveIt2, GripperInterface
from vector_map import get_map, get_map_ROS, SimulationSpace, init_visualize
import pyrealsense2 as rs

from ros_actor import actor, SubSystem
from .approach_action import ApproachAction
from .cognitive import CognitiveNetwork
from .manipulator import ManipulatorNetwork
from .tools import Tools

#######################################################
#
#   Turtlebot3 dependent definitions
#
#######################################################

MOVE_GROUP_ARM: str = "arm"
MOVE_GROUP_GRIPPER: str = "gripper"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.019, 0.019]
# CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.007, 0.007]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [-0.010, -0.010]

def joint_names() -> List[str]:
    return [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
    ]

def base_link_name() -> str:
    return "base_link"

def end_effector_name() -> str:
    return "end_effector_link"

def gripper_joint_names() -> List[str]:
    return [
        "gripper_left_joint",
        "gripper_right_joint",
    ]

class Tb3(SubSystem):
    def __init__(self, name, parent):
        super().__init__(name, parent)
        self.add_subsystem('navigation', Tb3NavigationSystem)
        self.add_subsystem('manipulator', Tb3ManipulatorSystem)
        self.add_subsystem('camera', Tb3CameraSystem)
        self.register_subscriber('model_states', ModelStates, 'model_states', 10)
        self.register_subscriber('link_states', LinkStates, 'link_states', 10)
        self.add_network(Tools)
        node = self.get_value('node')
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node)
        self.set_value('tf_buffer', tf_buffer)
    
    def get_trans(self, from_frame, to_frame):
        tf_buffer = self.get_value('tf_buffer')
        if not tf_buffer.can_transform(
                to_frame,
                from_frame,
                rclpy.time.Time()):
            return None
        tf = None
        try:
            tf = tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                rclpy.time.Time())
        except TransformException as ex:
            print(ex)
        return tf

    @actor
    def map_trans(self, src="camera_link"):
        while True:
            ret = self.get_trans(src, "map")
            if ret: return ret
            self.run_actor('sleep', 1)
    
    @actor
    def var_trans(self, target="link1"):
        while True:
            ret = self.get_trans("camera_link", target)
            if ret: return ret
            self.run_actor('sleep', 1)
    
    @actor
    def uni_trans(self, src="camera_link", target="map"):
        while True:
            ret = self.get_trans(src, target)
            if ret: return ret
            self.run_actor('sleep', 1)
    
    @actor
    def base_trans(self):
        while True:
            ret = self.get_trans("camera_link", "base_link")
            if ret: return ret
            self.run_actor('sleep', 1)
    
    @actor
    def gripper_trans(self):
        while True:
            ret = self.get_trans("link5", "base_link")
            if ret: return ret
            self.run_actor('sleep', 1)
    
    @actor
    def sleep(self, st):
        time.sleep(st)
    
class Tb3NavigationSystem(SubSystem):
    def __init__(self, name, parent):
        super().__init__(name, parent)
        self.register_action('navigate', NavigateToPose, "/navigate_to_pose")
        self.register_publisher('motor', Twist, 'cmd_vel', 10)
        self.register_subscriber('odom', Odometry, 'odom', 10)
        self.add_network(ApproachAction)
        self.set_value('current_pose', (0.0, 0.0, 0.0))
    
    def create_move_base_goal(self, x, y, theta):
        """ Creates a MoveBaseGoal message from a 2D navigation pose """
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        node = self.get_value('node')
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        quat = transforms3d.euler.euler2quat(0, 0, theta)
        goal.pose.pose.orientation.w = quat[0]
        goal.pose.pose.orientation.x = quat[1]
        goal.pose.pose.orientation.y = quat[2]
        goal.pose.pose.orientation.z = quat[3]
        return goal

    @actor
    def goto(self, x, y, theta):
        goal = self.create_move_base_goal(x, y, theta)
        result = self.run_actor('navigate', goal)
        self.set_value('current_pose', (x, y, theta))
#        return (result.status == GoalStatus.STATUS_SUCCEEDED)
        return True

    @actor
    def goto_deg(self, x, y, degree):
        rad = radians(degree)
        theta = round(rad, 2)
        x, y = float(x), float(y)

        goal = self.create_move_base_goal(x, y, theta)
        result = self.run_actor('navigate', goal)
        self.set_value('current_pose', (x, y, theta))
#        return (result.status == GoalStatus.STATUS_SUCCEEDED)
        return True

    
    @actor
    def migrate(self, dx=0.0, dy=0.0, dtheta=0.0):
        pose = self.get_value('current_pose')
        pose = list(map(add, pose, (dx, dy, dtheta)))
        self.run_actor('goto', *pose)

class Tb3CameraSystem(SubSystem):
    def __init__(self, name, parent):
        super().__init__(name, parent)
        self.set_value('cv_bridge', CvBridge())
        self.register_subscriber('pic',Image,"/camera/camera/color/image_raw",10)
        self.register_subscriber('depth',Image,"/camera/camera/depth/image_rect_raw",10)
        self.register_subscriber('camera_info',CameraInfo,"/camera/camera/color/camera_info",10)
        self.add_network(CognitiveNetwork)
    
    @actor
    def init_realsense(self):
        info_mes = self.run_actor('camera_info')
        it = rs.intrinsics()
        it.width = info_mes.width
        it.height = info_mes.height
        it.fx = info_mes.k[0]
        it.ppx = info_mes.k[2]
        it.fy = info_mes.k[4]
        it.ppy = info_mes.k[5]
        self.set_value('intrinsics', it)

class Tb3ManipulatorSystem(SubSystem):
    def __init__(self, name, parent):
        super().__init__(name, parent)
        self.add_network(ManipulatorNetwork)
        
        node = self.get_value('node')
        cg = self.get_value('callback_group')
        arm = MoveIt2(
            node=node,
            joint_names=joint_names(),
            base_link_name=base_link_name(),
            end_effector_name=end_effector_name(),
            group_name=MOVE_GROUP_ARM,
            callback_group=cg,
            use_move_group_action=True,
        )
        gripper = GripperInterface(
            node=node,
            gripper_joint_names=gripper_joint_names(),
            open_gripper_joint_positions=OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=MOVE_GROUP_GRIPPER,
            callback_group=cg,
#            follow_joint_trajectory_action_name="gripper_trajectory_controller/follow_joint_trajectory",
#            gripper_command_action_name="gripper_action_controller/gripper_cmd",
            follow_joint_trajectory_action_name="arm_controller/follow_joint_trajectory",
            gripper_command_action_name="gripper_controller/gripper_cmd",
            use_move_group_action=True,
        )
        arm.max_velocity = 0.5
        arm.max_acceleration = 0.5
        self.set_value('arm', arm)
        self.set_value('gripper', gripper) 
        self.set_value('joint_stat', [0.0, 0.0, 0.0, 0.0])  
        self.register_subscriber('joints',JointState,"/joint_states",10)
    
class MapSystem(SubSystem):
    def __init__(self, name, parent, map_file=None) -> None:
        super().__init__(name, parent)
        cache_file = os.path.expanduser('~/.actordemo/map_cache')
        if os.path.isfile(cache_file):
            with open(cache_file, 'rb') as f:
                world = pickle.load(f)
        else:
            if map_file:
                world = get_map_ROS(self.map_file)
            else:
                qos_profile=QoSProfile(
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=1)
                self.register_subscriber(
                    'map_topic', 
                    OccupancyGrid,
                    "/map",
                    qos_profile)
                data = self.run_actor('map_topic')
                width = data.info.width #map width, from nav_msgs/msg/MapMetaData
                height = data.info.height #map height, from nav_msgs/msg/MapMetaData
                map_array = np.array(data.data).reshape(height, width)
                map_array = np.flipud(map_array)

                # make ndarray that has the same size as of map_array
                pgm_array = np.zeros((height, width), dtype=np.uint8)
                pgm_array[(map_array == -1)] = 254
                pgm_array[(map_array < 90)] = 254 
                pgm_array[(map_array >= 90)] = 0 

                resolution = data.info.resolution
                o = data.info.origin.position
                origin = (o.x, o.y)                
                world = get_map(pgm_array, resolution, origin)
                
            cache_dir = os.path.expanduser('~/.actordemo')
            os.makedirs(cache_dir, exist_ok=True)
            with open(cache_file, 'wb') as f:
                pickle.dump(world, f)
        self.set_value('world', world)
        
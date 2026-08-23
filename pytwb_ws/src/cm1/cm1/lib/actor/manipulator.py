import array
import operator
from math import radians, atan2, degrees, pi
import numpy as np
import quaternion
from simple_pid.pid import PID
from time import sleep

from control_msgs.action import GripperCommand as GripperCommandAction

from ros_actor import actor, SubNet
from pymoveit2 import MoveIt2State

adjust_plus = 1.1  
adjust_minus = 1.25

def wait_until_executed(arm):
    while(arm.query_state() != MoveIt2State.IDLE):
        sleep(0.5)

class ManipulatorNetwork(SubNet):
    # Lift the end effector straight up from wherever it currently is
    # (relative joint adjustment, not an absolute target), before handing
    # off to home(). home() plans a joint-space move from the current pose
    # with no idea an object is being held, so on a short/close pick it can
    # dip through a low path and drag the object along the ground; lifting
    # clear first avoids that regardless of where the pick happened.
    @actor
    def pick_up(self):
        joint = [0.0, radians(34), radians(87), 0.0, radians(-78), 0.0]
        self.run_actor('move_joint', *joint)
        return True


    # set to home position
    @actor
    def home(self):
#        joint = [0.0, 1.0, 1.575, 0.0, -1.0, 0.0]
        joint = [0.0, radians(-63), radians(107), 0.0, radians(56), 0.0]
        self.run_actor('move_joint', *joint)
        return True

    @actor
    def pos_home(self):
        angle = (0, 90, 0)
        pos = (0.3, 0.0, 0.15)
        self.run_actor('arm_pose', pos, list(map(radians, angle)))
        return True
        
    @actor
    def arm0(self):
        '''
        angle = (0, 90, 0)
        pos = (0.3, 0.0, 0.15)
        self.run_actor('arm_pose', pos, list(map(radians, angle)))
        '''
        joint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.run_actor('move_joint', *joint)
        return True

    @actor    
    def move_joint(self, *joint_values):
        joint_positions = array.array('d', joint_values)
#        node = self.get_value('node')
#        node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")
        res = self.run_actor("move_to_configuration", joint_positions)
        self.set_value('joint_stat', joint_values)
        return res
    
    # joint angle in degree units
    @actor
    def move_joint_degree(self, *args):
        return self.move_joint(*list(map(radians, args)))

    # designate diff value of each joint
    @actor
    def adjust_joint(self, *args):
        value = self.get_value('joint_stat')
        off = list(map(radians, args))
        return self.move_joint(*list(map(operator.add, value, off)))

    @actor
    def adjust_joint_radian(self, *args):
        value = self.get_value('joint_stat')
        return self.move_joint(*list(map(operator.add, value, *args)))

    def move_position(self, position, orientation):
        position = array.array('d', position)
        orientation = array.array('d', orientation)
        self.node.get_logger().info(
            f"Moving to {{position: {list(position)}, quat_xyzw: {list(orientation)}}}"
        )
        self.run_actor('move_to_pose', position=position, quat_xyzw=orientation)
    
    # move arm and wait until done
    @actor
    def move_to_configuration(self, joint_positions):
        arm = self.get_value('arm')
        arm.move_to_configuration(joint_positions, joint_names=arm.joint_names)
        wait_until_executed(arm)
        return True         
#        return arm.wait_until_executed() # Never use this
    
    # open gripper
    @actor
    def open(self):
        gripper = self.get_value('gripper')
        gripper.open()
        wait_until_executed(gripper)
        return True

    # close gripper
    @actor
    def close(self):
        gripper = self.get_value('gripper')
        gripper.close()
        wait_until_executed(gripper)
        return True

    @actor
    def get_status(self):
        return {
            'joint': self.get_value('jstat')
        }
    
    # adjust arm angle
    @actor
    def ad(self):
        x, y, angle = self.run_actor('object_loc')
        if angle > 0:
            angle *= adjust_plus
        else:
            angle *= adjust_minus
        jstat = self.run_actor('jstat')
        cur = []
        for k in ('joint2','joint3','joint4','joint5','joint6'):
            cur.append(jstat[k])
        cur.insert(0,angle)
#        print(f'adjust.angle:{degrees(angle)}')
        self.run_actor('move_joint', *cur)
        self.run_actor('sleep', 1)
        return True
    
    @actor
    def arm_turn(self, value=0):
        jstat = self.run_actor('jstat')
        cur = []
        for k in ('joint2','joint3','joint4','joint5','joint6'):
            cur.append(jstat[k])
        cur.insert(0,value)
#        print(f'arm_turn.angle:{degrees(angle)}')
        self.run_actor('move_joint', *cur)
        self.run_actor('sleep', 1)
        return True

    @actor
    def ad0(self):
        jstat = self.run_actor('jstat')
        cur = []
        for k in ('joint2','joint3','joint4','joint5','joint6'):
            cur.append(jstat[k])
        cur.insert(0,0.0)
        self.run_actor('move_joint', *cur)
        self.run_actor('sleep', 1)
        return True

    # adjust arm base direction to the center of coke can
    # when arm down with variable distance
    @actor
    def fit(self, assumed_distance=0.15):
        _, _, angle, distance = self.run_actor('measure_center', assumed=-1)
        if distance < 0.1:
            print('could not gain depth')
            self.run_actor('fit2')
            return
        print(f'arm angle: {degrees(angle)}, distance: {distance}')
        jstat = self.run_actor('jstat')
        cur = []
        for k in ('joint2','joint3','joint4','joint5','joint6'):
            cur.append(jstat[k])
#        cur.insert(0,angle-radians(0.4))
        cur.insert(0,angle)
        print(f'fit: {cur}')
        self.run_actor('move_joint', *cur)
        self.run_actor('sleep', 1)
        return True

    # adjust arm base direction
    # when arm down and fixed distance (ready for grasp)
    @actor
    def fit2(self, distance):
        jstat = self.run_actor('jstat')
        cur = []
        for k in ('joint1', 'joint2','joint3','joint4','joint5','joint6'):
            cur.append(jstat[k])
        _,_,target_angle,_ = self.run_actor('measure_center2', assumed=distance)
        cur[0] = target_angle
        self.run_actor('move_joint', *cur)
        self.run_actor('sleep', 3)
        return True
    
    @actor
    def tf(self):
        pic_diff, edge = self.run_actor('find_object_pic')
        off = pic_diff - edge - 0.491
        print(f'off:{off}')
        angle = -atan2(off, 1)
        print(f'angle:{degrees(angle)}')
        return True

    # set arm to pick position
    @actor
    def pick(self, dir=0.0):
        '''
        angle = (0, 90, 0)
        pos = (0.3, 0.0, 0.05)
        self.run_actor('arm_pose', pos, list(map(radians, angle)))
        self.run_actor('sleep', 3.0)
        return True
        '''
        # joint = [0.0, 1.75, 1.4, 0.0, -1.6, 0.0]
        joint = [0.0, radians(97), radians(83), 0.0, radians(-83), 0.0]
        joint[0] = dir
        self.run_actor('move_joint', *joint)
        return True
    
    # set arm to place position
    @actor
    def place(self):
        val = (0.0, 50.0, -33.0, -16.0)
        self.run_actor('move_joint', *map(radians, val))
        return self.run_actor('open')

    @actor    
    def pid(self, *joint_values):
        arm = self.get_value('arm')        
        print(f'planner_id:{arm.planner_id}')
        return True

    @actor    
    def pose0(self, *joint_values):
        angle = (0, 90, 0)
        pos = (0.098, 0.0, 0.2497)
        self.run_actor('arm_pose', pos, list(map(radians, angle)))
        return True
    
    @actor    
    def pose1(self, *joint_values):
        angle = (0, 90, 0)
        pos = (0.3, 0.0, 0.05)
        self.run_actor('arm_pose', pos, list(map(radians, angle)))
        return True
    
    @actor
    def arm_pose(self, pos, angle):       
        q = quaternion.from_euler_angles(angle)
        quat_xyzw = (q.x, q.y, q.z, q.w)
        arm = self.get_value('arm')
        arm.move_to_pose(position=pos, quat_xyzw=quat_xyzw, cartesian=False)
        wait_until_executed(arm)
        return True

    @actor
    def jstat(self):
        arm = self.get_value('arm')
        state = arm.joint_state
        state_dict = dict(zip(state.name, state.position))
        return state_dict
        
    @actor
    def arm_angle(self, j1=0, j2=0, j3=0, j4=0, j5=0, j6=0):
        joint = [radians(j1), radians(j2), radians(j3), radians(j4), radians(j5), radians(j6)]
        self.run_actor('move_joint', *joint)
        return True
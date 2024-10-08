from math import atan2, degrees
import os

from pyquaternion import Quaternion

from pytwb import lib_main
from ros_actor import SubNet, actor, register_bt
from ..pointlib import PointEx

class Tools(SubNet):
    # command version
    @actor
    def go(self, *arg):
        arg = list(map(float, arg))
        while len(arg) < 3:
            arg.append(0.0)
        return self.run_actor('goto', arg[0], arg[1], arg[2])

    # update behavior tree name table
    @actor
    def update_bt(self):
        package = lib_main.get_package()
        dir = os.path.join(package.path, 'trees')
        for d in os.listdir(dir):
            if not d.endswith('.xml'): continue
            name = d[:-4]
            register_bt(name)
        return True
    
    # show gripper pose
    @actor
    def gl(self):
        trans = self.run_actor("gripper_trans")
        trans = trans.transform
        offset = trans.translation
        rot = trans.rotation
        point = (0.0, 0.0, 0.0)
        q_rot = Quaternion(rot.w, rot.x, rot.y, rot.z)
        rot_point = q_rot.rotate(point)
        x = rot_point[0] + offset.x
        y = rot_point[1] + offset.y
        print(f'gripper angle:{degrees(atan2(y, x))}')
    
    @actor
    def forward(self, value):
        self.run_actor('adjust_joint', 0.0, value, 0.0, 0.0)
    
    @actor
    def ol(self):
        _,_,target_angle,distance = self.run_actor('measure_center', target='base_link', assumed=0.2)
        print(f'target_angle:{degrees(target_angle)}, distance:{distance}')
    
    @actor
    def tl(self):
        _, _, angle = self.run_actor('object_loc', 'base_link')
        point = self.run_actor('find_object')
        print(f'object angle:{degrees(angle)}, distance:{point.distance}') 
    
    @actor    
    def js(self):
        value = self.get_value('joint_stat')
        d_value = tuple(map(degrees, value))
        print(d_value)
    
    @actor
    def cpos(self):
        root = PointEx()
        ref = PointEx(1.0, 0.0)
        trans = self.run_actor('map_trans')
        if not trans: return
        root.setTransform(trans.transform)
        ref.setTransform(trans.transform)
        print(f'x:{root.x}, y:{root.y}, z:{degrees(root.z)}')
        rx = ref.x - root.x
        ry = ref.y - root.y
        robot_angle = atan2(ry, rx)
        print(f'robot angle to X axis: {degrees(robot_angle)}')
    
    @actor
    def pause(self, is_on=True):
        if is_on: input('debug pause')
        return True
    
    @actor
    def key(self):
        return input()

    @actor
    def angle(self):
        print(f'assumed:{degrees(atan2(0.5, 1.0))}')
        x, y, angle = self.run_actor('object_loc')
        print(f'angle:{degrees(angle)}')
    
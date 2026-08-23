import py_trees

from pytwb.common import behavior
from ros_actor import run_actor, run_actor_async

from lib.pointlib import PointEx

@behavior
class Viewer(py_trees.behaviour.Behaviour):
    desc = 'just camera snap shot'

    def __init__(self, name, node, mode="permanent"):
        super(Viewer, self).__init__(name)
        self.mode = mode
        self.found = False
    
    def initialise(self):
        if self.mode == 'one_shot':
            num_thresh = 0
            i_num = 1
        elif self.mode == 'permanent':
            num_thresh = 0
            i_num = -1
        else:
            num_thresh = 1
            i_num = -1
        run_actor_async('find_coke', self.actor_callback, num_thresh, i_num)

    def update(self):
        if self.mode == 'one_shot':
            return py_trees.common.Status.SUCCESS
        elif self.mode == 'permanent':
            return py_trees.common.Status.RUNNING
        if self.found:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def actor_callback(self, candidate):
        if candidate:
            self.found = True
        
@behavior
class LookForCoke(py_trees.behaviour.Behaviour):
    desc = 'decide target location by a single glance'

    def __init__(self, name, node, debug=False):
        super(LookForCoke, self).__init__(name)
        self.target = None
        self.node = node
        self.bb = py_trees.blackboard.Blackboard()
        self.debug = debug
        self.tran = None

    def initialise(self):
        self.tran = run_actor_async('object_glance', self.actor_callback)
        self.logger.info("start looking for coke")

    def update(self):
        target = self.target
        if not target:
            return py_trees.common.Status.RUNNING
        trans = run_actor('map_trans')
        target.setTransform(trans.transform)
        self.bb.set('glanced_point', target)
        return py_trees.common.Status.SUCCESS
    
    def actor_callback(self, candidate):
        if not candidate: return
        self.target = PointEx(candidate)
    
    def terminate(self, new_status):
        if self.target:
            self.logger.info(f"found at x:{self.target.x} y:{self.target.y}")
        else:
            if self.tran:
                self.tran.abort(self.tran)
            self.logger.info(f"Terminated {new_status}")
 
class ResPoint:
    def __init__(self, point_bag) -> None:
        self.x = point_bag.x
        self.y = point_bag.y
        self.distance = point_bag.last_point.distance

@behavior
class Watch(py_trees.behaviour.Behaviour):
    desc = 'watch target repeatedly to get reliable location data'

    def __init__(self, name, node, debug=False):
        super(Watch, self).__init__(name)
        self.bb = py_trees.blackboard.Blackboard()
        self.debug = debug
        self.candidate = None

    def initialise(self):
        self.running = True
        run_actor_async('get_found', self.actor_callback, 10, 3)
        self.logger.info("watching target")

    def update(self):
        if self.candidate:
            self.bb.set('found_point', self.candidate)
            return py_trees.common.Status.SUCCESS
        if self.running:
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.FAILURE

    def actor_callback(self, point_bag):
        self.candidate = point_bag
        self.running = False
    
    def terminate(self, new_status):
        if self.candidate:
            self.bb.set('target_pose', [self.candidate.x, self.candidate.y, 1.57])
            self.logger.info(f"Concluded x:{self.candidate.x}, y:{self.candidate.y}")
        else:
            self.logger.info(f"Terminated {new_status}")

@behavior
class GetObjFront(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, debug=False):
        super(GetObjFront, self).__init__(name)
        self.bb = py_trees.blackboard.Blackboard()
        self.debug = False

    def initialise(self):
        self.running = True
        self.x, self.y, self.theta = run_actor('object_front', "map")
        self.logger.info(f"Concluded x:{self.x}, y:{self.y}")

    def update(self):
        if self.x and self.y and self.theta:
            self.bb.set('target_pose', [self.x, self.y, self.theta])
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
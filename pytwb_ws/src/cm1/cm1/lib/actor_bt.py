import py_trees

from pytwb.common import behavior
from ros_actor import run_actor_async

from threading import Semaphore

class SharedData:
    def __init__(self) -> None:
        self.sem = Semaphore()
        self.callee = []
        self.status = py_trees.common.Status.INVALID
    
    def set_callee(self, callee):
        with self.sem:
            self.callee = callee

    def execute(self):
        type, args = self.callee.pop(0)
        if not args: args = ()
        self.tran = run_actor_async(type, self.actor_callback, *args)
        
    def actor_callback(self, result):
        has_next = False
        with self.sem:
            if len(self.callee) == 0:
                if result == False:
                    self.status = py_trees.common.Status.FAILURE
                else:
                    self.status = py_trees.common.Status.SUCCESS
            else:
                has_next = True
        if has_next:
            self.execute()
    
    def get_status(self):
        with self.sem:
            ret = self.status
        return ret

    def initialise(self):
        with self.sem:
            self.status = py_trees.common.Status.RUNNING
        self.execute()
    
    def close(self):
        self.tran.abort(self.tran)

class ActorBT(py_trees.behaviour.Behaviour):
    def __init__(self, name, type, *args):
        super().__init__(name)
        self.args = args
        self.type = type
    
    def prepare(self):
        type = self.type
        self.shared = SharedData()
        if isinstance(type, tuple):
            self.callee = list(type)
        else:
            self.callee = [(type, self.args)]
        self.shared.set_callee(self.callee)
        self.logger.info(f'start {type}')
    
    def run(self):
        self.shared.initialise()
    
    def initialise(self):
        self.prepare()
        self.run()

    def update(self):
        return self.shared.get_status()          

    def terminate(self, new_status):
        self.shared.close()
        return True
#        self.logger.info(f"Terminated with status {new_status}")

    def set_callee(self, callee):
        self.shared.set_callee(callee)

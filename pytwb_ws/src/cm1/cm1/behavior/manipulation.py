import py_trees

from pytwb.common import behavior

from lib.actor_bt import ActorBT

@behavior
class Adjust(ActorBT):
    desc = 'adjust arm angle'

    def __init__(self, name, node):
        super().__init__(name, 'ad')

@behavior
class Fit(ActorBT):
    desc = 'adjust arm angle after gripper down'

    def __init__(self, name, node):
        super().__init__(name, 'fit')

@behavior
class Fit2(ActorBT):
    desc = 'adjust arm angle after gripper down'

    def __init__(self, name, node, distance):
        super().__init__(name, 'fit2', distance)
        
@behavior
class Pick(ActorBT):
    desc = 'pick object'

    def __init__(self, name, node):
        super().__init__(name, 
            (
                ('open', None),
                ('pick', None)
            )
        )

@behavior
class Place(ActorBT):
    desc = 'pick object'

    def __init__(self, name, node):
        super().__init__(name, 
            (
                ('pick', None),
                ('open', None)
            )
        )


@behavior
class Open(ActorBT):
    desc = 'gripper open'

    def __init__(self, name, node):
        super().__init__(name, 'open')
        
@behavior
class PickUp(ActorBT):
    desc = 'lift the end effector straight up before returning home'

    def __init__(self, name, node):
        super().__init__(name, 'pick_up')

@behavior
class ArmHome(ActorBT):
    desc = 'set arm home position'

    def __init__(self, name, node):
#        super().__init__(name, 'home')
        super().__init__(name, 
            (
                ('sleep', (1,)),
                ('home', None)
            )
        )
# add
@behavior
class Close(ActorBT):
    def __init__(self, name, node):
        super().__init__(name, 'close')

@behavior
class Arm0(ActorBT):
    def __init__(self, name, node):
        super().__init__(name, 'arm0')

@behavior
class FullClose(ActorBT):
    def __init__(self, name, node):
        super().__init__(name, 'full_close')

@behavior
class ArmTurn(ActorBT):
    def __init__(self, name, node, value=0):
        super().__init__(name, 'arm_turn', value=0)

@behavior
class Ad0(ActorBT):
    def __init__(self, name, node):
        super().__init__(name, 'ad0')

@behavior
class ArmAngle(ActorBT):
    def __init__(self, name, node, j1, j2, j3, j4, j5, j6):
        super().__init__(name, 'arm_angle', j1, j2, j3, j4, j5, j6)

